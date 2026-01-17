#!/usr/bin/env python3
"""
Operator Control Panel - CLI for Human Operator Intervention

This tool provides an easy-to-use command-line interface for operators
to monitor and control the cafeteria robot's state machine.

Usage:
    ros2 run cafeteria_robot_fsm operator_panel

Commands:
    status  - Show current robot state
    resume  - Resume from emergency stop
    abort   - Abort current mission
    theft   - Confirm theft incident
    return  - Force return to home
    deliver - Start new delivery
    help    - Show help
    quit    - Exit panel

Author: Hakan
Date: 2026-01-17
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import sys
import threading

from cafeteria_interfaces.msg import RobotState
from cafeteria_interfaces.srv import OperatorCommand, StartDelivery


class OperatorPanel(Node):
    """
    Interactive Operator Control Panel for the Cafeteria Robot.
    
    Provides both real-time state monitoring and command injection.
    """
    
    # State name mapping
    STATE_NAMES = {
        0: 'IDLE',
        1: 'NAVIGATING',
        2: 'THEFT_GRACE',
        3: 'EMERGENCY_STOP',
        4: 'RECOVERY_SEARCH',
        5: 'ARRIVED',
        6: 'RETURN_WITH_FOOD',
    }
    
    # State emoji/icons
    STATE_ICONS = {
        0: '🟢',  # IDLE
        1: '🚀',  # NAVIGATING
        2: '⚠️',  # THEFT_GRACE
        3: '🛑',  # EMERGENCY_STOP
        4: '🔍',  # RECOVERY_SEARCH
        5: '📍',  # ARRIVED
        6: '🏠',  # RETURN_WITH_FOOD
    }

    def __init__(self):
        super().__init__('operator_panel')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Current state tracking
        self.current_state = None
        self.current_state_name = 'UNKNOWN'
        self.time_in_state = 0.0
        self.status_message = ''
        
        # Subscribe to robot state
        self.state_sub = self.create_subscription(
            RobotState, '/robot_state',
            self.state_callback, 10,
            callback_group=self.callback_group
        )
        
        # Service clients
        self.operator_client = self.create_client(
            OperatorCommand, '/operator_command',
            callback_group=self.callback_group
        )
        self.delivery_client = self.create_client(
            StartDelivery, '/start_delivery',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('='*50)
        self.get_logger().info('🎛️  OPERATOR CONTROL PANEL')
        self.get_logger().info('='*50)
        self.get_logger().info('Type "help" for available commands')
        self.get_logger().info('')

    def state_callback(self, msg: RobotState):
        """Update current state from robot."""
        self.current_state = msg.current_state
        self.current_state_name = msg.state_name
        self.time_in_state = msg.time_in_state
        self.status_message = msg.status_message

    def print_status(self):
        """Print current robot status."""
        icon = self.STATE_ICONS.get(self.current_state, '❓')
        print('\n' + '='*50)
        print('📊 ROBOT STATUS')
        print('='*50)
        print(f'   State: {icon} {self.current_state_name}')
        print(f'   Time in state: {self.time_in_state:.1f}s')
        print(f'   Status: {self.status_message}')
        print('='*50 + '\n')

    def send_operator_command(self, command: int, command_name: str):
        """Send an operator command to the robot."""
        if not self.operator_client.wait_for_service(timeout_sec=2.0):
            print('❌ Error: Operator command service not available!')
            return
        
        request = OperatorCommand.Request()
        request.command = command
        request.operator_message = f'Command from operator panel: {command_name}'
        
        print(f'📤 Sending {command_name} command...')
        
        future = self.operator_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print(f'✅ {response.message}')
                print(f'   New state: {self.STATE_NAMES.get(response.new_state, "UNKNOWN")}')
            else:
                print(f'❌ Failed: {response.message}')
        else:
            print('❌ Error: No response from robot')

    def start_delivery(self, table_id: int):
        """Start a new delivery to specified table."""
        if not self.delivery_client.wait_for_service(timeout_sec=2.0):
            print('❌ Error: Delivery service not available!')
            return
        
        request = StartDelivery.Request()
        request.table_id = table_id
        request.priority = 0
        
        print(f'📤 Starting delivery to Table {table_id}...')
        
        future = self.delivery_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.accepted:
                print(f'✅ Delivery accepted!')
                print(f'   Mission ID: {response.mission_id}')
                print(f'   Estimated time: {response.estimated_time:.0f}s')
            else:
                print(f'❌ Rejected: {response.error_message}')
        else:
            print('❌ Error: No response from robot')

    def print_help(self):
        """Print help message."""
        print('''
╔════════════════════════════════════════════════════════════╗
║              🎛️  OPERATOR PANEL COMMANDS                   ║
╠════════════════════════════════════════════════════════════╣
║  status          - Show current robot state                ║
║  resume          - Resume from EMERGENCY_STOP              ║
║  abort           - Abort mission, return to IDLE           ║
║  theft           - Confirm theft incident                  ║
║  return          - Force immediate return to home          ║
║  deliver <1-5>   - Start delivery to table number          ║
║  help            - Show this help message                  ║
║  quit / exit     - Exit the operator panel                 ║
╚════════════════════════════════════════════════════════════╝
        ''')

    def run_interactive(self):
        """Run interactive command loop."""
        self.print_help()
        
        while rclpy.ok():
            try:
                # Print prompt with current state
                icon = self.STATE_ICONS.get(self.current_state, '❓')
                prompt = f'[{icon} {self.current_state_name}] operator> '
                
                user_input = input(prompt).strip().lower()
                
                if not user_input:
                    continue
                
                parts = user_input.split()
                command = parts[0]
                
                if command in ['quit', 'exit', 'q']:
                    print('👋 Goodbye!')
                    break
                
                elif command == 'help':
                    self.print_help()
                
                elif command == 'status':
                    self.print_status()
                
                elif command == 'resume':
                    self.send_operator_command(0, 'RESUME')
                
                elif command == 'abort':
                    self.send_operator_command(1, 'ABORT')
                
                elif command == 'theft':
                    self.send_operator_command(2, 'THEFT_CONFIRMED')
                
                elif command == 'return':
                    self.send_operator_command(3, 'FORCE_RETURN')
                
                elif command == 'deliver':
                    if len(parts) < 2:
                        print('❌ Usage: deliver <table_number>')
                        print('   Example: deliver 3')
                        continue
                    try:
                        table_id = int(parts[1])
                        if table_id < 1 or table_id > 5:
                            print('❌ Table must be 1-5')
                            continue
                        self.start_delivery(table_id)
                    except ValueError:
                        print('❌ Invalid table number')
                
                else:
                    print(f'❓ Unknown command: {command}')
                    print('   Type "help" for available commands')
                
            except KeyboardInterrupt:
                print('\n👋 Goodbye!')
                break
            except EOFError:
                break


def main(args=None):
    """Main entry point for the Operator Panel."""
    rclpy.init(args=args)
    
    panel = OperatorPanel()
    
    # Run state subscription in background thread
    executor = MultiThreadedExecutor()
    executor.add_node(panel)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # Give time for initial state to be received
    import time
    time.sleep(1.0)
    
    # Run interactive loop
    try:
        panel.run_interactive()
    finally:
        panel.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
