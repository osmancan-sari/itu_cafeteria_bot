#!/usr/bin/env python3
"""
Robot State Machine - The Central Brain of the Cafeteria Robot

This node implements a Finite State Machine (FSM) that orchestrates:
- Navigation to delivery targets
- Theft prevention (food monitoring)
- Emergency stop handling
- Recovery behaviors
- Operator intervention

States:
    IDLE            - Waiting for delivery command
    NAVIGATING      - Traveling to target, listening for events
    THEFT_GRACE     - Food lifted, waiting 30s for return
    EMERGENCY_STOP  - Collision detected, awaiting operator
    RECOVERY_SEARCH - Marker lost, executing search pattern
    ARRIVED         - At destination, waiting for pickup
    RETURN_WITH_FOOD- Returning home after pickup timeout

Author: Hakan
Date: 2026-01-17
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus
from enum import Enum, auto
from typing import Optional, Dict
import time
import math

# Standard ROS2 messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav2_msgs.action import NavigateToPose

# Our custom interfaces
from cafeteria_interfaces.msg import FoodStatus, CollisionAlert, RobotState
from cafeteria_interfaces.srv import OperatorCommand, StartDelivery


class RobotStateEnum(Enum):
    """
    Enumeration of all possible robot states.
    
    Each state represents a distinct behavior mode of the robot.
    The FSM transitions between these states based on events.
    """
    IDLE = 0              # Waiting at home for delivery command
    NAVIGATING = 1        # Traveling to target table
    THEFT_GRACE = 2       # Food lifted - 30s grace period
    EMERGENCY_STOP = 3    # Collision detected - operator needed
    RECOVERY_SEARCH = 4   # Marker lost - searching
    ARRIVED = 5           # At destination - waiting for pickup
    RETURN_WITH_FOOD = 6  # Returning home with untaken food


class StateHandler:
    """
    Base class for state-specific behavior handlers.
    
    Each state has an associated handler that defines:
    - on_enter: Called when entering this state
    - on_exit: Called when leaving this state  
    - on_update: Called periodically while in this state
    """
    
    def __init__(self, fsm: 'RobotStateMachine', state: RobotStateEnum):
        self.fsm = fsm
        self.state = state
        self.entry_time: Optional[float] = None
    
    def on_enter(self):
        """Called when the FSM transitions INTO this state."""
        self.entry_time = time.time()
        self.fsm.get_logger().info(f'[{self.state.name}] Entering state')
    
    def on_exit(self):
        """Called when the FSM transitions OUT OF this state."""
        duration = time.time() - self.entry_time if self.entry_time else 0
        self.fsm.get_logger().info(f'[{self.state.name}] Exiting state (was in state for {duration:.1f}s)')
    
    def on_update(self):
        """Called periodically while in this state. Override for state logic."""
        pass
    
    def get_time_in_state(self) -> float:
        """Returns seconds spent in this state."""
        if self.entry_time is None:
            return 0.0
        return time.time() - self.entry_time


class IdleHandler(StateHandler):
    """Handler for IDLE state - robot is waiting for delivery command."""
    
    def __init__(self, fsm: 'RobotStateMachine'):
        super().__init__(fsm, RobotStateEnum.IDLE)
    
    def on_enter(self):
        super().on_enter()
        # Stop all movement
        self.fsm.stop_robot()
        # Cancel any active navigation
        self.fsm.cancel_navigation()
        # Clear mission data
        self.fsm.current_target_table = None
        self.fsm.current_mission_id = None
        self.fsm.get_logger().info('Robot is idle. Waiting for delivery command...')


class NavigatingHandler(StateHandler):
    """
    Handler for NAVIGATING state - robot is traveling to target.
    
    This state:
    1. Uses Nav2 action client to navigate to the target table
    2. Monitors /food_status for theft detection
    3. Monitors /collision_alert for emergency stops
    4. Transitions to ARRIVED when goal is reached
    """
    
    def __init__(self, fsm: 'RobotStateMachine'):
        super().__init__(fsm, RobotStateEnum.NAVIGATING)
        self.navigation_started = False
        self.goal_handle: Optional[ClientGoalHandle] = None
    
    def on_enter(self):
        super().on_enter()
        self.navigation_started = False
        self.goal_handle = None
        
        table_id = self.fsm.current_target_table
        self.fsm.get_logger().info(f'🚀 Starting navigation to Table {table_id}')
        
        # Get target position for the table
        target_pose = self.fsm.get_table_position(table_id)
        if target_pose is None:
            self.fsm.get_logger().error(f'Unknown table ID: {table_id}')
            self.fsm.transition_to(RobotStateEnum.IDLE)
            return
        
        # Send navigation goal
        self.send_navigation_goal(target_pose)
    
    def on_exit(self):
        super().on_exit()
        # If we're leaving NAVIGATING for any reason, cancel the goal
        if self.goal_handle is not None:
            self.fsm.get_logger().info('Cancelling active navigation goal...')
            self.fsm.cancel_navigation()
    
    def send_navigation_goal(self, target_pose: PoseStamped):
        """Send a navigation goal to Nav2."""
        if not self.fsm.nav_client.wait_for_server(timeout_sec=5.0):
            self.fsm.get_logger().error('Nav2 action server not available!')
            self.fsm.transition_to(RobotStateEnum.EMERGENCY_STOP)
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        self.fsm.get_logger().info(
            f'📍 Sending goal: x={target_pose.pose.position.x:.2f}, '
            f'y={target_pose.pose.position.y:.2f}'
        )
        
        # Send goal asynchronously
        send_goal_future = self.fsm.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)
        self.navigation_started = True
    
    def navigation_goal_response_callback(self, future):
        """Called when Nav2 accepts/rejects the goal."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.fsm.get_logger().error('Navigation goal rejected!')
            self.fsm.transition_to(RobotStateEnum.EMERGENCY_STOP)
            return
        
        self.fsm.get_logger().info('Navigation goal accepted!')
        self.goal_handle = goal_handle
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """Called periodically with navigation progress."""
        feedback = feedback_msg.feedback
        # Log progress occasionally
        distance = feedback.distance_remaining
        if int(time.time()) % 5 == 0:  # Log every 5 seconds
            self.fsm.get_logger().info(f'📏 Distance remaining: {distance:.2f}m')
    
    def navigation_result_callback(self, future):
        """Called when navigation completes (success or failure)."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.fsm.get_logger().info('✅ Navigation succeeded! Arrived at destination.')
            self.fsm.transition_to(RobotStateEnum.ARRIVED)
        elif status == GoalStatus.STATUS_CANCELED:
            self.fsm.get_logger().warn('Navigation was cancelled.')
            # Don't transition - another state handler triggered this
        elif status == GoalStatus.STATUS_ABORTED:
            self.fsm.get_logger().error('Navigation aborted! Starting recovery...')
            self.fsm.transition_to(RobotStateEnum.RECOVERY_SEARCH)
        else:
            self.fsm.get_logger().error(f'Navigation failed with status: {status}')
            self.fsm.transition_to(RobotStateEnum.EMERGENCY_STOP)
    
    def on_update(self):
        """Monitor navigation progress and check for events."""
        # The actual monitoring is done through callbacks
        # This update loop can be used for additional checks
        pass


class TheftGraceHandler(StateHandler):
    """
    Handler for THEFT_GRACE state - food was lifted, waiting for return.
    
    This state:
    1. Stops the robot immediately
    2. Starts a 30-second countdown timer
    3. If food returns -> resume NAVIGATING
    4. If timeout -> transition to EMERGENCY_STOP
    """
    
    GRACE_PERIOD_SECONDS = 30.0
    
    def __init__(self, fsm: 'RobotStateMachine'):
        super().__init__(fsm, RobotStateEnum.THEFT_GRACE)
    
    def on_enter(self):
        super().on_enter()
        self.fsm.stop_robot()
        self.fsm.cancel_navigation()  # Pause navigation
        self.fsm.get_logger().warn(
            f'⚠️ FOOD LIFTED! Robot stopped. '
            f'Waiting {self.GRACE_PERIOD_SECONDS}s for food to be returned...'
        )
    
    def on_update(self):
        """Check if grace period has expired."""
        elapsed = self.get_time_in_state()
        
        # Log countdown every 5 seconds
        remaining = self.GRACE_PERIOD_SECONDS - elapsed
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            self.fsm.get_logger().warn(f'⏳ Theft grace: {remaining:.0f}s remaining...')
        
        if elapsed >= self.GRACE_PERIOD_SECONDS:
            self.fsm.get_logger().error(
                '❌ THEFT TIMEOUT! Food was not returned. Requesting operator intervention.'
            )
            self.fsm.transition_to(RobotStateEnum.EMERGENCY_STOP)


class EmergencyStopHandler(StateHandler):
    """
    Handler for EMERGENCY_STOP state - requires operator intervention.
    
    This state:
    1. Stops the robot immediately
    2. Cancels any active navigation
    3. Waits for operator command (Resume, Abort, etc.)
    """
    
    def __init__(self, fsm: 'RobotStateMachine'):
        super().__init__(fsm, RobotStateEnum.EMERGENCY_STOP)
    
    def on_enter(self):
        super().on_enter()
        self.fsm.stop_robot()
        self.fsm.cancel_navigation()
        self.fsm.get_logger().error(
            '🛑 EMERGENCY STOP ACTIVATED!\n'
            '   Robot is halted. Awaiting operator command.\n'
            '   Use: ros2 service call /operator_command ...'
        )


class RecoverySearchHandler(StateHandler):
    """
    Handler for RECOVERY_SEARCH state - searching for lost marker.
    
    This state:
    1. Executes a rotation search pattern
    2. Looks for ArUco marker or other landmarks
    3. If found -> resume NAVIGATING
    4. If search fails -> EMERGENCY_STOP
    """
    
    ROTATION_SPEED = 0.5  # rad/s
    MAX_SEARCH_TIME = 30.0  # seconds
    
    def __init__(self, fsm: 'RobotStateMachine'):
        super().__init__(fsm, RobotStateEnum.RECOVERY_SEARCH)
        self.rotations_completed = 0
    
    def on_enter(self):
        super().on_enter()
        self.rotations_completed = 0
        self.fsm.get_logger().warn(
            '🔍 Starting recovery search pattern...\n'
            '   Rotating to find markers/landmarks.'
        )
    
    def on_update(self):
        """Execute rotation search pattern."""
        elapsed = self.get_time_in_state()
        
        if elapsed >= self.MAX_SEARCH_TIME:
            self.fsm.get_logger().error('❌ Recovery search failed! Requesting operator.')
            self.fsm.transition_to(RobotStateEnum.EMERGENCY_STOP)
            return
        
        # Publish rotation command
        twist = Twist()
        twist.angular.z = self.ROTATION_SPEED
        self.fsm.cmd_vel_pub.publish(twist)
        
        # TODO: Check for marker detection and transition to NAVIGATING
    
    def on_exit(self):
        super().on_exit()
        self.fsm.stop_robot()


class ArrivedHandler(StateHandler):
    """
    Handler for ARRIVED state - at destination, waiting for food pickup.
    
    This state:
    1. Stops and waits for customer to pick up food
    2. Monitors /food_status for pickup confirmation
    3. If pickup -> return to IDLE (mission complete)
    4. If timeout (60s) -> RETURN_WITH_FOOD
    """
    
    PICKUP_TIMEOUT_SECONDS = 60.0
    
    def __init__(self, fsm: 'RobotStateMachine'):
        super().__init__(fsm, RobotStateEnum.ARRIVED)
    
    def on_enter(self):
        super().on_enter()
        self.fsm.stop_robot()
        self.fsm.get_logger().info(
            f'📍 ARRIVED at Table {self.fsm.current_target_table}!\n'
            f'   Waiting {self.PICKUP_TIMEOUT_SECONDS}s for customer pickup...'
        )
    
    def on_update(self):
        """Check for pickup timeout."""
        elapsed = self.get_time_in_state()
        
        # Log countdown every 15 seconds
        remaining = self.PICKUP_TIMEOUT_SECONDS - elapsed
        if int(elapsed) % 15 == 0 and int(elapsed) > 0:
            self.fsm.get_logger().info(f'⏳ Pickup timeout: {remaining:.0f}s remaining...')
        
        if elapsed >= self.PICKUP_TIMEOUT_SECONDS:
            self.fsm.get_logger().warn(
                '⏰ Pickup timeout expired! Returning to home with food.'
            )
            self.fsm.transition_to(RobotStateEnum.RETURN_WITH_FOOD)


class ReturnWithFoodHandler(StateHandler):
    """
    Handler for RETURN_WITH_FOOD state - returning home after timeout.
    
    This state:
    1. Navigates back to home position
    2. On arrival -> transition to IDLE
    """
    
    def __init__(self, fsm: 'RobotStateMachine'):
        super().__init__(fsm, RobotStateEnum.RETURN_WITH_FOOD)
        self.navigation_started = False
    
    def on_enter(self):
        super().on_enter()
        self.navigation_started = False
        self.fsm.get_logger().info('🏠 Navigating back to home position...')
        
        # Navigate to home
        self.send_home_navigation_goal()
    
    def send_home_navigation_goal(self):
        """Send navigation goal to home position."""
        if not self.fsm.nav_client.wait_for_server(timeout_sec=5.0):
            self.fsm.get_logger().error('Nav2 action server not available!')
            self.fsm.transition_to(RobotStateEnum.EMERGENCY_STOP)
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.fsm.home_position
        
        self.fsm.get_logger().info(
            f'📍 Returning to home: x={self.fsm.home_position.pose.position.x:.2f}, '
            f'y={self.fsm.home_position.pose.position.y:.2f}'
        )
        
        send_goal_future = self.fsm.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.navigation_started = True
    
    def goal_response_callback(self, future):
        """Called when home navigation goal is accepted/rejected."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.fsm.get_logger().error('Home navigation goal rejected!')
            self.fsm.transition_to(RobotStateEnum.EMERGENCY_STOP)
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Called when home navigation completes."""
        result = future.result()
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.fsm.get_logger().info('✅ Returned home successfully!')
            self.fsm.transition_to(RobotStateEnum.IDLE)
        else:
            self.fsm.get_logger().error('Failed to return home!')
            self.fsm.transition_to(RobotStateEnum.EMERGENCY_STOP)


class RobotStateMachine(Node):
    """
    Main State Machine Node for the Cafeteria Robot.
    
    This node acts as the central coordinator, receiving inputs from
    various sensors and perception systems, and commanding the robot's
    behavior accordingly.
    
    Subscriptions:
        /food_status (FoodStatus) - From FSR sensor
        /collision_alert (CollisionAlert) - From IMU/Bumper
        
    Publishers:
        /cmd_vel (Twist) - Motor commands
        /robot_state (RobotState) - Current FSM state
        
    Services:
        /operator_command (OperatorCommand) - Operator intervention
        /start_delivery (StartDelivery) - Start new delivery
        
    Actions:
        /navigate_to_pose (NavigateToPose) - Nav2 navigation client
    """
    
    # ==================== TABLE POSITIONS ====================
    # Define table positions in the cafeteria (x, y, theta)
    # These should be calibrated to your actual environment
    TABLE_POSITIONS = {
        1: (2.0, 1.0, 0.0),     # Table 1
        2: (2.0, -1.0, 0.0),    # Table 2
        3: (4.0, 1.0, 0.0),     # Table 3
        4: (4.0, -1.0, 0.0),    # Table 4
        5: (6.0, 0.0, 0.0),     # Table 5
    }
    
    # Home/Kitchen position
    HOME_POSITION = (0.0, 0.0, 0.0)

    def __init__(self):
        super().__init__('robot_state_machine')
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # ========== STATE MANAGEMENT ==========
        self.current_state = RobotStateEnum.IDLE
        self.previous_state: Optional[RobotStateEnum] = None
        
        # Initialize state handlers
        self.state_handlers: Dict[RobotStateEnum, StateHandler] = {
            RobotStateEnum.IDLE: IdleHandler(self),
            RobotStateEnum.NAVIGATING: NavigatingHandler(self),
            RobotStateEnum.THEFT_GRACE: TheftGraceHandler(self),
            RobotStateEnum.EMERGENCY_STOP: EmergencyStopHandler(self),
            RobotStateEnum.RECOVERY_SEARCH: RecoverySearchHandler(self),
            RobotStateEnum.ARRIVED: ArrivedHandler(self),
            RobotStateEnum.RETURN_WITH_FOOD: ReturnWithFoodHandler(self),
        }
        
        # ========== MISSION STATE ==========
        self.current_target_table: Optional[int] = None
        self.current_mission_id: Optional[str] = None
        
        # Set home position
        self.home_position = PoseStamped()
        self.home_position.header.frame_id = 'map'
        self.home_position.pose.position.x = self.HOME_POSITION[0]
        self.home_position.pose.position.y = self.HOME_POSITION[1]
        self.home_position.pose.orientation.w = 1.0
        
        # ========== NAV2 ACTION CLIENT ==========
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.callback_group
        )
        self._current_goal_handle: Optional[ClientGoalHandle] = None
        
        # ========== PUBLISHERS ==========
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.state_pub = self.create_publisher(
            RobotState, '/robot_state', 10
        )
        
        # ========== SUBSCRIBERS ==========
        self.food_status_sub = self.create_subscription(
            FoodStatus, '/food_status',
            self.food_status_callback, 10,
            callback_group=self.callback_group
        )
        self.collision_alert_sub = self.create_subscription(
            CollisionAlert, '/collision_alert',
            self.collision_alert_callback, 10,
            callback_group=self.callback_group
        )
        
        # ========== SERVICES ==========
        self.operator_cmd_srv = self.create_service(
            OperatorCommand, '/operator_command',
            self.operator_command_callback,
            callback_group=self.callback_group
        )
        self.start_delivery_srv = self.create_service(
            StartDelivery, '/start_delivery',
            self.start_delivery_callback,
            callback_group=self.callback_group
        )
        
        # ========== TIMERS ==========
        # Main update loop - runs at 10 Hz
        self.update_timer = self.create_timer(
            0.1, self.update_loop,
            callback_group=self.callback_group
        )
        # State publisher - runs at 1 Hz
        self.state_pub_timer = self.create_timer(
            1.0, self.publish_state,
            callback_group=self.callback_group
        )
        
        # Enter initial state
        self.state_handlers[self.current_state].on_enter()
        
        # Log startup
        self.get_logger().info('='*60)
        self.get_logger().info('🤖 Robot State Machine Initialized')
        self.get_logger().info(f'   Initial State: {self.current_state.name}')
        self.get_logger().info(f'   Tables configured: {list(self.TABLE_POSITIONS.keys())}')
        self.get_logger().info('   Subscriptions: /food_status, /collision_alert')
        self.get_logger().info('   Services: /operator_command, /start_delivery')
        self.get_logger().info('   Action: /navigate_to_pose (Nav2)')
        self.get_logger().info('='*60)

    # ==================== TABLE POSITIONS ====================
    
    def get_table_position(self, table_id: int) -> Optional[PoseStamped]:
        """
        Get the PoseStamped for a given table ID.
        
        Args:
            table_id: The table number (1-5)
            
        Returns:
            PoseStamped for the table, or None if invalid ID
        """
        if table_id not in self.TABLE_POSITIONS:
            return None
        
        x, y, theta = self.TABLE_POSITIONS[table_id]
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion (yaw only)
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        
        return pose

    # ==================== STATE TRANSITIONS ====================
    
    def transition_to(self, new_state: RobotStateEnum):
        """
        Transition to a new state.
        
        Calls on_exit for old state and on_enter for new state.
        
        Args:
            new_state: The state to transition to
        """
        if new_state == self.current_state:
            return  # No transition needed
        
        self.get_logger().info(
            f'🔄 State Transition: {self.current_state.name} -> {new_state.name}'
        )
        
        # Exit current state
        self.state_handlers[self.current_state].on_exit()
        
        # Update state
        self.previous_state = self.current_state
        self.current_state = new_state
        
        # Enter new state
        self.state_handlers[new_state].on_enter()

    # ==================== MAIN UPDATE LOOP ====================
    
    def update_loop(self):
        """Main update loop - called at 10 Hz."""
        # Call current state's update handler
        self.state_handlers[self.current_state].on_update()

    # ==================== MOTOR CONTROL ====================
    
    def stop_robot(self):
        """Send zero velocity command to stop the robot."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().debug('Robot stopped')
    
    def cancel_navigation(self):
        """Cancel any active Nav2 navigation goal."""
        if self._current_goal_handle is not None:
            self.get_logger().info('Cancelling navigation goal...')
            cancel_future = self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    # ==================== CALLBACKS ====================
    
    def food_status_callback(self, msg: FoodStatus):
        """
        Handle food status updates from FSR sensor.
        
        Triggers THEFT_GRACE if food is lifted during navigation.
        """
        if msg.status == FoodStatus.STATUS_LIFTED:
            if self.current_state == RobotStateEnum.NAVIGATING:
                self.get_logger().warn(f'🍽️ Food lifted! Weight: {msg.weight_kg:.2f} kg')
                self.transition_to(RobotStateEnum.THEFT_GRACE)
        
        elif msg.status == FoodStatus.STATUS_PRESENT:
            if self.current_state == RobotStateEnum.THEFT_GRACE:
                self.get_logger().info('✅ Food returned! Resuming navigation...')
                self.transition_to(RobotStateEnum.NAVIGATING)
        
        elif msg.status == FoodStatus.STATUS_REMOVED:
            if self.current_state == RobotStateEnum.ARRIVED:
                self.get_logger().info('✅ Food picked up by customer! Mission complete.')
                self.transition_to(RobotStateEnum.IDLE)

    def collision_alert_callback(self, msg: CollisionAlert):
        """
        Handle collision alerts from IMU/Bumper.
        
        Triggers EMERGENCY_STOP on any collision.
        """
        collision_type = 'BUMPER' if msg.collision_type == CollisionAlert.TYPE_BUMPER else 'IMU'
        self.get_logger().error(
            f'💥 COLLISION DETECTED! Type: {collision_type}, Severity: {msg.severity}'
        )
        
        # Don't interrupt if already in emergency stop
        if self.current_state != RobotStateEnum.EMERGENCY_STOP:
            self.transition_to(RobotStateEnum.EMERGENCY_STOP)

    def operator_command_callback(self, request: OperatorCommand.Request, 
                                   response: OperatorCommand.Response):
        """
        Handle operator commands from control panel.
        """
        cmd = request.command
        
        if cmd == OperatorCommand.Request.CMD_RESUME:
            if self.current_state == RobotStateEnum.EMERGENCY_STOP:
                self.get_logger().info('👨‍💼 Operator: RESUME command received')
                if self.previous_state and self.previous_state != RobotStateEnum.EMERGENCY_STOP:
                    self.transition_to(self.previous_state)
                else:
                    self.transition_to(RobotStateEnum.IDLE)
                response.success = True
                response.message = 'Resumed operation'
            else:
                response.success = False
                response.message = f'Cannot resume from state {self.current_state.name}'
        
        elif cmd == OperatorCommand.Request.CMD_ABORT:
            self.get_logger().info('👨‍💼 Operator: ABORT command received')
            self.transition_to(RobotStateEnum.IDLE)
            response.success = True
            response.message = 'Mission aborted'
        
        elif cmd == OperatorCommand.Request.CMD_THEFT_CONFIRMED:
            self.get_logger().error('👨‍💼 Operator: THEFT CONFIRMED')
            # Log theft, stay in emergency stop for investigation
            response.success = True
            response.message = 'Theft logged and confirmed'
        
        elif cmd == OperatorCommand.Request.CMD_FORCE_RETURN:
            self.get_logger().info('👨‍💼 Operator: FORCE RETURN command received')
            self.transition_to(RobotStateEnum.RETURN_WITH_FOOD)
            response.success = True
            response.message = 'Returning to home'
        
        else:
            response.success = False
            response.message = f'Unknown command: {cmd}'
        
        response.new_state = self.current_state.value
        return response

    def start_delivery_callback(self, request: StartDelivery.Request,
                                 response: StartDelivery.Response):
        """
        Handle request to start a new delivery mission.
        """
        if self.current_state != RobotStateEnum.IDLE:
            response.accepted = False
            response.error_message = f'Robot is busy (state: {self.current_state.name})'
            return response
        
        # Validate table ID
        if request.table_id not in self.TABLE_POSITIONS:
            response.accepted = False
            response.error_message = f'Invalid table ID: {request.table_id}. Valid: {list(self.TABLE_POSITIONS.keys())}'
            return response
        
        # Accept the mission
        self.current_target_table = request.table_id
        self.current_mission_id = f'delivery_{request.table_id}_{int(time.time())}'
        
        self.get_logger().info(
            f'📦 New delivery mission accepted!\n'
            f'   Mission ID: {self.current_mission_id}\n'
            f'   Target: Table {request.table_id}'
        )
        
        # Start navigation
        self.transition_to(RobotStateEnum.NAVIGATING)
        
        response.accepted = True
        response.mission_id = self.current_mission_id
        response.estimated_time = 60.0  # TODO: Calculate actual ETA
        return response

    # ==================== STATE PUBLISHING ====================
    
    def publish_state(self):
        """Publish current state to /robot_state topic."""
        msg = RobotState()
        msg.current_state = self.current_state.value
        msg.previous_state = self.previous_state.value if self.previous_state else 0
        msg.state_name = self.current_state.name
        msg.time_in_state = self.state_handlers[self.current_state].get_time_in_state()
        msg.status_message = f'Table: {self.current_target_table}' if self.current_target_table else 'No active mission'
        
        self.state_pub.publish(msg)


def main(args=None):
    """Main entry point for the Robot State Machine node."""
    rclpy.init(args=args)
    
    node = RobotStateMachine()
    
    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Robot State Machine...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
