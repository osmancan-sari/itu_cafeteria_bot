#!/usr/bin/env python3
"""
Mock FSR sensor node.

Publishes FoodStatus on /food_status at a fixed rate so the FSM can run
without a real tray weight sensor.
"""

import rclpy
from rclpy.node import Node

from cafeteria_interfaces.msg import FoodStatus


class FSRSensorMock(Node):
    """Publish a steady STATUS_PRESENT message for testing."""

    def __init__(self) -> None:
        super().__init__("fsr_sensor_mock")
        self.publisher = self.create_publisher(FoodStatus, "/food_status", 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info("FSR sensor mock started (publishing STATUS_PRESENT).")

    def publish_status(self) -> None:
        msg = FoodStatus()
        msg.status = FoodStatus.STATUS_PRESENT
        msg.weight_kg = 0.5
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FSRSensorMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down FSR sensor mock...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
