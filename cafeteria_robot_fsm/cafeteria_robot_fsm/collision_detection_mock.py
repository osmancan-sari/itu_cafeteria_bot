#!/usr/bin/env python3
"""
Mock collision detection node.

Listens to /bumper/contact and publishes CollisionAlert messages when
contact states are present.
"""

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Vector3

from cafeteria_interfaces.msg import CollisionAlert


class CollisionDetectionMock(Node):
    """Publish CollisionAlert when bumper contact is detected."""

    def __init__(self) -> None:
        super().__init__("collision_detection_mock")
        self.publisher = self.create_publisher(CollisionAlert, "/collision_alert", 10)
        self.subscription = self.create_subscription(
            ContactsState, "/bumper/contact", self.bumper_callback, 10
        )
        self._last_contact = False
        self.get_logger().info("Collision detection mock started (listening to /bumper/contact).")

    def bumper_callback(self, msg: ContactsState) -> None:
        has_contact = len(msg.states) > 0
        if not has_contact:
            self._last_contact = False
            return

        if self._last_contact:
            return

        alert = CollisionAlert()
        alert.collision_type = CollisionAlert.TYPE_BUMPER
        alert.impact_direction = Vector3(x=0.0, y=0.0, z=0.0)
        alert.severity = 50
        alert.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(alert)
        self._last_contact = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionDetectionMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down collision detection mock...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
