#!/usr/bin/env python3
"""
Collision detection node.

Bridges bumper contact (Gazebo) and optional IMU spikes into CollisionAlert
messages expected by the FSM.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

from cafeteria_interfaces.msg import CollisionAlert


class CollisionDetection(Node):
    """Publish CollisionAlert messages from bumper contact and optional IMU."""

    def __init__(self) -> None:
        super().__init__("collision_detection")

        self.declare_parameter("enable_imu", False)
        self.declare_parameter("imu_accel_threshold", 2.5)
        self.declare_parameter("contact_cooldown_sec", 0.5)

        self.enable_imu = bool(self.get_parameter("enable_imu").value)
        self.imu_accel_threshold = float(self.get_parameter("imu_accel_threshold").value)
        self.contact_cooldown_sec = float(self.get_parameter("contact_cooldown_sec").value)

        self.publisher = self.create_publisher(CollisionAlert, "/collision_alert", 10)

        self.bumper_sub = self.create_subscription(
            ContactsState, "/bumper/contact", self.bumper_callback, 10
        )

        self.imu_sub: Optional[rclpy.subscription.Subscription] = None
        if self.enable_imu:
            self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self._last_contact_publish = 0.0
        self._last_imu_publish = 0.0

        self.get_logger().info(
            "Collision detection started (bumper: /bumper/contact, imu enabled: %s)."
            % self.enable_imu
        )

    def bumper_callback(self, msg: ContactsState) -> None:
        if not msg.states:
            return

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self._last_contact_publish < self.contact_cooldown_sec:
            return

        alert = CollisionAlert()
        alert.collision_type = CollisionAlert.TYPE_BUMPER
        alert.impact_direction = Vector3(x=0.0, y=0.0, z=0.0)
        alert.severity = 50
        alert.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(alert)
        self._last_contact_publish = now

    def imu_callback(self, msg: Imu) -> None:
        accel = msg.linear_acceleration
        magnitude = math.sqrt(accel.x**2 + accel.y**2 + accel.z**2)

        if magnitude < self.imu_accel_threshold:
            return

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self._last_imu_publish < self.contact_cooldown_sec:
            return

        direction = Vector3(x=0.0, y=0.0, z=0.0)
        if magnitude > 0.0:
            direction = Vector3(
                x=accel.x / magnitude,
                y=accel.y / magnitude,
                z=accel.z / magnitude,
            )

        alert = CollisionAlert()
        alert.collision_type = CollisionAlert.TYPE_IMU
        alert.impact_direction = direction
        alert.severity = min(100, int(magnitude * 10))
        alert.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(alert)
        self._last_imu_publish = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down collision detection...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
