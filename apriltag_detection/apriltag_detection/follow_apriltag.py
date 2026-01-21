import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import time

class FollowTag(Node):
    def __init__(self):
        super().__init__('follow_tag')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/detected_tag_pose',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 1.2)
        self.declare_parameter("forward_chase_speed", 0.15)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("stop_distance_m", 0.6)
        self.declare_parameter("filter_value", 0.9)

        self.rcv_timeout_secs = float(self.get_parameter('rcv_timeout_secs').value)
        self.angular_chase_multiplier = float(self.get_parameter('angular_chase_multiplier').value)
        self.forward_chase_speed = float(self.get_parameter('forward_chase_speed').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.filter_value = float(self.get_parameter('filter_value').value)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.target_val = 0.0     # yaw control signal (from x offset)
        self.target_dist = 999.0  # forward distance (meters)
        self.lastrcvtime = time.time() - 10000

    def timer_callback(self):
        cmd = Twist()

        fresh = (time.time() - self.lastrcvtime) < self.rcv_timeout_secs
        if fresh:
            # Move forward until we reach stop distance
            if self.target_dist > self.stop_distance_m:
                cmd.linear.x = self.forward_chase_speed

            # Turn toward tag (sign might need flipping)
            cmd.angular.z = -self.angular_chase_multiplier * self.target_val
        else:
            cmd.angular.z = self.search_angular_speed

        self.publisher_.publish(cmd)

    def listener_callback(self, msg: PoseStamped):
        f = self.filter_value

        # In ROS optical frame (camera_link_optical):
        # x = right, y = down, z = forward
        x_right = msg.pose.position.x
        z_forward = msg.pose.position.z

        self.target_val = self.target_val * f + x_right * (1 - f)
        self.target_dist = self.target_dist * f + z_forward * (1 - f)

        self.lastrcvtime = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = FollowTag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
