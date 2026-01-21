import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class DetectTag3d(Node):
    def __init__(self):
        super().__init__('detect_tag_3d')

        self.get_logger().info('Visualizing AprilTag pose in 3D')

        self.pose_sub = self.create_subscription(
            PoseStamped, "/detected_tag_pose", self.pose_callback, 10
        )
        self.marker_pub = self.create_publisher(Marker, "/tag_3d_marker", 1)

        self.declare_parameter("tag_size", 0.12)  # meters
        self.declare_parameter("camera_frame", "camera_link_optical")

        self.tag_size = float(self.get_parameter("tag_size").value)
        self.camera_frame = self.get_parameter("camera_frame").value

    def pose_callback(self, msg: PoseStamped):
        # Publish a flat square marker (CUBE with tiny thickness)
        m = Marker()
        m.header = msg.header
        m.header.frame_id = msg.header.frame_id or self.camera_frame

        m.ns = "apriltag"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.pose = msg.pose

        # A tag is a square; visualize as thin plate
        m.scale.x = self.tag_size
        m.scale.y = self.tag_size
        m.scale.z = 0.002

        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.9

        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = DetectTag3d()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



