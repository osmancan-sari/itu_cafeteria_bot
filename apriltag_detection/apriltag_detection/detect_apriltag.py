import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
from pupil_apriltags import Detector


def crop_from_window_perc(image, window_perc):
    """window_perc = [x_min, y_min, x_max, y_max] in 0..100"""
    h, w = image.shape[:2]
    x0 = int(w * window_perc[0] / 100.0)
    y0 = int(h * window_perc[1] / 100.0)
    x1 = int(w * window_perc[2] / 100.0)
    y1 = int(h * window_perc[3] / 100.0)

    x0 = max(0, min(w - 1, x0))
    y0 = max(0, min(h - 1, y0))
    x1 = max(1, min(w, x1))
    y1 = max(1, min(h, y1))

    roi = image[y0:y1, x0:x1]
    return roi, (x0, y0, x1, y1)


class DetectAprilTag(Node):
    def __init__(self):
        super().__init__("detect_apriltag")

        self.get_logger().info("Looking for AprilTags...")

        self.image_sub = self.create_subscription(
            Image, "/image_in", self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)

        # Publish best tag id (you can change topic name)
        self.tag_id_pub = self.create_publisher(Int32, "/detected_tag_id", 1)

        # Parameters (keep ROI + tuning_mode)
        self.declare_parameter("tuning_mode", False)

        self.declare_parameter("x_min", 0)
        self.declare_parameter("x_max", 100)
        self.declare_parameter("y_min", 0)
        self.declare_parameter("y_max", 100)

        # AprilTag-specific params
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("min_decision_margin", 20.0)  # filter weak detections
        self.declare_parameter("quad_decimate", 1.0)         # 2.0 = faster, less accurate

        self.tuning_mode = self.get_parameter("tuning_mode").value

        self.tuning_params = {
            "x_min": self.get_parameter("x_min").value,
            "x_max": self.get_parameter("x_max").value,
            "y_min": self.get_parameter("y_min").value,
            "y_max": self.get_parameter("y_max").value,
        }

        fam = self.get_parameter("tag_family").value
        quad_decimate = float(self.get_parameter("quad_decimate").value)

        # Create detector once
        self.detector = Detector(
            families=fam,
            nthreads=2,
            quad_decimate=quad_decimate,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        self.min_decision_margin = float(self.get_parameter("min_decision_margin").value)

        self.bridge = CvBridge()

        # If you want a GUI trackbar tuning window like before,
        # you can add it later; for now we keep ROS params only.

    def callback(self, data: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # If tuning_mode, you could refresh params dynamically by reading self.get_parameter(...) here
        # For simplicity, we keep the initial values unless you want live updates.
        window = [
            self.tuning_params["x_min"],
            self.tuning_params["y_min"],
            self.tuning_params["x_max"],
            self.tuning_params["y_max"],
        ]

        roi_bgr, (x0, y0, x1, y1) = crop_from_window_perc(frame_bgr, window)
        roi_gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)

        dets = self.detector.detect(roi_gray)

        # Output images
        out = frame_bgr.copy()
        cv2.rectangle(out, (x0, y0), (x1, y1), (255, 0, 0), 2)

        best_det = None
        for d in dets:
            if float(d.decision_margin) < self.min_decision_margin:
                continue

            # Convert ROI coords -> full image coords
            corners = d.corners.copy()
            corners[:, 0] += x0
            corners[:, 1] += y0
            center = (float(d.center[0] + x0), float(d.center[1] + y0))

            # draw border
            pts = corners.astype(int).reshape(-1, 1, 2)
            cv2.polylines(out, [pts], True, (0, 255, 0), 2)
            cv2.circle(out, (int(center[0]), int(center[1])), 4, (0, 0, 255), -1)

            tag_id = int(d.tag_id)
            cv2.putText(
                out,
                f"id={tag_id} dm={d.decision_margin:.1f}",
                (int(center[0]) + 5, int(center[1]) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            if best_det is None or float(d.decision_margin) > float(best_det.decision_margin):
                best_det = d

        # publish annotated image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(out, "bgr8")
            img_msg.header = data.header
            self.image_out_pub.publish(img_msg)

            # For now, publish same as tuning image (or you can publish ROI-only debug)
            tun_msg = self.bridge.cv2_to_imgmsg(out, "bgr8")
            tun_msg.header = data.header
            self.image_tuning_pub.publish(tun_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge publish error: {e}")

        # publish best tag id (if any)
        if best_det is not None:
            msg = Int32()
            msg.data = int(best_det.tag_id)
            self.tag_id_pub.publish(msg)
            self.get_logger().info(f"Best tag: {msg.data} (dm={best_det.decision_margin:.1f})")


def main(args=None):
    rclpy.init(args=args)
    node = DetectAprilTag()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
