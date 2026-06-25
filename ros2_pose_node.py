"""
ROS2 Humble — MediaPipe BlazePose node.

Subscribes:
  ~image_topic  (sensor_msgs/msg/Image, default /image_raw)

Publishes:
  /pointed_location  (geometry_msgs/msg/PoseArray)
      Always 2 poses. index 0 = left arm, index 1 = right arm.
      position.x/y/z = NaN when that arm is not pointing toward the ground.
      Coordinate space: normalized MediaPipe (x 0-1 horizontal, y depth, z=-lm.y; ground z=-1.2).

  /annotated_image   (sensor_msgs/msg/Image)
      Input frame with MediaPipe skeleton overlay.

Run:
  source /opt/ros/humble/setup.bash
  python3 ros2_pose_node.py
"""

import os
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header

from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pose_landmarker_full.task")

CONNECTIONS = vision.PoseLandmarksConnections.POSE_LANDMARKS

# BGR colors for skeleton drawing
_CONN_COLOR = (180, 180, 180)
_KP_COLOR   = (0, 255, 0)
GROUND_Z = -1.2
VIS_THRESHOLD = 0.4
NAN = math.nan


def to_plot(lm):
    return np.array([lm.x, lm.z, -lm.y])


def arm_ray_ground_hit(landmarks, elbow_idx, wrist_idx):
    """Return hit point (np array) or None."""
    elbow = landmarks[elbow_idx]
    wrist = landmarks[wrist_idx]
    if elbow.visibility < VIS_THRESHOLD or wrist.visibility < VIS_THRESHOLD:
        return None
    pe = to_plot(elbow)
    pw = to_plot(wrist)
    direction = pw - pe
    dz = direction[2]
    if abs(dz) < 1e-6:
        return None
    t = (GROUND_Z - pe[2]) / dz
    if t < 1.0:
        return None
    return pe + t * direction


def _nan_pose():
    return Pose(
        position=Point(x=NAN, y=NAN, z=NAN),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _hit_pose(hit):
    return Pose(
        position=Point(x=float(hit[0]), y=float(hit[1]), z=float(hit[2])),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


class PoseNode(Node):
    def __init__(self):
        super().__init__("mediapipe_pose")

        self.declare_parameter("image_topic", "/image_raw")
        topic = self.get_parameter("image_topic").get_parameter_value().string_value

        if not os.path.exists(MODEL_PATH):
            self.get_logger().fatal(f"Model not found: {MODEL_PATH}")
            sys.exit(1)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._bridge = CvBridge()
        self._frame_ms = 0

        base_opts = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
        opts = vision.PoseLandmarkerOptions(
            base_options=base_opts,
            running_mode=vision.RunningMode.VIDEO,
            num_poses=2,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            output_segmentation_masks=False,
        )
        self._landmarker = vision.PoseLandmarker.create_from_options(opts)

        self._sub = self.create_subscription(Image, topic, self._image_cb, sensor_qos)
        self._pub_points = self.create_publisher(PoseArray, "/pointed_location", 10)
        self._pub_image = self.create_publisher(Image, "/annotated_image", sensor_qos)

        self.get_logger().info(f"Subscribed to {topic}")

    def _image_cb(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return

        self._frame_ms += 33  # approximate; MediaPipe VIDEO mode needs monotonic ms

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(mp_image, self._frame_ms)

        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)

        # --- pointed_location ---
        left_hit = None
        right_hit = None
        if result.pose_landmarks:
            lms = result.pose_landmarks[0]
            left_hit = arm_ray_ground_hit(lms, 13, 15)
            right_hit = arm_ray_ground_hit(lms, 14, 16)

        pa = PoseArray(header=header)
        pa.poses = [
            _hit_pose(left_hit) if left_hit is not None else _nan_pose(),
            _hit_pose(right_hit) if right_hit is not None else _nan_pose(),
        ]
        self._pub_points.publish(pa)

        # --- annotated_image ---
        annotated = bgr.copy()
        if result.pose_landmarks:
            h, w = annotated.shape[:2]
            for pose_landmarks in result.pose_landmarks:
                pts = [(int(lm.x * w), int(lm.y * h)) for lm in pose_landmarks]
                for conn in CONNECTIONS:
                    cv2.line(annotated, pts[conn.start], pts[conn.end], _CONN_COLOR, 2)
                for pt in pts:
                    cv2.circle(annotated, pt, 4, _KP_COLOR, -1)
        try:
            img_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = header
            self._pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"cv2_to_imgmsg: {e}")

    def destroy_node(self):
        self._landmarker.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
