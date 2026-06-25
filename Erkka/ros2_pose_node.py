"""
ROS2 Humble — MediaPipe BlazePose node with monocular distance estimation.

Subscribes:
  ~image_topic  (sensor_msgs/msg/Image, default /image_raw)

Publishes:
  /pointed_location  (geometry_msgs/msg/PoseArray)
      Always 2 poses. index 0 = left arm, index 1 = right arm.
      position.x/y/z = NaN when that arm is not pointing toward the ground.
      Coordinate space: METRES in the camera optical frame
      (x right, y down, z forward / into the scene), origin at the camera.

  /pose_skeleton     (visualization_msgs/msg/MarkerArray)
      The full body skeleton in metres in the camera optical frame.
      Constant real-world size; only its position (esp. z = depth) changes
      as the person moves toward/away from the camera.

  /annotated_image   (sensor_msgs/msg/Image)
      Input frame with MediaPipe skeleton overlay (2D).

How depth is recovered:
  MediaPipe's normalized `pose_landmarks` carry a `z` that is depth *relative
  to the hips*, not distance from the camera — which is why a person walking
  toward the lens used to keep the same z while the skeleton scaled. Instead we:
    1. Estimate depth Z from the *apparent* shoulder width in pixels against an
       assumed real shoulder width (pinhole model). Pose-rigid, unlike height.
    2. Back-project the hip-center pixel through the pinhole to get X, Y.
    3. Take the *shape* from MediaPipe's metric `pose_world_landmarks` (already
       in metres, hip-centered, constant size) and translate it to (X, Y, Z).

Run:
  source /opt/ros/humble/setup.bash
  python3 ros2_pose_node.py
"""

import os
import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Duration

from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pose_landmarker_full.task")

CONNECTIONS = vision.PoseLandmarksConnections.POSE_LANDMARKS

# BGR colors for 2D skeleton drawing
_CONN_COLOR = (180, 180, 180)
_KP_COLOR   = (0, 255, 0)
VIS_THRESHOLD = 0.4
NAN = math.nan

# --- Assumed real-world body dimensions (metres) ---------------------------
# Used to convert apparent pixel size -> metric depth. Population averages;
# good enough for ranging. Shoulders are preferred (rigid); hips are a fallback.
REAL_SHOULDER_WIDTH_M = 0.40   # adult biacromial breadth
REAL_HIP_WIDTH_M      = 0.26   # adult bi-iliac breadth

# --- Camera intrinsics -----------------------------------------------------
# TODO: replace with the REAL camera intrinsics on the Go2.
# Hardcoded for a Logitech HD Pro Webcam C920 at its native 16:9 aspect.
# Focal length (px) is derived from the horizontal FOV and the actual incoming
# image width, so it stays correct across capture resolutions.
CAMERA_HFOV_DEG = 70.42        # C920 horizontal field of view (16:9)

# Landmark indices
L_SHOULDER, R_SHOULDER = 11, 12
L_HIP, R_HIP = 23, 24
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
FOOT_IDX = (27, 28, 29, 30, 31, 32)  # ankles, heels, foot index

ARMS_UP_HOLD_S  = 3.0  # both wrists must be above shoulders for this long to activate
FILTER_WINDOW_S = 3.0  # epoch length for the pointing-location average


def focal_px(width):
    """Focal length in pixels from horizontal FOV and image width."""
    return (width / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG) / 2.0)


def _px_dist(a, b, w, h):
    return math.hypot((a.x - b.x) * w, (a.y - b.y) * h)


def estimate_depth_m(image_lms, w, h, focal):
    """Distance (m) from camera to the person's torso, from apparent size.

    Z = focal_px * real_width_m / apparent_width_px  (pinhole model).
    Prefers shoulder width (rigid, robust to pose); falls back to hip width.
    Returns None if neither pair is reliably visible.
    """
    ls, rs = image_lms[L_SHOULDER], image_lms[R_SHOULDER]
    if ls.visibility >= VIS_THRESHOLD and rs.visibility >= VIS_THRESHOLD:
        px = _px_dist(ls, rs, w, h)
        if px > 1.0:
            return focal * REAL_SHOULDER_WIDTH_M / px

    lh, rh = image_lms[L_HIP], image_lms[R_HIP]
    if lh.visibility >= VIS_THRESHOLD and rh.visibility >= VIS_THRESHOLD:
        px = _px_dist(lh, rh, w, h)
        if px > 1.0:
            return focal * REAL_HIP_WIDTH_M / px

    return None


def back_project_skeleton(image_lms, world_lms, w, h, focal):
    """Metric skeleton in the camera optical frame (x right, y down, z forward).

    Returns an (N, 3) array of joint positions in metres, or None if depth
    could not be estimated.
    """
    z = estimate_depth_m(image_lms, w, h, focal)
    if z is None:
        return None

    cx, cy = w / 2.0, h / 2.0

    # Anchor the metric (hip-centered) world skeleton to a back-projected pixel.
    # Use hips when visible (matches the world-landmark origin); else shoulders.
    if image_lms[L_HIP].visibility >= VIS_THRESHOLD and image_lms[R_HIP].visibility >= VIS_THRESHOLD:
        a, b = L_HIP, R_HIP
    elif image_lms[L_SHOULDER].visibility >= VIS_THRESHOLD and image_lms[R_SHOULDER].visibility >= VIS_THRESHOLD:
        a, b = L_SHOULDER, R_SHOULDER
    else:
        return None

    u = ((image_lms[a].x + image_lms[b].x) / 2.0) * w
    v = ((image_lms[a].y + image_lms[b].y) / 2.0) * h
    anchor = np.array([(u - cx) * z / focal, (v - cy) * z / focal, z])

    world = np.array([[wl.x, wl.y, wl.z] for wl in world_lms])
    world_origin = (world[a] + world[b]) / 2.0  # ~0 for hips, but exact for either pair
    return world - world_origin + anchor


def ground_level_m(cam_pts, image_lms):
    """Camera-frame y of the (flat) ground plane = lowest visible foot."""
    ys = [cam_pts[i][1] for i in FOOT_IDX if image_lms[i].visibility >= VIS_THRESHOLD]
    if ys:
        return max(ys)          # +y is down, so the foot is the largest y
    return float(cam_pts[:, 1].max())


def _both_arms_up(cam_pts, image_lms):
    """True when both wrists are above (lower camera-y) their respective shoulders."""
    for sh_idx, wr_idx in ((L_SHOULDER, L_WRIST), (R_SHOULDER, R_WRIST)):
        if (image_lms[sh_idx].visibility < VIS_THRESHOLD or
                image_lms[wr_idx].visibility < VIS_THRESHOLD):
            return False
        if cam_pts[wr_idx][1] >= cam_pts[sh_idx][1]:  # y-down: wrist must be smaller y
            return False
    return True


def arm_ray_ground_hit(cam_pts, image_lms, elbow_idx, wrist_idx, y_ground):
    """Where the extended elbow->wrist ray meets the ground plane. None if not
    pointing down toward the ground."""
    if image_lms[elbow_idx].visibility < VIS_THRESHOLD or image_lms[wrist_idx].visibility < VIS_THRESHOLD:
        return None
    pe = cam_pts[elbow_idx]
    pw = cam_pts[wrist_idx]
    dy = pw[1] - pe[1]
    if abs(dy) < 1e-6:
        return None
    t = (y_ground - pe[1]) / dy
    if t < 1.0:                  # ground must be beyond the wrist (pointing down)
        return None
    return pe + t * (pw - pe)


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


def _pt(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


class PoseNode(Node):
    def __init__(self):
        super().__init__("mediapipe_pose")

        self.declare_parameter("image_topic", "/image_raw")
        topic = self.get_parameter("image_topic").get_parameter_value().string_value
        # Subscribe to the COMPRESSED transport instead of raw (much higher effective
        # FPS over a network, e.g. the real Go2). image_topic stays the BASE topic;
        # '/compressed' is appended automatically.
        self.declare_parameter("use_compressed", False)
        use_compressed = self.get_parameter("use_compressed").get_parameter_value().bool_value

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
        self._last_stamp_ms = None

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

        if use_compressed:
            comp_topic = topic if topic.endswith("/compressed") else topic + "/compressed"
            self._sub = self.create_subscription(
                CompressedImage, comp_topic, self._image_cb_compressed, sensor_qos)
        else:
            self._sub = self.create_subscription(Image, topic, self._image_cb, sensor_qos)
        self._pub_points = self.create_publisher(PoseArray, "/pointed_location", 10)
        self._pub_skel = self.create_publisher(MarkerArray, "/pose_skeleton", 10)
        self._pub_image = self.create_publisher(Image, "/annotated_image", sensor_qos)

        # Arms-up activation gate (toggle: hold 3s to activate, hold 3s again to deactivate)
        self._arms_up_since: float | None = None
        self._gate_triggered = False  # prevents re-triggering while arms stay up
        self._active = False

        # 3-second epoch filter for pointed location
        self._filter_epoch_start = 0.0
        self._filter_left_buf:  list = []
        self._filter_right_buf: list = []
        self._filter_left_avg:  np.ndarray | None = None
        self._filter_right_avg: np.ndarray | None = None

        self.get_logger().info(f"Subscribed to {topic}")

    def _image_cb(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return
        self._process(bgr, msg.header)

    def _image_cb_compressed(self, msg: CompressedImage):
        try:
            bgr = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge (compressed): {e}")
            return
        self._process(bgr, msg.header)

    def _process(self, bgr, in_header):
        # MediaPipe VIDEO mode needs a strictly monotonic millisecond timestamp.
        # Use the message stamp; fall back to incrementing if stamps aren't set.
        stamp_ms = int(in_header.stamp.sec * 1000 + in_header.stamp.nanosec / 1e6)
        if self._last_stamp_ms is None or stamp_ms > self._last_stamp_ms:
            self._frame_ms = stamp_ms
        else:
            self._frame_ms += 1
        self._last_stamp_ms = self._frame_ms

        h, w = bgr.shape[:2]
        focal = focal_px(w)

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(mp_image, self._frame_ms)

        header = Header(stamp=in_header.stamp, frame_id=in_header.frame_id)

        n_poses = len(result.pose_landmarks) if result.pose_landmarks else 0

        # Metric camera-frame skeleton for every detected person.
        cam_skeletons = []
        for i in range(n_poses):
            pts = back_project_skeleton(
                result.pose_landmarks[i], result.pose_world_landmarks[i], w, h, focal
            )
            cam_skeletons.append(pts)

        # --- pointed_location (person 0, in metres) ---
        now = time.monotonic()
        left_hit = right_hit = None
        arms_up = False
        if cam_skeletons and cam_skeletons[0] is not None:
            pts0 = cam_skeletons[0]
            y_ground = ground_level_m(pts0, result.pose_landmarks[0])
            left_hit  = arm_ray_ground_hit(pts0, result.pose_landmarks[0], L_ELBOW, L_WRIST, y_ground)
            right_hit = arm_ray_ground_hit(pts0, result.pose_landmarks[0], R_ELBOW, R_WRIST, y_ground)
            arms_up   = _both_arms_up(pts0, result.pose_landmarks[0])

        # Arms-up toggle gate: track hold duration, fire once per raise/lower cycle
        if arms_up:
            if self._arms_up_since is None:
                self._arms_up_since = now
            if (not self._gate_triggered
                    and now - self._arms_up_since >= ARMS_UP_HOLD_S):
                self._gate_triggered = True
                self._active = not self._active
                if self._active:
                    self._filter_epoch_start = now
                    self._filter_left_buf.clear()
                    self._filter_right_buf.clear()
                else:
                    self._filter_left_avg  = None
                    self._filter_right_avg = None
        else:
            self._arms_up_since  = None
            self._gate_triggered = False

        pa = PoseArray(header=header)

        # Show sentinel 99 while arms are raised (ARMING or DISARMING feedback)
        if self._arms_up_since is not None:
            sentinel = Pose(
                position=Point(x=99.0, y=99.0, z=99.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            pa.poses = [sentinel, sentinel]

        elif self._active:
            # Pointing active — collect hits and publish 3-second epoch averages
            if left_hit  is not None:
                self._filter_left_buf.append(left_hit)
            if right_hit is not None:
                self._filter_right_buf.append(right_hit)

            if now - self._filter_epoch_start >= FILTER_WINDOW_S:
                if self._filter_left_buf:
                    self._filter_left_avg  = np.mean(self._filter_left_buf,  axis=0)
                if self._filter_right_buf:
                    self._filter_right_avg = np.mean(self._filter_right_buf, axis=0)
                self._filter_epoch_start = now
                self._filter_left_buf.clear()
                self._filter_right_buf.clear()

            pa.poses = [
                _hit_pose(self._filter_left_avg)  if self._filter_left_avg  is not None else _nan_pose(),
                _hit_pose(self._filter_right_avg) if self._filter_right_avg is not None else _nan_pose(),
            ]

        else:
            # IDLE
            pa.poses = [_nan_pose(), _nan_pose()]

        self._pub_points.publish(pa)

        # --- pose_skeleton MarkerArray (metric, constant size) ---
        self._pub_skel.publish(self._skeleton_markers(header, cam_skeletons, result.pose_landmarks))

        # --- annotated_image (2D overlay) ---
        annotated = bgr.copy()
        if result.pose_landmarks:
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

    def _skeleton_markers(self, header, cam_skeletons, all_image_lms):
        ma = MarkerArray()
        lifetime = Duration(sec=0, nanosec=200_000_000)  # auto-clear stale people
        identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Clear everything first, then redraw what is currently visible.
        clear = Marker(header=header)
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for idx, pts in enumerate(cam_skeletons):
            if pts is None:
                continue
            image_lms = all_image_lms[idx]

            lines = Marker(header=header, ns="skeleton", id=idx * 2)
            lines.type = Marker.LINE_LIST
            lines.action = Marker.ADD
            lines.scale.x = 0.02
            lines.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)
            lines.pose.orientation = identity
            lines.lifetime = lifetime
            for conn in CONNECTIONS:
                if image_lms[conn.start].visibility < VIS_THRESHOLD or image_lms[conn.end].visibility < VIS_THRESHOLD:
                    continue
                lines.points.append(_pt(pts[conn.start]))
                lines.points.append(_pt(pts[conn.end]))

            joints = Marker(header=header, ns="skeleton", id=idx * 2 + 1)
            joints.type = Marker.SPHERE_LIST
            joints.action = Marker.ADD
            joints.scale.x = joints.scale.y = joints.scale.z = 0.04
            joints.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            joints.pose.orientation = identity
            joints.lifetime = lifetime
            for i, p in enumerate(pts):
                if image_lms[i].visibility < VIS_THRESHOLD:
                    continue
                joints.points.append(_pt(p))

            ma.markers.append(lines)
            ma.markers.append(joints)

        return ma

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
