#!/usr/bin/env python3
"""
Bridge: Erkka skeleton pointing -> Nav2 goal.

Subscribes
----------
/pointed_location  (geometry_msgs/msg/PoseArray)
    Erkka MediaPipe pose node output. Always 2 poses: index 0 = left arm,
    index 1 = right arm. Each is the arm->ground hit point in METRES, in the
    CAMERA OPTICAL frame (x right, y down, z forward; origin at the camera).
      * position = NaN        -> that arm is not pointing at the ground.
      * position = (99,99,99) -> arming/disarming sentinel (user holding arms up);
                                 ignored, it is not a goal.
    Pointing is gated: the user raises both arms ~3 s to toggle it on, and the
    published point is a 3 s running average -> stable, deliberate goals.

Action
------
navigate_to_pose  (nav2_msgs/action/NavigateToPose)
    Relative name -> under namespace robot1 (sim) it is /robot1/navigate_to_pose;
    no namespace (real) -> /navigate_to_pose.

Pipeline
--------
1. Pick a valid arm pose (param `arm`: right|left|either); drop NaN and the 99 sentinel.
2. Convert camera-optical (x right, y down, z fwd) -> robot base frame
   (x fwd, y left, z up):
       base_x = optical_z + camera_forward_offset
       base_y = -optical_x
   (height/z dropped -> ground goal). `camera_forward_offset` accounts for the
   camera sitting ahead of base_link.
3. TF base_link -> goal_frame (map in sim, odom on the mapless real stack). This is
   the "+ odometry" step (adds the robot's pose AND heading), done via TF.
4. Send a NavigateToPose goal facing the point. Rate-limited + de-duplicated so it
   doesn't spam Nav2.

Run
---
  # sim (map-based, namespaced):
  ros2 run quadropted_controller pointed_goal.py --ros-args -r __ns:=/robot1 -p goal_frame:=map
  # real (mapless):
  python3 pointed_goal.py --ros-args -p goal_frame:=odom
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion
from nav2_msgs.action import NavigateToPose

import tf2_ros
try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

SENTINEL = 99.0          # Erkka "arms up" feedback value
SENTINEL_TOL = 1.0


def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class PointedGoal(Node):
    def __init__(self):
        super().__init__("pointed_goal")

        self.declare_parameter("input_topic", "/pointed_location")
        self.declare_parameter("arm", "either")              # right | left | either
        self.declare_parameter("goal_frame", "map")          # map (sim) | odom (real)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("camera_forward_offset", 0.0)  # m camera is ahead of base_link
        self.declare_parameter("min_goal_interval", 1.0)     # s between goals
        self.declare_parameter("min_goal_delta", 0.25)       # m; ignore near-identical goals

        gp = self.get_parameter
        self.arm = gp("arm").value
        self.goal_frame = gp("goal_frame").value
        self.base_frame = gp("robot_base_frame").value
        self.cam_fwd = float(gp("camera_forward_offset").value)
        self.min_goal_interval = float(gp("min_goal_interval").value)
        self.min_goal_delta = float(gp("min_goal_delta").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(PoseArray, gp("input_topic").value, self._on_points, 10)

        # One-shot gate: send exactly ONE goal per arms-up. The rising edge of the
        # 99-sentinel (arms up) re-arms; the first valid point after that is sent,
        # then further points are ignored until the next arms-up.
        self._in_sentinel = False
        self._armed = False
        self._sent_this_cycle = False

        self.get_logger().info(
            f"pointed_goal ready ({self.base_frame} -> {self.goal_frame}). "
            "Raise both arms (~3 s) to choose a point.")

    # ------------------------------------------------------------------
    @staticmethod
    def _is_nan(p):
        return any(math.isnan(v) for v in (p.position.x, p.position.y, p.position.z))

    @staticmethod
    def _is_sentinel(p):
        return all(abs(v - SENTINEL) < SENTINEL_TOL
                   for v in (p.position.x, p.position.y, p.position.z))

    def _pick(self, poses):
        def ok(i):
            return len(poses) > i and not self._is_nan(poses[i]) and not self._is_sentinel(poses[i])
        if self.arm == "left":
            return poses[0] if ok(0) else None
        if self.arm == "right":
            return poses[1] if ok(1) else None
        if ok(1):
            return poses[1]
        if ok(0):
            return poses[0]
        return None

    def _on_points(self, msg: PoseArray):
        poses = msg.poses

        # Arms-up (99 sentinel): the RISING edge re-arms for exactly one goal.
        if any(self._is_sentinel(p) for p in poses):
            if not self._in_sentinel:
                self._in_sentinel = True
                self._armed = True
                self._sent_this_cycle = False
                self.get_logger().info(
                    "🙌 Arms up detected — WAITING for you to point at a spot...")
            return
        self._in_sentinel = False

        # Only act when armed and not already fired this cycle.
        if not self._armed or self._sent_this_cycle:
            return

        pose = self._pick(poses)
        if pose is None:
            return  # armed, but no valid pointing hit yet — keep waiting

        # camera-optical (x right, y down, z fwd) -> base_link (x fwd, y left, z up)
        bx = pose.position.z + self.cam_fwd
        by = -pose.position.x

        pt = PointStamped()
        pt.header.frame_id = self.base_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x, pt.point.y, pt.point.z = bx, by, 0.0

        # base_link -> goal_frame: this adds the robot's current odom pose + heading.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.goal_frame, self.base_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.3))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"TF {self.base_frame}->{self.goal_frame} unavailable: {e}")
            return
        g = do_transform_point(pt, tf)
        gx, gy = g.point.x, g.point.y

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("navigate_to_pose action server not available yet")
            return

        yaw = math.atan2(gy - tf.transform.translation.y, gx - tf.transform.translation.x)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.goal_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.orientation = yaw_to_quat(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.nav_client.send_goal_async(goal)

        # One-shot: stop until the next arms-up.
        self._armed = False
        self._sent_this_cycle = True
        self.get_logger().info(
            f"✅ Goal sent ({self.goal_frame} x={gx:.2f} y={gy:.2f}). "
            "Raise both arms again to choose a new point.")


def main(args=None):
    rclpy.init(args=args)
    node = PointedGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
