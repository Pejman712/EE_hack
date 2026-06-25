#!/usr/bin/env python3
"""
Bridge: skeleton "pointed location" -> Nav2 goal.

Subscribes
----------
/pointed_location  (geometry_msgs/msg/PoseArray)
    Published by the MediaPipe pose node. Always 2 poses:
        index 0 = left arm, index 1 = right arm.
    position = NaN when that arm is not pointing at the ground.
    Coordinates are normalized MediaPipe space (x 0-1 horizontal, y = depth,
    z = vertical) -- NOT meters.

Action
------
navigate_to_pose  (nav2_msgs/action/NavigateToPose)
    Sends the computed goal to Nav2. Relative name, so launching this node under a
    namespace (e.g. robot1) targets /robot1/navigate_to_pose; on the real robot with
    no namespace it targets /navigate_to_pose.

Pipeline
--------
1. Pick a valid (non-NaN) arm pose (param `arm`: right | left | either).
2. Map the MediaPipe point to a metric offset in the robot frame:
       forward (base_link +x) = scale_forward * point.y          (MediaPipe depth)
       left    (base_link +y) = scale_lateral * (center_x - point.x)  (horizontal)
   Scales/center are PARAMETERS -- MediaPipe units are not meters, so tune these to
   your setup. (Set scale=1, center_x=0, and the right axes if your upstream already
   emits metric base_link coordinates.)
3. Transform that point from `source_frame` (default base_link) to `goal_frame`
   (default map) using TF -- this is the "sum with odometry" step, done properly
   (accounts for the robot's position AND heading).
4. Build a PoseStamped facing from the robot toward the point and send it as a
   NavigateToPose goal, rate-limited and de-duplicated so it doesn't spam Nav2.

This node only RECEIVES the point and feeds Nav2; it does not drive the robot
(Nav2 + the cmd_vel bridge do that).

Same logic, two targets (only params differ -- the code is identical):
  * SIM  (Jazzy, map-based Nav2):
        ros2 run quadropted_controller pointed_goal.py \
            --ros-args -r __ns:=/robot1 -p goal_frame:=map
  * REAL (Humble, mapless Nav2 in EE_hack/nav2 -- costmaps in odom):
        python3 pointed_goal.py --ros-args -p goal_frame:=odom
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion
from nav2_msgs.action import NavigateToPose

import tf2_ros
# do_transform_point lives in tf2_geometry_msgs on both Humble and Jazzy, but some
# builds only expose it via the submodule -- import defensively so the SAME file
# runs unchanged on the real robot (Humble) and in sim (Jazzy).
try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point


def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class PointedGoal(Node):
    def __init__(self):
        super().__init__("pointed_goal")

        # --- parameters ---
        self.declare_parameter("input_topic", "/pointed_location")
        self.declare_parameter("arm", "either")          # right | left | either
        self.declare_parameter("goal_frame", "map")      # frame Nav2 goals are sent in
        self.declare_parameter("source_frame", "base_link")  # frame the offset is in
        self.declare_parameter("scale_forward", 1.0)     # meters per MediaPipe depth unit
        self.declare_parameter("scale_lateral", 1.0)     # meters per MediaPipe horiz unit
        self.declare_parameter("center_x", 0.5)          # MediaPipe horizontal center
        self.declare_parameter("min_goal_interval", 2.0)  # s between goals (rate limit)
        self.declare_parameter("min_goal_delta", 0.3)    # m; ignore goals closer than this to the last

        gp = self.get_parameter
        self.arm = gp("arm").value
        self.goal_frame = gp("goal_frame").value
        self.source_frame = gp("source_frame").value
        self.scale_forward = float(gp("scale_forward").value)
        self.scale_lateral = float(gp("scale_lateral").value)
        self.center_x = float(gp("center_x").value)
        self.min_goal_interval = float(gp("min_goal_interval").value)
        self.min_goal_delta = float(gp("min_goal_delta").value)

        # --- tf, action client, subscription ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(
            PoseArray, gp("input_topic").value, self._on_points, 10)

        self._last_goal = None       # (x, y) in goal_frame
        self._last_sent_time = None

        self.get_logger().info(
            f"pointed_goal: {gp('input_topic').value} -> navigate_to_pose "
            f"(arm={self.arm}, {self.source_frame}->{self.goal_frame})")

    # ------------------------------------------------------------------
    def _pick_pose(self, poses):
        """Return a valid (non-NaN) pose per the `arm` preference, or None."""
        def valid(i):
            return (len(poses) > i and
                    not any(math.isnan(v) for v in
                            (poses[i].position.x, poses[i].position.y, poses[i].position.z)))
        if self.arm == "left":
            return poses[0] if valid(0) else None
        if self.arm == "right":
            return poses[1] if valid(1) else None
        # either: prefer right, fall back to left
        if valid(1):
            return poses[1]
        if valid(0):
            return poses[0]
        return None

    def _on_points(self, msg: PoseArray):
        pose = self._pick_pose(msg.poses)
        if pose is None:
            return  # no arm pointing at the ground this frame

        # rate limit
        now = self.get_clock().now()
        if self._last_sent_time is not None:
            dt = (now - self._last_sent_time).nanoseconds / 1e9
            if dt < self.min_goal_interval:
                return

        # MediaPipe normalized point -> metric offset in source_frame (base_link)
        fwd = self.scale_forward * pose.position.y
        lat = self.scale_lateral * (self.center_x - pose.position.x)

        pt = PointStamped()
        pt.header.frame_id = self.source_frame
        pt.header.stamp = rclpy.time.Time().to_msg()  # latest available transform
        pt.point.x = fwd
        pt.point.y = lat
        pt.point.z = 0.0

        # source_frame -> goal_frame (this is the "+ odom/pose" step, via TF)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.goal_frame, self.source_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.3))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"TF {self.source_frame}->{self.goal_frame} unavailable: {e}")
            return
        gp_pt = do_transform_point(pt, tf)
        gx, gy = gp_pt.point.x, gp_pt.point.y

        # de-duplicate: skip goals too close to the last one
        if self._last_goal is not None:
            if math.hypot(gx - self._last_goal[0], gy - self._last_goal[1]) < self.min_goal_delta:
                return

        # face from the robot toward the goal
        yaw = math.atan2(gy - tf.transform.translation.y, gx - tf.transform.translation.x)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.goal_frame
        goal_pose.header.stamp = now.to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.orientation = yaw_to_quat(yaw)

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 navigate_to_pose action server not available yet")
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.nav_client.send_goal_async(goal)
        self._last_goal = (gx, gy)
        self._last_sent_time = now
        self.get_logger().info(f"Sent Nav2 goal in {self.goal_frame}: x={gx:.2f} y={gy:.2f}")


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
