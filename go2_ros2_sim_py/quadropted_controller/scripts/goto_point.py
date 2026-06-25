#!/usr/bin/env python3
"""
Send the robot to a chosen (x, y[, yaw]) point via Nav2.

A manual counterpart to pointed_goal.py: instead of a camera pointing gesture, you
pick the coordinate. Same Nav2 interface (NavigateToPose), same sim/real split.

Frame choice:
  frame:=map   absolute coordinate in your SLAM map (needs map_server + localization)
  frame:=odom  relative to where the robot started (mapless stacks, e.g. EE_hack/nav2)

Usage
-----
  # Sim (Jazzy, map-based, namespaced):
  ros2 run quadropted_controller goto_point.py \
      --ros-args -r __ns:=/robot1 -p frame:=map -p x:=2.0 -p y:=1.0 -p yaw:=0.0

  # Real robot (Humble, mapless):
  python3 goto_point.py --ros-args -p frame:=odom -p x:=2.0 -p y:=0.0

The node sends ONE goal, prints feedback (distance remaining), reports the result,
then exits.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class GotoPoint(Node):
    def __init__(self):
        super().__init__("goto_point")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame", "map")

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.frame = self.get_parameter("frame").value

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info("Waiting for navigate_to_pose action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available — is Nav2 up?")
            rclpy.shutdown()
            return
        self._send()

    def _send(self):
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = self.frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = self.x
        p.pose.position.y = self.y
        p.pose.orientation = yaw_to_quat(self.yaw)
        goal.pose = p

        self.get_logger().info(
            f"Sending goal: frame={self.frame} x={self.x:.2f} y={self.y:.2f} yaw={self.yaw:.2f}")
        fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _on_feedback(self, fb):
        self.get_logger().info(
            f"distance remaining: {fb.feedback.distance_remaining:.2f} m",
            throttle_duration_sec=2.0)

    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("Goal REJECTED by Nav2")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted — navigating...")
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _fut):
        self.get_logger().info("Navigation finished.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GotoPoint()
    # Spin until a callback (result, rejection, or no server) calls rclpy.shutdown().
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
