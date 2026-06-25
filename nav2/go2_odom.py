#!/usr/bin/env python3
"""go2_odom — turn the Go2's `/sportmodestate` into nav2-shaped odometry.

slam_toolbox (and nav2 in general) localise the laser by chaining TF:

    map  ->  odom  ->  base_link  ->  <lidar frame>
    └ slam_toolbox  └ THIS node    └ static_transform_publisher (launch)

slam_toolbox publishes `map -> odom` (the SLAM correction). It is NOT an
odometry source — it expects something else to publish `odom -> base_link`, the
robot's dead-reckoning motion. The Go2 gives us exactly that on
`/sportmodestate` (unitree_go/msg/SportModeState): an integrated position +
body orientation in an `odom`-style frame, the same field the bridge reads for
`/go2/pose`. We republish it as:

  * TF        odom -> base_link        (what slam_toolbox consumes)
  * /odom     nav_msgs/Odometry        (what the nav2 stack / costmaps consume)

SportModeState fields used (see unitree_go/msg/SportModeState, and bridge/app.py
`_on_sport`): `position` [x,y,z] (m, odom frame), `velocity` [vx,vy,vz] (m/s,
body frame), `yaw_speed` (rad/s), `imu_state.quaternion` [w,x,y,z]. Unitree
orders the quaternion w-first; ROS wants x,y,z,w — we reorder.

UNVERIFIED on a live EDU+: the exact frame `position` integrates in (treated as
`odom` here) and any drift/reset behaviour aren't confirmed. This is dead
reckoning — it drifts; correcting that drift is precisely slam_toolbox's job.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster
from unitree_go.msg import SportModeState

SPORT_STATE_TOPIC = "/sportmodestate"


class Go2Odom(Node):
    def __init__(self):
        super().__init__("go2_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # The Go2 publishes sportmodestate BEST_EFFORT — match it or we get nothing.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(SportModeState, SPORT_STATE_TOPIC, self._on_state, qos)

        self._count = 0
        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info(
            f"go2_odom up: {SPORT_STATE_TOPIC} -> TF {self._odom_frame}->{self._base_frame} + /odom")

    def _heartbeat(self):
        if self._count:
            self.get_logger().info(
                f"{SPORT_STATE_TOPIC}: {self._count} msgs in last 5s (odom flowing)")
        else:
            self.get_logger().warn(
                f"{SPORT_STATE_TOPIC}: 0 msgs in last 5s — no odom; check DDS/domain. "
                "slam_toolbox can't track without odom->base_link.")
        self._count = 0

    def _on_state(self, msg: SportModeState):
        self._count += 1
        now = self.get_clock().now().to_msg()
        px, py, pz = (float(v) for v in msg.position[:3])
        w, x, y, z = (float(v) for v in msg.imu_state.quaternion[:4])  # unitree: w-first
        # Normalise — a non-unit quaternion makes TF reject the transform.
        n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
        w, x, y, z = w / n, x / n, y / n, z / n

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.x = x
        tf.transform.rotation.y = y
        tf.transform.rotation.z = z
        tf.transform.rotation.w = w
        self._tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        vx, vy, vz = (float(v) for v in msg.velocity[:3])
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = Go2Odom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
