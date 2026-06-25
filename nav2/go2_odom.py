#!/usr/bin/env python3
"""go2_odom — publish nav2-shaped odometry (TF odom->base_link + /odom) for the Go2.

nav2 / AMCL localise the laser by chaining TF:

    map  ->  odom  ->  base_link  ->  <lidar frame>
    └ amcl          └ THIS node    └ static_transform_publisher (launch)

AMCL publishes the `map -> odom` correction; it is NOT an odometry source — it
expects something else to publish `odom -> base_link`, the robot's continuous
dead-reckoning motion. This node provides exactly that, from one of two sources:

  * odom_source=utlidar   (DEFAULT) — the L1 LiDAR-inertial odometry the Go2
        publishes on `/utlidar/robot_odom` (nav_msgs/Odometry). Lower drift than
        the sport estimator; this is what we want for map-based navigation.
  * odom_source=sportmodestate      — the sport-mode dead-reckoning on
        `/sportmodestate` (unitree_go/msg/SportModeState): an integrated
        position + body orientation. Kept as a fallback if utlidar odom is not
        available on a given firmware.

Either way we republish, in the frames nav2 expects:

  * TF        odom -> base_link        (what AMCL / the costmaps chain on)
  * /odom     nav_msgs/Odometry        (what the nav2 controller consumes)

Why republish utlidar odom instead of using it directly? The Go2 stamps its
topics with the robot clock and labels them with its own frame ids; we re-stamp
with our clock and relabel to odom/base_link so the whole nav2 TF tree is
internally consistent and free of clock-skew extrapolation errors.

Verify the source exists on the live robot before trusting it:
    ros2 topic type /utlidar/robot_odom        # expect nav_msgs/msg/Odometry
    ros2 topic hz   /utlidar/robot_odom
If utlidar odom is missing, relaunch with odom_source:=sportmodestate.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster

UTLIDAR_ODOM_TOPIC = "/utlidar/robot_odom"
SPORT_STATE_TOPIC = "/sportmodestate"


class Go2Odom(Node):
    def __init__(self):
        super().__init__("go2_odom")
        self.declare_parameter("odom_source", "utlidar")  # utlidar | sportmodestate
        self.declare_parameter("utlidar_odom_topic", UTLIDAR_ODOM_TOPIC)
        self.declare_parameter("sport_state_topic", SPORT_STATE_TOPIC)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self._source = self.get_parameter("odom_source").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # The Go2 publishes both odometry sources BEST_EFFORT — match it or we
        # receive nothing.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        if self._source == "sportmodestate":
            from unitree_go.msg import SportModeState  # noqa: PLC0415 (image-only dep)
            topic = self.get_parameter("sport_state_topic").value
            self.create_subscription(SportModeState, topic, self._on_sport, qos)
            self._src_topic = topic
        else:  # default: utlidar
            self._source = "utlidar"
            topic = self.get_parameter("utlidar_odom_topic").value
            self.create_subscription(Odometry, topic, self._on_utlidar, qos)
            self._src_topic = topic

        self._count = 0
        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info(
            f"go2_odom up [{self._source}]: {self._src_topic} -> "
            f"TF {self._odom_frame}->{self._base_frame} + /odom")

    def _heartbeat(self):
        if self._count:
            self.get_logger().info(
                f"{self._src_topic}: {self._count} msgs in last 5s (odom flowing)")
        else:
            self.get_logger().warn(
                f"{self._src_topic}: 0 msgs in last 5s — no odom; check the topic "
                f"type/DDS domain. If on utlidar, try odom_source:=sportmodestate. "
                "nav2 cannot plan or localise without odom->base_link.")
        self._count = 0

    def _publish(self, now, px, py, pz, x, y, z, w, vx, vy, vz, wz):
        """Emit the odom->base_link TF and a /odom message from a normalised pose."""
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
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)

    def _on_utlidar(self, msg: Odometry):
        """L1 LiDAR-inertial odometry — re-stamp + relabel into odom/base_link."""
        self._count += 1
        now = self.get_clock().now().to_msg()
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist
        self._publish(
            now, float(p.x), float(p.y), float(p.z),
            float(o.x), float(o.y), float(o.z), float(o.w),
            float(t.linear.x), float(t.linear.y), float(t.linear.z),
            float(t.angular.z))

    def _on_sport(self, msg):
        """Sport-mode dead reckoning (SportModeState) — quaternion is w-first."""
        self._count += 1
        now = self.get_clock().now().to_msg()
        px, py, pz = (float(v) for v in msg.position[:3])
        w, x, y, z = (float(v) for v in msg.imu_state.quaternion[:4])  # unitree: w-first
        vx, vy, vz = (float(v) for v in msg.velocity[:3])
        self._publish(now, px, py, pz, x, y, z, w, vx, vy, vz, float(msg.yaw_speed))


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
