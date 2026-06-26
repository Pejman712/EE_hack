#!/usr/bin/env python3
"""ogaboga — the dumbest reactive walker. No gaps, no steering-to-openness, no map.

Look straight ahead over a narrow FOV; if nothing is close, walk forward; if
something is close, stop and spin RIGHT until the front clears, then walk again:

    nearest return in front ±(fov_deg/2):
        > clear_dist (0.3 m)  -> drive forward  (fwd_vx)
        otherwise             -> turn right     (-turn_wz) until clear

Reads /scan (2D, from pointcloud_to_laserscan) and publishes /cmd_vel (relayed to
the dog by cmd_vel_to_sport). A stale/missing scan publishes zero so the bridge
watchdog halts the dog instead of letting it run blind.

Params (ros2 -p name:=value):
  fov_deg 30 · clear_dist 0.3 · fwd_vx 0.3 · turn_wz 0.6 · rate_hz 12
  scan_topic /scan · cmd_vel_topic /cmd_vel
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class OgaBoga(Node):
    def __init__(self):
        super().__init__("ogaboga")
        self.half = math.radians(float(self.declare_parameter("fov_deg", 30.0).value) / 2.0)
        self.clear_dist = float(self.declare_parameter("clear_dist", 0.3).value)
        self.fwd_vx = float(self.declare_parameter("fwd_vx", 0.3).value)
        self.turn_wz = float(self.declare_parameter("turn_wz", 0.6).value)
        rate = float(self.declare_parameter("rate_hz", 12.0).value)
        scan_topic = self.declare_parameter("scan_topic", "/scan").value
        cmd_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value

        self._scan = None
        self._scan_t = 0.0
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self._cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"ogaboga up: front ±{math.degrees(self.half):.0f}°  clear>{self.clear_dist}m "
            f"-> forward {self.fwd_vx}m/s  else turn right {self.turn_wz}rad/s")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg: LaserScan):
        self._scan = msg
        self._scan_t = self._now()

    def _tick(self):
        if self._scan is None or (self._now() - self._scan_t) > 0.5:
            self._cmd.publish(Twist())  # no fresh scan -> hold still
            return
        s = self._scan
        rmax = s.range_max if math.isfinite(s.range_max) and s.range_max > 0 else 30.0

        # nearest valid return inside the front FOV
        nearest = rmax
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if -self.half <= a <= self.half and math.isfinite(r) and s.range_min <= r <= rmax:
                if r < nearest:
                    nearest = r

        cmd = Twist()
        if nearest > self.clear_dist:
            cmd.linear.x = self.fwd_vx       # clear -> go forward
        else:
            cmd.angular.z = -self.turn_wz    # blocked -> turn right (CW) until clear
        self._cmd.publish(cmd)


def main():
    rclpy.init()
    node = OgaBoga()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._cmd.publish(Twist())  # stop on exit
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
