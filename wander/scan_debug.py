#!/usr/bin/env python3
"""scan_debug — publish average /scan range in three sectors for the UI debugger.

Subscribes the 2D /scan (after ground filtering) and, every scan, publishes a
std_msgs/String "front,left,right" of the MEAN range (metres) in each sector:

    left  ◤   ◥        front = |angle| <= front_deg      (straight ahead)
       ◤ front ◥       left  =  front_deg < angle <= side_max_deg
   ───◀───┼───▶───     right = -side_max_deg <= angle < -front_deg
       ◣  right ◢      (angle 0 = ahead, +angle = left/CCW, matching cmd_vel +wz)

No-return / invalid beams count as range_max (open), so a larger average = more
open space on that side — the same "openness" wander steers by. server.py tails
this String over the ros2 CLI and shows it; this keeps the web process rclpy-free.

Params: front_deg (45), side_max_deg (135), scan_topic (/scan), out_topic
(/wander/debug).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanDebug(Node):
    def __init__(self):
        super().__init__("scan_debug")
        self.front = math.radians(float(self.declare_parameter("front_deg", 45.0).value))
        self.side = math.radians(float(self.declare_parameter("side_max_deg", 135.0).value))
        scan_topic = self.declare_parameter("scan_topic", "/scan").value
        out_topic = self.declare_parameter("out_topic", "/wander/debug").value

        self._pub = self.create_publisher(String, out_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f"scan_debug up: {scan_topic} -> {out_topic} "
            f"(front=±{math.degrees(self.front):.0f}° sides up to ±{math.degrees(self.side):.0f}°)")

    def _on_scan(self, s: LaserScan):
        rmax = s.range_max if math.isfinite(s.range_max) and s.range_max > 0 else 30.0
        fs = fn = ls = ln = rs = rn = 0.0
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            d = r if (math.isfinite(r) and s.range_min <= r <= rmax) else rmax
            if -self.front <= a <= self.front:
                fs += d; fn += 1
            elif self.front < a <= self.side:
                ls += d; ln += 1
            elif -self.side <= a < -self.front:
                rs += d; rn += 1
        front = fs / fn if fn else rmax
        left = ls / ln if ln else rmax
        right = rs / rn if rn else rmax
        self._pub.publish(String(data=f"{front:.2f},{left:.2f},{right:.2f}"))


def main():
    rclpy.init()
    node = ScanDebug()
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
