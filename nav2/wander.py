#!/usr/bin/env python3
"""wander — reactive "drive toward open space" for the Go2, straight off /scan.

No map, no AMCL, no planner. Each cycle we read the latest LaserScan, find the
widest-open bearing in the front sector, and steer there by publishing
geometry_msgs/Twist on /cmd_vel — which the existing cmd_vel_to_sport bridge
relays to the dog's sport Move API. It keeps moving until something stops it
(the server SIGINTs this process when you press Stop in the UI).

Behaviour
---------
* Open ahead  -> drive forward, steering toward the freest bearing; speed scales
  down as the nearest front obstacle gets closer (slow_dist -> stop_dist).
* Boxed in    -> stop forward motion and ROTATE in place toward the side with
  more clearance until the front opens up again, then resume. Never stops on its
  own.
* Stale scan  -> publish zero (and rely on the bridge's watchdog) so a dropped
  laser halts the dog instead of letting it run blind.

The freest bearing is chosen by binning the steering sector and picking the bin
with the largest *minimum* range (widest gap, not a single far pixel) — robust
against darting through a thin gap.

Frames: /scan is in base_link (pointcloud_to_laserscan target_frame), so angle 0
is straight ahead and +angle is to the left (CCW), matching cmd_vel's +wz = left.

Params (ros2 -p name:=value): front_deg, steer_deg, stop_dist, slow_dist,
max_vx, max_wz, k_steer, rate_hz, scan_topic, cmd_vel_topic.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Wander(Node):
    def __init__(self):
        super().__init__("wander")
        p = self.declare_parameter
        self.front_deg = float(p("front_deg", 30.0).value)   # half-cone for "blocked ahead"
        self.steer_deg = float(p("steer_deg", 90.0).value)   # half-sector searched for a gap
        self.stop_dist = float(p("stop_dist", 0.6).value)    # < this in front -> rotate
        self.slow_dist = float(p("slow_dist", 1.5).value)    # start slowing below this
        self.max_vx = float(p("max_vx", 0.30).value)
        self.max_wz = float(p("max_wz", 0.7).value)
        self.k_steer = float(p("k_steer", 1.2).value)        # target-bearing (rad) -> wz gain
        self.rate_hz = float(p("rate_hz", 12.0).value)
        scan_topic = p("scan_topic", "/scan").value
        cmd_topic = p("cmd_vel_topic", "/cmd_vel").value

        self._scan = None
        self._scan_t = 0.0
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self._cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self._count = 0
        self.create_timer(3.0, self._heartbeat)
        self.get_logger().info(
            f"wander up: {scan_topic} -> {cmd_topic} | front=±{self.front_deg}° "
            f"steer=±{self.steer_deg}° stop={self.stop_dist}m max_vx={self.max_vx}")

    def _on_scan(self, msg: LaserScan):
        self._scan = msg
        self._scan_t = self.now()
        self._count += 1

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _heartbeat(self):
        if not self._count:
            self.get_logger().warn(
                "no /scan in last 3s — dog held still. Check pointcloud_to_laserscan.")
        self._count = 0

    @staticmethod
    def _valid(r, lo, hi):
        return (r is not None) and math.isfinite(r) and lo <= r <= hi

    def _tick(self):
        # Stale / missing scan -> stop and let the bridge watchdog catch it.
        if self._scan is None or (self.now() - self._scan_t) > 0.5:
            self._cmd.publish(Twist())
            return

        s = self._scan
        front = math.radians(self.front_deg)
        steer = math.radians(self.steer_deg)
        rmax = s.range_max if math.isfinite(s.range_max) and s.range_max > 0 else 30.0

        # Front-cone nearest obstacle, and a binned "widest gap" over the steer sector.
        nbins = 13
        bin_w = (2.0 * steer) / nbins
        bin_min = [float("inf")] * nbins
        d_front = float("inf")
        left_sum = right_sum = 0.0
        left_n = right_n = 0

        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if a < -steer or a > steer:
                continue
            d = r if self._valid(r, s.range_min, rmax) else rmax
            if -front <= a <= front and d < d_front:
                d_front = d
            b = int((a + steer) / bin_w)
            b = _clamp(b, 0, nbins - 1)
            if d < bin_min[b]:
                bin_min[b] = d
            if a >= 0:
                left_sum += d; left_n += 1
            else:
                right_sum += d; right_n += 1

        cmd = Twist()
        if d_front < self.stop_dist:
            # Boxed in: rotate toward the freer side until the front clears.
            left = left_sum / left_n if left_n else 0.0
            right = right_sum / right_n if right_n else 0.0
            cmd.angular.z = self.max_wz if left >= right else -self.max_wz
        else:
            # Steer toward the center of the widest-open bin.
            best = max(range(nbins), key=lambda b: bin_min[b])
            target = -steer + (best + 0.5) * bin_w          # bearing of that gap
            wz = _clamp(self.k_steer * target, -self.max_wz, self.max_wz)
            # Forward speed: scale with front clearance, and back off while turning hard.
            scale = _clamp((d_front - self.stop_dist) / max(1e-3, self.slow_dist - self.stop_dist), 0.0, 1.0)
            vx = self.max_vx * scale * max(0.2, 1.0 - abs(wz) / self.max_wz)
            cmd.linear.x = vx
            cmd.angular.z = wz
        self._cmd.publish(cmd)

    def stop(self):
        for _ in range(3):
            self._cmd.publish(Twist())


def main():
    rclpy.init()
    node = Wander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
