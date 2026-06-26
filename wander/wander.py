#!/usr/bin/env python3
"""wander — reactive driving for the Go2, straight off /scan.

NOTE: this file keeps the wander SERVICE's interface (read /scan, publish
geometry_msgs/Twist on /cmd_vel, started/stopped by server.py, tuned via the
same -p params) but the ALGORITHM inside is the **disparity extender**
(F1Tenth/Vanderbilt, after Nathan Otterness) ported to the legged Go2 — the
original "widest-open bin" heuristic has been removed. server.py, entrypoint.sh,
pipeline.launch.py and cmd_vel_to_sport.py are unchanged: this still just turns
/scan into /cmd_vel, which the bridge relays to the dog's sport Move API.

Algorithm (unchanged in spirit from the racing original):
  1. read the latest LaserScan,
  2. find DISPARITIES — adjacent samples whose range jumps by more than
     `disparity_threshold` (the edge of an obstacle),
  3. EXTEND the nearer side of each disparity across the angular width the dog's
     body needs to clear it (so any gap we pick is one the whole body fits),
  4. steer toward the deepest remaining gap, slowing as forward clearance drops.

Adapted for a legged robot (vs. the Ackermann car it came from): the car emits a
steering angle; the Go2 cannot steer, it yaws — so the chosen gap bearing becomes
a YAW RATE on /cmd_vel.angular.z, and forward speed is /cmd_vel.linear.x scaled by
forward clearance and cos(gap) (slow when the path is short OR the turn is sharp).
All scan indices are derived from the LaserScan's own angle_min/angle_increment —
nothing hardcoded — so it runs unchanged on the real L1 /scan and in sim.

Frames: /scan is in base_link (pointcloud_to_laserscan target_frame), so angle 0
is straight ahead and +angle is to the left (CCW), matching /cmd_vel +wz = left.

Robustness (so the known L1 failure modes can't bite):
  * BOXED IN -> never freezes: it rotates in place toward the freer side until the
    front opens (the deepest-gap bearing can be ~0° when blocked, which would
    stall — so when blocked we steer by side clearance instead).
  * Sparse/noisy L1 -> EMA smoothing (`smooth`) damps per-frame gap hopping so the
    dog doesn't twitch.
  * Self-returns (the dog's own body at 0–0.5 m) are dropped upstream by
    range_min in pointcloud_to_laserscan.yaml, so /scan never reads a phantom
    "obstacle 0.2 m ahead."

Params (ros2 -p name:=value) — the three server.py forwards plus the disparity
knobs: max_vx, max_wz, stop_dist, slow_dist, disparity_threshold,
robot_half_width, front_deg, yaw_gain, escape_wz, smooth, rate_hz, scan_topic,
cmd_vel_topic.
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
        super().__init__("disparity_extender")
        p = self.declare_parameter
        # --- the three server.py forwards (kept named-compatible) ---
        self.max_vx = float(p("max_vx", 0.30).value)
        self.max_wz = float(p("max_wz", 0.70).value)        # yaw-rate clamp
        self.stop_dist = float(p("stop_dist", 0.60).value)  # < this ahead -> no fwd
        # --- disparity-extender knobs ---
        self.slow_dist = float(p("slow_dist", 2.0).value)   # full speed at/above this
        self.disparity_threshold = float(p("disparity_threshold", 0.30).value)
        self.robot_half_width = float(p("robot_half_width", 0.25).value)  # body half-width + tol
        self.front_deg = float(p("front_deg", 90.0).value)  # half-cone gaps are chosen in
        self.cone_deg = float(p("forward_cone_deg", 12.0).value)  # fwd-clearance cone
        self.side_guard_deg = float(p("side_guard_deg", 60.0).value)
        self.side_clearance = float(p("side_clearance", 0.30).value)
        self.yaw_gain = float(p("yaw_gain", 1.2).value)     # gap bearing (rad) -> wz
        self.escape_wz = float(p("escape_wz", 0.60).value)  # rotate rate when boxed in
        self.smooth = float(p("smooth", 0.4).value)         # EMA alpha (0=off,->1 snappy)
        self.rate_hz = float(p("rate_hz", 12.0).value)
        # Set true if the lidar is mounted UPSIDE DOWN: forward stays forward but
        # left/right are mirrored, so the dog would steer the wrong way (into walls).
        # This reverses the scan to restore correct left/right.
        self.flip_scan = bool(p("flip_scan", False).value)
        scan_topic = p("scan_topic", "/scan").value
        cmd_topic = p("cmd_vel_topic", "/cmd_vel").value

        self._scan = None
        self._scan_t = 0.0
        self._prev_vx = 0.0       # EMA state (twitch suppression)
        self._prev_wz = 0.0
        self._escape_sign = 0     # latched dead-end rotate dir (0 = not escaping)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self._cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self._count = 0
        self.create_timer(3.0, self._heartbeat)
        self.get_logger().info(
            f"disparity-extender up: {scan_topic} -> {cmd_topic} | front=±{self.front_deg}° "
            f"thr={self.disparity_threshold} half_width={self.robot_half_width} "
            f"stop={self.stop_dist}m max_vx={self.max_vx} max_wz={self.max_wz}")

    # ------------------------------------------------------------------ scan ---
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

    def _a2i(self, s, angle):
        i = int(round((angle - s.angle_min) / s.angle_increment))
        return _clamp(i, 0, len(s.ranges) - 1)

    def _clean(self, s):
        """ranges -> list with inf/nan/<=0 -> range_max, finite capped to it."""
        cap = s.range_max if (math.isfinite(s.range_max) and s.range_max > 0) else 30.0
        out = []
        for r in s.ranges:
            if r is None or not math.isfinite(r) or r <= 0.0:
                out.append(cap)
            else:
                out.append(min(r, cap))
        if self.flip_scan:
            out.reverse()   # upside-down lidar: un-mirror left/right
        return out, cap

    def _find_disparities(self, r, lo, hi):
        thr = self.disparity_threshold
        return [i for i in range(lo, hi) if abs(r[i] - r[i + 1]) >= thr]

    def _extend_disparities(self, r, disparities, s, lo, hi):
        """Extend the nearer side of each disparity over the body's angular width
        at that range, clamped to the front window [lo, hi]."""
        out = list(r)
        inc = s.angle_increment
        for i in disparities:
            v1, v2 = r[i], r[i + 1]
            if v1 < v2:                       # nearer obstacle on the left edge
                nearer, idx, step = v1, i, +1
            else:                             # nearer obstacle on the right edge
                nearer, idx, step = v2, i + 1, -1
            n = int(math.ceil(self.robot_half_width / (inc * max(nearer, 1e-3))))
            cur = idx
            for _ in range(n):
                if cur < lo or cur > hi:
                    break
                if out[cur] > nearer:
                    out[cur] = nearer
                cur += step
        return out

    # --------------------------------------------------------------- control ---
    def _tick(self):
        # Stale / missing scan -> stop and let the bridge watchdog catch it.
        if self._scan is None or (self.now() - self._scan_t) > 0.5:
            self._prev_vx = self._prev_wz = 0.0
            self._cmd.publish(Twist())
            return

        s = self._scan
        if len(s.ranges) < 8:
            self._prev_vx = self._prev_wz = 0.0
            self._cmd.publish(Twist())
            return
        ranges, cap = self._clean(s)

        front = math.radians(self.front_deg)
        lo, hi = self._a2i(s, -front), self._a2i(s, front)
        if hi <= lo:
            lo, hi = 0, len(ranges) - 1

        # 1-2-3: disparities -> extend -> pick the deepest gap (middle of the plateau).
        ext = self._extend_disparities(ranges, self._find_disparities(ranges, lo, hi), s, lo, hi)
        window = ext[lo:hi + 1]
        best = max(window)
        cand = [lo + k for k, v in enumerate(window) if v >= best - 1e-3]
        target = cand[len(cand) // 2]
        gap = s.angle_min + target * s.angle_increment      # bearing to the gap (rad)

        # Forward clearance: nearest RAW return in a narrow cone straight ahead.
        cone = math.radians(self.cone_deg)
        fl, fh = self._a2i(s, -cone), self._a2i(s, cone)
        d_front = min(ranges[fl:fh + 1]) if fh > fl else ranges[self._a2i(s, 0.0)]

        if d_front <= self.stop_dist:
            # BOXED IN: never sit frozen. Rotate IN PLACE toward the side with more
            # clearance until the front opens (the gap bearing can be ~0° here, which
            # would stall — so we ignore it and use side clearance, like the original
            # wander's escape). Rotating in place doesn't translate, so it's safe even
            # when close; no side-guard applied here.
            vx = 0.0
            # Latch the rotate direction on entry so it commits to ONE way out
            # instead of wobbling as the scan flickers; reset when the front opens.
            if self._escape_sign == 0:
                self._escape_sign = 1 if self._side_balance(ranges, s, lo, hi) >= 0 else -1
            wz = self.escape_wz * self._escape_sign
        else:
            self._escape_sign = 0
            # Drive toward the deepest gap; slow with clearance and on sharp turns.
            wz = _clamp(self.yaw_gain * gap, -self.max_wz, self.max_wz)
            span = max(1e-3, self.slow_dist - self.stop_dist)
            scale = _clamp((d_front - self.stop_dist) / span, 0.0, 1.0)
            vx = self.max_vx * scale * max(0.15, math.cos(gap))
            wz = self._guard_sides(wz, ranges, s)   # don't yaw into a wall while moving

        # EMA smoothing: suppress per-frame gap hopping / twitch on the sparse L1.
        # SAFETY: only smooth speeding UP — a slow-down/stop takes effect instantly,
        # so EMA lag can never carry the dog forward into an obstacle it just decided
        # to stop for. (Turning stays smoothed both ways; turn lag isn't dangerous.)
        a = _clamp(self.smooth, 0.0, 1.0)
        vx = vx if vx < self._prev_vx else a * vx + (1.0 - a) * self._prev_vx
        wz = a * wz + (1.0 - a) * self._prev_wz
        self._prev_vx, self._prev_wz = vx, wz

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self._cmd.publish(cmd)

    def _side_balance(self, ranges, s, lo, hi):
        """>0 if the LEFT half of the front window is more open than the RIGHT.
        Used only to choose which way to rotate out of a dead-end."""
        c = self._a2i(s, 0.0)
        left = ranges[c:hi + 1]
        right = ranges[lo:c]
        ml = sum(left) / len(left) if left else 0.0
        mr = sum(right) / len(right) if right else 0.0
        return ml - mr

    def _guard_sides(self, wz, ranges, s):
        """Don't yaw the body into a wall right beside it. Left=+angle, right=-angle."""
        cone = math.radians(self.cone_deg)
        guard = math.radians(self.side_guard_deg)
        ll, lh = self._a2i(s, cone), self._a2i(s, guard)
        rl, rh = self._a2i(s, -guard), self._a2i(s, -cone)
        min_left = min(ranges[ll:lh + 1]) if lh > ll else float("inf")
        min_right = min(ranges[rl:rh + 1]) if rh > rl else float("inf")
        if wz > 0.0 and min_left <= self.side_clearance:
            return 0.0
        if wz < 0.0 and min_right <= self.side_clearance:
            return 0.0
        return wz

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
