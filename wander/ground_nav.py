#!/usr/bin/env python3
"""ground_nav — reactive "drive over the floor" straight off the RAW 3D L2 cloud.

This is the 3D answer to the tilted-lidar problem. Instead of flattening the
cloud to a 2D /scan (which needs the mount pitch dialed in by hand, or the floor
falls outside the height band and every beam reads max range), we work in 3D:

  1. PLANE FIT (RANSAC).  Find the plane with the most points in the cloud — for a
     down-forward L2 that is the FLOOR.
  2. LEVEL.  Rotate the whole cloud so that plane's normal points along +Z. This
     makes the floor horizontal and the frame gravity-aligned NO MATTER how the
     lidar is tilted — the tilt is measured from the data, not configured.
  3. SEGMENT.  In the leveled frame, points within a thin band of the plane are
     FLOOR = drivable space; points standing above it (up to robot height) are
     OBSTACLES. (Points far below the plane are holes/cliffs — also not drivable.)
  4. STEER.  Bin the obstacles by bearing over the front sector, drive toward the
     widest gap, slow as the nearest front obstacle nears, rotate in place when
     boxed in. Same control law as wander.py, but obstacles are now real 3D
     above-floor returns instead of a mis-binned 2D slice.

Publishes geometry_msgs/Twist on /cmd_vel (relayed to the dog by cmd_vel_to_sport)
and, for the existing web UI, a std_msgs/String "front,left,right" of mean free
range per sector on /wander/debug. Optionally republishes the leveled obstacle
points on /wander/obstacles (PointCloud2) so you can watch the segmentation in
Foxglove.

Subscribes the RAW cloud (default /utlidar/cloud) — NOT the 2D /scan and NOT
pointcloud_to_laserscan; this node replaces both. No static TF / mount pitch is
needed; leveling is derived from the floor plane every frame.

Params (ros2 -p name:=value):
  cloud_topic /utlidar/cloud · cmd_vel_topic /cmd_vel
  plane_thresh 0.06   RANSAC inlier band (m) — a point this close to the plane is floor
  obs_min 0.08        a point must stand this far above the floor to count as an obstacle
  obs_max 1.20        ignore returns higher than this (ceiling / overhangs above the dog)
  ransac_iter 80 · max_pts 6000   plane-fit iterations / cloud subsample cap (speed)
  front_deg 30 · steer_deg 90 · stop_dist 0.6 · slow_dist 1.5
  max_vx 0.30 · max_wz 0.7 · k_steer 1.2 · range_max 8.0 · rate_hz 12
  publish_obstacles false   set true to emit /wander/obstacles for Foxglove
  flip_x_180 true   undo the L2 cloud's 180° X mount flip (negate y,z); RANSAC
                    levels tilt but NOT this mirror, so leave true on the real dog
"""
import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

# Params the UI / `ros2 param set` can change live — read every tick.
LIVE_PARAMS = ("front_deg", "steer_deg", "stop_dist", "slow_dist", "max_vx",
               "max_wz", "k_steer", "plane_thresh", "obs_min", "obs_max", "range_max")


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def cloud_to_xyz(msg: PointCloud2, flip_x_180: bool = True) -> np.ndarray:
    """sensor_msgs/PointCloud2 -> (N,3) float32 xyz, parsed straight from the buffer.

    Reads each field by its declared offset; assumes x/y/z are FLOAT32 (datatype 7),
    which is what the Unitree cloud uses. Returns finite points only.

    The Go2 L2's /utlidar/cloud is ROLLED 180° ABOUT X (mounted flipped), so we undo
    it by negating Y and Z, giving a forward(+x)/left(+y)/up(+z) cloud. RANSAC
    leveling fixes the floor TILT but NOT this handedness mirror, so without the
    un-flip the steering comes out left/right reversed. Set flip_x_180=False if the
    cloud is already upright.
    """
    off = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
    if not all(k in off for k in ("x", "y", "z")):
        return np.empty((0, 3), np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    n = raw.size // msg.point_step
    raw = raw[: n * msg.point_step].reshape(n, msg.point_step)

    def col(o):
        return raw[:, o : o + 4].copy().view(np.float32).ravel()

    xyz = np.stack([col(off["x"]), col(off["y"]), col(off["z"])], axis=1)
    if flip_x_180:
        xyz[:, 1:] *= -1.0  # 180° about X: y -> -y, z -> -z
    return xyz[np.isfinite(xyz).all(axis=1)]


def fit_plane_ransac(pts: np.ndarray, iters: int, thresh: float):
    """Largest-support plane via RANSAC. Returns (unit normal, d, inlier_mask) for
    plane  n·x + d = 0,  or None if it can't fit one. Normal is refined by SVD on
    the winning inliers."""
    n_pts = len(pts)
    if n_pts < 50:
        return None
    best_count, best_mask = 0, None
    rng = np.random.default_rng()
    for _ in range(iters):
        i, j, k = rng.integers(0, n_pts, 3)
        p0, p1, p2 = pts[i], pts[j], pts[k]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-6:
            continue
        nrm = nrm / ln
        dist = np.abs(pts @ nrm + (-nrm @ p0))
        mask = dist < thresh
        c = int(mask.sum())
        if c > best_count:
            best_count, best_mask = c, mask
    if best_mask is None or best_count < 50:
        return None
    inl = pts[best_mask]
    centroid = inl.mean(axis=0)
    # SVD: plane normal is the singular vector of least variance.
    _, _, vt = np.linalg.svd(inl - centroid, full_matrices=False)
    nrm = vt[-1]
    nrm = nrm / np.linalg.norm(nrm)
    d = float(-nrm @ centroid)
    return nrm, d, best_mask


def level_rotation(n_up: np.ndarray) -> np.ndarray:
    """Rotation matrix that maps the (upward) floor normal onto +Z (Rodrigues)."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n_up, z)
    s = np.linalg.norm(v)
    c = float(n_up @ z)
    if s < 1e-8:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


class GroundNav(Node):
    def __init__(self):
        super().__init__("ground_nav")
        p = self.declare_parameter
        self.plane_thresh = float(p("plane_thresh", 0.06).value)
        self.obs_min = float(p("obs_min", 0.08).value)
        self.obs_max = float(p("obs_max", 1.20).value)
        self.ransac_iter = int(p("ransac_iter", 80).value)
        self.max_pts = int(p("max_pts", 6000).value)
        self.front_deg = float(p("front_deg", 30.0).value)
        self.steer_deg = float(p("steer_deg", 90.0).value)
        self.stop_dist = float(p("stop_dist", 0.6).value)
        self.slow_dist = float(p("slow_dist", 1.5).value)
        self.max_vx = float(p("max_vx", 0.30).value)
        self.max_wz = float(p("max_wz", 0.7).value)
        self.k_steer = float(p("k_steer", 1.2).value)
        self.range_max = float(p("range_max", 8.0).value)
        self.rate_hz = float(p("rate_hz", 12.0).value)
        self.publish_obstacles = bool(p("publish_obstacles", False).value)
        self.flip_x_180 = bool(p("flip_x_180", True).value)  # L2 cloud is 180° X-flipped
        cloud_topic = p("cloud_topic", "/utlidar/cloud").value
        cmd_topic = p("cmd_vel_topic", "/cmd_vel").value

        self._cloud = None
        self._cloud_t = 0.0
        self._count = 0
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self._cmd = self.create_publisher(Twist, cmd_topic, 10)
        self._dbg = self.create_publisher(String, "/wander/debug", 10)
        self._obs_pub = (self.create_publisher(PointCloud2, "/wander/obstacles", 1)
                         if self.publish_obstacles else None)
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.create_timer(3.0, self._heartbeat)
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().info(
            f"ground_nav up: {cloud_topic} -> {cmd_topic} | plane±{self.plane_thresh}m "
            f"obs[{self.obs_min},{self.obs_max}]m front±{self.front_deg}° stop={self.stop_dist}m")

    def _on_set_params(self, params):
        for pr in params:
            if pr.name in LIVE_PARAMS:
                setattr(self, pr.name, float(pr.value))
        return SetParametersResult(successful=True)

    def _on_cloud(self, msg: PointCloud2):
        self._cloud = msg
        self._cloud_t = self.now()
        self._count += 1

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _heartbeat(self):
        if not self._count:
            self.get_logger().warn("no cloud in last 3s — dog held still. Check cloud_topic.")
        self._count = 0

    def _tick(self):
        # Stale / missing cloud -> stop and let the bridge watchdog catch it.
        if self._cloud is None or (self.now() - self._cloud_t) > 0.5:
            self._cmd.publish(Twist())
            return

        pts = cloud_to_xyz(self._cloud, self.flip_x_180)
        if len(pts) < 50:
            self._cmd.publish(Twist())
            return

        # Subsample for a fast, steady-rate plane fit.
        if len(pts) > self.max_pts:
            sel = np.random.default_rng().integers(0, len(pts), self.max_pts)
            sample = pts[sel]
        else:
            sample = pts

        fit = fit_plane_ransac(sample, self.ransac_iter, self.plane_thresh)
        if fit is None:
            self._cmd.publish(Twist())
            return
        nrm, d, _ = fit

        # Orient the normal "up": most returns sit ABOVE the floor, so flip the
        # normal until the bulk of the cloud is on its positive side.
        signed = sample @ nrm + d
        if signed.mean() < 0:
            nrm, d = -nrm, -d

        # Level: rotate so the floor normal is +Z. Floor height -> z ≈ -d (the
        # plane sits at signed-distance 0; leveled z = n·x + d).
        R = level_rotation(nrm)
        lev = sample @ R.T
        height = sample @ nrm + d              # height above floor, per point (m)

        # Segment: obstacles stand above the floor within the dog's height band.
        obs_mask = (height > self.obs_min) & (height < self.obs_max)
        obs = lev[obs_mask]
        if self._obs_pub is not None:
            self._obs_pub.publish(self._to_cloud(obs))

        # Horizontal geometry in the leveled frame: x forward, y left.
        cmd = self._steer(obs)
        self._cmd.publish(cmd)

    def _steer(self, obs: np.ndarray) -> Twist:
        front = math.radians(self.front_deg)
        steer = math.radians(self.steer_deg)
        nbins = 13
        bin_w = (2.0 * steer) / nbins
        bin_min = np.full(nbins, self.range_max)
        d_front = self.range_max
        left_n = right_n = 0
        left_sum = right_sum = 0.0

        if len(obs):
            ox, oy = obs[:, 0], obs[:, 1]
            rng = np.hypot(ox, oy)
            ang = np.arctan2(oy, ox)
            keep = (ang >= -steer) & (ang <= steer) & (rng >= 0.1) & (rng <= self.range_max)
            ang, rng = ang[keep], rng[keep]
            if len(rng):
                d_front = self._front_min(ang, rng, front)
                b = np.clip(((ang + steer) / bin_w).astype(int), 0, nbins - 1)
                for bi, ri in zip(b, rng):
                    if ri < bin_min[bi]:
                        bin_min[bi] = ri
                left = ang >= 0
                left_sum, left_n = float(rng[left].sum()), int(left.sum())
                right_sum, right_n = float(rng[~left].sum()), int((~left).sum())

        # Publish sector means for the UI (open sectors read range_max).
        f = d_front
        l = left_sum / left_n if left_n else self.range_max
        r = right_sum / right_n if right_n else self.range_max
        self._dbg.publish(String(data=f"{f:.2f},{l:.2f},{r:.2f}"))

        cmd = Twist()
        if d_front < self.stop_dist:
            # Boxed in: rotate toward the side with more clearance.
            cmd.angular.z = self.max_wz if l >= r else -self.max_wz
        else:
            best = int(np.argmax(bin_min))
            target = -steer + (best + 0.5) * bin_w
            wz = _clamp(self.k_steer * target, -self.max_wz, self.max_wz)
            scale = _clamp((d_front - self.stop_dist) / max(1e-3, self.slow_dist - self.stop_dist), 0.0, 1.0)
            cmd.linear.x = self.max_vx * scale * max(0.2, 1.0 - abs(wz) / self.max_wz)
            cmd.angular.z = wz
        return cmd

    @staticmethod
    def _front_min(ang, rng, front):
        m = (ang >= -front) & (ang <= front)
        return float(rng[m].min()) if m.any() else float("inf")

    def _to_cloud(self, pts: np.ndarray) -> PointCloud2:
        """Pack leveled obstacle points into a base_link-stamped PointCloud2."""
        msg = PointCloud2()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(("x", "y", "z"))]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.astype(np.float32).tobytes()
        return msg

    def stop(self):
        for _ in range(3):
            self._cmd.publish(Twist())


def main():
    rclpy.init()
    node = GroundNav()
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
