"""geometry_msgs/PoseStamped IDL for direct CycloneDDS (no ROS 2).

Matches the ROS 2 sensor_msgs schema so we can subscribe to the Go2's
rt/utlidar/robot_pose — the onboard SLAM's drift-corrected pose, as opposed to
the dead-reckoning position in rt/sportmodestate that `app.py` already uses
for /go2/pose and /tf.

Do NOT add `from __future__ import annotations` — IdlStruct resolves the type
hints by name at class-definition time and PEP-563 breaks it.
"""
from dataclasses import dataclass, field

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import float64

from pointcloud2 import _Header  # shared std_msgs/Header + builtin Time


@dataclass
class _Point(IdlStruct, typename="geometry_msgs::msg::dds_::Point_"):
    x: float64 = 0.0
    y: float64 = 0.0
    z: float64 = 0.0


@dataclass
class _Quaternion(IdlStruct, typename="geometry_msgs::msg::dds_::Quaternion_"):
    x: float64 = 0.0
    y: float64 = 0.0
    z: float64 = 0.0
    w: float64 = 1.0


@dataclass
class _Pose(IdlStruct, typename="geometry_msgs::msg::dds_::Pose_"):
    position: _Point = field(default_factory=_Point)
    orientation: _Quaternion = field(default_factory=_Quaternion)


@dataclass
class PoseStamped_(IdlStruct, typename="geometry_msgs::msg::dds_::PoseStamped_"):
    header: _Header = field(default_factory=_Header)
    pose: _Pose = field(default_factory=_Pose)
