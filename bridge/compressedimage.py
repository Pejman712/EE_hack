"""sensor_msgs/CompressedImage IDL for direct CycloneDDS (no ROS 2).

Matches the ROS 2 sensor_msgs schema so the bridge can *publish* the JPEG frames
the camera service forwards as a real DDS topic that the unitree_ros2 layer (and
`ros2 topic echo` / image_transport / nav2 / slam containers on the same DDS
domain) sees as sensor_msgs/CompressedImage — the same trick pointcloud2.py /
posestamped.py use to read the Go2's native rt/... topics as ROS 2 types.

ROS 2 maps a topic `/foo` to the DDS topic `rt/foo`, so publishing on the DDS
topic `rt/go2/camera/compressed` here surfaces as the ROS 2 topic
`/go2/camera/compressed` (the conventional image_transport "compressed" name for
a base `/go2/camera`).

Do NOT add `from __future__ import annotations` — IdlStruct resolves the type
hints by name at class-definition time and PEP-563 breaks it.
"""
from dataclasses import dataclass, field

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence, uint8

from pointcloud2 import _Header  # shared std_msgs/Header + builtin Time


@dataclass
class CompressedImage_(IdlStruct, typename="sensor_msgs::msg::dds_::CompressedImage_"):
    header: _Header = field(default_factory=_Header)
    format: str = ""  # e.g. "jpeg" (image_transport expects "<encoding>; jpeg")
    data: sequence[uint8] = field(default_factory=list)
