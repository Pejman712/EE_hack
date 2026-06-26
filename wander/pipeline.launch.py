#!/usr/bin/env python3
"""pipeline.launch.py — the minimal pipeline the reactive wander needs.

NO nav2, NO map, NO AMCL, NO odometry. Just the three things between the L1
LiDAR and the dog's legs:

  + static TF        base_link -> <lidar frame>   (so /scan has a frame)
  + pointcloud_to_laserscan   /utlidar/cloud -> /scan (2D slice, ground removed)
  + cmd_vel_to_sport          /cmd_vel -> /api/sport/request   (drive the dog)

The wander node itself is NOT started here — server.py spawns/kills it on the
UI's Start/Stop. This launch only stands up the sensing + actuation pipeline so
that wander has a /scan to read and a bridge to push /cmd_vel through.

We use the RAW /utlidar/cloud (not /utlidar/cloud_deskewed). The L1 is mounted
looking DOWN-forward, so most returns are the floor; the raw cloud is in the
tilted sensor frame, so the static base_link->lidar TF's PITCH must match the L1
mount or the floor lands inside the height band. Ground removal then happens in
pointcloud_to_laserscan.yaml via the min_height cut (raise it until the floor
disappears from /scan). See that file's header.

The Go2 L2's /utlidar/cloud comes out ROLLED 180° ABOUT X relative to base_link
(it's mounted flipped — Y and Z are negated). We undo that in the static TF with
lidar_roll = pi, so pointcloud_to_laserscan transforms the cloud into a correct,
gravity-aligned base_link (floor below, obstacles above, +Y to the left). Without
this the floor lands inside the height band as a fake wall AND left/right steering
is mirrored. Any remaining downward mount tilt goes on top via lidar_pitch.

Args mirror the lidar mounting used elsewhere (override name:=value):
  lidar_topic /utlidar/cloud · lidar_frame utlidar_lidar
  lidar_x 0.28 · lidar_y 0.0 · lidar_z -0.05 · lidar_yaw 0.0
  lidar_roll 3.14159  -> the L2's 180° X flip (leave at pi unless the cloud is upright)
  lidar_pitch 0.0     -> ADD the L2's downward tilt here if the floor still shows up.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    lidar_topic = LaunchConfiguration("lidar_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_roll = LaunchConfiguration("lidar_roll")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    p2l_params = os.path.join(HERE, "pointcloud_to_laserscan.yaml")

    args = [
        DeclareLaunchArgument("lidar_topic", default_value="/utlidar/cloud"),
        DeclareLaunchArgument("lidar_frame", default_value="utlidar_lidar"),
        DeclareLaunchArgument("lidar_x", default_value="0.28"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="-0.05"),
        DeclareLaunchArgument("lidar_roll", default_value="3.14159"),  # L2 cloud is 180° X-flipped
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
    ]

    # Static base_link -> lidar frame so pointcloud_to_laserscan (target_frame
    # base_link) can transform the cloud and /scan has a valid frame.
    static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_lidar",
        arguments=[
            "--x", lidar_x, "--y", lidar_y, "--z", lidar_z,
            "--roll", lidar_roll, "--pitch", lidar_pitch, "--yaw", lidar_yaw,
            "--frame-id", "base_link", "--child-frame-id", lidar_frame,
        ])

    # 3D cloud -> 2D scan that wander reads.
    p2l = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", parameters=[p2l_params],
        remappings=[("cloud_in", lidar_topic), ("scan", "/scan")])

    # /cmd_vel -> sport Move bridge (the dog has no /cmd_vel input).
    bridge = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "cmd_vel_to_sport.py")], output="screen")

    # Always-on sector debugger: /scan -> /wander/debug "front,left,right" averages
    # (the UI shows these; handy for tuning the ground filter even before wandering).
    scan_debug = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "scan_debug.py")], output="screen")

    return LaunchDescription(args + [static_tf, p2l, bridge, scan_debug])
