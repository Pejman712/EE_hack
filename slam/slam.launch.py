#!/usr/bin/env python3
"""slam.launch.py — nav2-style 2D SLAM for the Go2 with the Unitree L1 LiDAR.

Brings up the four pieces that turn the Go2's 3D point cloud into a live 2D
occupancy-grid map (`/map`) that nav2 can plan on:

  1. go2_odom           /sportmodestate            -> TF odom->base_link + /odom
  2. static TF          base_link -> <lidar frame> (L1 mounting pose)
  3. pointcloud_to_laserscan  /utlidar/cloud_deskewed -> /scan (2D slice)
  4. slam_toolbox (async/online)  /scan + TF       -> /map + TF map->odom

The robot's only LiDAR is the **bottom-mounted Unitree L1** — a forward/down-
biased dome-FOV sensor, NOT a 360° puck. Consequences for 2D SLAM, all honest
caveats to verify on the robot:
  * Coverage behind the robot is poor, so loop closure is weak — drive/rotate
    to keep walls in view, and expect more drift than a 360° lidar.
  * The L1 sits low and looks down: the 2D height slice (min/max_height in
    config/pointcloud_to_laserscan.yaml) must be tuned so it cuts walls, not
    the floor. Start by echoing the cloud and watching /scan in Foxglove.
  * For the L1, a 3D LiDAR-inertial odometry (point-lio / fast-lio) is arguably
    a better fit than 2D slam_toolbox — see README. This launch is the nav2
    (slam_toolbox) path because that's what plugs straight into nav2 costmaps.

Args (override with `name:=value`):
  lidar_topic   (default /utlidar/cloud_deskewed)  — L1 deskewed cloud
  lidar_frame   (default utlidar_lidar)            — the cloud's header frame_id;
                  read the real one: `ros2 topic echo --field header.frame_id <topic>`
  lidar_xyz / lidar_rpy  — base_link -> lidar_frame static transform (METERS /
                  RADIANS). Approximate L1 mount; measure on the robot. If the
                  deskewed cloud is already gravity-aligned, leave rpy at 0.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    lidar_topic = LaunchConfiguration("lidar_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")

    # Approximate L1 mount on the Go2: front of the body, low. MEASURE/TUNE.
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_roll = LaunchConfiguration("lidar_roll")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    p2l_params = os.path.join(HERE, "config", "pointcloud_to_laserscan.yaml")
    slam_params = os.path.join(HERE, "config", "slam_toolbox.yaml")

    args = [
        DeclareLaunchArgument("lidar_topic", default_value="/utlidar/cloud_deskewed"),
        DeclareLaunchArgument("lidar_frame", default_value="utlidar_lidar"),
        DeclareLaunchArgument("lidar_x", default_value="0.28"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="-0.05"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
    ]

    # 1. Odometry: /sportmodestate -> TF odom->base_link + /odom. Run as a plain
    #    script (no colcon package needed), matching this repo's lightweight nodes.
    go2_odom = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "go2_odom.py")],
        output="screen",
    )

    # 2. Static base_link -> lidar frame. tf2_ros named-arg form (Humble).
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar",
        arguments=[
            "--x",
            lidar_x,
            "--y",
            lidar_y,
            "--z",
            lidar_z,
            "--roll",
            lidar_roll,
            "--pitch",
            lidar_pitch,
            "--yaw",
            lidar_yaw,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            lidar_frame,
        ],
    )

    # 3. 3D cloud -> 2D scan. Slices a horizontal band into a LaserScan.
    p2l = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[p2l_params],
        remappings=[("cloud_in", lidar_topic), ("scan", "/scan")],
    )

    # 4. slam_toolbox online async: /scan + TF -> /map + TF map->odom.
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params, {"use_sim_time": False}],
        output="screen",
    )

    return LaunchDescription(args + [go2_odom, static_tf, p2l, slam])
