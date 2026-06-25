#!/usr/bin/env python3
"""nav2_mapped.launch.py — MAP-BASED nav2 stack for the Go2 (L1 LiDAR).

Same as nav2.launch.py but localizes against a saved map so you can send goals in
ABSOLUTE map coordinates (e.g. `goto_point.py frame:=map x:=.. y:=..`):

  + map_server   loads nav2/maps/map.yaml  -> /map (latched)
  + amcl         localizes /scan in the map -> publishes map -> odom TF
  global_costmap is static in the `map` frame (see config/nav2_params_mapped.yaml);
  local_costmap still rolls in `odom`.

Everything else (go2_odom, static base->lidar TF, pointcloud_to_laserscan, the nav2
servers, and the cmd_vel -> sport bridge) is identical to nav2.launch.py.

Localization note: amcl starts at the map origin (set_initial_pose). If the robot
does not actually start there, give it the real start pose in RViz (2D Pose Estimate)
or edit `initial_pose` in config/nav2_params_mapped.yaml.

Args (override with name:=value): same lidar_* args as nav2.launch.py, plus
  map          (default nav2/maps/map.yaml)  — the occupancy map to localize against.
  odom_source  (default utlidar)             — go2_odom source: utlidar | sportmodestate.
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
    map_yaml = LaunchConfiguration("map")
    odom_source = LaunchConfiguration("odom_source")

    p2l_params = os.path.join(HERE, "config", "pointcloud_to_laserscan.yaml")
    nav2_params = os.path.join(HERE, "config", "nav2_params_mapped.yaml")

    args = [
        DeclareLaunchArgument("lidar_topic", default_value="/utlidar/cloud_deskewed"),
        DeclareLaunchArgument("lidar_frame", default_value="utlidar_lidar"),
        DeclareLaunchArgument("lidar_x", default_value="0.28"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="-0.05"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        DeclareLaunchArgument("map", default_value=os.path.join(HERE, "maps", "map.yaml")),
        DeclareLaunchArgument("odom_source", default_value="utlidar"),
    ]

    # Odometry: utlidar (default) or sportmodestate -> TF odom->base_link + /odom
    go2_odom = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "go2_odom.py"),
             "--ros-args", "-p", ["odom_source:=", odom_source]],
        output="screen")

    # Static base_link -> lidar frame
    static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="base_to_lidar",
        arguments=[
            "--x", lidar_x, "--y", lidar_y, "--z", lidar_z,
            "--roll", lidar_roll, "--pitch", lidar_pitch, "--yaw", lidar_yaw,
            "--frame-id", "base_link", "--child-frame-id", lidar_frame,
        ])

    # 3D cloud -> 2D scan
    p2l = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", parameters=[p2l_params],
        remappings=[("cloud_in", lidar_topic), ("scan", "/scan")])

    # Map-based localization: map_server (loads the map) + amcl (map->odom)
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen", parameters=[nav2_params, {"yaml_filename": map_yaml}])
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl",
        output="screen", parameters=[nav2_params])

    # nav2 servers (global costmap is now static in the map frame)
    nav2_nodes = [
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[nav2_params]),
    ]

    # Lifecycle manager: map_server + amcl FIRST, then the nav servers.
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "map_server", "amcl",
                "controller_server", "planner_server",
                "behavior_server", "bt_navigator",
            ],
        }])

    # /cmd_vel -> sport Move bridge
    bridge = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "cmd_vel_to_sport.py")], output="screen")

    return LaunchDescription(
        args + [go2_odom, static_tf, p2l, map_server, amcl]
        + nav2_nodes + [lifecycle, bridge])
