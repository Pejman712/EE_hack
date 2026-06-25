#!/usr/bin/env python3
"""nav2.launch.py — mapless nav2 "walk N metres" stack for the Go2 (L1 LiDAR).

Brings up everything nav2 needs to take a goal N metres ahead and drive the dog
there, avoiding obstacles the bottom L1 sees — without any prior map or SLAM:

  1. go2_odom              /sportmodestate            -> TF odom->base_link + /odom
  2. static TF             base_link -> <lidar frame> (L1 mounting pose)
  3. pointcloud_to_laserscan  /utlidar/cloud_deskewed -> /scan (2D slice)
  4. nav2 servers          controller / planner / behaviors / bt_navigator
       + lifecycle_manager (autostart) — rolling costmaps in `odom` (see
       config/nav2_params.yaml); the controller publishes /cmd_vel.
  5. cmd_vel_to_sport      /cmd_vel (Twist) -> /api/sport/request Move (CLI-only
       bridge; the Go2 has no native /cmd_vel input).

The server (server.py) launches this whole file as a subprocess and then drives
it purely over the ros2 CLI (`ros2 action send_goal /navigate_to_pose …`).

The L1 is the bottom-mounted Unitree L1 — a forward/down-biased dome, not a 360°
puck — so obstacle coverage behind the robot is poor and the height-band slice in
config/pointcloud_to_laserscan.yaml must be tuned to cut walls, not the floor.
Watch /scan in Foxglove against the live cloud before trusting avoidance.

Args (override with `name:=value`):
  lidar_topic   (default /utlidar/cloud_deskewed)  — L1 deskewed cloud
  lidar_frame   (default utlidar_lidar)            — the cloud's header frame_id;
                  read the real one: `ros2 topic echo --field header.frame_id <topic>`
  lidar_x/y/z, lidar_roll/pitch/yaw  — base_link -> lidar static transform
                  (METRES / RADIANS). Approximate L1 mount; measure on the robot.
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

    p2l_params = os.path.join(HERE, "config", "pointcloud_to_laserscan.yaml")
    nav2_params = os.path.join(HERE, "config", "nav2_params.yaml")

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

    # 1. Odometry: /sportmodestate -> TF odom->base_link + /odom (plain script,
    #    same as the slam/ stack — nav2 needs this transform to plan/control).
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
            "--x", lidar_x, "--y", lidar_y, "--z", lidar_z,
            "--roll", lidar_roll, "--pitch", lidar_pitch, "--yaw", lidar_yaw,
            "--frame-id", "base_link", "--child-frame-id", lidar_frame,
        ],
    )

    # 3. 3D cloud -> 2D scan for the costmaps' obstacle layer.
    p2l = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[p2l_params],
        remappings=[("cloud_in", lidar_topic), ("scan", "/scan")],
    )

    # 4. nav2 servers + a lifecycle manager that configures/activates them on
    #    start (autostart). Minimal set for "drive to a pose" — no map_server,
    #    no AMCL (mapless); costmaps roll in `odom` per nav2_params.yaml.
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
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
    )

    # 5. /cmd_vel -> sport Move bridge (CLI-only; the Go2 takes no /cmd_vel).
    bridge = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "cmd_vel_to_sport.py")],
        output="screen",
    )

    return LaunchDescription(
        args + [go2_odom, static_tf, p2l] + nav2_nodes + [lifecycle, bridge]
    )
