#!/usr/bin/env python3
"""
SLAM mapping for the real Unitree Go2 -- builds a map of YOUR environment.

Runs slam_toolbox (online async) against the robot's `/scan`, using the Go2's
`odom -> base_link` TF, and publishes `map -> odom` so you get a live map. Drive
the robot around (teleop or the sport remote) to cover the space, then SAVE the map
and feed it to `nav2_real.launch.py` via `map:=`.

Prerequisites
-------------
* Robot publishing `/scan` (sensor_msgs/LaserScan) and `/tf` with `odom`/`base_link`.
* `ros-jazzy-slam-toolbox` installed (it is, on this machine).
* NO Gazebo, NO namespace (real robot).

Usage
-----
    source /home/daino/colcon_ws/install/local_setup.bash
    ros2 launch gazebo_sim slam_real.launch.py

    # ... drive the robot around to cover the area, watching the map grow in RViz ...

    # then SAVE the map (run in another sourced terminal):
    ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/my_area --ros-args -p use_sim_time:=false

    # finally navigate with the saved map:
    ros2 launch gazebo_sim nav2_real.launch.py map:=~/go2_maps/my_area.yaml

Open RViz separately (Fixed Frame = map, add Map + LaserScan displays) to watch it build.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gazebo_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Real robot uses the wall clock -> false.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'slam_real.yaml'),
            description='slam_toolbox parameters.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description="Robot's LaserScan topic."),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('/scan', scan_topic)],
        ),
    ])
