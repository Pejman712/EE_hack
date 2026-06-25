#!/usr/bin/env python3
"""
Real Unitree Go2 Nav2 bringup -- NO Gazebo, NO namespace.

Brings up the same Nav2 stack used in simulation, but pointed at the *real* robot's
topics (`/scan`, `/odom`, `/tf`) and frames, and starts the cmd_vel -> Unitree sport
API bridge (`/api/sport/request`). Everything upstream of `cmd_vel` (costmaps,
planner, controller, AMCL) is identical to the sim -- only the sensor topics, the
absence of a namespace, `use_sim_time:=false`, and the control bridge differ.

Prerequisites
-------------
* Official Unitree ROS2 SDK sourced (provides `unitree_api` / `unitree_go` msgs):
  https://github.com/unitreerobotics/unitree_ros2
* The robot publishing `/scan` (sensor_msgs/LaserScan), `/odom` (nav_msgs/Odometry),
  and `/tf` with frames matching `nav2_params_real.yaml`
  (`base_link` / `odom` / `map`).
* A map of YOUR environment (the cafe map here is a placeholder -- SLAM-map your space
  and pass it via `map:=/path/to/your_map.yaml`).

Examples
--------
    # source BOTH this workspace and the Unitree SDK first
    ros2 launch gazebo_sim nav2_real.launch.py map:=/path/to/your_map.yaml

    # point at a different odom/scan topic if needed
    ros2 launch gazebo_sim nav2_real.launch.py \
        map:=/path/to/map.yaml odom_topic:=/utlidar/robot_odom scan_topic:=/scan
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('gazebo_sim')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    start_bridge = LaunchConfiguration('start_bridge')

    declare_args = [
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg, 'maps', 'cafe_world_map.yaml'),
            description='Map YAML for YOUR environment (placeholder = sim cafe map).'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params_real.yaml'),
            description='Nav2 params (de-namespaced real-robot variant).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Real robot uses the wall clock -> false.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Auto-activate the Nav2 lifecycle nodes.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description="Robot's LaserScan topic."),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description="Robot's odometry topic (e.g. /odom or /utlidar/robot_odom)."),
        DeclareLaunchArgument(
            'start_bridge', default_value='true',
            description='Start the cmd_vel -> /api/sport/request bridge.'),
    ]

    # Substitute map path, sim-time and topics into the params at launch.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',  # no namespace on the real robot
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
            'odom_topic': odom_topic,
            'scan_topic': scan_topic,
        },
        convert_types=True)

    # Real robot: no namespace, so scan/odom sit at the root. Remap lets you override
    # without editing params; default values are no-ops.
    remappings = [('/scan', scan_topic), ('/odom', odom_topic)]

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = ['controller_server', 'planner_server',
                        'behavior_server', 'smoother_server', 'bt_navigator']

    common = dict(parameters=[configured_params], remappings=remappings, output='screen')

    nodes = [
        # ---- Localization ----
        Node(package='nav2_map_server', executable='map_server', name='map_server', **common),
        Node(package='nav2_amcl', executable='amcl', name='amcl', **common),

        # ---- Navigation ----
        Node(package='nav2_controller', executable='controller_server', name='controller_server', **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', **common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', **common),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', **common),

        # ---- Lifecycle managers (ordered: map_server activates before amcl) ----
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': localization_nodes}]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': navigation_nodes}]),
    ]

    # ---- cmd_vel -> Unitree sport API bridge (real target) ----
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('quadropted_controller'),
                         'launch', 'cmd_vel_bridge.launch.py')),
        condition=IfCondition(start_bridge),
        launch_arguments={'sim': 'false', 'namespace': ''}.items(),
    )

    return LaunchDescription(declare_args + nodes + [bridge])
