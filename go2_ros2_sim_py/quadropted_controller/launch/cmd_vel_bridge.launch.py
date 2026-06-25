#!/usr/bin/env python3
"""
Portable cmd_vel bridge selector.

Picks the correct velocity bridge for the target while keeping everything upstream
of ``cmd_vel`` (Nav2, costmaps, planner, controller) identical:

    sim:=true  (default) -> cmd_vel_pub.py       (Gazebo IK controller, via robot_velocity)
    sim:=false           -> cmd_vel_to_sport.py  (real Go2, via /api/sport/request)

Examples
--------
Simulation (namespaced like the rest of the sim)::

    ros2 launch quadropted_controller cmd_vel_bridge.launch.py sim:=true namespace:=robot1

Real robot (no namespace -- matches the Go2's /cmd_vel and /api/sport/request)::

    ros2 launch quadropted_controller cmd_vel_bridge.launch.py sim:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='true: Gazebo bridge (cmd_vel_pub.py); '
                        'false: real Go2 sport-API bridge (cmd_vel_to_sport.py)'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace for the bridge node. Sim uses e.g. "robot1"; '
                        'the real robot uses an empty namespace.'),

        # --- Simulation target: cmd_vel -> robot_velocity -> IK controller ---
        Node(
            condition=IfCondition(sim),
            package='quadropted_controller',
            executable='cmd_vel_pub.py',
            name='cmd_vel_pub',
            namespace=namespace,
            output='screen',
        ),

        # --- Real robot target: cmd_vel -> /api/sport/request (Unitree sport mode) ---
        Node(
            condition=UnlessCondition(sim),
            package='quadropted_controller',
            executable='cmd_vel_to_sport.py',
            name='cmd_vel_to_sport',
            namespace=namespace,
            output='screen',
        ),
    ])
