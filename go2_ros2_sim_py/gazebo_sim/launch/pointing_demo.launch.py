#!/usr/bin/env python3
"""
Pointing pipeline for the simulation: external camera -> skeleton -> Nav2 goal.

Runs the three pieces that turn a person's pointing gesture into a Nav2 goal, on
top of an already-running sim (`ros2 launch gazebo_sim launch.py`):

  1. webcam        Logitech (v4l2) -> /<ns>/color/image_raw   (the robot's camera)
  2. Erkka pose    /<ns>/color/image_raw -> /pointed_location  (MediaPipe skeleton;
                     external script, needs mediapipe -- path via `pose_node`)
  3. pointed_goal  /pointed_location -> /<ns>/navigate_to_pose (camera-optical point
                     -> base_link -> map via TF, sent as a NavigateToPose goal)

Usage
-----
  # 1) sim + Nav2 already up:    ros2 launch gazebo_sim launch.py
  # 2) put the dog in TROT:      ros2 topic pub --once /robot1/robot_mode \
  #        quadropted_msgs/msg/RobotModeCommand "{mode: 'TROT', robot_id: 1}"
  # 3) this pipeline:
  ros2 launch gazebo_sim pointing_demo.launch.py

  # override the Erkka script location / camera / arm:
  ros2 launch gazebo_sim pointing_demo.launch.py \
        pose_node:=/home/daino/EE_hack/Erkka/ros2_pose_node.py \
        video_device:=/dev/video3 arm:=right

Then a person raises BOTH arms ~3 s (arming), then points at the ground -> the dog
walks to that point. (Goal in the `map` frame; use goal_frame:=odom for a mapless stack.)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gazebo_sim')

    namespace = LaunchConfiguration('namespace')
    video_device = LaunchConfiguration('video_device')
    image_topic = LaunchConfiguration('image_topic')
    pose_node = LaunchConfiguration('pose_node')
    goal_frame = LaunchConfiguration('goal_frame')
    arm = LaunchConfiguration('arm')

    args = [
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('video_device', default_value='/dev/video3',
                              description='Logitech webcam device'),
        DeclareLaunchArgument('image_topic', default_value='/robot1/color/image_raw',
                              description='Camera topic the pose node reads'),
        DeclareLaunchArgument(
            'pose_node',
            default_value='/home/daino/EE_hack/Erkka/ros2_pose_node.py',
            description='Path to the Erkka MediaPipe pose script (needs mediapipe)'),
        DeclareLaunchArgument('goal_frame', default_value='map',
                              description='map (sim) | odom (mapless)'),
        DeclareLaunchArgument('arm', default_value='either',
                              description='which arm to follow: right|left|either'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Open a live view of the skeleton overlay'),
        DeclareLaunchArgument('use_compressed', default_value='false',
                              description='Pose node reads the COMPRESSED camera stream '
                                          '(higher FPS over a network / real robot)'),
    ]
    viz = LaunchConfiguration('viz')
    use_compressed = LaunchConfiguration('use_compressed')

    # 1. webcam -> /<ns>/color/image_raw  (reuses webcam.launch.py)
    webcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'webcam.launch.py')),
        launch_arguments={'namespace': namespace, 'video_device': video_device}.items(),
    )

    # 2. Erkka MediaPipe pose node -> /pointed_location
    pose = ExecuteProcess(
        cmd=['python3', pose_node, '--ros-args',
             '-p', ['image_topic:=', image_topic],
             '-p', ['use_compressed:=', use_compressed]],
        output='screen',
    )

    # 3. pointing -> Nav2 goal bridge (under the robot namespace)
    bridge = Node(
        package='quadropted_controller',
        executable='pointed_goal.py',
        name='pointed_goal',
        namespace=namespace,
        output='screen',
        parameters=[{'goal_frame': goal_frame, 'arm': arm}],
    )

    # 4. Live visualization of the skeleton overlay (camera + detection + arms-up +
    #    pointing all visible in one window). Confirms the whole input side at a glance.
    viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='pointing_viewer',
        arguments=['/annotated_image'],
        condition=IfCondition(viz),
        output='screen',
    )

    return LaunchDescription(args + [webcam, pose, bridge, viewer])
