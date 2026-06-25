#!/usr/bin/env python3
"""
Feed a real USB webcam into the simulation's camera topics.

Publishes a Logitech C920 (or any V4L2 webcam) onto the same topics the Gazebo
camera used (`/<namespace>/color/image_raw` + `/color/camera_info`), so any node
that consumed the sim camera now runs on real video. The Gazebo camera bridge is
disabled in gazebo_multi_nav2_world.launch.py to avoid two publishers on one topic.

NOTE: the feed shows whatever the webcam points at -- NOT the simulated world.
Useful for hardware-in-the-loop perception (detection / AprilTags / etc.), not for
visual navigation inside the sim. camera_info here is uncalibrated.

Usage
-----
    source /home/daino/colcon_ws/install/local_setup.bash
    ros2 launch gazebo_sim webcam.launch.py                      # defaults: C920 on /dev/video3, ns robot1
    ros2 launch gazebo_sim webcam.launch.py video_device:=/dev/video0
    ros2 launch gazebo_sim webcam.launch.py namespace:=robot1

To change resolution, edit IMAGE_SIZE below (v4l2_camera needs integer values).

Requires: ros-jazzy-v4l2-camera
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# v4l2_camera wants integers here; keep it a plain Python list (not a launch arg).
# 640x480 YUYV runs far faster than 720p (the C920's 720p YUYV caps ~10 fps and the
# RGB conversion is CPU-heavy). MediaPipe pose works fine at this resolution.
# (MJPG would give 30 fps but this v4l2_camera build can't output MJPG.)
IMAGE_SIZE = [640, 480]


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')
    namespace = LaunchConfiguration('namespace')

    # Run the node IN the "<ns>/color" namespace so its default 'image_raw' and
    # 'camera_info' (and ALL image_transport sub-topics: compressed/theora/zstd)
    # resolve to /<ns>/color/image_raw[/...] automatically. A plain topic remap
    # would only catch the base topic and leave the transports at /image_raw/*.
    cam_namespace = [namespace, '/color']
    camera_frame = PythonExpression(["'", namespace, "/camera_link'"])

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video3',
                              description='V4L2 device (C920 reported /dev/video3).'),
        DeclareLaunchArgument('namespace', default_value='robot1',
                              description='Robot namespace the sim camera used.'),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='webcam',
            namespace=cam_namespace,
            output='screen',
            parameters=[{
                'video_device': video_device,
                'image_size': IMAGE_SIZE,
                'camera_frame_id': camera_frame,
            }],
        ),
    ])
