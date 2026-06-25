#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash

# Kill all background jobs on Ctrl-C or exit
trap 'kill $(jobs -p) 2>/dev/null; exit' INT TERM EXIT

ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 &

sleep 3
python3.10 "$SCRIPT_DIR/ros2_pose_node.py" &

sleep 3
DISPLAY="${DISPLAY:-:0}" rviz2 -d "$SCRIPT_DIR/pose.rviz"
