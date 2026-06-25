#!/usr/bin/env bash
# Source ROS 2 + the unitree message workspace, then exec whatever command the
# container was given (default: stay up so you can `ros2 topic echo|pub`).
set -e
source /opt/ros/humble/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash

# WendyOS `frameworks.ros2` (wendy.json) injects ROS_DOMAIN_ID + RMW_IMPLEMENTATION
# for us — keep those. But it ALSO injects ROS_LOCALHOST_ONLY=1 and an
# interface-less CYCLONEDDS_URI, because Wendy's ROS 2 graphs are intra-host
# (loopback only). The Go2 is a PHYSICAL robot on the LAN, so we must undo both:
# unconditionally re-point CycloneDDS at our NIC-bound config and re-enable
# off-loopback DDS, overriding the agent-injected OCI env at PID 1.
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///app/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"   # framework sets this (domainId: 0)

exec "$@"
