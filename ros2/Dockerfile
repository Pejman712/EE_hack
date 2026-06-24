# go2-foxglove — ros2 (Go2 native DDS ⇄ ROS 2 via unitree_ros2).
#
# Replaces the old HTTP `control` service with the official unitree_ros2 layer.
# unitree_ros2 ships the ROS 2 message packages (unitree_go, unitree_api) whose
# IDL matches the Go2's native DDS types, plus a CycloneDDS RMW. With the RMW
# bound to the robot LAN, the Go2's native `rt/...` topics show up as ROS 2
# topics carrying those types:
#   rt/lowstate            -> /lowstate           (unitree_go/msg/LowState)
#   rt/sportmodestate      -> /sportmodestate     (unitree_go/msg/SportModeState)
#   rt/utlidar/cloud_*     -> /utlidar/cloud_*     (sensor_msgs/PointCloud2)
#   rt/api/sport/request   -> /api/sport/request  (unitree_api/msg/Request)
# so you drive/read the robot with `ros2 topic pub|echo` and ROS 2 nodes instead
# of the curl HTTP API the old control service exposed.
#
# Humble path: apt's ros-humble-rmw-cyclonedds-cpp already bundles CycloneDDS
# 0.10.x (the robot runs 0.10.2), so we skip unitree_ros2's from-source
# CycloneDDS build (that's the Foxy-only step) and just colcon-build the message
# packages under cyclonedds_ws.
#
# Build for arm64 (the Go2's Orin): `--platform linux/arm64` from an x86 host.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        python3-colcon-common-extensions \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
        libyaml-cpp-dev \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
ARG UNITREE_ROS2_REF=master
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git \
    && git -C unitree_ros2 checkout "${UNITREE_ROS2_REF}"

# Build the Unitree message packages (unitree_go, unitree_api, …) that live under
# cyclonedds_ws/unitree. We use apt's rmw_cyclonedds, so cyclonedds_ws/src stays
# empty and `colcon build` just generates the message typesupport.
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/unitree_ros2/cyclonedds_ws \
    && colcon build

WORKDIR /app
COPY cyclonedds.xml ros_entrypoint.sh ./
RUN chmod +x /app/ros_entrypoint.sh

# Pin the DDS NIC by address (the Go2 Orin is multi-homed) and route the RMW
# through CycloneDDS — same binding the bridge/control services use.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///app/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/app/ros_entrypoint.sh"]
# Stay up as a ROS 2 participant on the robot LAN; attach to run ros2 commands.
CMD ["bash", "-c", "ros2 topic list 2>/dev/null || true; echo 'DIAG ros2 up — unitree topics listed above; attach and run ros2 commands'; exec sleep infinity"]
