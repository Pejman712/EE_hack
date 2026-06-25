# go2-foxglove — sit_stand (ROS 2 node: sit the Go2 down, then stand it up).
#
# A rclpy node that publishes unitree_api/msg/Request to /api/sport/request with
# the Sit (1009) / RiseSit (1010) sport api ids, exposed behind a std_srvs/Trigger
# service (sit_and_stand). Needs the same unitree_ros2 message packages the `ros2`
# service builds (unitree_api), so it repeats that message build on ros:humble.
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

# Build the Unitree message packages (unitree_go, unitree_api, …) the node imports.
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/unitree_ros2/cyclonedds_ws \
    && colcon build

WORKDIR /app
COPY cyclonedds.xml ros_entrypoint.sh sit_stand.py ./
RUN chmod +x /app/ros_entrypoint.sh

# Pin the DDS NIC by address (the Go2 Orin is multi-homed); the entrypoint
# re-asserts these over WendyOS's loopback-only frameworks.ros2 injection.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///app/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

# The entrypoint sources ROS 2 + fixes the DDS env, then launches the node
# (pass a command to override, e.g. `bash` for an interactive debug shell).
ENTRYPOINT ["/app/ros_entrypoint.sh"]
