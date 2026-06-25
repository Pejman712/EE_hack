# go2-foxglove — slam (nav2-style 2D SLAM from the Go2's Unitree L1 LiDAR).
#
# Same shape as the recorder image: ROS 2 Humble + CycloneDDS + the Unitree
# message packages (built into /unitree_ws), with entrypoint.sh binding DDS to
# the NIC that reaches the Go2 (GO2_IP) before launching. On top we add
# slam_toolbox, pointcloud_to_laserscan and the nav2 stack: the L1's 3D cloud is
# sliced to a 2D LaserScan and fed to slam_toolbox, which emits a
# nav_msgs/OccupancyGrid (/map) + the map->odom TF that nav2 plans on. See
# slam.launch.py for the pipeline and the L1-specific caveats.
#
# arm64 (the Go2's Orin). Build natively on Apple Silicon via `wendy run`, or
# `--platform linux/arm64` from an x86 host.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-tf2-ros \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-slam-toolbox \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        python3-pip git iproute2 \
 && rm -rf /var/lib/apt/lists/*

# Unitree message packages — go2_odom subscribes to unitree_go/msg/SportModeState.
# Identical copy-and-colcon-build as recorder/Dockerfile (robust to the repo's
# nested layout): grab anything that looks like a unitree message package and
# build just those (rmw_cyclonedds_cpp comes from apt above).
RUN git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2 /tmp/unitree_ros2
RUN mkdir -p /unitree_ws/src \
 && for d in $(find /tmp/unitree_ros2 -name package.xml -exec dirname {} \;); do \
        name=$(basename "$d"); \
        case "$name" in unitree_*|common) echo "  + $name"; cp -r "$d" /unitree_ws/src/ ;; esac; \
    done \
 && echo "packages to build:" && ls /unitree_ws/src
RUN . /opt/ros/humble/setup.sh \
 && cd /unitree_ws \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log /tmp/unitree_ros2

COPY entrypoint.sh /entrypoint.sh
COPY go2_odom.py /go2_odom.py
COPY slam.launch.py /slam.launch.py
COPY config /config
RUN chmod +x /entrypoint.sh

# GO2_IP: any address on the robot's internal DDS network — used only to pick the
# right NIC to bind CycloneDDS to (same as recorder). ROS_DOMAIN_ID 0 is the Go2
# default.
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    GO2_IP=192.168.123.161

CMD ["/entrypoint.sh"]
