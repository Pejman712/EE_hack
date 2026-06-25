# go2-foxglove — nav2 (mapless "walk N metres" for the Go2's Unitree L1 LiDAR).
#
# Same base as the slam image: ROS 2 Humble + CycloneDDS + the Unitree message
# packages (built into /unitree_ws), with entrypoint.sh binding DDS to the NIC
# that reaches the Go2 (GO2_IP). On top we install the FULL nav2 stack
# (navigation2 + nav2-bringup) plus pointcloud_to_laserscan, and run a small
# FastAPI server that launches nav2.launch.py and drives it over the ros2 CLI.
# See nav2.launch.py / server.py and the L1-specific caveats in the README.
#
# arm64 (the Go2's Orin). Build natively on Apple Silicon via `wendy run`, or
# `--platform linux/arm64` from an x86 host.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# nav2 itself (navigation2 + nav2-bringup) plus the DDS RMW, TF, and the
# pointcloud->laserscan node that feeds the costmaps' obstacle layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-tf2-ros \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        python3-pip git iproute2 \
 && rm -rf /var/lib/apt/lists/*

# Unitree message packages — go2_odom subscribes to unitree_go/msg/SportModeState
# and the cmd_vel bridge publishes unitree_api/msg/Request. Same copy-and-colcon
# build as the recorder/slam images (robust to the repo's nested layout).
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

RUN pip3 install --no-cache-dir fastapi==0.115.6 uvicorn==0.34.0 pydantic==2.10.4

COPY entrypoint.sh /entrypoint.sh
COPY server.py /server.py
COPY nav2.launch.py /nav2.launch.py
COPY cmd_vel_to_sport.py /cmd_vel_to_sport.py
COPY go2_odom.py /go2_odom.py
COPY config /config
RUN chmod +x /entrypoint.sh

# GO2_IP: any address on the robot's internal DDS network — used only to pick the
# right NIC to bind CycloneDDS to (same as recorder/slam). ROS_DOMAIN_ID 0 is the
# Go2 default.
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    GO2_IP=192.168.123.161 \
    PORT=7100

EXPOSE 7100
CMD ["/entrypoint.sh"]
