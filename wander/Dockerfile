# go2-wander — reactive "drive toward open space" for the Unitree Go2 L1 LiDAR.
#
# Deliberately NOT the nav2 image: no navigation2, no nav2-bringup, no map_server
# / AMCL / planner, no map rendering. Just the reactive pipeline:
#   pointcloud_to_laserscan (/utlidar/cloud_deskewed -> /scan)  ·  wander.py
#   (/scan -> /cmd_vel)  ·  cmd_vel_to_sport (/cmd_vel -> /api/sport/request)
# fronted by a tiny FastAPI Start/Stop page (server.py). entrypoint.sh binds
# CycloneDDS to the NIC that reaches the Go2 (GO2_IP).
#
# arm64 (the Go2's Orin). Build via `wendy run`, or `--platform linux/arm64`.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# DDS RMW, TF (static_transform_publisher + tf2_echo), and the pointcloud->scan
# node. NO navigation2 / nav2-bringup — wander needs none of it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-tf2-ros \
        ros-humble-pointcloud-to-laserscan \
        python3-pip git iproute2 \
 && rm -rf /var/lib/apt/lists/*

# Unitree message packages — cmd_vel_to_sport publishes unitree_api/msg/Request
# over the ros2 CLI, which needs the type available. Same copy-and-colcon build
# as the other images.
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

# FastAPI server deps (no Pillow/PyYAML — there is no map to render or parse).
# numpy: ground_nav.py does the 3D floor-plane RANSAC + leveling in numpy.
RUN pip3 install --no-cache-dir \
        fastapi==0.115.6 uvicorn==0.34.0 pydantic==2.10.4 numpy==1.26.4

COPY entrypoint.sh /entrypoint.sh
COPY server.py /server.py
COPY wander.py /wander.py
COPY ground_nav.py /ground_nav.py
COPY scan_debug.py /scan_debug.py
COPY cmd_vel_to_sport.py /cmd_vel_to_sport.py
COPY pipeline.launch.py /pipeline.launch.py
COPY pointcloud_to_laserscan.yaml /pointcloud_to_laserscan.yaml
RUN chmod +x /entrypoint.sh

# GO2_IP: any address on the robot's internal DDS LAN — picks the NIC to bind
# CycloneDDS to. ROS_DOMAIN_ID 0 is the Go2 default.
#   WANDER_MAX_VX / WANDER_MAX_WZ / WANDER_STOP_DIST  optional wander tuning.
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    GO2_IP=192.168.123.161 \
    PORT=7100

EXPOSE 7100
CMD ["/entrypoint.sh"]
