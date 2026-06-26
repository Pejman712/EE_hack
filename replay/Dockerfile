# go2-replay — re-drive a RECORDED Go2 run by replaying its sport command stream.
#
# Sibling of go2-wander, minus the sensing pipeline: replay publishes the dog's
# sport Move API directly, so there is NO pointcloud_to_laserscan and NO
# cmd_vel_to_sport bridge. Just:
#   replay.py  (recorded /api/sport/request stream -> dog, original timing)
#   server.py  (tiny FastAPI Start/Stop page)
# entrypoint.sh binds CycloneDDS to the NIC that reaches the Go2 (GO2_IP).
#
# arm64 (the Go2's Orin). Build via `wendy run`, or `--platform linux/arm64`.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# DDS RMW + the message-codegen tools the Unitree packages need to build. NO
# navigation2, NO pointcloud-to-laserscan — replay needs neither.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-generator-dds-idl \
        python3-pip git iproute2 \
 && rm -rf /var/lib/apt/lists/*

# Unitree message packages — replay.py builds & publishes unitree_api/msg/Request,
# which needs the type available. Same copy-and-colcon build as the other images.
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

# FastAPI server + the self-describing-mcap reader replay.py uses to pull the
# recorded command stream (api_id + parameter) out of the bag.
RUN pip3 install --no-cache-dir \
        fastapi==0.115.6 uvicorn==0.34.0 pydantic==2.10.4 \
        mcap==1.2.1 mcap-ros2-support==0.5.5

COPY entrypoint.sh /entrypoint.sh
COPY server.py /server.py
COPY replay.py /replay.py
COPY run.mcap /run.mcap
RUN chmod +x /entrypoint.sh

# GO2_IP: any address on the robot's internal DDS LAN — picks the NIC to bind
# CycloneDDS to. ROS_DOMAIN_ID 0 is the Go2 default.
#   REPLAY_SPEED / REPLAY_START_DELAY / REPLAY_TOPICS_EXTRA  optional replay tuning.
#   BAG_PATH overrides the baked-in /run.mcap (e.g. a mounted volume).
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    GO2_IP=192.168.123.161 \
    BAG_PATH=/run.mcap \
    PORT=7100

EXPOSE 7100
CMD ["/entrypoint.sh"]
