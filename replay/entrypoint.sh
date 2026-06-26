#!/bin/bash
# Source ROS + the built Unitree messages, bind CycloneDDS to the interface that
# reaches the Go2, then launch the replay web server (which spawns/kills the
# replay node on demand). Same NIC-binding as the wander/nav2/recorder services —
# self-contained, no nav2 stack.
set -e

source /opt/ros/humble/setup.bash
[ -f /unitree_ws/install/setup.bash ] && source /unitree_ws/install/setup.bash

# Declared as a WendyOS `frameworks.ros2` service: the agent injects ROS_DOMAIN_ID
# (0) + RMW, but also ROS_LOCALHOST_ONLY=1 and an interface-less CYCLONEDDS_URI
# (Wendy graphs are loopback-only). The Go2 is a PHYSICAL robot on the LAN, so
# undo both and re-bind to our NIC below.
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

GO2_IP="${GO2_IP:-192.168.123.161}"

# Find the local IP / interface that routes to the Go2 (internal LAN, eth0 @
# 192.168.123.x typically).
read IFNAME LOCALIP <<EOF
$(python3 - "$GO2_IP" <<'PY'
import socket, subprocess, sys
ip = sys.argv[1]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
local = ""
try:
    s.connect((ip, 1))          # no packets sent; resolves egress address
    local = s.getsockname()[0]
except Exception:
    pass
finally:
    s.close()
name = ""
try:
    out = subprocess.check_output(["ip", "-o", "-4", "addr", "show"]).decode()
    for line in out.splitlines():
        p = line.split()
        if local and len(p) >= 4 and p[3].split("/")[0] == local:
            name = p[1]
            break
except Exception:
    pass
print(name, local)
PY
)
EOF

echo "[replay] Go2=${GO2_IP}  iface=${IFNAME:-auto}  local=${LOCALIP:-auto}  domain=${ROS_DOMAIN_ID}"

# Bind CycloneDDS to that interface so discovery finds the robot's topics.
URI=/tmp/cyclonedds.xml
IFXML=""
[ -n "$LOCALIP" ] && IFXML="<NetworkInterface address=\"$LOCALIP\"/>"
cat > "$URI" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>${IFXML}</Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <EnableMulticastLoopback>true</EnableMulticastLoopback>
    </General>
  </Domain>
</CycloneDDS>
XML
export CYCLONEDDS_URI="file://$URI"

exec python3 /server.py
