# go2-foxglove

Stream a **Unitree Go2**'s live data into **Foxglove** over a single WebSocket —
LiDAR point cloud, pose + TF, body state (IMU / battery / foot forces) and UWB,
plus the front camera — and drive it with the Unitree Python SDK's Sport API.

```
Go2 controller ──DDS──┐
  192.168.123.161      │   ┌──────────────────────────────┐
                       ├──▶│ bridge  (DDS → Foxglove WS)    │──▶ ws://<device>:8765 ──▶ Foxglove
  Jetson .123.18  ─────┤   │   /go2/points /go2/pose /tf    │
                       │   │   /go2/state /go2/uwb          │
                       │   └──────────────────────────────┘
                       │   ┌──────────────────────────────┐
                       ├──▶│ ros2 (unitree_ros2 ⇄ DDS)      │◀─▶ /api/sport/request
                       │   │   /lowstate /sportmodestate    │
                       │   └──────────────────────────────┘
        front cam ──WebRTC─▶│ camera ──localhost JPEG──▶ /go2/camera
                           └──────────────────────────────┘
```

**Independent containers, one robot LAN.** The `camera` service does the heavy
WebRTC decode in isolation and forwards JPEG frames to the `bridge` over localhost,
so the camera appears on the *same* Foxglove connection — but if WebRTC fails, the
3D/LiDAR view stays up. The `ros2` service is a separate DDS participant that
exposes the Go2 to the ROS 2 ecosystem; the `bridge` keeps its own direct read
path to Foxglove, so the two don't depend on each other.

## ros2 (drive/read the robot over ROS 2)

The `ros2` service is the official
[`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) layer. It ships
the ROS 2 message packages (`unitree_go`, `unitree_api`) whose IDL matches the
Go2's native DDS types and runs a CycloneDDS RMW bound to the robot LAN, so the
robot's native `rt/...` topics appear as plain ROS 2 topics — drive and read it
with `ros2` commands and nodes instead of an HTTP API. (This replaces the old
`unitree_sdk2py`-backed `control` service; the curl endpoints no longer exist.)

It's declared as a WendyOS **`frameworks.ros2`** service (`wendy.json`), so the
agent injects `ROS_DOMAIN_ID=0` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
the `wendy device ros2` inspection CLI can target it:

```bash
wendy device ros2 topics                 # list the Go2's topics
wendy device ros2 echo /sportmodestate   # stream pose / velocity / foot state
wendy device ros2 nodes                  # list active nodes
```

You can also attach and run `ros2` directly (env pre-sourced):

```bash
wendy container attach <device> ros2     # shell into the service

ros2 topic list
ros2 topic echo /lowstate                # IMU / battery / motor state
```

> **LAN, not loopback.** `frameworks.ros2` is built for *intra-host* ROS 2 graphs:
> it injects `ROS_LOCALHOST_ONLY=1` and an interface-less `CYCLONEDDS_URI` that
> pin DDS to `lo`. The Go2 is a physical robot on the LAN, so `ros2/ros_entrypoint.sh`
> overrides both — re-enabling off-loopback DDS and re-pointing `CYCLONEDDS_URI`
> at the NIC-bound `ros2/cyclonedds.xml` — and the service runs with `network: host`.

**Driving** is publishing a `unitree_api/msg/Request` to `/api/sport/request` with
the high-level **sport API id** for the command (e.g. `Move`) and a JSON
`parameter`. Constructing those by hand is awkward, so for real driving use the
ready-made client nodes under `unitree_ros2/example` (build that workspace inside
the container, then `ros2 run …`) — its `ros2_sport_client` holds the
authoritative api-id table (`Move`, `StandUp`, `Damp`, `StopMove`, …) and wraps
the velocity-watchdog re-send loop the Go2 expects. The raw shape, for reference:

```bash
ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
  '{header: {identity: {api_id: <SPORT_API_ID>}}, parameter: "{\"x\":0.3,\"y\":0.0,\"z\":0.0}"}'
```

### SLAM command topics — highly unverified

The Go2's onboard SLAM (see **Mapping / SLAM** below) exposes two `std_msgs/String`
command topics the Unitree phone app drives — `/uslam/client_command` and
`/utlidar/mapping_cmd` (native `rt/uslam/client_command`, `rt/utlidar/mapping_cmd`).
Under ROS 2 they're a plain `ros2 topic pub`:

```bash
ros2 topic pub --once /uslam/client_command std_msgs/msg/String '{data: "start_mapping"}'
ros2 topic pub --once /utlidar/mapping_cmd  std_msgs/msg/String '{data: "save_map"}'
```

No public documentation for the accepted command strings was found — the values
above are guesses. Discover real values empirically (e.g. `ros2 topic echo` these
topics while driving the phone app's mapping UI) before relying on this.

Config:
- **ROS_DOMAIN_ID** — the Go2's native DDS domain (`0`), set via `frameworks.ros2.domainId`
  in `wendy.json`. Must be `0`; omitting it makes WendyOS derive a non-zero domain
  from the appId, which won't match the robot.
- **GO2_DDS_ADDRESS** — *this device's* IP on the robot LAN, set in `ros2/cyclonedds.xml`
  (same multi-homed-NIC caveat as `bridge`).

## sit_stand (a ROS 2 node that sits the robot down, then up)

A worked example of implementing a custom node against the Go2's ROS 2 graph,
following the WendyOS ROS 2 example's per-service layout. The `sit_stand` service
runs a `rclpy` node that publishes `unitree_api/msg/Request` to `/api/sport/request`
with the sport api ids `Sit` (1009) then, after `sit_seconds`, `RiseSit` (1010) —
the same ids the `unitree_ros2` example's `SportClient` uses. The routine is behind
a `std_srvs/Trigger` service so the container stays healthy and doesn't move the
robot on every boot:

```bash
wendy device ros2 call /sit_and_stand std_srvs/srv/Trigger   # sit, wait, stand back up
# or, attached to the container directly:
ros2 service call /sit_and_stand std_srvs/srv/Trigger
```

**Logging**: each request is tagged with a unique id and logged when sent; the
node also subscribes to `/api/sport/response` and logs the robot's reply
(`code == 0` = ok, nonzero = "investigate" — the meaning of nonzero codes per
api_id isn't confirmed against Unitree's docs). Watch it with:

```bash
wendy device logs --app sitstand --follow
```

**Built-in connectivity self-check** — runs continuously, no robot motion, no
manual `ros2 topic` commands needed: every 5s the node logs how many real
`/lowstate` messages it received from the robot (`0` means the DDS path to the
robot isn't up — check network/domain before suspecting the sport commands
themselves), and publishes+receives its own heartbeat on `/sit_stand/heartbeat`
to confirm this node's *own* publisher is reaching the DDS bus at all,
independent of whether the robot is reachable. Both show up in the same log
stream above.

Same WendyOS framework setup as `ros2` (`frameworks.ros2` domain `0`, `network: host`,
loopback-only injection overridden in `sit_stand/ros_entrypoint.sh`). Params:
`sit_seconds` (default `4.0`) and `auto_run_on_start` (default `false` — set `true`
to run the routine once a couple seconds after launch instead of on a service call).

Preconditions (same as any sport command): the robot must hold the sport lease and
be in normal sport mode (not AI/advanced mode, not damped), standing on a flat,
clear area before you trigger it. `Sit` from another posture may no-op.

## Mapping / SLAM (`/go2/map`, `/go2/slam_pose`) — unverified

The Go2 EDU+'s onboard Orin runs a SLAM stack continuously by default — no
command needed to "turn it on." `bridge` additionally subscribes to two more
native topics:

- `rt/uslam/cloud_map` → **`/go2/map`** (`sensor_msgs/PointCloud2`, same decode
  path as `/go2/points`) — the onboard SLAM's accumulated map, which should
  build up as the robot moves/turns rather than showing only the current frame.
- `rt/utlidar/robot_pose` → **`/go2/slam_pose`** (`geometry_msgs/PoseStamped`)
  — the SLAM's drift-corrected pose, as opposed to the dead-reckoning pose on
  `/go2/pose` (from `sportmodestate`).

Both are published as their own root frame in the 3D panel (not linked into the
existing `odom → base_link` tf tree — there's no verified `map → odom`
transform to publish alongside them), so switch the panel's **Display frame**
to see them. Override topic names with `MAP_TOPIC` / `SLAM_POSE_TOPIC` env vars
if they don't match your firmware.

## Deploy

```bash
wendy init --template go2-foxglove --language python --app-id go2viz
cd go2viz
wendy run --device <go2>.local
```

Variables (`wendy init` prompts, or pass `--var`):
- **GO2_IP** — the robot controller IP for the camera (default `192.168.123.161`).
- **GO2_DDS_ADDRESS** — *this device's* IP on the robot LAN (default `192.168.123.18`).
  See **Where does this run?** below.
- **FOXGLOVE_PORT** — the WebSocket port (default `8765`).

## View in Foxglove

1. Open Foxglove (desktop app or <https://app.foxglove.dev>).
2. **Open connection → Foxglove WebSocket** → `ws://<device>:8765`.
3. **Layout → Import from file…** → `foxglove-layout.json` (in this template) to get
   the 3D + camera + plots + UWB panels pre-arranged.

You should see the point cloud under the moving robot, the camera image, battery/IMU
and pose/foot-force plots, and the raw UWB message.

## Where does this run? (matters for GO2_DDS_ADDRESS)

DDS binds to **this machine's** IP on the robot LAN — set `GO2_DDS_ADDRESS` to it:
- **On the Go2's onboard Jetson:** usually `192.168.123.18` (the default).
- **On an external Jetson** bridged to the robot LAN: that machine's `192.168.123.x`.

Binding by **address** (not interface name) is deliberate — the Go2 Orin is
multi-homed (`eth1` carries two subnets), so a name is ambiguous and DDS can
advertise the wrong subnet.

## Notes / caveats (unverified on a live EDU+ — verify on the robot)

- **foxglove-sdk API**: the bridge uses the `foxglove-sdk` channel/schema classes;
  pin the version you validate (`bridge/requirements.txt`).
- **LiDAR**: assumes `rt/utlidar/cloud_deskewed` (override with `LIDAR_TOPIC`). The
  EDU+ ships the **Livox MID-360**; confirm that topic is published on your firmware.
- **Camera**: the Go2 allows **one** WebRTC client — if the Unitree phone app is
  connected, the camera can't connect until it disconnects.
- **arm64**: the Go2's Orin is arm64. Build with `--platform linux/arm64` if building
  the images from an x86 host.
- **Frames**: the 3D panel's *Display frame* is `base_link`; if the cloud or pose
  looks off, switch the display frame in the panel settings.
- **ros2 / sport lease**: high-level sport commands need the sport lease
  (`/api/sport_lease`) — if the Unitree phone app or another client holds it, your
  `Request` publishes to `/api/sport/request` will silently no-op or error. Make
  sure the robot is on a flat, clear area before driving; nothing in this service
  stops it from walking into something.
- **Mapping/SLAM topics** (`rt/uslam/cloud_map`, `rt/utlidar/robot_pose`,
  `rt/uslam/client_command`, `rt/utlidar/mapping_cmd`): names, types, and (for the
  two command topics) accepted string values are unconfirmed on a live EDU+ —
  found via web research, not verified against the robot. See **Mapping / SLAM**
  above.
