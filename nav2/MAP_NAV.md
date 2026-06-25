# Map navigation web UI for the real Go2 (Wendy ROS2 service)

Drive the **real** Go2 around a saved map from a web page: load your map
(`yaml` + `pgm`), **set the initial pose** so AMCL knows where the dog is, then
**set a goal** — by clicking on the map or typing `x/y/yaw` — and watch the dog's
live pose move on the map as it walks there.

This is the map-based counterpart to the mapless "walk N metres" walker. It runs
the same Nav2 stack the Gazebo sim (`go2_ros2_sim_py`) demonstrates, but on real
hardware: the robot's onboard sport mode does the walking (we bridge
`/cmd_vel` → `/api/sport/request`), and odometry comes from the **L1 utlidar**.

```
 browser (map UI)                       nav2/server.py  (FastAPI :7100, ros2 CLI only)
   ├─ click/drag or type  ──POST /api/nav/initialpose──▶ ros2 topic pub /initialpose ─▶ AMCL relocalise
   ├─ click/drag or type  ──POST /api/nav/goal────────▶ ros2 action send_goal /navigate_to_pose (map frame)
   ├─ ■ Stop              ──POST /api/nav/stop────────▶ cancel goal + StopMove
   └─ live robot marker   ◀─GET  /api/nav/status──────  tf2_echo map base_link

   server.py also spawns:  ros2 launch nav2_mapped.launch.py map:=$MAP_YAML odom_source:=$ODOM_SOURCE
     go2_odom (utlidar → /odom + TF odom→base_link) · static base→lidar TF ·
     pointcloud_to_laserscan (/utlidar/cloud_deskewed → /scan) · map_server · amcl ·
     nav2 servers · cmd_vel_to_sport
```

## What changed for "real robot"

- **Odometry = utlidar.** `go2_odom.py` now defaults to `odom_source=utlidar`,
  republishing `/utlidar/robot_odom` (`nav_msgs/Odometry`) as `/odom` + the
  `odom→base_link` TF. `odom_source=sportmodestate` is the fallback if utlidar
  odom isn't published on your firmware.
- **Web map UI.** `server.py` launches `nav2_mapped.launch.py` (not the mapless
  one) and serves the map + the initial/goal/live-pose controls.
- **Configurable map.** `MAP_YAML` (env) points at any `yaml`+`pgm`; the page
  renders it and converts clicks ↔ map metres using the yaml's `resolution` +
  `origin`.

## Deploy as a Wendy service

The `nav2` service is already declared in `../wendy.json` (`frameworks.ros2`,
`domainId: 0`, host network). Build + run it on the dog's compute:

```bash
# from the repo root, with a device discovered (wendy discover --json)
wendy run --yes --detach --device <go2-hostname>

# point it at your own map instead of the bundled placeholder:
#   edit ../wendy.json → services.nav2 → add an env, OR rebuild with a map under
#   /maps, OR mount a volume and set MAP_YAML=/data/maps/<your>.yaml
```

Open the UI at **`http://<go2-hostname>:7100/`** (the service binds `0.0.0.0:7100`,
host network).

> Code-only: nothing here moves the robot until *you* press **Send goal** in the
> UI. Make sure the Go2 is standing (`BalanceStand`) first — sport mode ignores
> Move commands when sitting.

## Using the UI

1. **Set the initial pose** (green). The robot must be localised before goals mean
   anything. Switch to *Initial pose* mode and click where the dog actually is on
   the map, dragging to set its heading — or type `x/y/yaw` and press *Set initial
   pose*. This publishes `/initialpose`; AMCL snaps the laser onto the map.
2. **Check the scan lines up** with the map walls (live robot marker = amber). If
   it's off, set the initial pose again.
3. **Set a goal** (teal). Switch to *Goal* mode, click+drag the destination, or
   type `x/y/yaw`, then *Send goal ▸*. The dog plans + walks there in absolute map
   coordinates. **■ Stop** cancels and halts immediately.

## Configuration (env on the `nav2` service)

| Env | Default | Meaning |
|-----|---------|---------|
| `MAP_YAML` | `/maps/map.yaml` | map the UI renders + AMCL localises against (`pgm` resolved beside it) |
| `ODOM_SOURCE` | `utlidar` | `utlidar` (L1 LiDAR-inertial) or `sportmodestate` |
| `GO2_IP` | `192.168.123.161` | any address on the robot's DDS LAN — picks the NIC to bind CycloneDDS |
| `PORT` | `7100` | web server port |

## Verify on the live robot

```bash
# odom source actually present? (expect nav_msgs/msg/Odometry)
wendy device ros2 exec --device <go2> -- topic type /utlidar/robot_odom
wendy device ros2 hz   --device <go2> /utlidar/robot_odom
# our republished odom + TF chain:
wendy device ros2 echo --device <go2> /odom
wendy device ros2 nodes --device <go2>            # go2_odom, amcl, *_server, bt_navigator
# map served + AMCL active:
wendy device ros2 exec --device <go2> -- lifecycle get /amcl     # 'active'
```

If `/utlidar/robot_odom` is missing or a non-`Odometry` type, set
`ODOM_SOURCE=sportmodestate` and redeploy.

## Local dev (no Wendy, sourced Humble + Unitree SDK)

```bash
cd nav2
MAP_YAML=$PWD/maps/map.yaml ODOM_SOURCE=utlidar python3 server.py
# open http://localhost:7100/
```

See `REAL_WORLD_POINTING.md` for the camera-pointing flow and the per-link Nav2
debug checklist (scan/costmap/cmd_vel/sport) — all of it applies here too.
