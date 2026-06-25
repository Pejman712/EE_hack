# Go2 — running on the real robot

Bringing up Nav2 + the cmd_vel→Unitree sport-API bridge on a **physical Unitree Go2**
(ROS 2 Jazzy). For the sim↔real topic mapping see [SIM_TO_REAL.md](SIM_TO_REAL.md).

> Status: this real-robot path **builds and launches but has NOT been validated on
> hardware**. Expect to confirm topics/frames and debug the bridge. See
> "Things that may need fixing" below.

---

## Prerequisites

- Official Unitree ROS2 SDK installed and sourced (provides `unitree_api` /
  `unitree_go` messages): <https://github.com/unitreerobotics/unitree_ros2>
- This workspace built and sourced:
  ```bash
  cd ~/colcon_ws && colcon build --symlink-install
  source install/local_setup.bash
  source ~/unitree_ros2/install/setup.bash      # the Unitree SDK
  ```
- The robot publishing `/scan` (sensor_msgs/LaserScan), `/odom` (nav_msgs/Odometry),
  and `/tf` with frames `base_link` / `odom` / `map`.
- The Go2 standing / in `BalanceStand` (its own sport mode) before sending commands.

---

## 1. Map your environment (one-time per space)

```bash
ros2 launch gazebo_sim slam_real.launch.py
# ... drive the robot slowly around the whole area (remote or teleop) ...
```
Open RViz (Fixed Frame = `map`, add **Map** + **LaserScan**) to watch it build, then save:
```bash
mkdir -p ~/go2_maps
ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/my_area --ros-args -p use_sim_time:=false
```

## 2. Navigate

```bash
ros2 launch gazebo_sim nav2_real.launch.py map:=~/go2_maps/my_area.yaml
```
This starts Nav2 (no namespace, `use_sim_time:=false`) **and** the
`cmd_vel → /api/sport/request` bridge. Then in RViz: **2D Pose Estimate** → **Nav2 Goal**.

Useful overrides:
```bash
ros2 launch gazebo_sim nav2_real.launch.py \
    map:=~/go2_maps/my_area.yaml \
    odom_topic:=/utlidar/robot_odom \
    scan_topic:=/scan
```

### The velocity bridge (run standalone if needed)
```bash
ros2 launch quadropted_controller cmd_vel_bridge.launch.py sim:=false
```
`cmd_vel_to_sport.py` converts Nav2's `cmd_vel` (Twist) into Unitree sport `Move`
requests on `/api/sport/request`. (`sim:=true` instead selects the Gazebo bridge.)

---

## Things that may need fixing

### The bridge (`cmd_vel_to_sport.py`) — UNVALIDATED stub
- The sport-API `api_id`s (`Move`=1008, `StopMove`=1003) and `unitree_api/msg/Request`
  field names are from standard `unitree_ros2` but **never run on hardware**. Verify
  them against YOUR SDK version (`ros2 interface show unitree_api/msg/Request`).
- **Test with the robot on a stand first** — publish a tiny `cmd_vel` and confirm the
  legs respond as expected before letting it free-run on the floor.
- The Go2 must be in `BalanceStand` (sport mode active) or it will ignore Move commands.
- Decide control level: high-level sport `Move` (what this does, recommended) vs
  low-level `/lowcmd` (full joint control, much harder).

### Topics / frames
- Confirm the robot's topics: `ros2 topic list`, and frames: `ros2 run tf2_tools view_frames`.
- If odom is `/utlidar/robot_odom`, pass `odom_topic:=/utlidar/robot_odom`.
- `nav2_params_real.yaml` assumes frames `base_link` / `odom` / `map`; adjust if the
  robot uses different names, or AMCL and the costmaps will fail.

### Map / localization
- The default map in `nav2_real.launch.py` is the **sim cafe map** — replace it with a
  real SLAM map of your space (Step 1).
- In RViz the **Map** display needs **Durability = Transient Local** to show (it's latched).
- `Waiting for map` / `frame "map" does not exist` → check the lifecycle manager brings
  up `map_server` before `amcl` (already ordered in `localization_launch.py`).

### Build
- After any `apt upgrade` of `ros2-control`, rebuild the Gazebo control plugin so its ABI
  matches: `colcon build --packages-select gz_ros2_control --cmake-clean-cache`.
- `undefined symbol ... diagnostic_updater::Updater ...` → `sudo apt update && sudo apt upgrade`
  to sync `ros2-control` / `diagnostic-updater` / `tl-expected`, then rebuild.
