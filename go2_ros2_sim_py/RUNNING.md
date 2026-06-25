# Go2 sim — how to run + things that may need fixing

Unitree Go2 quadruped simulation on **ROS 2 Jazzy** + **Gazebo Sim**, with Nav2 and
an inverse-kinematics walking controller. This file is the practical run guide;
see [SIM_TO_REAL.md](SIM_TO_REAL.md) for the sim↔real-robot topic mapping.

---

## 1. Build

Put this folder inside a colcon workspace `src/` and build:

```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y     # 2 unresolved keys are harmless (see Troubleshooting)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

Packages: `gazebo_sim` (launch/worlds/maps), `go2_description` (URDF/meshes),
`quadropted_controller` (IK controller + bridges), `quadropted_msgs` (custom msgs).

---

## 2. Run the simulation

```bash
source install/local_setup.bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix gazebo_sim)/share/gazebo_sim/models
ros2 launch gazebo_sim launch.py
```

Wait ~10 s until the robot is **standing** in Gazebo and RViz's Nav2 panel shows
**Navigation: active / Localization: active**.

### Drive it manually
The robot only moves in **TROT** mode — REST/STAND cannot translate. In a second
sourced terminal:

```bash
# 1. enable walking gait (REQUIRED before any motion)
ros2 topic pub --once /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand "{mode: 'TROT', robot_id: 1}"

# 2a. keyboard teleop ...
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
# 2b. ... or a direct velocity
ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
```

Modes (publish to `/robot1/robot_mode`): `REST` (stand, can't move, default),
`STAND` (rotate in place), `TROT` (walk).

### Navigate with Nav2
1. Make sure it's in **TROT** (above).
2. In RViz: **2D Pose Estimate** → click the robot's location, drag toward its heading.
3. **Nav2 Goal** → click a destination. Robot plans + walks there.

> RViz **Map** display shows nothing until you set its **Durability = Transient Local**
> (the map is latched).

---

## 3. Use a real USB webcam as the camera (optional)

Replaces the simulated camera topic with a real webcam (e.g. Logitech C920).

```bash
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-image-transport-plugins
ros2 launch gazebo_sim webcam.launch.py video_device:=/dev/video3   # adjust device
ros2 run rqt_image_view rqt_image_view                              # /robot1/color/image_raw
```
The Gazebo camera is disabled in `gazebo_multi_nav2_world.launch.py` to avoid two
publishers on one topic (clearly commented; re-enable to use the sim camera).

---

## 4. Real Unitree Go2 (NOT yet hardware-validated — see Troubleshooting)

Requires the official Unitree ROS2 SDK sourced: <https://github.com/unitreerobotics/unitree_ros2>

```bash
# A) map your environment with slam_toolbox
ros2 launch gazebo_sim slam_real.launch.py        # drive around, then:
ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/my_area --ros-args -p use_sim_time:=false

# B) navigate on the real robot (Nav2 + cmd_vel -> Unitree sport API bridge)
ros2 launch gazebo_sim nav2_real.launch.py map:=~/go2_maps/my_area.yaml
```
The bridge selector: `cmd_vel_bridge.launch.py sim:=true` (Gazebo) /
`sim:=false` (real Go2 → `/api/sport/request`).

---

## Things that may need fixing

### Build / runtime
- **`rosdep` reports `gazebo_plugins` / `ros_ign_utils` unresolved** — harmless. They're
  Gazebo-Classic leftovers in `go2_description/package.xml`; the sim uses Gazebo Sim.
- **`undefined symbol ... diagnostic_updater::Updater ...`** (controller_manager or
  robot_localization crash) — partial apt state. Fix: `sudo apt update && sudo apt upgrade`
  to sync `ros2-control` / `diagnostic-updater` / `tl-expected`, then **rebuild
  `gz_ros2_control`**: `colcon build --packages-select gz_ros2_control --cmake-clean-cache`.
  Re-run that rebuild any time you `apt upgrade` ros2_control — the Gazebo plugin must
  match the installed `hardware_interface` ABI or Gazebo segfaults on hardware init.
- **colcon symlink errors** (`failed to create symbolic link ... Is a directory`) — stale
  build mixing symlink/non-symlink installs. Fix: `rm -rf build install log && colcon build --symlink-install`.

### Robot won't stand / collapses on spawn
- Tune **spawn height** `z_pose` in `gazebo_sim/config/robots.yaml` (too high = tumbles,
  too low = clips through floor; ~0.5–0.7 worked).
- Tune the leg **`position_proportional_gain`** in `go2_description/xacro/gazebo.xacro`
  (default 0.1 is too soft to hold the body; ~5.0 works). Higher = stiffer but can jitter.
- Default startup mode is **REST** (set in `quadropted_controller/.../RobotController.py`)
  so it stands rather than booting into TROT and falling.

### Nav2 plans but the robot doesn't move
- It must be in **TROT** — REST/STAND ignore velocity.
- `cmd_vel_pub.py` scales velocity down hard (`×0.035`), so the dog walks slowly and Nav2's
  progress checker may abort with **"Failed to make progress"**. Relax it in
  `gazebo_sim/config/nav2_params.yaml` (lower `required_movement_radius`, raise
  `movement_time_allowance`) or reduce the scaling in `cmd_vel_pub.py`.
- **`Waiting for map` / `frame "map" does not exist`** — the localization launch was fixed
  to use one ordered lifecycle manager (`map_server` then `amcl`). If it regresses, check
  `gazebo_sim/launch/nav2/localization_launch.py`.

### Real robot (`cmd_vel_to_sport.py`) — UNVALIDATED stub
- The sport-API `api_id`s (`Move`=1008, `StopMove`=1003) and `unitree_api/msg/Request`
  field names are taken from standard `unitree_ros2` but **never run on hardware**.
  Verify against your SDK version and **test with the robot on a stand first**.
- Confirm the robot publishes `/scan`, `/odom`, `/tf` with frames `base_link`/`odom`/`map`;
  pass `odom_topic:=/utlidar/robot_odom` if needed.
- The Go2 must be standing / in `BalanceStand` before it accepts Move commands.
- The default map in `nav2_real.launch.py` is the sim cafe map — replace with a real
  SLAM map of your space.
