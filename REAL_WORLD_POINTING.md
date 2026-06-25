# Pointing → Nav2 on the real Go2 (Humble)

## ⭐ Quick start — map-based navigation, no camera

> **Want a web map UI instead of the CLI?** See **[nav2/MAP_NAV.md](nav2/MAP_NAV.md)**:
> a web page that loads your map, sets the **initial** + **goal** pose by click or
> typing, and shows the dog's live pose — driven by the same stack, with **utlidar**
> odometry. Runs as the `nav2` Wendy service on `:7100`.

Localize in your saved map and drive to a point you choose. **No camera / pointing.**

```bash
# Terminal 1 — the WHOLE stack:
#   odom + scan + map_server + AMCL + Nav2 + cmd_vel→sport bridge
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/install/setup.bash
cd EE_hack/nav2
ros2 launch nav2_mapped.launch.py
```
This brings up everything for map-based navigation (no camera/pointing): loads
`nav2/maps/map.yaml`, AMCL localizes (`map→odom`), the Nav2 servers run, and the
`cmd_vel→sport` bridge drives the dog.

```bash
# Terminal 2 — go to a chosen point (ABSOLUTE map coordinates)
cd EE_hack/nav2
python3 goto_point.py --ros-args -p frame:=map -p x:=3.0 -p y:=-1.5
```

**Before the first goal:** AMCL starts at the map origin. If the robot isn't
physically there, set its real start pose once (Foxglove pose tool, or publish to
`/initialpose`) — otherwise goals go to the wrong place. Confirm the laser lines up
with the map walls, then send goals.

> The rest of this doc covers the **camera pointing** mode (point at the ground →
> the dog goes there) and full debugging. For just "localize + drive to a point,"
> the two commands above are all you need.

---

Point at the ground; the dog walks there. This wires the MediaPipe pose node
(`ros2_pose_node.py`) into the existing **mapless Nav2** stack (`nav2/`) via a small
bridge (`nav2/pointed_goal.py`).

```
camera ─▶ ros2_pose_node.py ─▶ /pointed_location (PoseArray, arm→ground hit)
                                      │
                                      ▼
                          nav2/pointed_goal.py
        (pick arm → point is camera-relative (robot=0) → + odometry via TF
         → goal in `odom` frame) ─▶ /navigate_to_pose (NavigateToPose action)
                                      │
                                      ▼
        nav2 (controller/planner/bt) ─▶ /cmd_vel ─▶ cmd_vel_to_sport ─▶ /api/sport/request
```

> The Nav2 stack here is **mapless** — rolling costmaps in the `odom` frame, no SLAM
> or map needed. So goals are sent in `odom`, which is exactly "the pointed offset
> summed with odometry."

---

## Prerequisites

- ROS 2 **Humble** sourced, plus the **Unitree ROS2 SDK** (`unitree_api`/`unitree_go`):
  <https://github.com/unitreerobotics/unitree_ros2>
- The robot reachable, in **sport mode / standing** (`BalanceStand`) — it must be
  standing before it accepts `Move` commands.
- MediaPipe deps for the pose node: `pip install mediapipe opencv-python` and
  `pose_landmarker_full.task` present (it is, in repo root).
- Same `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` in **every** terminal (the Go2 stack
  typically uses CycloneDDS — match it everywhere or nothing discovers anything).

---

## Run it (each in its own sourced terminal)

```bash
# 0. EVERY terminal first:
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/install/setup.bash
# (and the SAME ROS_DOMAIN_ID / RMW_IMPLEMENTATION the robot uses)
```

```bash
# 1. Nav2 stack (odom + scan + servers + cmd_vel_to_sport bridge)
cd EE_hack/nav2
ros2 launch nav2.launch.py
```

```bash
# 2. Pose node — point it at the robot's camera topic
cd EE_hack
python3 ros2_pose_node.py --ros-args -p image_topic:=/<your_camera_image_topic>
```

```bash
# 3. Pointing → Nav2 bridge (mapless => goal_frame:=odom)
cd EE_hack/nav2
python3 pointed_goal.py --ros-args -p goal_frame:=odom
```

Now stand in front of the camera and **point at the ground**. `pointed_goal.py`
logs `Sent Nav2 goal in odom: x=… y=…` and the dog walks there.

### Tuning (MediaPipe units are NOT metres)
```bash
python3 pointed_goal.py --ros-args -p goal_frame:=odom \
    -p scale_forward:=2.0 -p scale_lateral:=2.0 \
    -p arm:=right -p min_goal_interval:=2.0 -p min_goal_delta:=0.3
```
- `scale_forward` / `scale_lateral` — metres per MediaPipe unit. Start ~2.0, adjust
  until the goal distance matches where you actually point.
- `arm` — `right` | `left` | `either`.
- `min_goal_interval` / `min_goal_delta` — stop it spamming Nav2 with goals.

### Test the bridge WITHOUT a person (fake a point 1.5 m ahead)
```bash
ros2 topic pub --once /pointed_location geometry_msgs/msg/PoseArray \
"{header: {frame_id: 'base_link'}, poses: [{position: {x: 0.0, y: 0.0, z: 0.0}}, {position: {x: 0.5, y: 1.5, z: 0.0}}]}"
```
Expect a `Sent Nav2 goal` log and the dog to move.

---

## Why it's not working — debug each link in order

Work down the chain; the first broken link is the culprit.

### A. Camera → pose node
```bash
ros2 topic hz /<your_camera_image_topic>     # frames arriving?
ros2 topic echo /pointed_location --once      # PoseArray of 2 poses?
```
- All `nan` positions = no arm detected pointing at the ground. Stand fully in frame,
  point clearly downward. Check `/annotated_image` (the skeleton overlay) to see what
  MediaPipe sees.
- Nothing on `/pointed_location` = pose node not receiving images → wrong `image_topic`.

### B. pointed_goal.py → Nav2
```bash
ros2 node info /pointed_goal                  # subscribed to /pointed_location? action client to /navigate_to_pose?
ros2 action list | grep navigate_to_pose      # action server present?
```
- Log says **"navigate_to_pose action server not available"** → Nav2 not up (fix C).
- Log says **"TF base_link->odom unavailable"** → odom/TF missing (fix D).
- No `Sent Nav2 goal` though `/pointed_location` has valid poses → likely rate-limited
  (`min_goal_interval`) or too close to last goal (`min_goal_delta`); or the chosen
  `arm` is the one that's NaN — try `arm:=either`.

### C. Nav2 stack
```bash
ros2 lifecycle get /bt_navigator               # should be 'active'
ros2 topic echo /scan --once                   # 2D scan from the L1 cloud?
ros2 topic echo /cmd_vel --once                # Nav2 emitting velocity while a goal is active?
```
- Costmap/planner errors → check `nav2/config/nav2_params.yaml` and the
  `pointcloud_to_laserscan` height band (must cut walls, not the floor).
- Goal accepted but robot still → see E.

### D. Odometry / TF
```bash
ros2 run tf2_tools view_frames                 # need odom -> base_link (from go2_odom)
ros2 topic echo /odom --once                   # go2_odom publishing?
```
- No `odom->base_link` → `go2_odom.py` not running or `/sportmodestate` absent.
- Goal frame mismatch: this stack is **mapless**, so use `goal_frame:=odom`. A `map`
  frame won't exist (no SLAM/AMCL) and goals will be rejected/un-transformable.

### E. Nav2 commands but the dog doesn't move
```bash
ros2 topic echo /cmd_vel                        # non-zero while navigating?
ros2 topic echo /api/sport/request --once       # cmd_vel_to_sport forwarding Move?
```
- `/cmd_vel` non-zero but no `/api/sport/request` → `cmd_vel_to_sport.py` not running.
- Both present but no motion → robot not in sport `BalanceStand`, or `MAX_VX/VY/WZ`
  env limits too low (see `nav2/cmd_vel_to_sport.py`).

### F. Nothing discovers anything (empty `ros2 topic list`)
- `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` mismatch between terminals. Make them
  identical everywhere, matching the robot's DDS (usually CycloneDDS).

---

---

## Go to a chosen X,Y point (`goto_point.py`)

Manual alternative to pointing — pick a coordinate and the dog drives there. Same
Nav2 interface, sends one goal and reports progress.

```bash
# mapless stack (frame=odom, relative to where the robot started):
cd EE_hack/nav2
python3 goto_point.py --ros-args -p frame:=odom -p x:=2.0 -p y:=0.0 -p yaw:=0.0
```

## Navigating by ABSOLUTE map coordinates (map-based mode)

`nav2.launch.py` is mapless (goals relative to start). To use your saved map and
send goals in **absolute map coordinates**, use the map-based stack — it adds
`map_server` (loads `nav2/maps/map.yaml`) + `amcl` (localizes, publishes `map->odom`):

```bash
# 1. map-based nav2 (instead of nav2.launch.py)
cd EE_hack/nav2
ros2 launch nav2_mapped.launch.py            # or map:=/path/to/other_map.yaml

# 2. localize: amcl starts at the map origin. If the robot isn't there, set its
#    real pose in RViz (2D Pose Estimate) or edit initial_pose in
#    config/nav2_params_mapped.yaml.

# 3. drive to an absolute point in the map:
python3 goto_point.py --ros-args -p frame:=map -p x:=3.0 -p y:=-1.5
#    ...or point at the ground with the camera:
python3 pointed_goal.py --ros-args -p goal_frame:=map
```

Debug map mode (in addition to the chain above):
```bash
ros2 topic echo /map --once --qos-durability transient_local   # map served?
ros2 run tf2_tools view_frames                                  # need map->odom (amcl) and odom->base_link
ros2 lifecycle get /amcl                                        # 'active'?
```
If the laser doesn't line up with the map walls in RViz, fix the initial pose
(2D Pose Estimate) — a wrong start pose makes every goal go to the wrong place.

---

## Notes
- `nav2/pointed_goal.py` is byte-identical to the sim copy in
  `go2_ros2_sim_py/quadropted_controller/scripts/pointed_goal.py` — same logic, only
  the params differ (real: `goal_frame=odom`, no namespace; sim: `goal_frame=map`,
  namespace `robot1`).
- Coverage of the L1 lidar is forward/down-biased — obstacle avoidance behind the
  robot is poor. Validate `/scan` against the live cloud before trusting it.
