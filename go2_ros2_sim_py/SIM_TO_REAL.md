# Sim ↔ Real Go2: topic mapping & portability

This document maps the **Gazebo simulation** interface in this repo to the **real
Unitree Go2** interface, and explains how to run the *same* Nav2 stack on both.

## TL;DR

Nav2 and the sensor pipeline are **portable as-is**. Only the last hop —
`cmd_vel` → robot — differs, and that is absorbed by a swappable bridge node:

```
            ┌──────────────── identical on both targets ────────────────┐
 sensors → tf / odom / scan → Nav2 (costmaps, planner, controller) → cmd_vel
                                                                         │
                                    ┌────────────────────────────────────┴───────────┐
                              sim:=true                                          sim:=false
                          cmd_vel_pub.py                                   cmd_vel_to_sport.py
                    (cmd_vel → robot_velocity →                       (cmd_vel → /api/sport/request,
                      Gazebo IK controller)                             Unitree sport "Move" API)
```

Select the bridge with one launch argument:

```bash
# Simulation
ros2 launch quadropted_controller cmd_vel_bridge.launch.py sim:=true  namespace:=robot1
# Real robot
ros2 launch quadropted_controller cmd_vel_bridge.launch.py sim:=false
```

A **rename is not enough** to port the control side: Nav2 emits
`geometry_msgs/Twist`, but the real Go2 has **no Twist velocity input** — it is
driven by a `unitree_api/msg/Request` (`Move`, api_id `1008`) on
`/api/sport/request`. Converting the *message type* is the bridge's job.

## Control interface — fundamentally different (needs the bridge)

| Purpose            | Simulation (this repo)                                  | Real Go2                                             |
|--------------------|---------------------------------------------------------|------------------------------------------------------|
| Command motion     | `cmd_vel` (`geometry_msgs/Twist`)                       | `/api/sport/request` (`unitree_api/msg/Request`, Move)|
| Mode switching     | `robot_mode` (`quadropted_msgs/RobotModeCommand`)       | sport API requests (`/api/sport/request`)            |
| Joint commands     | `joint_group_controller/commands` (`Float64MultiArray`)| `/lowcmd` (`unitree_go/msg/LowCmd`)                  |
| Robot state        | `joint_states`, `imu`                                   | `/sportmodestate`, `/lowstate` (`unitree_go` msgs)   |

These sim topics **do not exist** on the real robot, and the real `/api/*`,
`/lowcmd`, `/sportmodestate` **do not exist** in the sim.

## Sensing / Nav2 — corresponds (align names, then reuse config)

| Purpose      | Simulation                         | Real Go2                          |
|--------------|------------------------------------|-----------------------------------|
| Odometry     | `/robot1/odometry/filtered`        | `/odom`, `/utlidar/robot_odom`    |
| Laser scan   | `/robot1/scan`                     | `/scan`                           |
| Transforms   | `/robot1/tf`                       | `/tf`                             |
| Point cloud  | `/robot1/scan/points`              | `/utlidar/cloud(_deskewed)`       |
| Nav2 servers | costmaps / planner / bt_navigator  | same (Go2 already runs Nav2)      |

## To make the *same* config run on both

1. **Frame names must match** — both must use the same `base_link` / `odom` /
   `map` frame ids, or AMCL and the costmaps break. Pick one convention and apply
   it to both targets.
2. **Topic names**: either drop the `robot1` namespace in the sim, or namespace
   both identically, so one `nav2_params.yaml` applies to both. The bridge nodes
   subscribe to the *relative* `cmd_vel`, so launching under a namespace handles it.
3. **Swap only the bridge** via `sim:=true|false` (see above).

## Real-robot prerequisites (not needed in sim)

- Unitree ROS2 SDK installed: <https://github.com/unitreerobotics/unitree_ros2>
  (provides the `unitree_api` / `unitree_go` messages the real bridge imports).
- Decide the control level:
  - **High-level sport `Move`** (recommended, what `cmd_vel_to_sport.py` does) —
    the robot keeps its own balance/odometry.
  - **Low-level `/lowcmd`** — full joint control, much harder, bypasses sport mode.
- `cmd_vel_to_sport.py` is a **stub**: verify `api_id` values and message field
  names against your `unitree_ros2` version, and test with the robot on a stand.

## Files added for portability

- `quadropted_controller/scripts/cmd_vel_to_sport.py` — real-robot bridge (stub).
- `quadropted_controller/launch/cmd_vel_bridge.launch.py` — `sim:=true|false` selector.
- `quadropted_controller/scripts/cmd_vel_pub.py` — existing sim bridge (unchanged).
