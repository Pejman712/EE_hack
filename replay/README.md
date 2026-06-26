# go2-replay

Re-drive a **recorded** Go2 run by replaying its sport command stream — the
playback sibling of [`../wander`](../wander). Where wander turns *live* `/scan`
into motion, replay turns a *recorded bag* back into the **same motion**.

The run in [`run.mcap`](run.mcap) was teleop, captured as a stream of Unitree
**Move** commands (`unitree_api/msg/Request`, `api_id 1008`,
`{"x":vx,"y":vy,"z":yaw_rate}`) on `/api/sport/request` at ~40 Hz. `replay.py`
re-publishes that exact stream, with the original inter-message timing, straight
to the dog's sport API — no `/scan`, no `/cmd_vel`, no bridge.

## Important: it's open-loop

Replaying velocities reproduces the same **motion profile**, not a map-tracked
path. Nothing corrects drift, so:

- **Place the robot at the recording's start pose and heading** before starting.
- Turns diverge most; the path is *approximately* the recorded one, not exact.

## Safety

The recorded run's **last Move is non-zero** (the dog was still turning), so a
raw `ros2 bag play` would leave it driving when playback ends. `replay.py`
**always** sends `Move(0,0,0) → StopMove → BalanceStand` on finish **and** on
Stop/SIGINT. It replays only `/api/sport/request` (+ any `REPLAY_TOPICS_EXTRA`) —
never `/lowcmd` or recorded state/sensor topics.

## Run

Web UI (same pattern as wander): `Start replay` / `Stop`.

```bash
# build for the Go2's Orin (arm64)
docker build --platform linux/arm64 -t go2-replay .
docker run --rm --network host -e GO2_IP=192.168.123.161 -p 7100:7100 go2-replay
# open http://<host>:7100  → Start replay
```

Or run the node directly on a machine that already has ROS 2 + the Unitree
messages sourced and reaches the robot:

```bash
pip install mcap mcap-ros2-support
BAG_PATH=run.mcap REPLAY_SPEED=0.5 python3 replay.py   # first run at half speed
```

## Env

| var | default | meaning |
|-----|---------|---------|
| `BAG_PATH` | `/run.mcap` | run to replay (override to mount a different bag) |
| `REPLAY_SPEED` | `1.0` | playback multiplier (`0.5` = half speed, safer) |
| `REPLAY_START_DELAY` | `0.0` | seconds to wait before driving |
| `REPLAY_TOPICS_EXTRA` | — | extra request topics, e.g. `/api/obstacles_avoid/request` |
| `GO2_IP` | `192.168.123.161` | any address on the robot's DDS LAN (picks the NIC) |
| `PORT` | `7100` | web UI port |
