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
                       ├──▶│ control (HTTP → SportClient)   │──▶ rt/api/sport/request
                       │   │   /standup /move /stop /damp   │
                       │   └──────────────────────────────┘
        front cam ──WebRTC─▶│ camera ──localhost JPEG──▶ /go2/camera
                           └──────────────────────────────┘
```

**Independent containers, one robot LAN.** The `camera` service does the heavy
WebRTC decode in isolation and forwards JPEG frames to the `bridge` over localhost,
so the camera appears on the *same* Foxglove connection — but if WebRTC fails, the
3D/LiDAR view stays up. The `control` service is a separate DDS participant that only
writes (it never subscribes), so driving the robot can't be slowed down by the
bridge's read load.

## control (drive the robot)

The `control` service wraps `unitree_sdk2py`'s high-level `SportClient` — the same
SDK the `bridge` already vendors to read `lowstate`/`sportmodestate` — behind a tiny
HTTP API, so you can drive the robot with `curl` instead of writing DDS code:

```bash
curl -X POST http://<device>:8767/standup
curl -X POST http://<device>:8767/move -H 'content-type: application/json' \
  -d '{"vx": 0.3, "vy": 0, "vyaw": 0, "duration_s": 1.0}'
curl -X POST http://<device>:8767/stop
```

`Move()` is a velocity command the robot's watchdog expects repeated, so `/move`
re-sends it at 10 Hz for `duration_s` and calls `StopMove()` when done. Other
endpoints: `/standdown`, `/damp`, `/healthz`.

Env (set in `control/Dockerfile`, override at build/run):
- **CONTROL_PORT** — the HTTP API port (default `8767`).
- **GO2_DDS_ADDRESS** — *this device's* IP on the robot LAN, set in `control/cyclonedds.xml`
  (same multi-homed-NIC caveat as `bridge`).

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
- **control / sport lease**: `SportClient` needs the sport lease (`/api/sport_lease`)
  — if the Unitree phone app or another SDK client holds it, `/standup` and `/move`
  will silently no-op or error. Make sure the robot is on a flat, clear area before
  calling `/move`; nothing in this service stops it from walking into something.
