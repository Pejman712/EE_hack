#!/usr/bin/env python3
"""cmd_vel_to_sport — bridge nav2's /cmd_vel to the Go2's sport Move API using
ONLY the ros2 CLI (no rclpy).

The Go2 has no /cmd_vel input. It walks via unitree_api/msg/Request "Move"
commands (api_id 1008, parameter {"x":vx,"y":vy,"z":yaw_rate}) on
/api/sport/request, and a velocity watchdog stops the dog if it stops hearing
them (~0.4 s). nav2's controller, on the other hand, drives geometry_msgs/Twist
on /cmd_vel. This script connects the two with two long-lived `ros2` CLI
processes — no rclpy node:

  reader:  `ros2 topic echo /cmd_vel --csv`                       -> latest (vx, vy, wz)
  writer:  `ros2 topic pub -r N /api/sport/request <Request> '…'` -> feeds the dog

A running `ros2 topic pub` publishes a FIXED message, so we can't stream a
changing velocity through one. Instead we **quantize** the command and only
restart the writer when the quantized command changes. During a straight "walk N
metres" the controller's command is nearly constant, so the steady writer keeps
the watchdog fed and we respawn rarely. When the command is ~zero we publish
StopMove (1003) once so the dog holds position instead of timing out mid-stride.
On exit we StopMove so a dying bridge never leaves the dog walking.

This CLI-only design is deliberate (see README): the cost of not running an
rclpy node is the quantization + respawn-on-change behaviour below. If smoother
following is ever needed, a ~20-line rclpy node would forward every Twist 1:1.

Env overrides: CMD_VEL_TOPIC (/cmd_vel), BRIDGE_RATE (10 Hz writer),
MAX_VX/MAX_VY/MAX_WZ velocity clamps.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

SPORT_TOPIC = "/api/sport/request"
SPORT_TYPE = "unitree_api/msg/Request"
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
MOVE_API_ID = 1008
STOP_API_ID = 1003

WRITE_RATE = float(os.environ.get("BRIDGE_RATE", "10"))   # Hz the writer feeds the dog
LOOP_PERIOD = 0.05                                         # 20 Hz command re-evaluation
CMD_TIMEOUT = 0.5                                          # /cmd_vel staleness -> stop

# Safety clamps (m/s, rad/s). The Go2 can go faster; keep "walk N metres" gentle.
MAX_VX = float(os.environ.get("MAX_VX", "0.4"))
MAX_VY = float(os.environ.get("MAX_VY", "0.3"))
MAX_WZ = float(os.environ.get("MAX_WZ", "0.8"))

# Quantization steps — coarse enough that a steady walk doesn't churn the writer.
STEP_V = 0.02
STEP_W = 0.05

_latest = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "t": 0.0}
_lock = threading.Lock()
_writer = {"proc": None, "cmd": None}  # cmd == quantized (vx,vy,wz) currently being published


def _reader():
    """Stream /cmd_vel as CSV and keep the latest velocity. geometry_msgs/Twist
    --csv order is linear.x,linear.y,linear.z,angular.x,angular.y,angular.z."""
    while True:
        proc = subprocess.Popen(
            ["ros2", "topic", "echo", CMD_VEL_TOPIC, "--csv"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1,
        )
        for line in proc.stdout:
            parts = line.strip().split(",")
            if len(parts) < 6:
                continue
            try:
                vx, vy, wz = float(parts[0]), float(parts[1]), float(parts[5])
            except ValueError:
                continue
            with _lock:
                _latest.update(vx=vx, vy=vy, wz=wz, t=time.time())
        # echo died (publisher gone / DDS hiccup) — back off and respawn.
        time.sleep(1.0)


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _quant(v, step):
    return round(v / step) * step


def _stop_writer():
    p = _writer["proc"]
    if p is not None and p.poll() is None:
        p.terminate()
        try:
            p.wait(timeout=1.0)
        except Exception:  # noqa: BLE001
            p.kill()
    _writer["proc"] = None


def _start_move(vx, vy, wz):
    param = json.dumps({"x": vx, "y": vy, "z": wz}, separators=(",", ":"))
    # YAML for unitree_api/msg/Request; parameter is a string holding that JSON.
    msg = "{header: {identity: {api_id: %d}}, parameter: '%s'}" % (MOVE_API_ID, param)
    _writer["proc"] = subprocess.Popen(
        ["ros2", "topic", "pub", "-r", str(WRITE_RATE), SPORT_TOPIC, SPORT_TYPE, msg],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    _writer["cmd"] = (vx, vy, wz)


def _publish_stop():
    msg = "{header: {identity: {api_id: %d}}}" % STOP_API_ID
    try:
        subprocess.run(
            ["ros2", "topic", "pub", "--once", SPORT_TOPIC, SPORT_TYPE, msg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8,
        )
    except Exception:  # noqa: BLE001
        pass


def _shutdown(*_):
    _stop_writer()
    _publish_stop()
    sys.exit(0)


def main():
    print(f"[bridge] {CMD_VEL_TOPIC} (Twist) -> {SPORT_TOPIC} Move@{WRITE_RATE:g}Hz "
          f"(clamps vx={MAX_VX} vy={MAX_VY} wz={MAX_WZ})", flush=True)
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    threading.Thread(target=_reader, daemon=True).start()

    while True:
        time.sleep(LOOP_PERIOD)

        # If the writer died on its own, force a respawn on the next change.
        p = _writer["proc"]
        if p is not None and p.poll() is not None:
            _writer["proc"] = None
            _writer["cmd"] = None

        with _lock:
            vx, vy, wz, t = _latest["vx"], _latest["vy"], _latest["wz"], _latest["t"]
        if (time.time() - t) > CMD_TIMEOUT:
            vx = vy = wz = 0.0  # stale -> stop

        vx = _clamp(_quant(vx, STEP_V), MAX_VX)
        vy = _clamp(_quant(vy, STEP_V), MAX_VY)
        wz = _clamp(_quant(wz, STEP_W), MAX_WZ)
        cmd = (vx, vy, wz)

        if cmd == _writer["cmd"]:
            continue  # nothing changed; the steady writer keeps feeding the dog

        is_zero = abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6
        _stop_writer()
        if is_zero:
            _publish_stop()
            _writer["cmd"] = (0.0, 0.0, 0.0)  # remember we're stopped; don't re-StopMove
        else:
            _start_move(vx, vy, wz)


if __name__ == "__main__":
    main()
