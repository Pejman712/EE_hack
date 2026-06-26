"""Go2 map-based nav2 server — a web map UI + JSON API that drives nav2 over the
ros2 CLI.

This is the MAP-BASED counterpart to the mapless "walk N metres" walker: it
launches the map-based nav2 stack (nav2_mapped.launch.py — map_server + AMCL +
the nav2 servers + cmd_vel->sport bridge) as a subprocess, loads a saved
occupancy map (yaml + pgm) from a path you provide, and serves a web page that
renders that map so you can:

  * set the INITIAL pose  — click/drag on the map (or type x/y/yaw) -> /initialpose
        (geometry_msgs/PoseWithCovarianceStamped) which AMCL uses to (re)localise.
  * set the GOAL pose     — click/drag on the map (or type x/y/yaw) -> a
        NavigateToPose goal in the **map** frame (absolute map coordinates).
  * watch the LIVE robot pose — the map->base_link TF (AMCL + utlidar odom),
        drawn as a moving marker/arrow.

Everything is driven purely with `ros2` CLI calls (same pattern as the recorder
and the mapless walker) — no rclpy in this process.

  GET  /                       map control UI
  GET  /api/map/meta           map yaml metadata (resolution, origin, size)
  GET  /api/map/image.png      the occupancy map rendered as PNG (for the canvas)
  GET  /api/nav/status         nav2 up? + navigating? + last result + live pose
  POST /api/nav/initialpose    {"x":X,"y":Y,"yaw":YAW}  -> publish /initialpose
  POST /api/nav/goal           {"x":X,"y":Y,"yaw":YAW}  -> NavigateToPose (map frame)
  POST /api/nav/stop           cancel the goal + StopMove the dog

Env:
  MAP_YAML      path to the map .yaml (default ./maps/map.yaml; its `image:` is
                resolved relative to the yaml). The matching .pgm sits beside it.
  ODOM_SOURCE   utlidar (default) | sportmodestate — passed to nav2_mapped.launch.py.
  PORT          web server port (default 7100).
"""
import io
import math
import os
import re
import signal
import subprocess
import threading
import time

import uvicorn
import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, Response
from PIL import Image
from pydantic import BaseModel

PORT = int(os.environ.get("PORT", "7100"))
HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_FILE = os.path.join(HERE, "nav2_mapped.launch.py")
WANDER_FILE = os.path.join(HERE, "wander.py")
MAP_YAML = os.environ.get("MAP_YAML", os.path.join(HERE, "maps", "map.yaml"))
ODOM_SOURCE = os.environ.get("ODOM_SOURCE", "utlidar")

NAV_ACTION = "/navigate_to_pose"
NAV_ACTION_TYPE = "nav2_msgs/action/NavigateToPose"
NAV_CANCEL_SRV = "/navigate_to_pose/_action/cancel_goal"
NAV_CANCEL_TYPE = "action_msgs/srv/CancelGoal"
NAV_CANCEL_ALL = "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}"
INITIALPOSE_TOPIC = "/initialpose"
INITIALPOSE_TYPE = "geometry_msgs/msg/PoseWithCovarianceStamped"
SPORT_TOPIC = "/api/sport/request"
SPORT_TYPE = "unitree_api/msg/Request"
SPORT_API_ID_STOP = 1003

# AMCL initial-pose covariance: modest confidence in x,y (0.25 m^2) and yaw
# (~0.068 rad^2) — the same diagonal RViz's "2D Pose Estimate" tool sends.
COV_XX, COV_YY, COV_YAW = 0.25, 0.25, 0.06853892326654787

app = FastAPI(title="go2-nav2-map")

# Long-lived nav2 stack + the single in-flight goal + latest robot pose.
#   pose      : map -> base_link  (AMCL-localised pose in the map)
#   odom      : odom -> base_link (raw utlidar dead-reckoning; tf2_echo)
#   odom_zero : snapshot taken on "reset" — DISPLAY tare only. Subtracted from
#               `odom` before it's shown so the readout reads (0,0,0) at reset.
#               Purely cosmetic: the real TF / AMCL chain is never touched.
_nav = {"launch": None, "goal": None, "x": None, "y": None, "yaw": None,
        "started": None, "result": None, "pose": None,
        "odom": None, "odom_zero": None,
        # liveness stamps for the per-link diagnostics ({"t": <epoch>} or None)
        "scan": None, "cmdvel": None,
        "wander": None}  # reactive /scan->/cmd_vel wander subprocess
_map = {"meta": None, "png": None, "png_mtime": None}


class PoseReq(BaseModel):
    x: float = 0.0     # metres, map frame
    y: float = 0.0
    yaw: float = 0.0   # radians, map frame (0 = +x)


# ----------------------------------------------------------------------------- map
def _load_map_meta() -> dict:
    """Parse the map yaml once; resolve the image path relative to the yaml."""
    if _map["meta"] is not None:
        return _map["meta"]
    with open(MAP_YAML) as f:
        y = yaml.safe_load(f)
    image = y["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(MAP_YAML)), image)
    with Image.open(image) as im:
        w, h = im.size
    res = float(y["resolution"])
    ox, oy = float(y["origin"][0]), float(y["origin"][1])
    oyaw = float(y["origin"][2]) if len(y["origin"]) > 2 else 0.0
    _map["meta"] = {
        "map_yaml": MAP_YAML, "image": image,
        "resolution": res, "origin": [ox, oy, oyaw],
        "width": w, "height": h,
        # world extent of the image (origin is the bottom-left corner)
        "bounds": {"min_x": ox, "min_y": oy,
                   "max_x": ox + w * res, "max_y": oy + h * res},
    }
    return _map["meta"]


def _map_png() -> bytes:
    """Render the occupancy .pgm to PNG bytes (cached on the image's mtime)."""
    meta = _load_map_meta()
    mtime = os.path.getmtime(meta["image"])
    if _map["png"] is None or _map["png_mtime"] != mtime:
        with Image.open(meta["image"]) as im:
            buf = io.BytesIO()
            im.convert("L").save(buf, format="PNG")
        _map["png"], _map["png_mtime"] = buf.getvalue(), mtime
    return _map["png"]


# --------------------------------------------------------------------------- nav2
def _launch_alive() -> bool:
    p = _nav["launch"]
    return p is not None and p.poll() is None


def _is_navigating() -> bool:
    p = _nav["goal"]
    return p is not None and p.poll() is None


def _wandering() -> bool:
    p = _nav["wander"]
    return p is not None and p.poll() is None


def _start_nav2():
    """Spawn `ros2 launch nav2_mapped.launch.py map:=.. odom_source:=..` in its
    own session so the whole nav2 tree can be torn down together."""
    if _launch_alive():
        return
    _nav["launch"] = subprocess.Popen(
        ["ros2", "launch", LAUNCH_FILE,
         f"map:={MAP_YAML}", f"odom_source:={ODOM_SOURCE}"],
        start_new_session=True,
    )
    print(f"[nav2] launched {LAUNCH_FILE} map={MAP_YAML} odom={ODOM_SOURCE} "
          f"(pid={_nav['launch'].pid})", flush=True)


def _stop_dog():
    """One-shot StopMove so the dog halts immediately, regardless of whether the
    action cancel has propagated through the controller yet."""
    msg = "{header: {identity: {api_id: %d}}}" % SPORT_API_ID_STOP
    try:
        subprocess.run(
            ["ros2", "topic", "pub", "--once", SPORT_TOPIC, SPORT_TYPE, msg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8)
    except Exception:  # noqa: BLE001
        pass


def _quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (z, w)


# Live robot pose: tail `tf2_echo map base_link` and keep the latest x/y/yaw.
_TRANS_RE = re.compile(r"Translation:\s*\[\s*([-\d.eE]+),\s*([-\d.eE]+)")
_QUAT_RE = re.compile(r"Quaternion\s*\[\s*([-\d.eE]+),\s*([-\d.eE]+),\s*([-\d.eE]+),\s*([-\d.eE]+)")


def _tf_poller(parent: str, child: str, key: str):
    """Tail `tf2_echo <parent> <child>` and keep the latest x/y/yaw in _nav[key]
    (~1 Hz). Restarts the subprocess if it dies (e.g. TF not available until the
    relevant node is active). Used for both map->base_link (AMCL pose) and
    odom->base_link (raw utlidar odom)."""
    while True:
        try:
            proc = subprocess.Popen(
                ["ros2", "run", "tf2_ros", "tf2_echo", parent, child],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                start_new_session=True)
        except Exception:  # noqa: BLE001
            time.sleep(3)
            continue
        tx = ty = None
        for line in proc.stdout:
            m = _TRANS_RE.search(line)
            if m:
                tx, ty = float(m.group(1)), float(m.group(2))
                continue
            q = _QUAT_RE.search(line)
            if q and tx is not None:
                qz, qw = float(q.group(3)), float(q.group(4))
                yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
                _nav[key] = {"x": tx, "y": ty, "yaw": yaw, "t": time.time()}
                tx = ty = None
        proc.wait()
        time.sleep(2)  # TF dropped — wait and re-attach


def _tared_odom():
    """Raw odom minus the reset snapshot, expressed in the reset-pose frame so a
    fresh reset reads (0,0,0) and +x is 'forward since reset'. None if no odom
    yet. DISPLAY only — does not affect TF/AMCL."""
    o = _nav["odom"]
    if not o or time.time() - o["t"] > 5.0:
        return None  # stale — odom stopped flowing
    z = _nav["odom_zero"]
    if not z:
        return {"x": o["x"], "y": o["y"], "yaw": o["yaw"]}
    dx, dy = o["x"] - z["x"], o["y"] - z["y"]
    c, s = math.cos(z["yaw"]), math.sin(z["yaw"])
    yaw = math.atan2(math.sin(o["yaw"] - z["yaw"]), math.cos(o["yaw"] - z["yaw"]))
    return {"x": c * dx + s * dy, "y": -s * dx + c * dy, "yaw": yaw}


def _fresh(d, ttl: float = 4.0) -> bool:
    """True if the liveness/pose dict `d` was stamped within the last `ttl` s."""
    return bool(d) and (time.time() - d["t"] < ttl)


def _liveness_poller(topic: str, key: str, field: str):
    """Tail `ros2 topic echo <topic> --field <field>` and stamp _nav[key]['t'] on
    every message — liveness only, no payload parsing. `--field` keeps the bytes
    printed tiny (e.g. just header.stamp) so this stays cheap even for /scan.
    Restarts the echo if it dies (topic not up until its publisher is)."""
    while True:
        try:
            proc = subprocess.Popen(
                ["ros2", "topic", "echo", topic, "--field", field],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                start_new_session=True)
        except Exception:  # noqa: BLE001
            time.sleep(3)
            continue
        for line in proc.stdout:
            if line.strip() and line.strip() != "---":
                _nav[key] = {"t": time.time()}
        proc.wait()
        time.sleep(2)  # publisher gone — wait and re-attach


# ---------------------------------------------------------------------------- API
@app.get("/api/map/meta")
def api_map_meta() -> dict:
    try:
        return {"ok": True, **_load_map_meta()}
    except Exception as e:  # noqa: BLE001
        raise HTTPException(500, f"could not load map {MAP_YAML}: {e}")


@app.get("/api/map/image.png")
def api_map_image() -> Response:
    try:
        return Response(content=_map_png(), media_type="image/png")
    except Exception as e:  # noqa: BLE001
        raise HTTPException(500, f"could not render map image: {e}")


@app.get("/api/nav/status")
def api_status() -> dict:
    p = _nav["goal"]
    if p is not None and p.poll() is not None and _nav["result"] is None:
        _nav["result"] = "ok" if p.returncode == 0 else f"exit {p.returncode}"
    pose = _nav["pose"]
    if pose and time.time() - pose["t"] > 5.0:
        pose = None  # stale — TF stopped flowing
    return {
        "nav2_up": _launch_alive(),
        "navigating": _is_navigating(),
        "goal": None if _nav["x"] is None else {"x": _nav["x"], "y": _nav["y"], "yaw": _nav["yaw"]},
        "started": _nav["started"],
        "result": _nav["result"],
        "pose": pose,
        "odom": _tared_odom(),
        "odom_tared": _nav["odom_zero"] is not None,
        "wandering": _wandering(),
    }


@app.get("/api/nav/diag")
def api_diag() -> dict:
    """Per-link health of the nav chain, in dependency order. Each step's `ok` is
    True/False, or None when the check only applies while navigating. The UI walks
    these top-to-bottom and stops at the first broken link (the `blocker`)."""
    navg = _is_navigating()
    active = navg or _wandering()  # /cmd_vel only flows while one of these drives
    steps = [
        {"name": "Nav2 stack running",
         "ok": _launch_alive(),
         "hint": "server.py auto-launches nav2_mapped.launch.py — if this stays red, "
                 "read the server log; a node probably crashed on startup.",
         "cmd": "ros2 node list   # expect map_server, amcl, *_server, bt_navigator"},
        {"name": "Odometry  (utlidar → odom→base_link)",
         "ok": _fresh(_nav["odom"]),
         "hint": "go2_odom isn't getting /utlidar/robot_odom. Check the topic exists "
                 "and the DDS domain matches; else relaunch with ODOM_SOURCE=sportmodestate.",
         "cmd": "ros2 topic hz /utlidar/robot_odom ; ros2 run tf2_ros tf2_echo odom base_link"},
        {"name": "Laser  /scan",
         "ok": _fresh(_nav["scan"]),
         "hint": "pointcloud_to_laserscan produced nothing. Usual causes: cloud frame_id "
                 "≠ utlidar_lidar, clock skew (raise transform_tolerance), or an empty "
                 "height band in pointcloud_to_laserscan.yaml.",
         "cmd": "ros2 topic hz /scan ; ros2 topic echo --field header.frame_id /utlidar/cloud_deskewed"},
        {"name": "Localization  (AMCL map→odom)",
         "ok": _fresh(_nav["pose"]),
         "hint": "AMCL isn't publishing map→odom. Set the initial pose so the scan lands "
                 "on the walls, and confirm amcl reached 'active'.",
         "cmd": "ros2 lifecycle get /amcl ; ros2 run tf2_ros tf2_echo map base_link"},
        {"name": "Controller output  /cmd_vel",
         "ok": _fresh(_nav["cmdvel"]) if active else None,
         "hint": "Only meaningful while a goal is active. If navigating but this is red, "
                 "the planner/controller is stuck: no valid path, costmap blocked, or the "
                 "goal sits in an obstacle/unknown cell.",
         "cmd": "ros2 topic echo /cmd_vel"},
        {"name": "Sport bridge  (/cmd_vel → /api/sport/request)",
         "ok": _fresh(_nav["cmdvel"]) if active else None,
         "hint": "cmd_vel_to_sport relays Twist to the dog. If cmd_vel flows but the dog "
                 "doesn't move: make sure it's in BalanceStand (sport ignores Move while "
                 "sitting) and watch /api/sport/request.",
         "cmd": "ros2 topic echo /api/sport/request"},
    ]
    blocker = next((i for i, s in enumerate(steps) if s["ok"] is False), None)
    return {"steps": steps, "blocker": blocker, "navigating": navg}


@app.post("/api/nav/odom_reset")
def api_odom_reset() -> dict:
    """Zero the displayed utlidar odom at the robot's current pose (display tare).
    Snapshots the live odom->base_link; the status endpoint subtracts it. Does
    NOT touch the TF tree or AMCL — navigation is unaffected."""
    o = _nav["odom"]
    if not o or time.time() - o["t"] > 5.0:
        raise HTTPException(503, "no live odom yet — is go2_odom publishing?")
    _nav["odom_zero"] = {"x": o["x"], "y": o["y"], "yaw": o["yaw"]}
    return {"ok": True}


@app.post("/api/nav/odom_unreset")
def api_odom_unreset() -> dict:
    """Clear the tare — show raw odom->base_link values again."""
    _nav["odom_zero"] = None
    return {"ok": True}


@app.post("/api/nav/initialpose")
def api_initialpose(req: PoseReq) -> dict:
    if not _launch_alive():
        raise HTTPException(503, "nav2 stack not running yet — try again in a few seconds")
    x, y, yaw = float(req.x), float(req.y), float(req.yaw)
    qz, qw = _quat(yaw)
    cov = [0.0] * 36
    cov[0], cov[7], cov[35] = COV_XX, COV_YY, COV_YAW
    msg = (
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: %f, y: %f, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}, covariance: [%s]}}"
        % (x, y, qz, qw, ", ".join(str(c) for c in cov))
    )
    # Publish a few times — AMCL may not have subscribed the instant we fire.
    subprocess.Popen(
        ["ros2", "topic", "pub", "--times", "3", "--rate", "2",
         INITIALPOSE_TOPIC, INITIALPOSE_TYPE, msg],
        start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return {"ok": True, "x": x, "y": y, "yaw": yaw}


@app.post("/api/nav/goal")
def api_goal(req: PoseReq) -> dict:
    if not _launch_alive():
        raise HTTPException(503, "nav2 stack not running yet — try again in a few seconds")
    if _is_navigating():
        raise HTTPException(409, "already navigating — stop first")
    if _wandering():
        raise HTTPException(409, "wander mode is running — stop it first")
    x, y, yaw = float(req.x), float(req.y), float(req.yaw)
    qz, qw = _quat(yaw)
    goal = (
        "{pose: {header: {frame_id: 'map'}, "
        "pose: {position: {x: %f, y: %f, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}}}" % (x, y, qz, qw)
    )
    # send_goal blocks until the goal finishes; own process group so Stop can SIGINT it.
    _nav["goal"] = subprocess.Popen(
        ["ros2", "action", "send_goal", NAV_ACTION, NAV_ACTION_TYPE, goal],
        start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    _nav.update(x=x, y=y, yaw=yaw, started=time.strftime("%Y-%m-%d %H:%M:%S"), result=None)
    return {"ok": True, "x": x, "y": y, "yaw": yaw}


@app.post("/api/nav/stop")
def api_stop() -> dict:
    cancelled = False
    try:
        subprocess.run(
            ["ros2", "service", "call", NAV_CANCEL_SRV, NAV_CANCEL_TYPE, NAV_CANCEL_ALL],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        cancelled = True
    except Exception:  # noqa: BLE001
        pass
    p = _nav["goal"]
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:  # noqa: BLE001
            pass
    _stop_wander()
    _stop_dog()
    _nav["result"] = "stopped"
    return {"ok": True, "cancelled": cancelled}


def _stop_wander():
    """SIGINT the wander node (it zeroes /cmd_vel on exit) and forget it."""
    p = _nav["wander"]
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            p.wait(timeout=2.0)
        except Exception:  # noqa: BLE001
            try:
                p.kill()
            except Exception:  # noqa: BLE001
                pass
    _nav["wander"] = None


@app.post("/api/nav/wander_start")
def api_wander_start() -> dict:
    """Start reactive wander (drive toward open space off /scan). Refuses while a
    goal is active — they'd both drive /cmd_vel. Needs /scan flowing + the bridge
    (both come up with the nav2 stack)."""
    if _is_navigating():
        raise HTTPException(409, "a goal is navigating — stop it first")
    if _wandering():
        return {"ok": True, "already": True}
    if not _fresh(_nav["scan"]):
        raise HTTPException(503, "no /scan yet — wander needs the laser (see diagnostics)")
    _nav["wander"] = subprocess.Popen(
        ["python3", WANDER_FILE], start_new_session=True)
    _nav["result"] = None
    return {"ok": True}


@app.post("/api/nav/wander_stop")
def api_wander_stop() -> dict:
    _stop_wander()
    _stop_dog()
    _nav["result"] = "stopped"
    return {"ok": True}


PAGE = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Go2 Map Nav</title>
<style>
  :root{--bg:#0c0e12;--panel:#14171d;--ink:#e7ebf0;--muted:#9aa6b2;--line:#262c36;
    --teal:#2dd4bf;--green:#67e480;--bad:#ff6b6b;--amber:#ffb429;}
  *{box-sizing:border-box;} body{margin:0;background:var(--bg);color:var(--ink);
    font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;line-height:1.5;}
  .wrap{max-width:980px;margin:0 auto;padding:24px 20px 60px;}
  h1{font-size:23px;font-weight:800;margin:0 0 2px;} .sub{color:var(--muted);font-size:14px;margin:0 0 18px;}
  .grid{display:flex;gap:18px;flex-wrap:wrap;align-items:flex-start;}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:18px;}
  .mapcard{padding:12px;}
  canvas{display:block;border-radius:8px;background:#0c0e12;cursor:crosshair;max-width:100%;touch-action:none;}
  label{display:block;font-size:12px;color:var(--muted);margin:0 0 5px;}
  input{width:90px;background:#0c0e12;border:1px solid var(--line);border-radius:9px;
    color:var(--ink);font-size:16px;padding:8px 10px;font-weight:700;}
  .row{display:flex;gap:8px;align-items:flex-end;flex-wrap:wrap;}
  .seg{display:inline-flex;border:1px solid var(--line);border-radius:10px;overflow:hidden;margin-bottom:12px;}
  .seg button{background:transparent;color:var(--muted);border:none;padding:9px 16px;font-weight:800;font-size:13px;cursor:pointer;}
  .seg button.on.init{background:var(--green);color:#0c0e12;}
  .seg button.on.goal{background:var(--teal);color:#0c0e12;}
  button{border:none;border-radius:10px;padding:11px 18px;font-weight:800;font-size:14px;cursor:pointer;}
  .send.init{background:var(--green);color:#0c0e12;} .send.goal{background:var(--teal);color:#0c0e12;}
  .send.amber{background:var(--amber);color:#0c0e12;}
  .stop{background:linear-gradient(92deg,#ff6b6b,#ff9a8b);color:#0c0e12;}
  button:disabled{opacity:.45;cursor:not-allowed;}
  .readout{font-size:18px;font-weight:800;color:var(--ink);padding:6px 0;min-width:74px;}
  h3{font-size:13px;margin:0 0 10px;letter-spacing:.04em;text-transform:uppercase;}
  h3.init{color:var(--green);} h3.goal{color:var(--teal);} h3.amber{color:var(--amber);}
  .status{margin-top:14px;font-size:13px;color:var(--muted);}
  .dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#444;margin-right:7px;vertical-align:middle;}
  .dot.on{background:var(--teal);box-shadow:0 0 8px var(--teal);animation:pulse 1s infinite;}
  .dot.warn{background:var(--amber);}
  @keyframes pulse{50%{opacity:.4;}}
  code{font-family:ui-monospace,Menlo,monospace;color:#cfe9e3;}
  .hint{font-size:12px;color:var(--muted);margin:8px 0 0;}
  .leg{font-size:12px;color:var(--muted);margin-top:8px;}
  .sw{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 5px 0 12px;vertical-align:middle;}
  .diag{list-style:none;margin:0;padding:0;counter-reset:step;}
  .diag li{display:flex;gap:11px;padding:11px 12px;border:1px solid var(--line);
    border-radius:11px;margin-bottom:8px;background:#0f1217;opacity:.5;}
  .diag li.ok{opacity:1;} .diag li.bad{opacity:1;border-color:var(--amber);background:#1c1813;}
  .diag .ic{font-size:16px;font-weight:900;line-height:1.5;width:18px;flex:none;text-align:center;}
  .diag li.ok .ic{color:var(--green);} .diag li.bad .ic{color:var(--amber);}
  .diag li.na .ic{color:var(--muted);}
  .diag .body{flex:1;min-width:0;}
  .diag .nm{font-weight:800;font-size:14px;}
  .diag .ht{font-size:12px;color:var(--muted);margin:3px 0 0;}
  .diag pre{margin:7px 0 0;padding:8px 10px;background:#0a0c10;border:1px solid var(--line);
    border-radius:8px;font-size:11.5px;color:#cfe9e3;overflow-x:auto;white-space:pre-wrap;}
</style></head><body><div class="wrap">
  <h1>🗺️ Go2 Map Navigation</h1>
  <p class="sub">Map-based nav2 on the real Go2 (utlidar odometry). Pick a mode, then
    <b>click+drag on the map</b> (click = position, drag = heading) — or type values — to set the
    <b style="color:var(--green)">initial pose</b> (tells AMCL where the dog is) and the
    <b style="color:var(--teal)">goal</b> (where to walk, in absolute map metres).</p>

  <div class="grid">
    <div class="card mapcard">
      <div class="seg">
        <button id="mInit" class="on init" onclick="setMode('init')">◎ Initial pose</button>
        <button id="mGoal" class="goal" onclick="setMode('goal')">▸ Goal</button>
      </div>
      <canvas id="cv" width="600" height="400"></canvas>
      <div class="leg">
        <span class="sw" style="background:var(--green)"></span>initial
        <span class="sw" style="background:var(--teal)"></span>goal
        <span class="sw" style="background:var(--amber)"></span>robot (live)
      </div>
      <div class="status" id="status"><span class="dot" id="dot"></span>—</div>
    </div>

    <div style="display:flex;flex-direction:column;gap:18px;flex:1;min-width:230px;">
      <div class="card">
        <h3 class="init">◎ Initial pose</h3>
        <div class="row">
          <div><label>x (m)</label><input id="ix" type="number" step="0.1" value="0.0"></div>
          <div><label>y (m)</label><input id="iy" type="number" step="0.1" value="0.0"></div>
          <div><label>yaw (rad)</label><input id="iyaw" type="number" step="0.1" value="0.0"></div>
        </div>
        <div class="row" style="margin-top:12px">
          <button class="send init" onclick="sendInit()">Set initial pose</button>
        </div>
        <p class="hint">Publishes <code>/initialpose</code> → AMCL relocalises here.</p>
      </div>

      <div class="card">
        <h3 class="goal">▸ Goal</h3>
        <div class="row">
          <div><label>x (m)</label><input id="gx" type="number" step="0.1" value="1.0"></div>
          <div><label>y (m)</label><input id="gy" type="number" step="0.1" value="0.0"></div>
          <div><label>yaw (rad)</label><input id="gyaw" type="number" step="0.1" value="0.0"></div>
        </div>
        <div class="row" style="margin-top:12px">
          <button class="send goal" id="goBtn" onclick="sendGoal()">Send goal ▸</button>
          <button class="stop" onclick="stop()">■ Stop</button>
        </div>
        <p class="hint">Sends <code>NavigateToPose</code> in the <code>map</code> frame (absolute).</p>
      </div>

      <div class="card">
        <h3 class="amber">⟳ Odom (utlidar)</h3>
        <div class="row" style="gap:18px;font-variant-numeric:tabular-nums">
          <div><label>x (m)</label><div class="readout" id="ox">—</div></div>
          <div><label>y (m)</label><div class="readout" id="oy">—</div></div>
          <div><label>yaw (rad)</label><div class="readout" id="oyaw">—</div></div>
        </div>
        <div class="row" style="margin-top:12px">
          <button class="send amber" onclick="resetOdom()">Reset to 0 here</button>
          <button onclick="unresetOdom()" style="background:var(--line);color:var(--ink)">Show raw</button>
        </div>
        <p class="hint" id="ohint">Live <code>odom→base_link</code> from <code>/utlidar/robot_odom</code>.</p>
      </div>

      <div class="card">
        <h3 class="goal">🧭 Wander (reactive)</h3>
        <div class="row">
          <button class="send goal" id="wanderBtn" onclick="wanderStart()">Start wander</button>
          <button class="stop" onclick="wanderStop()">■ Stop</button>
        </div>
        <p class="hint" id="whint">Drives toward open space off <code>/scan</code> (no map / planner).
          Runs until you stop it. Can't run while a goal is navigating.</p>
      </div>
    </div>
  </div>

  <div class="card" style="margin-top:18px">
    <h3 class="amber">✓ Nav chain — step by step</h3>
    <p class="hint" style="margin:0 0 12px">Each link depends on the one above it.
      Fix the <b style="color:var(--amber)">highlighted</b> step first, then the rest
      go green on their own. Greyed steps are waiting on an earlier link (or only
      apply while navigating).</p>
    <ol id="diag" class="diag"></ol>
  </div>
</div>
<script>
  const cv=document.getElementById("cv"), ctx=cv.getContext("2d");
  let META=null, IMG=null, scale=1, mode="init";
  let initPose=null, goalPose=null, robot=null, drag=null;

  async function j(u,m,b){const o={method:m||"GET"};
    if(b){o.headers={"Content-Type":"application/json"};o.body=JSON.stringify(b);}
    const r=await fetch(u,o);return r.json().catch(()=>({}));}

  function setMode(x){mode=x;
    document.getElementById("mInit").className="init"+(x==="init"?" on":"");
    document.getElementById("mGoal").className="goal"+(x==="goal"?" on":"");}

  // world<->image-pixel<->canvas mappings (origin = bottom-left of the image)
  function worldToCanvas(wx,wy){
    const ipx=(wx-META.origin[0])/META.resolution;
    const ipy=META.height-(wy-META.origin[1])/META.resolution;
    return [ipx*scale, ipy*scale];}
  function canvasToWorld(cx,cy){
    const ipx=cx/scale, ipy=cy/scale;
    return [META.origin[0]+ipx*META.resolution,
            META.origin[1]+(META.height-ipy)*META.resolution];}

  function arrow(cx,cy,yaw,color){
    const len=22;
    ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=3;
    ctx.beginPath();ctx.arc(cx,cy,6,0,7);ctx.fill();
    const ex=cx+len*Math.cos(-yaw), ey=cy+len*Math.sin(-yaw); // canvas y is down
    ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(ex,ey);ctx.stroke();
    ctx.beginPath();ctx.moveTo(ex,ey);
    ctx.lineTo(ex-9*Math.cos(-yaw-0.5),ey-9*Math.sin(-yaw-0.5));
    ctx.lineTo(ex-9*Math.cos(-yaw+0.5),ey-9*Math.sin(-yaw+0.5));
    ctx.closePath();ctx.fill();}

  function redraw(){
    if(!IMG)return;
    ctx.clearRect(0,0,cv.width,cv.height);
    ctx.drawImage(IMG,0,0,cv.width,cv.height);
    if(initPose){const[x,y]=worldToCanvas(initPose.x,initPose.y);arrow(x,y,initPose.yaw,"#67e480");}
    if(goalPose){const[x,y]=worldToCanvas(goalPose.x,goalPose.y);arrow(x,y,goalPose.yaw,"#2dd4bf");}
    if(robot){const[x,y]=worldToCanvas(robot.x,robot.y);arrow(x,y,robot.yaw,"#ffb429");}
    if(drag){const c=mode==="init"?"#67e480":"#2dd4bf";arrow(drag.cx,drag.cy,drag.yaw,c);}
  }

  function fillInputs(p,which){
    document.getElementById(which+"x").value=p.x.toFixed(2);
    document.getElementById(which+"y").value=p.y.toFixed(2);
    document.getElementById(which+"yaw").value=p.yaw.toFixed(2);}

  function evtXY(e){const r=cv.getBoundingClientRect();
    const t=e.touches?e.touches[0]:e;
    return [(t.clientX-r.left)*(cv.width/r.width),(t.clientY-r.top)*(cv.height/r.height)];}

  function down(e){e.preventDefault();const[cx,cy]=evtXY(e);drag={cx,cy,yaw:0};redraw();}
  function move(e){if(!drag)return;e.preventDefault();const[cx,cy]=evtXY(e);
    drag.yaw=Math.atan2(-(cy-drag.cy),cx-drag.cx);redraw();}
  function up(e){if(!drag)return;e.preventDefault();
    const[wx,wy]=canvasToWorld(drag.cx,drag.cy);
    const p={x:wx,y:wy,yaw:drag.yaw};
    if(mode==="init"){initPose=p;fillInputs(p,"i");}else{goalPose=p;fillInputs(p,"g");}
    drag=null;redraw();}

  cv.addEventListener("mousedown",down);cv.addEventListener("mousemove",move);
  window.addEventListener("mouseup",up);
  cv.addEventListener("touchstart",down,{passive:false});
  cv.addEventListener("touchmove",move,{passive:false});
  cv.addEventListener("touchend",up,{passive:false});

  function readInputs(which){return{
    x:parseFloat(document.getElementById(which+"x").value)||0,
    y:parseFloat(document.getElementById(which+"y").value)||0,
    yaw:parseFloat(document.getElementById(which+"yaw").value)||0};}

  async function sendInit(){const p=readInputs("i");initPose=p;redraw();
    const d=await j("/api/nav/initialpose","POST",p);
    if(d&&d.detail)setStatus("warn","error: "+d.detail);else setStatus("","initial pose sent → AMCL relocalising");}
  async function sendGoal(){const p=readInputs("g");goalPose=p;redraw();
    const btn=document.getElementById("goBtn");btn.disabled=true;
    try{const d=await j("/api/nav/goal","POST",p);
      if(d&&d.detail)setStatus("warn","error: "+d.detail);}
    finally{setTimeout(()=>{btn.disabled=false;refresh();},400);}}
  async function stop(){await j("/api/nav/stop","POST");setTimeout(refresh,300);}
  async function resetOdom(){await j("/api/nav/odom_reset","POST");refresh();}
  async function unresetOdom(){await j("/api/nav/odom_unreset","POST");refresh();}
  async function wanderStart(){const d=await j("/api/nav/wander_start","POST");
    if(d&&d.detail)setStatus("warn","error: "+d.detail);setTimeout(refresh,300);}
  async function wanderStop(){await j("/api/nav/wander_stop","POST");setTimeout(refresh,300);}

  function setStatus(cls,txt){
    document.getElementById("status").innerHTML=
      "<span class='dot"+(cls?" "+cls:"")+"'></span>"+txt;}

  // Walk the nav chain top-to-bottom: green = up, amber = the first broken link
  // (shows how to fix it), grey = waiting on an earlier link or nav-only.
  function renderDiag(d){
    const ol=document.getElementById("diag");
    if(!d||!d.steps){return;}
    ol.innerHTML=d.steps.map((s,i)=>{
      let cls,ic;
      if(s.ok===true){cls="ok";ic="✓";}
      else if(i===d.blocker){cls="bad";ic="✕";}
      else{cls="na";ic="○";}
      const detail = cls==="bad"
        ? "<p class='ht'>"+s.hint+"</p><pre>"+s.cmd+"</pre>"
        : (s.ok===null ? "<p class='ht'>only checked while navigating</p>" : "");
      return "<li class='"+cls+"'><div class='ic'>"+ic+"</div>"+
        "<div class='body'><div class='nm'>"+s.name+"</div>"+detail+"</div></li>";
    }).join("");
  }

  async function refresh(){
    const s=await j("/api/nav/status");
    robot=s.pose?{x:s.pose.x,y:s.pose.y,yaw:s.pose.yaw}:null;redraw();
    const o=s.odom;
    document.getElementById("ox").textContent  =o?o.x.toFixed(2):"—";
    document.getElementById("oy").textContent  =o?o.y.toFixed(2):"—";
    document.getElementById("oyaw").textContent=o?o.yaw.toFixed(3):"—";
    document.getElementById("ohint").innerHTML=s.odom_tared
      ?"Tared — showing displacement since reset (display only; TF/AMCL untouched)."
      :"Live <code>odom→base_link</code> from <code>/utlidar/robot_odom</code>.";
    renderDiag(await j("/api/nav/diag"));
    const wb=document.getElementById("wanderBtn");
    wb.textContent=s.wandering?"Wandering…":"Start wander";wb.disabled=s.wandering;
    document.getElementById("goBtn").disabled=s.wandering;
    if(!s.nav2_up){setStatus("warn","nav2 starting up…");return;}
    let m=robot?"":"  (no robot pose yet — set an initial pose)";
    if(s.wandering)setStatus("on","wandering → driving toward open space (Stop to halt)");
    else if(s.navigating)setStatus("on","navigating → ("+s.goal.x.toFixed(2)+", "+s.goal.y.toFixed(2)+") m"+m);
    else setStatus("","idle"+(s.result?" — last: "+s.result:"")+" · nav2 ready"+m);}

  async function boot(){
    META=await j("/api/map/meta");
    if(!META||!META.ok){setStatus("warn","map not loaded — check MAP_YAML");return;}
    // size the canvas to the map aspect, capped at 600px wide
    const W=Math.min(600,META.width), H=Math.round(W*META.height/META.width);
    cv.width=W;cv.height=H;scale=W/META.width;
    IMG=new Image();IMG.onload=redraw;IMG.src="/api/map/image.png?ts="+Date.now();
    refresh();setInterval(refresh,1000);
  }
  boot();
</script></body></html>"""


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return PAGE


def _bootstrap():
    # Give DDS discovery a moment so go2_odom/scan find the robot, then bring up
    # the map-based nav2 stack and start tailing the robot pose.
    time.sleep(4)
    _start_nav2()
    threading.Thread(target=_tf_poller, args=("map", "base_link", "pose"),
                     daemon=True).start()
    threading.Thread(target=_tf_poller, args=("odom", "base_link", "odom"),
                     daemon=True).start()
    # per-link liveness for the diagnostics panel
    threading.Thread(target=_liveness_poller, args=("/scan", "scan", "header.stamp"),
                     daemon=True).start()
    threading.Thread(target=_liveness_poller, args=("/cmd_vel", "cmdvel", "linear.x"),
                     daemon=True).start()


if __name__ == "__main__":
    threading.Thread(target=_bootstrap, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="info", access_log=False)
