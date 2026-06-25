"""Go2 nav2 walker — a tiny web UI + JSON API that drives nav2 over the ros2 CLI.

Mapless "walk N metres": the server launches the nav2 stack (nav2.launch.py) as
a subprocess, then drives it purely with `ros2` CLI calls — exactly the pattern
recorder/server.py uses for `ros2 bag record`. No rclpy in this process.

  GET  /                  control UI (enter x/y, Walk, Stop)
  GET  /api/nav/status    nav2 up? + currently walking? + last result
  POST /api/nav/walk      {"x": X, "y": Y}  -> ros2 action send_goal navigate_to_pose
  POST /api/nav/stop      cancel the goal + StopMove the dog

"Walk to (x, y)" sends a NavigateToPose goal at the point (x forward, y left)
relative to the robot's CURRENT pose, in the **base_link** frame — so nav2 (whose
costmaps live in `odom`) plans from wherever the robot is right now, with no map
and no fixed origin. (x, 0) is the old "walk x metres". The goal's final heading
faces the target direction (yaw = atan2(y, x)). The controller's /cmd_vel is
turned into Go2 sport Move commands by cmd_vel_to_sport.py (launched inside
nav2.launch.py).
"""
import math
import os
import signal
import subprocess
import time

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

PORT = int(os.environ.get("PORT", "7100"))
HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_FILE = os.path.join(HERE, "nav2.launch.py")

NAV_ACTION = "/navigate_to_pose"
NAV_ACTION_TYPE = "nav2_msgs/action/NavigateToPose"
# Action cancel service + the "cancel all goals" request (zero goal_id + zero
# stamp). Calling the server directly is authoritative — it doesn't depend on the
# send_goal CLI client cancelling on Ctrl-C (which varies across Humble builds).
NAV_CANCEL_SRV = "/navigate_to_pose/_action/cancel_goal"
NAV_CANCEL_TYPE = "action_msgs/srv/CancelGoal"
NAV_CANCEL_ALL = "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}"
SPORT_TOPIC = "/api/sport/request"
SPORT_TYPE = "unitree_api/msg/Request"
SPORT_API_ID_STOP = 1003

MAX_RANGE = 10.0  # sanity clamp on the goal distance (metres) for a single command

app = FastAPI(title="go2-nav2-walker")

# Long-lived nav2 stack + the single in-flight walk goal.
_nav = {"launch": None, "walk": None, "x": None, "y": None, "started": None, "result": None}


class WalkReq(BaseModel):
    x: float = 0.0  # metres forward (base_link +x)
    y: float = 0.0  # metres left    (base_link +y)


def _launch_alive() -> bool:
    p = _nav["launch"]
    return p is not None and p.poll() is None


def _is_walking() -> bool:
    p = _nav["walk"]
    return p is not None and p.poll() is None


def _start_nav2():
    """Spawn `ros2 launch nav2.launch.py` in its own process group so the whole
    nav2 tree can be signalled/torn down together."""
    if _launch_alive():
        return
    _nav["launch"] = subprocess.Popen(
        ["ros2", "launch", LAUNCH_FILE],
        start_new_session=True,
    )
    print(f"[nav2] launched {LAUNCH_FILE} (pid={_nav['launch'].pid})", flush=True)


def _stop_dog():
    """Publish a one-shot StopMove so the dog halts immediately, independent of
    whether the action cancel propagated through the controller yet."""
    msg = "{header: {identity: {api_id: %d}}}" % SPORT_API_ID_STOP
    try:
        subprocess.run(
            ["ros2", "topic", "pub", "--once", SPORT_TOPIC, SPORT_TYPE, msg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8,
        )
    except Exception:  # noqa: BLE001
        pass


@app.get("/api/nav/status")
def api_status() -> dict:
    p = _nav["walk"]
    if p is not None and p.poll() is not None and _nav["result"] is None:
        _nav["result"] = "ok" if p.returncode == 0 else f"exit {p.returncode}"
    return {
        "nav2_up": _launch_alive(),
        "walking": _is_walking(),
        "x": _nav["x"],
        "y": _nav["y"],
        "started": _nav["started"],
        "result": _nav["result"],
    }


@app.post("/api/nav/walk")
def api_walk(req: WalkReq) -> dict:
    if not _launch_alive():
        raise HTTPException(503, "nav2 stack not running yet — try again in a few seconds")
    if _is_walking():
        raise HTTPException(409, "already walking — stop first")
    x, y = float(req.x), float(req.y)
    dist = math.hypot(x, y)
    if dist == 0.0 or dist > MAX_RANGE:
        raise HTTPException(
            400, f"goal must be non-zero and within {MAX_RANGE} m of the robot (got {dist:.2f} m)"
        )

    # Goal at (x forward, y left) in base_link, facing the target direction
    # (yaw = atan2(y, x)). nav2 transforms it into the odom costmap frame at goal
    # time, so it's relative to wherever the dog is right now.
    yaw = math.atan2(y, x)
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    goal = (
        "{pose: {header: {frame_id: base_link}, "
        "pose: {position: {x: %f, y: %f, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}}}" % (x, y, qz, qw)
    )
    # send_goal blocks until the goal finishes; run it in its own process group
    # so Stop can SIGINT it (the ros2 CLI cancels the goal on Ctrl-C).
    _nav["walk"] = subprocess.Popen(
        ["ros2", "action", "send_goal", NAV_ACTION, NAV_ACTION_TYPE, goal],
        start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    _nav.update(x=x, y=y, started=time.strftime("%Y-%m-%d %H:%M:%S"), result=None)
    return {"ok": True, "x": x, "y": y}


@app.post("/api/nav/stop")
def api_stop() -> dict:
    # 1. Authoritatively cancel every active goal on the action server itself.
    cancelled = False
    try:
        subprocess.run(
            ["ros2", "service", "call", NAV_CANCEL_SRV, NAV_CANCEL_TYPE, NAV_CANCEL_ALL],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10,
        )
        cancelled = True
    except Exception:  # noqa: BLE001
        pass
    # 2. Tear down the send_goal CLI client so it stops waiting on the result.
    p = _nav["walk"]
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:  # noqa: BLE001
            pass
    # 3. Belt-and-suspenders: halt the dog immediately, regardless of the above.
    _stop_dog()
    _nav["result"] = "stopped"
    return {"ok": True, "cancelled": cancelled}


PAGE = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Go2 nav2 Walker</title>
<style>
  :root{--bg:#0c0e12;--panel:#14171d;--ink:#e7ebf0;--muted:#9aa6b2;--line:#262c36;
    --teal:#2dd4bf;--bad:#ff6b6b;}
  *{box-sizing:border-box;} body{margin:0;background:var(--bg);color:var(--ink);
    font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;line-height:1.5;}
  .wrap{max-width:680px;margin:0 auto;padding:28px 20px 60px;}
  h1{font-size:23px;font-weight:800;margin:0 0 2px;} .sub{color:var(--muted);font-size:14px;margin:0 0 22px;}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:20px;margin-bottom:18px;}
  label{display:block;font-size:13px;color:var(--muted);margin-bottom:6px;}
  input{width:160px;background:#0c0e12;border:1px solid var(--line);border-radius:10px;
    color:var(--ink);font-size:20px;padding:10px 12px;font-weight:700;}
  .row{display:flex;gap:10px;align-items:flex-end;flex-wrap:wrap;}
  button{border:none;border-radius:10px;padding:13px 22px;font-weight:800;font-size:15px;cursor:pointer;}
  .go{background:linear-gradient(92deg,var(--teal),#7af0e0);color:#0c0e12;}
  .stop{background:linear-gradient(92deg,#ff6b6b,#ff9a8b);color:#0c0e12;}
  button:disabled{opacity:.45;cursor:not-allowed;}
  .status{margin-top:16px;font-size:14px;color:var(--muted);}
  .dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#444;margin-right:7px;vertical-align:middle;}
  .dot.on{background:var(--teal);box-shadow:0 0 8px var(--teal);animation:pulse 1s infinite;}
  .dot.warn{background:#ffb429;}
  @keyframes pulse{50%{opacity:.4;}}
  code{font-family:ui-monospace,Menlo,monospace;color:#cfe9e3;}
</style></head><body><div class="wrap">
  <h1>🐕 Go2 nav2 Walker</h1>
  <p class="sub">Mapless nav2 — give a point <code>(x, y)</code> relative to the dog
    now (x forward, y left) and it plans there, avoiding what the L1 sees.
    Negative values go backward / right.</p>

  <div class="card">
    <div class="row">
      <div>
        <label for="x">x — forward (m)</label>
        <input id="x" type="number" step="0.25" value="1.0">
      </div>
      <div>
        <label for="y">y — left (m)</label>
        <input id="y" type="number" step="0.25" value="0.0">
      </div>
      <button class="go" id="goBtn" onclick="walk()">Walk ▸</button>
      <button class="stop" id="stopBtn" onclick="stop()">■ Stop</button>
    </div>
    <div class="status" id="status"><span class="dot" id="dot"></span>—</div>
  </div>
</div>
<script>
  async function j(u,m,b){const o={method:m||"GET"};
    if(b){o.headers={"Content-Type":"application/json"};o.body=JSON.stringify(b);}
    const r=await fetch(u,o);return r.json().catch(()=>({}));}
  async function walk(){
    const x=parseFloat(document.getElementById("x").value);
    const y=parseFloat(document.getElementById("y").value);
    const btn=document.getElementById("goBtn");btn.disabled=true;
    try{const d=await j("/api/nav/walk","POST",{x:x,y:y});
      if(d&&d.detail){setStatus("warn","error: "+d.detail);}}
    finally{setTimeout(()=>{btn.disabled=false;refresh();},400);}
  }
  async function stop(){await j("/api/nav/stop","POST");setTimeout(refresh,300);}
  function setStatus(cls,txt){
    document.getElementById("dot").className="dot"+(cls?" "+cls:"");
    document.getElementById("status").innerHTML=
      "<span class='dot"+(cls?" "+cls:"")+"'></span>"+txt;
  }
  async function refresh(){
    const s=await j("/api/nav/status");
    if(!s.nav2_up){setStatus("warn","nav2 starting up…");return;}
    if(s.walking){setStatus("on","walking to ("+s.x+", "+s.y+") m (since "+s.started+")");}
    else{setStatus("","idle"+(s.result?" — last: "+s.result:"")+" · nav2 ready");}
  }
  refresh();setInterval(refresh,1500);
</script></body></html>"""


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return PAGE


def _bootstrap():
    # Give DDS discovery a moment so go2_odom/scan find the robot, then bring up
    # the nav2 stack. Walks are accepted once the action server is alive.
    time.sleep(4)
    _start_nav2()


if __name__ == "__main__":
    import threading
    threading.Thread(target=_bootstrap, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="info", access_log=False)
