#!/usr/bin/env python3
"""server.py — tiny web UI for the reactive WANDER mode (no nav2, no map).

Stands up the sensing+actuation pipeline (pipeline.launch.py: static TF +
pointcloud_to_laserscan + cmd_vel_to_sport) and serves a one-button page:

  * Start wander  -> spawn wander.py (reads /scan, steers toward open space,
                     publishes /cmd_vel; the bridge relays it to the dog).
  * Stop          -> SIGINT wander.py (it zeroes /cmd_vel on exit) + StopMove.

It also tails /scan and /cmd_vel liveness so the page can show whether the laser
is feeding and whether wander is actually commanding velocity. Everything is
driven over the ros2 CLI — no rclpy in this process.

Endpoints
---------
  GET  /                      the page
  GET  /api/status            {pipeline_up, wandering, scan_ok, cmd_ok}
  POST /api/wander/start      start the wander node
  POST /api/wander/stop       stop it + StopMove

Env: GO2_IP (DDS NIC, used by entrypoint), PORT (7100), plus wander tunables
forwarded to wander.py: WANDER_MAX_VX, WANDER_MAX_WZ, WANDER_STOP_DIST.
"""
import os
import signal
import subprocess
import threading
import time

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse

PORT = int(os.environ.get("PORT", "7100"))
HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_FILE = os.path.join(HERE, "pipeline.launch.py")
WANDER_FILE = os.path.join(HERE, "wander.py")

SPORT_TOPIC = "/api/sport/request"
SPORT_TYPE = "unitree_api/msg/Request"
SPORT_API_ID_STOP = 1003

app = FastAPI(title="go2-wander")

# Long-lived pipeline launch + the wander subprocess + liveness stamps.
_state = {"launch": None, "wander": None, "scan": None, "cmd": None}


# --------------------------------------------------------------------------- procs
def _alive(key: str) -> bool:
    p = _state[key]
    return p is not None and p.poll() is None


def _start_pipeline():
    if _alive("launch"):
        return
    _state["launch"] = subprocess.Popen(
        ["ros2", "launch", LAUNCH_FILE], start_new_session=True)
    print(f"[wander] pipeline up (pid={_state['launch'].pid})", flush=True)


def _stop_dog():
    """One-shot StopMove so a halted wander leaves the dog holding position."""
    msg = "{header: {identity: {api_id: %d}}}" % SPORT_API_ID_STOP
    try:
        subprocess.run(
            ["ros2", "topic", "pub", "--once", SPORT_TOPIC, SPORT_TYPE, msg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8)
    except Exception:  # noqa: BLE001
        pass


def _stop_wander():
    p = _state["wander"]
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            p.wait(timeout=2.0)
        except Exception:  # noqa: BLE001
            try:
                p.kill()
            except Exception:  # noqa: BLE001
                pass
    _state["wander"] = None


def _fresh(d, ttl: float = 4.0) -> bool:
    return bool(d) and (time.time() - d["t"] < ttl)


def _liveness_poller(topic: str, key: str, field: str):
    """Tail `ros2 topic echo <topic> --field <field>` and stamp liveness."""
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
                _state[key] = {"t": time.time()}
        proc.wait()
        time.sleep(2)


# ----------------------------------------------------------------------------- API
@app.get("/api/status")
def api_status() -> dict:
    return {
        "pipeline_up": _alive("launch"),
        "wandering": _alive("wander"),
        "scan_ok": _fresh(_state["scan"]),
        "cmd_ok": _fresh(_state["cmd"]),
    }


@app.post("/api/wander/start")
def api_wander_start() -> dict:
    if _alive("wander"):
        return {"ok": True, "already": True}
    if not _fresh(_state["scan"]):
        raise HTTPException(503, "no /scan yet — laser pipeline not ready")
    env = dict(os.environ)
    params = []
    for env_key, pname in (("WANDER_MAX_VX", "max_vx"), ("WANDER_MAX_WZ", "max_wz"),
                           ("WANDER_STOP_DIST", "stop_dist")):
        if env.get(env_key):
            params += ["-p", f"{pname}:={env[env_key]}"]
    cmd = ["python3", WANDER_FILE]
    if params:
        cmd += ["--ros-args", *params]
    _state["wander"] = subprocess.Popen(cmd, start_new_session=True)
    return {"ok": True}


@app.post("/api/wander/stop")
def api_wander_stop() -> dict:
    _stop_wander()
    _stop_dog()
    return {"ok": True}


PAGE = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Go2 Wander</title>
<style>
  :root{--bg:#0c0e12;--panel:#14171d;--ink:#e7ebf0;--muted:#9aa6b2;--line:#262c36;
    --teal:#2dd4bf;--green:#67e480;--bad:#ff6b6b;--amber:#ffb429;}
  *{box-sizing:border-box;}
  body{margin:0;background:var(--bg);color:var(--ink);
    font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;line-height:1.5;}
  .wrap{max-width:560px;margin:0 auto;padding:36px 20px 60px;}
  h1{font-size:23px;font-weight:800;margin:0 0 2px;}
  .sub{color:var(--muted);font-size:14px;margin:0 0 22px;}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:20px;}
  .row{display:flex;gap:10px;margin:0 0 18px;}
  button{flex:1;border:none;border-radius:11px;padding:15px 18px;font-weight:800;font-size:16px;cursor:pointer;}
  .go{background:var(--teal);color:#0c0e12;} .stop{background:linear-gradient(92deg,#ff6b6b,#ff9a8b);color:#0c0e12;}
  button:disabled{opacity:.45;cursor:not-allowed;}
  ul{list-style:none;margin:0;padding:0;}
  li{display:flex;align-items:center;gap:10px;padding:9px 0;border-top:1px solid var(--line);font-size:14px;}
  li:first-child{border-top:none;}
  .dot{width:11px;height:11px;border-radius:50%;background:#444;flex:none;}
  .dot.ok{background:var(--green);} .dot.bad{background:var(--bad);} .dot.on{background:var(--amber);animation:pulse 1s infinite;}
  @keyframes pulse{50%{opacity:.4;}}
  .nm{flex:1;} .val{color:var(--muted);}
  .status{margin-top:16px;font-size:13px;color:var(--muted);}
  code{font-family:ui-monospace,Menlo,monospace;color:#cfe9e3;}
</style></head><body><div class="wrap">
  <h1>🧭 Go2 Wander</h1>
  <p class="sub">Reactive "drive toward open space" off the L1 <code>/scan</code> —
    no map, no planner. The dog keeps moving (and rotates to escape dead-ends)
    until you press <b>Stop</b>.</p>
  <div class="card">
    <div class="row">
      <button class="go" id="goBtn" onclick="start()">Start wander</button>
      <button class="stop" onclick="stop()">■ Stop</button>
    </div>
    <ul>
      <li><span class="dot" id="dPipe"></span><span class="nm">Pipeline (scan + bridge)</span><span class="val" id="vPipe">—</span></li>
      <li><span class="dot" id="dScan"></span><span class="nm">Laser <code>/scan</code></span><span class="val" id="vScan">—</span></li>
      <li><span class="dot" id="dCmd"></span><span class="nm">Velocity <code>/cmd_vel</code></span><span class="val" id="vCmd">—</span></li>
      <li><span class="dot" id="dWan"></span><span class="nm">Wander node</span><span class="val" id="vWan">—</span></li>
    </ul>
    <div class="status" id="status">—</div>
  </div>
<script>
  async function j(u,m){const r=await fetch(u,{method:m||"GET"});return r.json().catch(()=>({}));}
  function set(id,ok,txt,on){
    const d=document.getElementById("d"+id), v=document.getElementById("v"+id);
    d.className="dot"+(on?" on":(ok?" ok":" bad"));v.textContent=txt;}
  async function start(){const d=await j("/api/wander/start","POST");
    if(d&&d.detail)document.getElementById("status").textContent="error: "+d.detail;
    setTimeout(refresh,300);}
  async function stop(){await j("/api/wander/stop","POST");setTimeout(refresh,300);}
  async function refresh(){
    const s=await j("/api/status");
    set("Pipe",s.pipeline_up,s.pipeline_up?"up":"starting…");
    set("Scan",s.scan_ok,s.scan_ok?"flowing":"no data");
    set("Cmd",s.cmd_ok,s.cmd_ok?"flowing":"idle");
    set("Wan",false,s.wandering?"running":"stopped",s.wandering);
    document.getElementById("goBtn").disabled=s.wandering||!s.scan_ok;
    document.getElementById("status").textContent=
      s.wandering?"wandering → driving toward open space (Stop to halt)"
      :(s.scan_ok?"ready — press Start":"waiting for /scan…");
  }
  refresh();setInterval(refresh,1000);
</script></div></body></html>"""


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return PAGE


def _bootstrap():
    time.sleep(4)  # let DDS discovery find the robot
    _start_pipeline()
    threading.Thread(target=_liveness_poller, args=("/scan", "scan", "header.stamp"),
                     daemon=True).start()
    threading.Thread(target=_liveness_poller, args=("/cmd_vel", "cmd", "linear.x"),
                     daemon=True).start()


if __name__ == "__main__":
    threading.Thread(target=_bootstrap, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="info", access_log=False)
