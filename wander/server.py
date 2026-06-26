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

# Live-tunable wander params (defaults match wander.py). The UI edits these; we
# `ros2 param set /wander ...` while running (immediate) and pass them at start.
PARAM_KEYS = ("max_vx", "max_wz", "stop_dist", "slow_dist", "front_deg",
              "steer_deg", "k_steer")
_DEFAULT_PARAMS = {"max_vx": 0.30, "max_wz": 0.7, "stop_dist": 0.6,
                   "slow_dist": 1.5, "front_deg": 30.0, "steer_deg": 90.0,
                   "k_steer": 1.2}

# Long-lived pipeline launch + the wander subprocess + liveness stamps.
_state = {"launch": None, "wander": None, "scan": None, "cmd": None,
          "sectors": None,  # {"front":m,"left":m,"right":m,"t":epoch} from scan_debug
          "params": dict(_DEFAULT_PARAMS)}

# Optional env seeds for the headline params (documented in the Dockerfile).
for _env, _k in (("WANDER_MAX_VX", "max_vx"), ("WANDER_MAX_WZ", "max_wz"),
                 ("WANDER_STOP_DIST", "stop_dist")):
    if os.environ.get(_env):
        try:
            _state["params"][_k] = float(os.environ[_env])
        except ValueError:
            pass


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


def _sector_poller():
    """Tail /wander/debug (std_msgs/String "front,left,right") from scan_debug and
    keep the latest sector averages for the UI debugger."""
    while True:
        try:
            proc = subprocess.Popen(
                ["ros2", "topic", "echo", "/wander/debug", "--field", "data"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                start_new_session=True)
        except Exception:  # noqa: BLE001
            time.sleep(3)
            continue
        for line in proc.stdout:
            parts = line.strip().strip('"').split(",")
            if len(parts) == 3:
                try:
                    f, l, r = (float(p) for p in parts)
                except ValueError:
                    continue
                _state["sectors"] = {"front": f, "left": l, "right": r, "t": time.time()}
        proc.wait()
        time.sleep(2)


# ----------------------------------------------------------------------------- API
@app.get("/api/status")
def api_status() -> dict:
    sec = _state["sectors"]
    return {
        "pipeline_up": _alive("launch"),
        "wandering": _alive("wander"),
        "scan_ok": _fresh(_state["scan"]),
        "cmd_ok": _fresh(_state["cmd"]),
        "sectors": sec if _fresh(sec) else None,
        "params": _state["params"],
    }


@app.post("/api/wander/start")
def api_wander_start() -> dict:
    if _alive("wander"):
        return {"ok": True, "already": True}
    if not _fresh(_state["scan"]):
        raise HTTPException(503, "no /scan yet — laser pipeline not ready")
    # Launch with the UI's current params so a restart keeps your tuning.
    args = []
    for k in PARAM_KEYS:
        args += ["-p", f"{k}:={_state['params'][k]}"]
    _state["wander"] = subprocess.Popen(
        ["python3", WANDER_FILE, "--ros-args", *args], start_new_session=True)
    return {"ok": True}


@app.post("/api/wander/params")
def api_wander_params(body: dict) -> dict:
    """Update wander params from the UI. Applies IMMEDIATELY to a running node via
    `ros2 param set /wander ...` (the node's control loop reads them each tick);
    always stored so the next start uses them too. Unknown keys are ignored."""
    updated = {}
    for k, v in body.items():
        if k not in PARAM_KEYS:
            continue
        try:
            updated[k] = float(v)
        except (TypeError, ValueError):
            raise HTTPException(400, f"{k} must be a number")
    _state["params"].update(updated)
    applied = False
    if _alive("wander") and updated:
        for k, v in updated.items():
            try:
                subprocess.run(["ros2", "param", "set", "/wander", k, repr(v)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               timeout=6)
                applied = True
            except Exception:  # noqa: BLE001
                pass
    return {"ok": True, "params": _state["params"], "applied_live": applied}


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
  .sectors{display:flex;gap:10px;margin:16px 0 0;}
  .sec{flex:1;text-align:center;border:1px solid var(--line);border-radius:11px;padding:11px 6px;background:#0f1217;}
  .sec.front{border-color:var(--teal);}
  .sec label{display:block;font-size:11px;color:var(--muted);margin:0 0 4px;letter-spacing:.03em;}
  .sval{font-size:20px;font-weight:800;font-variant-numeric:tabular-nums;}
  .seclab{font-size:11px;color:var(--muted);margin:8px 0 0;}
  .ttl{font-weight:800;font-size:14px;margin:0 0 12px;}
  .params{display:grid;grid-template-columns:repeat(2,1fr);gap:10px 14px;margin:0 0 14px;}
  .params .p{display:flex;flex-direction:column;}
  .params label{font-size:11px;color:var(--muted);margin:0 0 3px;}
  .params input{background:#0c0e12;border:1px solid var(--line);border-radius:9px;color:var(--ink);
    font-size:15px;padding:8px 10px;font-weight:700;font-variant-numeric:tabular-nums;width:100%;}
  .apply{background:var(--green);color:#0c0e12;width:100%;}
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
    <div class="sectors">
      <div class="sec"><label>◀ LEFT</label><div class="sval" id="sLeft">—</div></div>
      <div class="sec front"><label>▲ FRONT</label><div class="sval" id="sFront">—</div></div>
      <div class="sec"><label>RIGHT ▶</label><div class="sval" id="sRight">—</div></div>
    </div>
    <p class="seclab">Mean <code>/scan</code> range per sector (m) — bigger = more open.
      The dog steers toward the larger side and slows as <b>Front</b> shrinks.</p>
    <div class="status" id="status">—</div>
  </div>

  <div class="card" style="margin-top:18px">
    <p class="ttl">⚙ Live tuning</p>
    <div class="params">
      <div class="p"><label>max_vx (m/s)</label><input id="p_max_vx" type="number" step="0.05"></div>
      <div class="p"><label>max_wz (rad/s)</label><input id="p_max_wz" type="number" step="0.05"></div>
      <div class="p"><label>stop_dist (m)</label><input id="p_stop_dist" type="number" step="0.05"></div>
      <div class="p"><label>slow_dist (m)</label><input id="p_slow_dist" type="number" step="0.1"></div>
      <div class="p"><label>front_deg (±°)</label><input id="p_front_deg" type="number" step="5"></div>
      <div class="p"><label>steer_deg (±°)</label><input id="p_steer_deg" type="number" step="5"></div>
      <div class="p"><label>k_steer (gain)</label><input id="p_k_steer" type="number" step="0.1"></div>
    </div>
    <button class="apply" onclick="applyParams()">Apply now</button>
    <p class="seclab" id="phint">Edits apply <b>immediately</b> while wandering (and are kept for the next start).</p>
  </div>
<script>
  const PKEYS=["max_vx","max_wz","stop_dist","slow_dist","front_deg","steer_deg","k_steer"];
  async function j(u,m,b){const o={method:m||"GET"};
    if(b){o.headers={"Content-Type":"application/json"};o.body=JSON.stringify(b);}
    const r=await fetch(u,o);return r.json().catch(()=>({}));}
  async function applyParams(){
    const body={};PKEYS.forEach(k=>{const v=parseFloat(document.getElementById("p_"+k).value);
      if(!isNaN(v))body[k]=v;});
    const d=await j("/api/wander/params","POST",body);
    const h=document.getElementById("phint");
    h.textContent=(d&&d.applied_live)?"applied live ✓":"saved — applies on next Start";
    setTimeout(()=>{h.innerHTML="Edits apply <b>immediately</b> while wandering (and are kept for the next start).";},1800);
  }
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
    const sc=s.sectors;
    document.getElementById("sLeft").textContent =sc?sc.left.toFixed(2)+" m":"—";
    document.getElementById("sFront").textContent=sc?sc.front.toFixed(2)+" m":"—";
    document.getElementById("sRight").textContent=sc?sc.right.toFixed(2)+" m":"—";
    if(s.params){PKEYS.forEach(k=>{const el=document.getElementById("p_"+k);
      if(el&&document.activeElement!==el&&s.params[k]!==undefined)el.value=s.params[k];});}
    document.getElementById("goBtn").disabled=s.wandering||!s.scan_ok;
    document.getElementById("status").textContent=
      s.wandering?"wandering → driving toward open space (Stop to halt)"
      :(s.scan_ok?"ready — press Start":"waiting for /scan…");
  }
  PKEYS.forEach(k=>document.getElementById("p_"+k)
    .addEventListener("change",applyParams));  // auto-apply on edit (Enter/blur)
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
    threading.Thread(target=_sector_poller, daemon=True).start()


if __name__ == "__main__":
    threading.Thread(target=_bootstrap, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="info", access_log=False)
