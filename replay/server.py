#!/usr/bin/env python3
"""server.py — tiny web UI for REPLAY mode (re-drive a recorded run).

Sibling of wander's server.py, minus the sensing pipeline: replay publishes the
dog's sport Move API directly, so there is no /scan or /cmd_vel bridge to stand
up. One-button page:

  * Start replay -> spawn replay.py (re-publishes the recorded /api/sport/request
                    stream with original timing straight to the dog).
  * Stop         -> SIGINT replay.py (it sends StopMove + BalanceStand on exit)
                    plus a belt-and-braces StopMove.

It tails /api/sport/request liveness so the page shows whether commands are
actually flowing to the robot. Everything is driven over the ros2 CLI here — no
rclpy in this process (replay.py is the only rclpy node).

Endpoints
---------
  GET  /                    the page
  GET  /api/status          {replaying, sport_ok}
  POST /api/replay/start    start the replay node
  POST /api/replay/stop     stop it + StopMove

Env: GO2_IP (DDS NIC, used by entrypoint), PORT (7100), plus replay tunables
forwarded to replay.py: BAG_PATH, REPLAY_SPEED, REPLAY_START_DELAY,
REPLAY_TOPICS_EXTRA.
"""
import os
import signal
import subprocess
import threading
import time

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse

PORT = int(os.environ.get("PORT", "7100"))
HERE = os.path.dirname(os.path.abspath(__file__))
REPLAY_FILE = os.path.join(HERE, "replay.py")

SPORT_TOPIC = "/api/sport/request"
SPORT_TYPE = "unitree_api/msg/Request"
SPORT_API_ID_STOP = 1003

app = FastAPI(title="go2-replay")

_state = {"replay": None, "sport": None}


def _alive(key: str) -> bool:
    p = _state[key]
    return p is not None and p.poll() is None


def _stop_dog():
    """One-shot StopMove so a halted replay leaves the dog holding position."""
    msg = "{header: {identity: {api_id: %d}}}" % SPORT_API_ID_STOP
    try:
        subprocess.run(
            ["ros2", "topic", "pub", "--once", SPORT_TOPIC, SPORT_TYPE, msg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8)
    except Exception:  # noqa: BLE001
        pass


def _stop_replay():
    p = _state["replay"]
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)  # replay stops the dog on SIGINT
            p.wait(timeout=3.0)
        except Exception:  # noqa: BLE001
            try:
                p.kill()
            except Exception:  # noqa: BLE001
                pass
    _state["replay"] = None


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


@app.get("/api/status")
def api_status() -> dict:
    return {
        "replaying": _alive("replay"),
        "sport_ok": _fresh(_state["sport"]),
    }


@app.post("/api/replay/start")
def api_replay_start() -> dict:
    if _alive("replay"):
        return {"ok": True, "already": True}
    _state["replay"] = subprocess.Popen(
        ["python3", REPLAY_FILE], start_new_session=True, env=dict(os.environ))
    return {"ok": True}


@app.post("/api/replay/stop")
def api_replay_stop() -> dict:
    _stop_replay()
    _stop_dog()
    return {"ok": True}


PAGE = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Go2 Replay</title>
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
  .warn{margin-top:14px;font-size:13px;color:var(--amber);}
  code{font-family:ui-monospace,Menlo,monospace;color:#cfe9e3;}
</style></head><body><div class="wrap">
  <h1>⏯️ Go2 Replay</h1>
  <p class="sub">Re-drives a <b>recorded run</b> by replaying its sport command
    stream with original timing — open-loop, no map. Place the dog at the
    recording's start pose first. It always stops cleanly on finish or
    <b>Stop</b>.</p>
  <div class="card">
    <div class="row">
      <button class="go" id="goBtn" onclick="start()">Start replay</button>
      <button class="stop" onclick="stop()">■ Stop</button>
    </div>
    <ul>
      <li><span class="dot" id="dRep"></span><span class="nm">Replay node</span><span class="val" id="vRep">—</span></li>
      <li><span class="dot" id="dSp"></span><span class="nm">Sport <code>/api/sport/request</code></span><span class="val" id="vSp">—</span></li>
    </ul>
    <div class="status" id="status">—</div>
    <div class="warn">⚠ Open-loop velocity replay. Clear the area and match the start pose.</div>
  </div>
<script>
  async function j(u,m){const r=await fetch(u,{method:m||"GET"});return r.json().catch(()=>({}));}
  function set(id,ok,txt,on){
    const d=document.getElementById("d"+id), v=document.getElementById("v"+id);
    d.className="dot"+(on?" on":(ok?" ok":" bad"));v.textContent=txt;}
  async function start(){await j("/api/replay/start","POST");setTimeout(refresh,300);}
  async function stop(){await j("/api/replay/stop","POST");setTimeout(refresh,300);}
  async function refresh(){
    const s=await j("/api/status");
    set("Rep",false,s.replaying?"running":"stopped",s.replaying);
    set("Sp",s.sport_ok,s.sport_ok?"flowing":"idle");
    document.getElementById("goBtn").disabled=s.replaying;
    document.getElementById("status").textContent=
      s.replaying?"replaying recorded run (Stop to halt)":"ready — press Start";
  }
  refresh();setInterval(refresh,1000);
</script></div></body></html>"""


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return PAGE


def _bootstrap():
    time.sleep(4)  # let DDS discovery find the robot
    threading.Thread(target=_liveness_poller,
                     args=(SPORT_TOPIC, "sport", "header.identity.api_id"),
                     daemon=True).start()


if __name__ == "__main__":
    threading.Thread(target=_bootstrap, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="info", access_log=False)
