"""go2-foxglove control — drive the Go2 over native DDS via unitree_sdk2py.

Same DDS setup as the `bridge` service (ChannelFactory bound to this device's
IP via cyclonedds.xml / GO2_DDS_ADDRESS, same multi-homed-NIC caveat), but
instead of reading topics this writes to them: it wraps unitree_sdk2py's
high-level SportClient (rt/api/sport/request) behind a small HTTP API, so you
can drive the robot with curl while the `bridge` shows you what it's doing in
Foxglove.

Move() is a velocity command — the robot keeps moving at the last commanded
velocity until a new one arrives or its internal watchdog times out, so /move
re-sends it at MOVE_HZ for the requested duration, then calls StopMove().

UNVERIFIED on a live Go2 EDU+. Verify the SportClient method set/signatures
against the unitree_sdk2py ref pinned in the Dockerfile, and make sure nothing
else (phone app, sport_lease) is holding the sport lease before driving it.
"""
import logging
import os
import threading
import time

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("go2-foxglove-control")

CONTROL_PORT = int(os.environ.get("CONTROL_PORT", "8767"))
DDS_DOMAIN = int(os.environ.get("DDS_DOMAIN", "0"))
MOVE_HZ = 10.0

api = FastAPI(title="go2-foxglove-control")
_sport = None
_move_lock = threading.Lock()
_move_stop = threading.Event()
_move_thread = None


class MoveRequest(BaseModel):
    vx: float = 0.3
    vy: float = 0.0
    vyaw: float = 0.0
    duration_s: float = 1.0


def _init_sport():
    global _sport
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    ChannelFactoryInitialize(DDS_DOMAIN)  # honours CYCLONEDDS_URI from the Dockerfile
    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()
    _sport = client
    log.info("SportClient ready")


def _move_loop(vx, vy, vyaw, duration_s):
    end = time.time() + duration_s
    period = 1.0 / MOVE_HZ
    try:
        while time.time() < end and not _move_stop.is_set():
            _sport.Move(vx, vy, vyaw)
            time.sleep(period)
    except Exception:  # noqa: BLE001
        log.exception("Move failed")
    finally:
        try:
            _sport.StopMove()
        except Exception:  # noqa: BLE001
            log.exception("StopMove failed")


@api.get("/healthz")
def healthz():
    return {"ok": _sport is not None}


@api.post("/standup")
def standup():
    _sport.StandUp()
    _sport.BalanceStand()
    return {"ok": True}


@api.post("/standdown")
def standdown():
    _sport.StandDown()
    return {"ok": True}


@api.post("/damp")
def damp():
    _sport.Damp()
    return {"ok": True}


@api.post("/move")
def move(req: MoveRequest):
    global _move_thread
    with _move_lock:
        _move_stop.set()
        if _move_thread is not None:
            _move_thread.join(timeout=2.0)
        _move_stop.clear()
        _move_thread = threading.Thread(
            target=_move_loop, args=(req.vx, req.vy, req.vyaw, req.duration_s), daemon=True)
        _move_thread.start()
    return {"ok": True, "vx": req.vx, "vy": req.vy, "vyaw": req.vyaw, "duration_s": req.duration_s}


@api.post("/stop")
def stop():
    _move_stop.set()
    _sport.StopMove()
    return {"ok": True}


def main():
    _init_sport()
    print(f"DIAG control up on :{CONTROL_PORT}", flush=True)
    uvicorn.run(api, host="0.0.0.0", port=CONTROL_PORT, log_level="warning")


if __name__ == "__main__":
    main()
