#!/usr/bin/env python3
"""roam_oa — let the Go2 roam using its OWN built-in obstacle avoidance.

No /scan, no pointcloud_to_laserscan, no wander gap-following. This uses the
Go2's native high-level *obstacle-avoidance* client (the firmware does the
sensing and dodging): we just turn the mode ON and then keep feeding it a gentle
forward velocity, and the dog walks around avoiding things on its own.

It talks the same unitree_api/msg/Request protocol the rest of this repo uses
(see sit_stand.py / cmd_vel_to_sport.py), but on the OBSTACLES-AVOID topic, not
the sport topic:

    /api/obstacles_avoid/request   unitree_api/msg/Request
      SwitchSet  api_id 1001  parameter '<true|false>'   enable/disable the mode
      Move       api_id 1003  parameter '{"x":vx,"y":vy,"yaw":vyaw}'  drive it
    /api/obstacles_avoid/response  unitree_api/msg/Response  (code==0 == ok)

SAFETY: we only start sending Move once SwitchSet has been ACK'd with code==0, so
if this firmware doesn't support the obstacle-avoid client the dog just stands
still instead of walking forward BLIND. On exit we send Move(0,0,0) then disable
the mode.

The api_ids / parameter formats match the unitree_sdk2 obstacles-avoid client but
are UNVERIFIED on this firmware — if the dog won't enable, check the codes logged
from /api/obstacles_avoid/response and adjust the constants below.

Params (ros2 -p name:=value):
  vx 0.25        forward speed (m/s) fed to the avoid mode
  vy 0.0         lateral speed (m/s)
  vyaw 0.0       constant yaw bias (rad/s) — small value => sweeps/curves while roaming
  move_hz 10     how often Move is re-sent (the mode has a velocity watchdog)
  enable_period 1.0   re-send SwitchSet(true) this often until it ACKs
"""
import json
import itertools

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

OA_REQUEST_TOPIC = "/api/obstacles_avoid/request"
OA_RESPONSE_TOPIC = "/api/obstacles_avoid/response"
OA_API_ID_SWITCH_SET = 1001
OA_API_ID_MOVE = 1003
LABELS = {OA_API_ID_SWITCH_SET: "SwitchSet", OA_API_ID_MOVE: "Move"}


class RoamOA(Node):
    def __init__(self):
        super().__init__("roam_oa")
        p = self.declare_parameter
        self.vx = float(p("vx", 0.25).value)
        self.vy = float(p("vy", 0.0).value)
        self.vyaw = float(p("vyaw", 0.0).value)
        self.move_hz = float(p("move_hz", 10.0).value)
        self.enable_period = float(p("enable_period", 1.0).value)

        self._ids = itertools.count(1)
        self._pending = {}          # request id -> label, awaiting a response
        self._enabled = False       # set True once SwitchSet(true) is ACK'd code==0

        self._pub = self.create_publisher(Request, OA_REQUEST_TOPIC, 10)
        self.create_subscription(Response, OA_RESPONSE_TOPIC, self._on_response, 10)
        # Keep nudging the mode ON until it ACKs, then Move takes over.
        self._enable_timer = self.create_timer(self.enable_period, self._try_enable)
        self.create_timer(1.0 / self.move_hz, self._tick)
        self.get_logger().info(
            f"roam_oa up: enabling obstacle-avoid on {OA_REQUEST_TOPIC}, then "
            f"Move(vx={self.vx}, vy={self.vy}, vyaw={self.vyaw}) @ {self.move_hz:g}Hz "
            "(stands still until the mode ACKs)")

    def _send(self, api_id, parameter=""):
        req = Request()
        req.header.identity.api_id = api_id
        req.header.identity.id = next(self._ids)
        if parameter:
            req.parameter = parameter
        self._pending[req.header.identity.id] = LABELS.get(api_id, str(api_id))
        self._pub.publish(req)

    def _try_enable(self):
        if self._enabled:
            self._enable_timer.cancel()
            return
        if self._pub.get_subscription_count() == 0:
            self.get_logger().warn(
                f"no subscriber on {OA_REQUEST_TOPIC} — robot reachable / obstacle-"
                "avoid service up? retrying")
        self._send(OA_API_ID_SWITCH_SET, json.dumps(True))  # '<true>'

    def _tick(self):
        if not self._enabled:
            return  # safety: never drive forward before the mode is confirmed on
        self._send(OA_API_ID_MOVE,
                   json.dumps({"x": self.vx, "y": self.vy, "yaw": self.vyaw}))

    def _on_response(self, msg: Response):
        label = self._pending.pop(msg.header.identity.id, None)
        code = msg.header.status.code
        if label == "SwitchSet" and code == 0 and not self._enabled:
            self._enabled = True
            self.get_logger().info("obstacle-avoid ENABLED (code=0) — roaming")
        elif label == "SwitchSet" and code != 0:
            self.get_logger().warn(f"SwitchSet ack code={code} — mode NOT enabled")

    def stop(self):
        """Halt and disable the mode (best-effort; we may be mid-shutdown)."""
        try:
            self._send(OA_API_ID_MOVE, json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0}))
            self._send(OA_API_ID_SWITCH_SET, json.dumps(False))
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = RoamOA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
