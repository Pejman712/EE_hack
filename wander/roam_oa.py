#!/usr/bin/env python3
"""roam_oa — let the Go2 roam using its OWN built-in obstacle avoidance.

No /scan, no pointcloud_to_laserscan, no wander gap-following. This uses the
Go2's native high-level *obstacle-avoidance* client (the firmware does the
sensing and dodging): we turn the mode ON, hand it API control, and then keep
feeding it a gentle forward velocity — the dog walks around avoiding things on
its own.

It talks the same unitree_api/msg/Request protocol the rest of this repo uses
(see sit_stand.py / cmd_vel_to_sport.py), but on the OBSTACLES-AVOID topic, and
follows the documented ObstaclesAvoidClient sequence:

    /api/obstacles_avoid/request   unitree_api/msg/Request
      SwitchSet              1001  '{"enable":true}'      turn the mode ON
      UseRemoteCommandFromApi 1004 '{"is_remote_commands_from_api":true}'
                                                          hand Move control to us
      Move                   1003  '{"x":vx,"y":vy,"yaw":vyaw,"mode":0}'  drive
    /api/obstacles_avoid/response  unitree_api/msg/Response  (code==0 == ok)

ORDER MATTERS. Sending Move before SwitchSet+UseRemoteCommandFromApi are ACK'd
gets rejected with code 3202 ("mode not enabled"). So we walk a small handshake
(switch -> remote -> ready) and only send Move once BOTH steps return code==0 —
which also means an unsupported firmware leaves the dog standing still instead of
walking BLIND. On exit we Move(0), drop API control, and disable the mode.

api_ids / parameter formats match unitree_sdk2py's ObstaclesAvoidClient.

Params (ros2 -p name:=value):
  vx 0.25        forward speed (m/s)
  vy 0.0         lateral speed (m/s)
  vyaw 0.0       constant yaw bias (rad/s) — small value => sweeps/curves while roaming
  move_hz 10     how often Move is re-sent (the mode has a velocity watchdog)
  handshake_period 1.0   re-send the current enable step this often until it ACKs
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
OA_API_ID_USE_REMOTE = 1004
LABELS = {OA_API_ID_SWITCH_SET: "SwitchSet", OA_API_ID_MOVE: "Move",
          OA_API_ID_USE_REMOTE: "UseRemoteCommandFromApi"}


class RoamOA(Node):
    def __init__(self):
        super().__init__("roam_oa")
        p = self.declare_parameter
        self.vx = float(p("vx", 0.25).value)
        self.vy = float(p("vy", 0.0).value)
        self.vyaw = float(p("vyaw", 0.0).value)
        self.move_hz = float(p("move_hz", 10.0).value)
        self.handshake_period = float(p("handshake_period", 1.0).value)

        self._ids = itertools.count(1)
        self._pending = {}        # request id -> label, awaiting a response
        # Handshake state machine: "switch" -> "remote" -> "ready".
        self._stage = "switch"

        self._pub = self.create_publisher(Request, OA_REQUEST_TOPIC, 10)
        self.create_subscription(Response, OA_RESPONSE_TOPIC, self._on_response, 10)
        # Drive the enable handshake until both steps ACK, then Move takes over.
        self._hs_timer = self.create_timer(self.handshake_period, self._handshake)
        self.create_timer(1.0 / self.move_hz, self._tick)
        self.get_logger().info(
            f"roam_oa up: SwitchSet -> UseRemoteCommandFromApi on {OA_REQUEST_TOPIC}, "
            f"then Move(vx={self.vx}, vy={self.vy}, vyaw={self.vyaw}) @ {self.move_hz:g}Hz "
            "(stands still until both ACK code=0)")

    def _send(self, api_id, parameter=""):
        req = Request()
        req.header.identity.api_id = api_id
        req.header.identity.id = next(self._ids)
        if parameter:
            req.parameter = parameter
        self._pending[req.header.identity.id] = LABELS.get(api_id, str(api_id))
        self._pub.publish(req)

    def _handshake(self):
        if self._stage == "ready":
            self._hs_timer.cancel()
            return
        if self._pub.get_subscription_count() == 0:
            self.get_logger().warn(
                f"no subscriber on {OA_REQUEST_TOPIC} — robot reachable / obstacle-"
                "avoid service up? retrying")
        if self._stage == "switch":
            self._send(OA_API_ID_SWITCH_SET, json.dumps({"enable": True}))
        elif self._stage == "remote":
            self._send(OA_API_ID_USE_REMOTE,
                       json.dumps({"is_remote_commands_from_api": True}))

    def _tick(self):
        if self._stage != "ready":
            return  # safety: never drive until SwitchSet + UseRemote both ACK'd
        self._send(OA_API_ID_MOVE,
                   json.dumps({"x": self.vx, "y": self.vy, "yaw": self.vyaw, "mode": 0}))

    def _on_response(self, msg: Response):
        label = self._pending.pop(msg.header.identity.id, None)
        code = msg.header.status.code
        if label == "SwitchSet":
            if code == 0 and self._stage == "switch":
                self._stage = "remote"
                self.get_logger().info("obstacle-avoid ON — taking API control")
            elif code != 0:
                self.get_logger().warn(f"SwitchSet ack code={code} — mode NOT enabled")
        elif label == "UseRemoteCommandFromApi":
            if code == 0 and self._stage == "remote":
                self._stage = "ready"
                self.get_logger().info("API control granted (code=0) — roaming")
            elif code != 0:
                self.get_logger().warn(
                    f"UseRemoteCommandFromApi ack code={code} — Move will be rejected")
        elif label == "Move" and code != 0:
            self.get_logger().warn(f"Move ack code={code} (3202 == mode not enabled)")

    def stop(self):
        """Halt, drop API control, and disable the mode (best-effort)."""
        try:
            self._send(OA_API_ID_MOVE,
                       json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0}))
            self._send(OA_API_ID_USE_REMOTE,
                       json.dumps({"is_remote_commands_from_api": False}))
            self._send(OA_API_ID_SWITCH_SET, json.dumps({"enable": False}))
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
