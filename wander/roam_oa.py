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

ESCAPE-TURN / MORE ROAMING. The firmware only avoids obstacles AHEAD, so against
a wall (or a dead-end) it just stops. We watch the dog's measured body speed on
/sportmodestate: if we're commanding forward but it's stalled (speed < stall_speed
for stall_time), we count it as BLOCKED and rotate IN PLACE for a randomized
duration/direction (turn_wz, turn_min_s..turn_max_s) to hunt for a new heading,
then resume forward. Random direction + duration = it explores instead of pacing
the same spot. If /sportmodestate isn't arriving, blocked-detection is simply off
and the firmware's own avoidance still runs.

api_ids / parameter formats match unitree_sdk2py's ObstaclesAvoidClient.

Params (ros2 -p name:=value):
  vx 0.25 · vy 0.0 · vyaw 0.0        forward / lateral / yaw-bias velocity
  move_hz 10                         Move re-send rate (the mode has a watchdog)
  handshake_period 1.0               re-send each enable step until it ACKs
  turn_wz 0.9                        in-place rotation speed when blocked (rad/s)
  turn_min_s 1.0 · turn_max_s 2.5    randomized escape-turn duration range (s)
  stall_speed 0.06 · stall_time 1.0  blocked = slower than this for this long
"""
import json
import itertools
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

OA_REQUEST_TOPIC = "/api/obstacles_avoid/request"
OA_RESPONSE_TOPIC = "/api/obstacles_avoid/response"
SPORT_STATE_TOPIC = "/sportmodestate"
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
        self.turn_wz = float(p("turn_wz", 0.9).value)
        self.turn_min_s = float(p("turn_min_s", 1.0).value)
        self.turn_max_s = float(p("turn_max_s", 2.5).value)
        self.stall_speed = float(p("stall_speed", 0.06).value)
        self.stall_time = float(p("stall_time", 1.0).value)

        self._ids = itertools.count(1)
        self._pending = {}        # request id -> label, awaiting a response
        # Enable handshake: "switch" -> "remote" -> "ready".
        self._stage = "switch"
        # While "ready", a sub-behaviour: "forward" or (escape) "turning".
        self._phase = "forward"
        self._stall_since = None  # epoch the dog first looked stalled (None = moving)
        self._turn_until = 0.0    # epoch the current escape-turn ends
        self._turn_dir = 1.0      # +1 = turn left (CCW), -1 = right
        self._grace_until = 0.0   # don't re-evaluate stall right after a turn
        self._meas_speed = 0.0    # latest measured body speed (m/s)
        self._meas_t = 0.0        # when we last got /sportmodestate
        self._warned_no_state = False

        self._pub = self.create_publisher(Request, OA_REQUEST_TOPIC, 10)
        self.create_subscription(Response, OA_RESPONSE_TOPIC, self._on_response, 10)
        self.create_subscription(SportModeState, SPORT_STATE_TOPIC, self._on_state,
                                 qos_profile_sensor_data)
        # Drive the enable handshake until both steps ACK, then Move takes over.
        self._hs_timer = self.create_timer(self.handshake_period, self._handshake)
        self.create_timer(1.0 / self.move_hz, self._tick)
        self.get_logger().info(
            f"roam_oa up: SwitchSet -> UseRemoteCommandFromApi on {OA_REQUEST_TOPIC}, "
            f"then Move(vx={self.vx}) @ {self.move_hz:g}Hz; rotate {self.turn_wz}rad/s "
            f"{self.turn_min_s}-{self.turn_max_s}s when stalled (stands still until ACK)")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _send(self, api_id, parameter=""):
        req = Request()
        req.header.identity.api_id = api_id
        req.header.identity.id = next(self._ids)
        if parameter:
            req.parameter = parameter
        self._pending[req.header.identity.id] = LABELS.get(api_id, str(api_id))
        self._pub.publish(req)

    def _move(self, vx, vy, vyaw):
        self._send(OA_API_ID_MOVE,
                   json.dumps({"x": vx, "y": vy, "yaw": vyaw, "mode": 0}))

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

    def _on_state(self, msg: SportModeState):
        try:
            vx, vy = float(msg.velocity[0]), float(msg.velocity[1])
        except Exception:  # noqa: BLE001
            return
        self._meas_speed = math.hypot(vx, vy)
        self._meas_t = self._now()

    def _tick(self):
        if self._stage != "ready":
            return  # safety: never drive until SwitchSet + UseRemote both ACK'd
        now = self._now()

        # Escape turn in progress: rotate in place until it elapses.
        if self._phase == "turning":
            if now < self._turn_until:
                self._move(0.0, 0.0, self._turn_dir * self.turn_wz)
                return
            self._phase = "forward"
            self._stall_since = None
            self._grace_until = now + 0.7  # let it pick up speed before judging stall

        # Normal roaming: drive forward (firmware steers around what it sees).
        self._move(self.vx, self.vy, self.vyaw)

        # Blocked detection needs fresh velocity and a settle grace after turning.
        if now < self._grace_until:
            return
        if (now - self._meas_t) > 0.5:
            if not self._warned_no_state:
                self.get_logger().warn(
                    "no /sportmodestate velocity — escape-turn off (firmware still "
                    "avoids). Check the topic / QoS.")
                self._warned_no_state = True
            return
        self._warned_no_state = False

        if self._meas_speed < self.stall_speed:
            if self._stall_since is None:
                self._stall_since = now
            elif (now - self._stall_since) >= self.stall_time:
                self._begin_turn(now)
        else:
            self._stall_since = None

    def _begin_turn(self, now):
        self._phase = "turning"
        self._turn_dir = random.choice((-1.0, 1.0))
        dur = random.uniform(self.turn_min_s, self.turn_max_s)
        self._turn_until = now + dur
        self._stall_since = None
        side = "left" if self._turn_dir > 0 else "right"
        self.get_logger().info(f"blocked (stalled) — rotating {side} {dur:.1f}s for a new heading")

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
            self._move(0.0, 0.0, 0.0)
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
