#!/usr/bin/env python3
"""sit_stand — a ROS 2 node that sits the Go2 down and then stands it back up.

It exposes a `std_srvs/Trigger` service (`sit_and_stand`) that, when called,
publishes `unitree_api/msg/Request` messages to `/api/sport/request` with the
high-level sport API ids — the same ids the unitree_ros2 example's SportClient
uses (example/src/include/common/ros2_sport_client.h):

    Sit      -> api_id 1009
    RiseSit  -> api_id 1010   (rise from the sit back to standing)

Drive it with:

    ros2 service call /sit_and_stand std_srvs/srv/Trigger

Like the example client, Sit/RiseSit carry no `parameter` — only the api_id. For
these to take effect the robot must hold the sport lease and be in normal sport
mode (not AI/advanced mode, not damped), standing on a flat, clear area. Set
`auto_run_on_start:=true` to run the routine once shortly after launch instead
of waiting for a service call.

Logging: every request is tagged with a unique `header.identity.id` and logged
when sent; the node also subscribes to `/api/sport/response` and logs the
robot's reply (matched back to the Sit/RiseSit label by that id) — `code == 0`
is success, matching the convention other unitree_sdk2py error tables use, but
this hasn't been confirmed against Unitree's docs for every api_id, so treat a
nonzero code as "investigate" rather than a specific known failure.
"""
import itertools
import os
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from unitree_api.msg import Request, Response

SPORT_REQUEST_TOPIC = "/api/sport/request"
SPORT_RESPONSE_TOPIC = "/api/sport/response"
SPORT_API_ID_SIT = 1009
SPORT_API_ID_RISESIT = 1010
SPORT_API_LABELS = {SPORT_API_ID_SIT: "Sit", SPORT_API_ID_RISESIT: "RiseSit"}


class SitStand(Node):
    def __init__(self):
        super().__init__("sit_stand")
        self.declare_parameter("sit_seconds", 4.0)
        self.declare_parameter("auto_run_on_start", False)

        self._request_ids = itertools.count(1)
        self._pending = {}  # request id -> label, awaiting a /api/sport/response

        self._pub = self.create_publisher(Request, SPORT_REQUEST_TOPIC, 10)
        self._sub = self.create_subscription(
            Response, SPORT_RESPONSE_TOPIC, self._on_response, 10)
        self._srv = self.create_service(Trigger, "sit_and_stand", self._on_trigger)

        self.get_logger().info(
            f"sit_stand up (domain={os.environ.get('ROS_DOMAIN_ID')}, "
            f"rmw={os.environ.get('RMW_IMPLEMENTATION')}, "
            f"sit_seconds={self.get_parameter('sit_seconds').value}, "
            f"auto_run_on_start={self.get_parameter('auto_run_on_start').value}) — "
            "call 'ros2 service call /sit_and_stand std_srvs/srv/Trigger' "
            f"(publishes to {SPORT_REQUEST_TOPIC}, listens on {SPORT_RESPONSE_TOPIC})")

        if self.get_parameter("auto_run_on_start").value:
            # Fire once, after the node is fully constructed and spinning.
            self._auto_timer = self.create_timer(2.0, self._auto_run)

    def _auto_run(self):
        self._auto_timer.cancel()
        self.get_logger().info("auto_run_on_start fired")
        self._sit_then_stand()

    def _on_response(self, msg):
        req_id = msg.header.identity.id
        label = self._pending.pop(req_id, None)
        if label is None:
            return  # response to a request from before this node started, or already handled
        code = msg.header.status.code
        if code == 0:
            self.get_logger().info(f"{label} ack: code=0 (ok)")
        else:
            self.get_logger().warn(f"{label} ack: code={code} (non-zero — check the robot)")

    def _send(self, api_id):
        label = SPORT_API_LABELS[api_id]
        req_id = next(self._request_ids)
        self._pending[req_id] = label
        req = Request()
        req.header.identity.api_id = api_id
        req.header.identity.id = req_id
        self.get_logger().info(f"sending {label} (id={req_id})")
        self._pub.publish(req)

    def _wait_for_robot(self, timeout_s=5.0):
        """Block until the robot's sport subscriber matches, so the first
        publish isn't dropped before discovery completes."""
        deadline = time.time() + timeout_s
        while time.time() < deadline and self._pub.get_subscription_count() == 0:
            time.sleep(0.1)
        return self._pub.get_subscription_count() > 0

    def _sit_then_stand(self):
        sit_seconds = float(self.get_parameter("sit_seconds").value)
        matched = self._wait_for_robot()
        if not matched:
            self.get_logger().warn(
                f"no subscriber on {SPORT_REQUEST_TOPIC} — robot reachable / sport "
                "service up? sending anyway")
        self._send(SPORT_API_ID_SIT)
        time.sleep(sit_seconds)
        self._send(SPORT_API_ID_RISESIT)
        return matched

    def _on_trigger(self, request, response):
        self.get_logger().info("sit_and_stand triggered")
        try:
            matched = self._sit_then_stand()
            response.success = True
            response.message = "sit→stand sent" + (
                "" if matched else " (no subscriber matched — check the robot)")
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"failed: {exc}"
            self.get_logger().error(response.message)
        return response


def main():
    rclpy.init()
    node = SitStand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
