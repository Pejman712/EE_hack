#!/usr/bin/env python3
"""replay — re-drive a recorded Go2 run by replaying its sport command stream.

Sibling of wander.py: where wander turns LIVE /scan into motion, replay turns a
RECORDED bag back into the SAME motion. The run was teleop captured as a stream
of Unitree "Move" commands (unitree_api/msg/Request, api_id 1008, parameter
{"x":vx,"y":vy,"z":yaw_rate}) on /api/sport/request at ~40 Hz. We re-publish that
exact stream, with the original inter-message timing, straight to the dog's sport
API — no /scan, no /cmd_vel, no bridge (replay IS the actuation).

This reproduces the same VELOCITY PROFILE, so it is OPEN-LOOP: there is no map or
localisation correcting drift. Place the robot at the SAME start pose/heading as
the recording for the path to line up; turns diverge most.

Safety (the reason this exists instead of `ros2 bag play`):
  * The recorded run's LAST Move is non-zero (the dog was still turning), so a
    raw bag play would leave it driving when playback ends. This node ALWAYS
    sends Move(0,0,0) -> StopMove(1003) -> BalanceStand(1002) on finish AND on
    SIGINT/SIGTERM, so the dog stops cleanly however it ends.
  * Replays ONLY /api/sport/request (+ optional keepalive topics). Never touches
    /lowcmd or the recorded state/sensor topics.

Reading the bag uses the self-describing mcap (mcap_ros2), so only api_id +
parameter are taken from each message; we then build FRESH unitree_api/msg/Request
messages to publish (the type comes from the colcon-built Unitree packages).

Env (server.py forwards these):
  BAG_PATH         path to the .mcap run            (default /run.mcap)
  REPLAY_SPEED     playback speed multiplier        (default 1.0; 0.5 = half)
  REPLAY_START_DELAY  seconds to wait before driving (default 0.0)
  REPLAY_TOPICS_EXTRA space/comma list of extra request topics to also replay,
                      e.g. /api/obstacles_avoid/request
"""
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rosidl_runtime_py.utilities import get_message

from mcap_ros2.reader import read_ros2_messages

SPORT_TOPIC = "/api/sport/request"
REQUEST_TYPE = "unitree_api/msg/Request"

API_MOVE = 1008           # {"x":vx,"y":vy,"z":yaw_rate}
API_STOP_MOVE = 1003      # parameter ""
API_BALANCE_STAND = 1002  # parameter ""


def _env_topics(val):
    return [t for t in val.replace(",", " ").split() if t]


class Replayer(Node):
    def __init__(self):
        super().__init__("go2_run_replayer")

        self.bag = os.environ.get("BAG_PATH", "/run.mcap")
        self.speed = max(1e-3, float(os.environ.get("REPLAY_SPEED", "1.0")))
        self.start_delay = float(os.environ.get("REPLAY_START_DELAY", "0.0"))
        extra = _env_topics(os.environ.get("REPLAY_TOPICS_EXTRA", ""))
        self.topics = [SPORT_TOPIC, *extra]

        if not os.path.exists(self.bag):
            self.get_logger().fatal(f"bag not found: {self.bag}")
            sys.exit(1)

        self.ReqType = get_message(REQUEST_TYPE)  # unitree_api/msg/Request

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pubs = {t: self.create_publisher(self.ReqType, t, qos) for t in self.topics}

        # Pre-load (t_ns, topic, api_id, parameter) from the self-describing mcap.
        self.events = []
        n_move = 0
        for m in read_ros2_messages(self.bag, topics=self.topics):
            api_id = int(m.ros_msg.header.identity.api_id)
            param = m.ros_msg.parameter
            if api_id == API_MOVE:
                n_move += 1
            self.events.append((m.log_time_ns, m.channel.topic, api_id, param))
        self.events.sort(key=lambda e: e[0])

        if not self.events:
            self.get_logger().fatal("no command messages found to replay")
            sys.exit(1)
        self.dur = (self.events[-1][0] - self.events[0][0]) / 1e9
        self.get_logger().info(
            f"loaded {len(self.events)} cmds ({n_move} Move) over {self.dur:.1f}s "
            f"from {self.topics} @ {self.speed}x | bag={self.bag}")

    def _request(self, api_id, parameter=""):
        m = self.ReqType()
        m.header.identity.api_id = api_id
        m.header.identity.id = int(time.time() * 1e9) % (2**31)
        m.parameter = parameter
        return m

    def stop(self):
        """Always leave the robot stationary and balanced."""
        pub = self._pubs[SPORT_TOPIC]
        for _ in range(5):
            pub.publish(self._request(API_MOVE, '{"x":0.0,"y":0.0,"z":0.0}'))
            time.sleep(0.02)
        pub.publish(self._request(API_STOP_MOVE))
        time.sleep(0.05)
        pub.publish(self._request(API_BALANCE_STAND))
        self.get_logger().info("sent StopMove + BalanceStand — robot stopped")

    def run(self):
        if self.start_delay > 0:
            self.get_logger().info(f"starting in {self.start_delay:.1f}s — clear the area")
            time.sleep(self.start_delay)
        self.get_logger().info("driving — replaying recorded command stream")

        t_first = self.events[0][0]
        wall0 = time.monotonic()
        for t_ns, topic, api_id, param in self.events:
            target = wall0 + ((t_ns - t_first) / 1e9) / self.speed
            while True:
                dt = target - time.monotonic()
                if dt <= 0:
                    break
                time.sleep(min(dt, 0.05))
            self._pubs[topic].publish(self._request(api_id, param))
        self.get_logger().info("replay finished")


def main():
    rclpy.init()
    node = Replayer()

    def _shutdown(*_):
        node.get_logger().warn("interrupted — stopping robot")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
