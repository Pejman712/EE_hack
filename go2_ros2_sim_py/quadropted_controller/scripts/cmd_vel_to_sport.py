#!/usr/bin/env python3
"""
Bridge: Nav2 / teleop ``cmd_vel`` (geometry_msgs/Twist)  ->  Unitree Go2 sport API.

This is the REAL-ROBOT counterpart of ``cmd_vel_pub.py`` (which targets the Gazebo
IK controller via the custom ``robot_velocity`` message). On the physical Go2,
motion is commanded through the high-level "sport mode" API: a
``unitree_api/msg/Request`` published on ``/api/sport/request``, where the *Move*
command (api_id 1008) carries a JSON payload ``{"x": vx, "y": vy, "z": vyaw}``.

Everything upstream of ``cmd_vel`` (Nav2, costmaps, planner, controller) is identical
between sim and real -- only this last hop differs, which is exactly what this node
absorbs. Swap ``cmd_vel_pub.py`` (sim) <-> ``cmd_vel_to_sport.py`` (real) and the rest
of the stack is unchanged.

IMPORTANT -- this is a hardware bridge STUB:
  * It requires the Unitree ROS2 SDK (``unitree_api`` messages), which is present on
    the robot but NOT in the simulation build. The import is deferred to runtime so
    this file installs cleanly in the sim; it only needs the SDK when actually run.
  * Verify ``api_id`` values and message field names against YOUR ``unitree_ros2``
    version before trusting it on hardware. Test with the robot on a stand first.
"""
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Unitree Go2 sport-mode API ids (see unitree_ros2 SportClient / sport_api headers).
SPORT_API_ID_STOPMOVE = 1003
SPORT_API_ID_MOVE = 1008


class CmdVelToSport(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_sport')

        # Deferred import: the sim build does not have the Unitree SDK, but this file
        # is still installed there. It is only imported when the node actually runs
        # (i.e. on the robot), so the sim never needs the dependency.
        from unitree_api.msg import Request
        self._Request = Request

        # Safety clamps (m/s, m/s, rad/s). Keep conservative for first hardware tests.
        self.declare_parameter('max_x', 1.0)
        self.declare_parameter('max_y', 0.5)
        self.declare_parameter('max_yaw', 1.5)
        self.max_x = float(self.get_parameter('max_x').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.max_yaw = float(self.get_parameter('max_yaw').value)

        # /api/sport/request is NOT namespaced on the real robot -> absolute topic.
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        # cmd_vel is relative so the node can be launched under a namespace if desired.
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.get_logger().info('cmd_vel -> /api/sport/request bridge started')

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def on_cmd_vel(self, msg: Twist):
        vx = self._clamp(msg.linear.x, self.max_x)
        vy = self._clamp(msg.linear.y, self.max_y)
        vyaw = self._clamp(msg.angular.z, self.max_yaw)

        req = self._Request()
        if vx == 0.0 and vy == 0.0 and vyaw == 0.0:
            # Explicit stop is safer than sending a zero-velocity Move.
            req.header.identity.api_id = SPORT_API_ID_STOPMOVE
        else:
            req.header.identity.api_id = SPORT_API_ID_MOVE
            req.parameter = json.dumps({'x': vx, 'y': vy, 'z': vyaw})
        self.pub.publish(req)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
