import rclpy
import time
import math
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeCmd_


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('move_forward_node')

    node.get_logger().info('Initializing Go2 movement...')

    try:
        ChannelFactoryInitialize(0)
        node.get_logger().info('DDS channel initialized')
    except Exception as e:
        node.get_logger().error(f'Failed to initialize DDS: {e}')
        rclpy.shutdown()
        return

    try:
        cmd_publisher = ChannelPublisher("rt/sportmodeCmd", SportModeCmd_)
        cmd_publisher.Init()
        node.get_logger().info('SportModeCmd publisher initialized')
    except Exception as e:
        node.get_logger().error(f'Failed to initialize command publisher: {e}')
        rclpy.shutdown()
        return

    cmd = SportModeCmd_()
    cmd.mode = 2  # Sport mode
    cmd.gait_type = 1  # Walk gait
    cmd.speed_level = 0
    cmd.footer_raise = 0.0
    cmd.body_height = 0.0
    cmd.euler = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]
    cmd.velocity = [0.5, 0.0]  # [forward, lateral] m/s
    cmd.yaw_speed = 0.0
    cmd.reserve = 0

    node.get_logger().info('Starting 2-second forward movement (target: ~1 meter at 0.5 m/s)')

    start_time = time.time()
    while time.time() - start_time < 2.0:
        try:
            cmd_publisher.Write(cmd)
            time.sleep(0.02)  # 50 Hz command rate
        except Exception as e:
            node.get_logger().error(f'Failed to publish command: {e}')
            break

    cmd.velocity = [0.0, 0.0]
    try:
        cmd_publisher.Write(cmd)
    except Exception:
        pass

    node.get_logger().info('Movement complete. Shutting down.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
