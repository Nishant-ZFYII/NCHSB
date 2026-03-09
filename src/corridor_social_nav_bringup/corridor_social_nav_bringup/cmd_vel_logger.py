"""Lightweight cmd_vel logger for debugging MPPI output."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_logger')
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cb, 10)
        self._count = 0

    def _cb(self, msg: Twist):
        self._count += 1
        if self._count % 20 == 0:
            self.get_logger().info(
                f'cmd_vel  vx={msg.linear.x:.3f}  vy={msg.linear.y:.3f}  '
                f'wz={msg.angular.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
