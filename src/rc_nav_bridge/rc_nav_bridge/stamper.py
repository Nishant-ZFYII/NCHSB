#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')

        # Parameters (with defaults)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('reference_topic', '/ackermann_steering_controller/reference')
        self.declare_parameter('frame_id', '')

        in_topic  = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('reference_topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # QoS: match controller's sub (BEST_EFFORT, depth 1) and teleop's pub (RELIABLE)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(TwistStamped, out_topic, pub_qos)
        self._sub = self.create_subscription(Twist, in_topic, self._on_twist, sub_qos)

        self.get_logger().info(
            f"Stamper ready: '{in_topic}' (Twist) → '{out_topic}' (TwistStamped) "
            f"[frame_id='{self._frame_id or ''}']"
        )

    def _on_twist(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        out.twist = msg
        self._pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(TwistToStamped())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
