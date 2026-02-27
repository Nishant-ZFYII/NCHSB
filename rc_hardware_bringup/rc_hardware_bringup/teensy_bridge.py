#!/usr/bin/env python3
"""
Teensy 4.1 serial bridge for the Traxxas Maxx 4S Ackermann RC car.

Bridges ROS 2 velocity commands to the Teensy microcontroller over USB
serial (ASCII protocol), and publishes odometry from AS5600 encoder
feedback.

Serial protocol (Jetson → Teensy):
    CMD <speed_mps> <steering_rad>\n
        speed_mps    : float, desired rear-axle speed in m/s
        steering_rad : float, desired steering angle in radians

Serial protocol (Teensy → Jetson):
    ODOM <x> <y> <theta> <vx> <vyaw> <steering_rad>\n
        Sent at ~50 Hz by the Teensy from AS5600 + VESC feedback.

The node also publishes a static /joint_states for the steering angle
so robot_state_publisher can keep the TF tree correct.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import tf2_ros

try:
    import serial
except ImportError:
    serial = None


class TeensyBridge(Node):
    def __init__(self):
        super().__init__('teensy_bridge')

        # --------------- parameters ---------------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheelbase', 0.1869)
        self.declare_parameter('wheel_radius', 0.055)
        self.declare_parameter('max_steering_angle', 0.4887)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('use_stamped_vel', False)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baud_rate').value
        self._wheelbase = self.get_parameter('wheelbase').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._max_steer = self.get_parameter('max_steering_angle').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value
        cmd_topic = self.get_parameter('cmd_topic').value
        use_stamped = self.get_parameter('use_stamped_vel').value
        self._cmd_timeout = self.get_parameter('cmd_timeout_sec').value

        # --------------- serial port ---------------
        self._ser = None
        self._serial_lock = threading.Lock()
        self._open_serial()

        # --------------- publishers ---------------
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._steer_pub = self.create_publisher(Float64, '/steering_angle', 10)

        if self._publish_tf:
            self._tf_br = tf2_ros.TransformBroadcaster(self)
        else:
            self._tf_br = None

        # --------------- subscribers ---------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        if use_stamped:
            self.create_subscription(
                TwistStamped, cmd_topic, self._on_twist_stamped, qos)
        else:
            self.create_subscription(
                Twist, cmd_topic, self._on_twist, qos)

        # --------------- state ---------------
        self._last_cmd_time = self.get_clock().now()
        self._current_speed = 0.0
        self._current_steer = 0.0

        # --------------- timers ---------------
        self.create_timer(0.02, self._read_serial_loop)   # 50 Hz read
        self.create_timer(0.05, self._write_serial_loop)   # 20 Hz write

        self.get_logger().info(
            f'TeensyBridge started  port={self._port}  baud={self._baud}'
        )

    # ------------------------------------------------------------------ serial
    def _open_serial(self):
        if serial is None:
            self.get_logger().error(
                'pyserial not installed. Run: pip install pyserial')
            return
        try:
            self._ser = serial.Serial(
                self._port, self._baud, timeout=0.01)
            self.get_logger().info(f'Opened serial port {self._port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {self._port}: {e}')
            self._ser = None

    # ------------------------------------------------------------ cmd_vel sub
    def _on_twist(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        vx = msg.linear.x
        wz = msg.angular.z
        self._update_command(vx, wz)

    def _on_twist_stamped(self, msg: TwistStamped):
        self._last_cmd_time = self.get_clock().now()
        vx = msg.twist.linear.x
        wz = msg.twist.angular.z
        self._update_command(vx, wz)

    def _update_command(self, vx: float, wz: float):
        if abs(vx) < 1e-4:
            steer = 0.0
        else:
            steer = math.atan2(wz * self._wheelbase, vx)
        steer = max(-self._max_steer, min(self._max_steer, steer))
        self._current_speed = vx
        self._current_steer = steer

    # --------------------------------------------------------- serial write
    def _write_serial_loop(self):
        if self._ser is None or not self._ser.is_open:
            return

        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self._cmd_timeout:
            speed = 0.0
            steer = 0.0
        else:
            speed = self._current_speed
            steer = self._current_steer

        line = f'CMD {speed:.4f} {steer:.4f}\n'
        with self._serial_lock:
            try:
                self._ser.write(line.encode('ascii'))
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial write error: {e}')

    # ---------------------------------------------------------- serial read
    def _read_serial_loop(self):
        if self._ser is None or not self._ser.is_open:
            return
        with self._serial_lock:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                return

        if not raw:
            return
        line = raw.decode('ascii', errors='ignore').strip()
        if not line.startswith('ODOM'):
            return

        parts = line.split()
        if len(parts) < 7:
            return
        try:
            x = float(parts[1])
            y = float(parts[2])
            theta = float(parts[3])
            vx = float(parts[4])
            vyaw = float(parts[5])
            steer_angle = float(parts[6])
        except (ValueError, IndexError):
            return

        now = self.get_clock().now().to_msg()
        self._publish_odom(now, x, y, theta, vx, vyaw)
        self._publish_joint_states(now, steer_angle, vx)
        self._publish_steering(steer_angle)

    # ------------------------------------------------------------- publishers
    def _publish_odom(self, stamp, x, y, theta, vx, vyaw):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = vyaw
        self._odom_pub.publish(msg)

        if self._tf_br is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_br.sendTransform(t)

    def _publish_joint_states(self, stamp, steer_angle, vx):
        wheel_vel = vx / self._wheel_radius if self._wheel_radius > 0 else 0.0
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            'left_wheel_mount_turn',
            'right_wheel_mount_turn',
            'back_left_wheel_joint',
            'back_right_wheel_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
        ]
        msg.position = [
            steer_angle, steer_angle,
            0.0, 0.0, 0.0, 0.0,
        ]
        msg.velocity = [
            0.0, 0.0,
            wheel_vel, wheel_vel,
            wheel_vel, wheel_vel,
        ]
        self._js_pub.publish(msg)

    def _publish_steering(self, angle):
        msg = Float64()
        msg.data = angle
        self._steer_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeensyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._ser is not None and node._ser.is_open:
            with node._serial_lock:
                node._ser.write(b'CMD 0.0 0.0\n')
                node._ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
