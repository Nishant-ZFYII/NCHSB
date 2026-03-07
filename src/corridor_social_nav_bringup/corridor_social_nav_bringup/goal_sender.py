"""
Navigation goal sender for automated experiment trials.

Sends a NavigateToPose action goal and monitors for completion
or timeout. Reports result via /trial_result topic and then exits.

Includes retry logic: bt_navigator creates its action server during
the 'configure' lifecycle phase but only accepts goals after the
'activate' phase.  If a goal is rejected, we retry after a short
delay (up to max_retries times).
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class GoalSender(Node):
    MAX_RETRIES = 10
    RETRY_DELAY_S = 3.0

    def __init__(self):
        super().__init__('goal_sender')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('timeout_s', 120.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.timeout_s = self.get_parameter('timeout_s').value

        self.result_pub = self.create_publisher(String, '/trial_result', 10)

        self.action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            f'GoalSender: target=({self.goal_x:.1f}, {self.goal_y:.1f}, '
            f'yaw={self.goal_yaw:.2f}), timeout={self.timeout_s}s')

        self.goal_sent = False
        self.start_time = None
        self._retries_remaining = self.MAX_RETRIES
        self._retry_timer = None
        self._timeout_timer = None
        self._done = False

        self.create_timer(2.0, self.send_goal_once)

    def send_goal_once(self):
        if self.goal_sent:
            return
        self.goal_sent = True

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.action_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('Action server not available after 60s')
            self.publish_result('TIMEOUT_NO_SERVER')
            return

        self.get_logger().info('Action server found, sending goal...')
        self._attempt_send_goal()

    def _attempt_send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)

        attempt = self.MAX_RETRIES - self._retries_remaining + 1
        self.get_logger().info(
            f'Sending goal: ({self.goal_x:.1f}, {self.goal_y:.1f}) '
            f'[attempt {attempt}/{self.MAX_RETRIES + 1}]')

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        if self._done:
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            if self._retries_remaining > 0:
                self._retries_remaining -= 1
                self.get_logger().warn(
                    f'Goal rejected (Nav2 still initializing), '
                    f'retrying in {self.RETRY_DELAY_S:.0f}s '
                    f'({self._retries_remaining} retries left)...')
                self._retry_timer = self.create_timer(
                    self.RETRY_DELAY_S, self._on_retry)
                return
            self.get_logger().warn('Goal rejected by Nav2 after all retries')
            self.publish_result('REJECTED')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

        if self._timeout_timer is None:
            self._timeout_timer = self.create_timer(
                self.timeout_s, self.timeout_cb)

    def _on_retry(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self._attempt_send_goal()

    def result_cb(self, future):
        if self._done:
            return
        result = future.result()
        status = result.status

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if status == 4:
            self.get_logger().info(
                f'Goal SUCCEEDED in {elapsed:.1f}s')
            self.publish_result(f'SUCCESS:{elapsed:.2f}')
        elif status == 6:
            self.get_logger().warn(
                f'Goal ABORTED after {elapsed:.1f}s')
            self.publish_result(f'ABORTED:{elapsed:.2f}')
        elif status == 5:
            self.get_logger().warn(
                f'Goal CANCELED after {elapsed:.1f}s')
            self.publish_result(f'CANCELED:{elapsed:.2f}')
        else:
            self.get_logger().warn(
                f'Goal ended with status {status} after {elapsed:.1f}s')
            self.publish_result(f'UNKNOWN:{elapsed:.2f}')

    def feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        if remaining < 0.5:
            self.get_logger().info(
                f'Almost there: {remaining:.2f}m remaining')

    def timeout_cb(self):
        if self._done:
            return
        elapsed = 0.0
        if self.start_time:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.get_logger().warn(
            f'Trial TIMEOUT after {elapsed:.1f}s (limit={self.timeout_s}s)')
        self.publish_result(f'TIMEOUT:{elapsed:.2f}')

    def publish_result(self, result_str: str):
        if self._done:
            return
        self._done = True
        msg = String()
        msg.data = result_str
        self.result_pub.publish(msg)
        self.get_logger().info(f'Published trial result: {result_str}')
        self.create_timer(1.0, self._shutdown)

    def _shutdown(self):
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
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
