#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quat_to_mat(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-8:
        return np.eye(3)

    s = 2.0 / n
    x, y, z, w = qx, qy, qz, qw

    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s

    return np.array([
        [1.0 - (yy + zz),      xy - wz,          xz + wy],
        [xy + wz,              1.0 - (xx + zz),  yz - wx],
        [xz - wy,              yz + wx,          1.0 - (xx + yy)],
    ])


def mat_to_quat(R):
    """3x3 rotation matrix -> quaternion (x,y,z,w)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]

    trace = m00 + m11 + m22

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (m21 - m12) * s
        qy = (m02 - m20) * s
        qz = (m10 - m01) * s
    elif (m00 > m11) and (m00 > m22):
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    return np.array([qx, qy, qz, qw])


def pose_to_matrix(position, orientation):
    """geometry_msgs/Pose -> 4x4 transform matrix."""
    R = quat_to_mat(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = np.array([position.x, position.y, position.z])
    return T


class WorldOdomAligner(Node):
    """
    Build a consistent TF chain:

      world --> odom --> base_link

    using:
      - /ground_truth_pose (Pose) from Gazebo (world frame)
      - /odom (Odometry) from the controller (odom frame)

    We compute T_world_odom so that:
      T_world_odom * T_odom_base = T_world_base
    """

    def __init__(self):
        super().__init__("world_odom_aligner")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.world_frame = (
            self.get_parameter("world_frame").get_parameter_value().string_value
        )
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_gt_pose = None      # geometry_msgs/Pose
        self.last_odom_msg = None     # nav_msgs/Odometry

        self.create_subscription(
            Pose,
            "/ground_truth_pose",
            self.gt_pose_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.get_logger().info(
            f"WorldOdomAligner started "
            f"(world={self.world_frame}, odom={self.odom_frame}, base={self.base_frame})"
        )

    def gt_pose_callback(self, msg: Pose):
        self.last_gt_pose = msg
        self.maybe_publish_tf()

    def odom_callback(self, msg: Odometry):
        self.last_odom_msg = msg
        self.maybe_publish_tf()

    def maybe_publish_tf(self):
        if self.last_gt_pose is None or self.last_odom_msg is None:
            return

        odom_pose = self.last_odom_msg.pose.pose
        gt_pose   = self.last_gt_pose

        # T_world_base from Gazebo (ground truth)
        T_world_base = pose_to_matrix(
            gt_pose.position,
            gt_pose.orientation,
        )

        # T_odom_base from controller odometry
        T_odom_base = pose_to_matrix(
            odom_pose.position,
            odom_pose.orientation,
        )

        # T_world_odom = T_world_base * inv(T_odom_base)
        T_odom_base_inv = np.linalg.inv(T_odom_base)
        T_world_odom = T_world_base @ T_odom_base_inv

        # Publish world -> odom (time-stamp from odom)
        self.publish_tf_from_matrix(
            parent=self.world_frame,
            child=self.odom_frame,
            T=T_world_odom,
            stamp=self.last_odom_msg.header.stamp,
        )

    def publish_tf_from_matrix(self, parent, child, T, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child

        tf_msg.transform.translation.x = float(T[0, 3])
        tf_msg.transform.translation.y = float(T[1, 3])
        tf_msg.transform.translation.z = float(T[2, 3])

        R = T[0:3, 0:3]
        qx, qy, qz, qw = mat_to_quat(R)
        tf_msg.transform.rotation.x = float(qx)
        tf_msg.transform.rotation.y = float(qy)
        tf_msg.transform.rotation.z = float(qz)
        tf_msg.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldOdomAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
