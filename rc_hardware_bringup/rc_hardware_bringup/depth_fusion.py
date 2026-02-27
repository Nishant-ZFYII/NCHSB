#!/usr/bin/env python3
"""
Confidence-gated depth fusion node.

Fuses hardware ToF depth from the Orbbec Femto Bolt with a lightweight
neural depth estimator (student model distilled from Depth Anything V2).

Per-pixel fusion rule (from the IROS paper):
    d_fused(p) = d_ToF(p)     if confidence(p) >= tau
                 d_student(p)  if confidence(p) <  tau

The student model is only triggered when the fraction of low-confidence
pixels exceeds alpha (default 5%), saving GPU cycles.

This node is a skeleton — the actual TensorRT student model inference
is marked with TODO placeholders.  The rest of the ROS plumbing
(subscribers, publisher, triggering logic) is fully functional.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthFusion(Node):
    def __init__(self):
        super().__init__('depth_fusion')

        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('trigger_fraction', 0.05)
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('confidence_topic', '/camera/depth/confidence')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('fused_topic', '/depth/fused')

        self._tau = self.get_parameter('confidence_threshold').value
        self._alpha = self.get_parameter('trigger_fraction').value

        depth_topic = self.get_parameter('depth_topic').value
        conf_topic = self.get_parameter('confidence_topic').value
        rgb_topic = self.get_parameter('rgb_topic').value
        fused_topic = self.get_parameter('fused_topic').value

        self._bridge = CvBridge()

        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth, 10)
        self._conf_sub = self.create_subscription(
            Image, conf_topic, self._on_confidence, 10)
        self._rgb_sub = self.create_subscription(
            Image, rgb_topic, self._on_rgb, 10)

        self._fused_pub = self.create_publisher(Image, fused_topic, 10)

        self._latest_depth = None
        self._latest_confidence = None
        self._latest_rgb = None

        self.get_logger().info('DepthFusion node started')

    def _on_depth(self, msg: Image):
        self._latest_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self._try_fuse(msg.header)

    def _on_confidence(self, msg: Image):
        self._latest_confidence = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _on_rgb(self, msg: Image):
        self._latest_rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _try_fuse(self, header):
        if self._latest_depth is None:
            return

        depth = self._latest_depth.astype(np.float32)

        if self._latest_confidence is not None:
            conf = self._latest_confidence.astype(np.float32)
            if conf.max() > 1.0:
                conf = conf / conf.max()
        else:
            conf = np.ones_like(depth)

        low_conf_frac = np.mean(conf < self._tau)

        if low_conf_frac > self._alpha and self._latest_rgb is not None:
            student_depth = self._run_student_model(self._latest_rgb)
            if student_depth is not None:
                mask = conf < self._tau
                depth[mask] = student_depth[mask]
                self.get_logger().debug(
                    f'Fused {np.sum(mask)} low-confidence pixels '
                    f'({low_conf_frac*100:.1f}% of frame)')

        fused_msg = self._bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        fused_msg.header = header
        self._fused_pub.publish(fused_msg)

    def _run_student_model(self, rgb_image: np.ndarray):
        """
        Run the distilled MobileNetV3-Small student model on an RGB frame.

        TODO: Load TensorRT engine and run inference.
        Expected workflow:
            1. Preprocess: resize to model input (e.g. 256x256), normalize
            2. Run TensorRT Int8 engine
            3. Post-process: resize output depth map back to original resolution
            4. Return np.ndarray of shape (H, W) with metric depth in meters
        """
        self.get_logger().debug(
            'Student model inference placeholder — returning None')
        return None


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
