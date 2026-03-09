#!/usr/bin/env python3
"""
ROS2 node that runs Depth Anything 3 inference on live RGB images.

Subscribes to an RGB camera topic, runs DA3 depth estimation on GPU,
and publishes metric-scale depth as a float32 Image message on
/depth_anything_3/depth.  Downstream nodes (da3_to_pointcloud, Nav2)
consume this topic unchanged.

Metric depth conversion:
  - DA3-Metric models (da3metric-*): focal * raw / 300
  - DA3-Relative models (da3-small/base/large): optional median-scaling
    against the sensor depth topic, or the same focal formula as a
    rough approximation (controlled by --scale_mode parameter).

Rate-limiting: only one inference runs at a time; frames that arrive
while the GPU is busy are silently dropped.
"""

import threading
import time

import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class Da3InferenceNode(Node):

    def __init__(self):
        super().__init__('da3_inference_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('model_id', 'depth-anything/DA3-SMALL')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/depth_anything_3/depth')
        self.declare_parameter('sensor_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('process_res', 504)
        self.declare_parameter('scale_mode', 'focal')
        self.declare_parameter('device', 'cuda')

        model_id = self.get_parameter('model_id').value
        rgb_topic = self.get_parameter('rgb_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        sensor_depth_topic = self.get_parameter('sensor_depth_topic').value
        self._process_res = self.get_parameter('process_res').value
        self._scale_mode = self.get_parameter('scale_mode').value
        device_str = self.get_parameter('device').value

        self._bridge = CvBridge()
        self._focal = None
        self._busy = False
        self._lock = threading.Lock()
        self._sensor_depth = None
        self._is_metric_model = 'metric' in model_id.lower()
        self._frame_count = 0
        self._total_infer_ms = 0.0

        # ── Load model ──────────────────────────────────────────────
        self.get_logger().info(f'Loading DA3 model: {model_id} ...')
        t0 = time.time()
        from depth_anything_3.api import DepthAnything3
        self._device = torch.device(device_str if torch.cuda.is_available() else 'cpu')
        self._model = DepthAnything3.from_pretrained(model_id)
        self._model = self._model.to(device=self._device)
        self._model.eval()
        load_s = time.time() - t0
        self.get_logger().info(
            f'DA3 loaded on {self._device} in {load_s:.1f}s  '
            f'(metric={self._is_metric_model}, scale_mode={self._scale_mode})')

        # ── QoS ─────────────────────────────────────────────────────
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers / publishers ────────────────────────────────
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, 10)
        self._rgb_sub = self.create_subscription(
            Image, rgb_topic, self._on_rgb, qos_sensor)
        self._depth_pub = self.create_publisher(Image, depth_topic, 10)
        self._depth_vis_pub = self.create_publisher(
            Image, depth_topic + '/colormap', 10)

        if self._scale_mode == 'median':
            self._sensor_depth_sub = self.create_subscription(
                Image, sensor_depth_topic, self._on_sensor_depth, qos_sensor)
            self.get_logger().info(
                f'Median-scaling enabled — subscribing to {sensor_depth_topic}')

        self.get_logger().info(
            f'DA3 inference node ready: {rgb_topic} → {depth_topic}')

    # ── Callbacks ───────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo):
        if self._focal is None:
            fx, fy = msg.k[0], msg.k[4]
            self._focal = (fx + fy) / 2.0
            self.get_logger().info(f'Camera focal length: {self._focal:.1f} px')

    def _on_sensor_depth(self, msg: Image):
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth = depth.astype(np.float32)
        if depth.max() > 100.0:
            depth = depth / 1000.0
        self._sensor_depth = depth

    def _on_rgb(self, msg: Image):
        if self._focal is None:
            return

        with self._lock:
            if self._busy:
                return
            self._busy = True

        try:
            self._run_inference(msg)
        except Exception as e:
            self.get_logger().error(f'DA3 inference failed: {e}')
        finally:
            with self._lock:
                self._busy = False

    # ── Inference ───────────────────────────────────────────────────

    def _run_inference(self, msg: Image):
        t0 = time.time()

        rgb_np = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        orig_h, orig_w = rgb_np.shape[:2]

        prediction = self._model.inference(
            [rgb_np],
            process_res=self._process_res,
        )
        raw_depth = prediction.depth[0]  # [H_proc, W_proc] float32

        metric_depth = self._to_metric(raw_depth, orig_h, orig_w)

        if metric_depth.shape != (orig_h, orig_w):
            from PIL import Image as PILImage
            depth_pil = PILImage.fromarray(metric_depth)
            depth_pil = depth_pil.resize((orig_w, orig_h), PILImage.BILINEAR)
            metric_depth = np.array(depth_pil, dtype=np.float32)

        out_msg = self._bridge.cv2_to_imgmsg(metric_depth, encoding='32FC1')
        out_msg.header = msg.header
        self._depth_pub.publish(out_msg)

        if self._depth_vis_pub.get_subscription_count() > 0:
            self._publish_colormap(metric_depth, msg.header)

        elapsed_ms = (time.time() - t0) * 1000
        self._frame_count += 1
        self._total_infer_ms += elapsed_ms
        if self._frame_count % 30 == 0:
            avg = self._total_infer_ms / self._frame_count
            fps = 1000.0 / avg if avg > 0 else 0
            rng = f'{metric_depth.min():.2f}–{metric_depth.max():.2f}m'
            self.get_logger().info(
                f'[DA3] frame {self._frame_count}: '
                f'{elapsed_ms:.0f}ms (avg {avg:.0f}ms, ~{fps:.1f}fps), '
                f'depth {rng}')

    def _publish_colormap(self, depth: np.ndarray, header):
        """Publish a TURBO-colormapped depth image for visualization."""
        valid = depth[depth > 0.01]
        if len(valid) == 0:
            return
        d_min, d_max = float(valid.min()), min(float(np.percentile(valid, 98)), 8.0)
        norm = np.clip((depth - d_min) / max(d_max - d_min, 1e-3), 0, 1)
        colored = cv2.applyColorMap(
            (norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        vis_msg = self._bridge.cv2_to_imgmsg(colored, encoding='bgr8')
        vis_msg.header = header
        self._depth_vis_pub.publish(vis_msg)

    def _to_metric(self, raw: np.ndarray, orig_h: int, orig_w: int) -> np.ndarray:
        """Convert raw DA3 output to metric depth (metres)."""
        if self._is_metric_model or self._scale_mode == 'focal':
            return (self._focal * raw / 300.0).astype(np.float32)

        if self._scale_mode == 'median' and self._sensor_depth is not None:
            sensor = self._sensor_depth
            if sensor.shape != raw.shape:
                from PIL import Image as PILImage
                sensor = np.array(
                    PILImage.fromarray(sensor).resize(
                        (raw.shape[1], raw.shape[0]), PILImage.BILINEAR),
                    dtype=np.float32)
            valid = (sensor > 0.1) & (sensor < 10.0) & (raw > 1e-6)
            if valid.sum() > 100:
                scale = float(np.median(sensor[valid]) / np.median(raw[valid]))
                return (raw * scale).astype(np.float32)

        return (self._focal * raw / 300.0).astype(np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = Da3InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
