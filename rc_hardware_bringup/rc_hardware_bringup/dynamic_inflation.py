#!/usr/bin/env python3
"""
Dynamic inflation controller for Nav2.

Subscribes to YOLO Detection2DArray messages and adjusts Nav2's local
costmap inflation_radius parameter in real-time based on detected
obstacle classes.

Class-aware radius mapping (from offline ablation study):
    person      -> 0.30m (unpredictable movement, max safety)
    glass       -> 0.20m (invisible to LiDAR/ToF)
    furniture   -> 0.15m (static but may protrude)
    other YOLO  -> 0.12m (generic obstacle)
    no detects  -> fallback to corridor_width or fixed strategy

Uses Nav2's SetParameters service to adjust inflation_radius on the fly.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import (
    Parameter as ParameterMsg,
    ParameterValue,
    ParameterType,
)
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np

YOLO_PERSON_IDS = {0}
YOLO_FURNITURE_IDS = {56, 57, 58, 59, 60, 62, 63}

CLASS_RADII = {
    'person': 0.30,
    'furniture': 0.15,
    'other': 0.12,
}

FIXED_RADIUS = 0.09


class DynamicInflation(Node):
    def __init__(self):
        super().__init__('dynamic_inflation')

        self.declare_parameter('detection_topic', '/yolo_node/detections')
        self.declare_parameter('depth_topic', '/depth_anything_3/depth')
        self.declare_parameter('strategy', 'class_aware')
        self.declare_parameter('fallback_radius', 0.09)
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('min_radius', 0.05)
        self.declare_parameter('max_radius', 0.30)
        self.declare_parameter('costmap_node',
                               '/local_costmap/local_costmap')

        det_topic = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self._strategy = self.get_parameter('strategy').value
        self._fallback = self.get_parameter('fallback_radius').value
        self._min_r = self.get_parameter('min_radius').value
        self._max_r = self.get_parameter('max_radius').value
        costmap_node = self.get_parameter('costmap_node').value
        update_rate = self.get_parameter('update_rate').value

        self._bridge = CvBridge()
        self._latest_depth = None
        self._latest_detections = None
        self._current_radius = self._fallback

        self._det_sub = self.create_subscription(
            Detection2DArray, det_topic, self._on_detections, 10)
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth, 10)

        self._set_param_client = self.create_client(
            SetParameters,
            f'{costmap_node}/set_parameters',
        )

        self._timer = self.create_timer(1.0 / update_rate, self._update)

        self.get_logger().info(
            f'DynamicInflation started: strategy={self._strategy}, '
            f'fallback={self._fallback}m')

    def _on_detections(self, msg: Detection2DArray):
        self._latest_detections = msg

    def _on_depth(self, msg: Image):
        self._latest_depth = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough').astype(np.float32)

    def _compute_radius(self) -> float:
        if self._strategy == 'fixed':
            return self._fallback

        if self._strategy == 'class_aware':
            return self._class_aware_radius()

        if self._strategy == 'corridor_width':
            return self._corridor_width_radius()

        if self._strategy == 'min_depth':
            return self._min_depth_radius()

        return self._fallback

    def _class_aware_radius(self) -> float:
        """Compute radius based on most dangerous detected class."""
        if self._latest_detections is None:
            return self._fallback_depth_radius()

        detections = self._latest_detections.detections
        if not detections:
            return self._fallback_depth_radius()

        max_radius = 0.0
        for det in detections:
            if not det.results:
                continue
            cls_id = int(det.results[0].hypothesis.class_id)
            conf = det.results[0].hypothesis.score

            if conf < 0.4:
                continue

            if cls_id in YOLO_PERSON_IDS:
                max_radius = max(max_radius, CLASS_RADII['person'])
            elif cls_id in YOLO_FURNITURE_IDS:
                max_radius = max(max_radius, CLASS_RADII['furniture'])
            else:
                max_radius = max(max_radius, CLASS_RADII['other'])

        if max_radius > 0:
            return float(np.clip(max_radius, self._min_r, self._max_r))

        return self._fallback_depth_radius()

    def _fallback_depth_radius(self) -> float:
        """Fall back to min_depth strategy when no detections."""
        if self._latest_depth is not None:
            return self._min_depth_radius()
        return self._fallback

    def _min_depth_radius(self) -> float:
        if self._latest_depth is None:
            return self._fallback

        depth = self._latest_depth
        valid = (depth >= 0.1) & (depth <= 5.0)
        if not np.any(valid):
            return self._fallback

        min_d = np.percentile(depth[valid], 5)

        if min_d < 0.3:
            radius = 0.25
        elif min_d < 1.0:
            radius = 0.25 - (min_d - 0.3) * (0.13 / 0.7)
        elif min_d < 3.0:
            radius = 0.12 - (min_d - 1.0) * (0.07 / 2.0)
        else:
            radius = 0.05

        return float(np.clip(radius, self._min_r, self._max_r))

    def _corridor_width_radius(self) -> float:
        if self._latest_depth is None:
            return self._fallback

        depth = self._latest_depth
        h, w = depth.shape
        band = depth[int(h * 0.4):int(h * 0.6), :]

        col_depths = np.nanmedian(
            np.where((band >= 0.1) & (band <= 5.0), band, np.nan), axis=0)
        valid_cols = ~np.isnan(col_depths)
        if np.sum(valid_cols) < 2:
            return self._fallback

        valid_depths = col_depths[valid_cols]
        wall_thresh = np.percentile(valid_depths, 30)
        wall_cols = np.where(valid_cols & (col_depths <= wall_thresh))[0]
        if len(wall_cols) < 2:
            return self._fallback

        left_d = col_depths[wall_cols[0]]
        right_d = col_depths[wall_cols[-1]]
        mean_d = (left_d + right_d) / 2.0
        span = wall_cols[-1] - wall_cols[0]
        width = 2.0 * mean_d * np.tan(np.radians(40.0)) * (span / w)
        width = max(0.5, width)

        if width <= 0.5:
            radius = 0.18
        elif width >= 3.0:
            radius = 0.05
        else:
            radius = 0.18 - (width - 0.5) * (0.13 / 2.5)

        return float(np.clip(radius, self._min_r, self._max_r))

    def _update(self):
        new_radius = self._compute_radius()

        if abs(new_radius - self._current_radius) < 0.005:
            return

        self._current_radius = new_radius
        self._set_inflation_radius(new_radius)

    def _set_inflation_radius(self, radius: float):
        if not self._set_param_client.service_is_ready():
            self.get_logger().debug('SetParameters service not ready')
            return

        param = ParameterMsg()
        param.name = 'inflation_layer.inflation_radius'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = radius

        req = SetParameters.Request()
        req.parameters = [param]

        future = self._set_param_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().debug(
                f'Inflation radius set to {radius:.3f}m'))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicInflation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
