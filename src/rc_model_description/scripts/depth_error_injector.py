#!/usr/bin/env python3
"""Depth Error Injector for Gazebo closed-loop simulation experiments.

Subscribes to GT depth from Gazebo, applies one of 10 error profiles
(derived from real 459-frame corridor benchmark), and publishes the
error-injected depth image.  Downstream nodes (da3_to_pointcloud.py,
Nav2 ObstacleLayer) consume the injected topic instead of raw GT.

Profiles 1-7 model the measured depth errors of DA3-Small and student
models V4-V9 using depth-bin-specific Gaussian noise whose std matches
the measured per-bin RMSE.  Profiles 8-9 add sensor dead-pixel patterns.

Usage (standalone):
  ros2 run rc_model_description depth_error_injector.py --ros-args \
      -p profile:=1 -p input_topic:=/camera/depth \
      -p output_topic:=/camera/depth_injected

Runtime profile switch:
  ros2 param set /depth_error_injector profile 3
"""

import numpy as np
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult


@dataclass(frozen=True)
class ErrorProfile:
    """Depth error profile derived from real corridor evaluation data.

    Per-bin RMSE values come from eval_corridor_depth.py run on 459 frames
    extracted from rosbag rgbd_imu_20260228_003828_0.mcap.  Values marked
    with (est) are interpolated from overall RMSE and observed bin ratios.
    """
    name: str
    near_rmse: float    # Gaussian noise std at 0.3-1.0 m
    mid_rmse: float     # Gaussian noise std at 1.0-2.0 m
    far_rmse: float     # Gaussian noise std at 2.0-4.0 m
    beyond_rmse: float  # Gaussian noise std at >4.0 m
    dead_pixel_ratio: float  # fraction of pixels set to inf [0, 1]
    description: str


PROFILES = {
    0: ErrorProfile(
        name='GT',
        near_rmse=0.0, mid_rmse=0.0, far_rmse=0.0, beyond_rmse=0.0,
        dead_pixel_ratio=0.0,
        description='Ground truth pass-through (no error injected)',
    ),
    1: ErrorProfile(
        name='DA3-Small',
        near_rmse=0.158, mid_rmse=0.486, far_rmse=1.320, beyond_rmse=1.5,
        dead_pixel_ratio=0.0,
        description='DA3-Small teacher: RMSE 0.596m, near delta<1.25=70.7%',
    ),
    2: ErrorProfile(
        name='V4',
        near_rmse=1.434, mid_rmse=1.182, far_rmse=1.283, beyond_rmse=1.4,
        dead_pixel_ratio=0.0,
        description='V4 (EfficientViT-B1, NYU-only): RMSE 1.374m, near delta<1.25=0.0%',
    ),
    3: ErrorProfile(
        name='V5',
        near_rmse=2.361, mid_rmse=2.0, far_rmse=1.469, beyond_rmse=1.5,
        dead_pixel_ratio=0.0,
        description='V5 (multi-task depth+seg): RMSE 2.186m, near delta<1.25=0.0%',
    ),
    4: ErrorProfile(
        name='V6',
        near_rmse=2.262, mid_rmse=2.0, far_rmse=1.5, beyond_rmse=1.6,
        dead_pixel_ratio=0.0,
        description='V6 (SUN+DIODE pretrain, NYU finetune): RMSE 2.158m, near delta<1.25=0.0%',
    ),
    5: ErrorProfile(
        name='V7',
        near_rmse=1.982, mid_rmse=1.453, far_rmse=0.732, beyond_rmse=0.9,
        dead_pixel_ratio=0.0,
        description='V7 corridor specialist (V5->LILocBench): RMSE 1.712m, far RMSE 0.732m',
    ),
    6: ErrorProfile(
        name='V8',
        near_rmse=2.368, mid_rmse=2.1, far_rmse=1.6, beyond_rmse=1.7,
        dead_pixel_ratio=0.0,
        description='V8 mixed (NYU+LILocBench joint): RMSE 2.266m, near delta<1.25=0.0%',
    ),
    7: ErrorProfile(
        name='V9',
        near_rmse=1.782, mid_rmse=1.3, far_rmse=1.0, beyond_rmse=1.1,
        dead_pixel_ratio=0.0,
        description='V9 (V6->LILocBench finetune): RMSE 1.589m, near delta<1.25=11.4%',
    ),
    8: ErrorProfile(
        name='sensor-fail',
        near_rmse=0.0, mid_rmse=0.0, far_rmse=0.0, beyond_rmse=0.0,
        dead_pixel_ratio=0.77,
        description='Sensor failure only: 77% dead pixels (matches Femto Bolt corridor data)',
    ),
    9: ErrorProfile(
        name='DA3+sensor-fail',
        near_rmse=0.158, mid_rmse=0.486, far_rmse=1.320, beyond_rmse=1.5,
        dead_pixel_ratio=0.77,
        description='DA3-Small noise + 77% dead pixels (worst-case combination)',
    ),
}

DEPTH_BIN_EDGES = (0.3, 1.0, 2.0, 4.0)  # near | mid | far | beyond


class DepthErrorInjector(Node):
    def __init__(self):
        super().__init__('depth_error_injector')

        self.declare_parameter('profile', 0)
        self.declare_parameter('input_topic', '/camera/depth')
        self.declare_parameter('output_topic', '/camera/depth_injected')
        self.declare_parameter('seed', 42)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 10.0)

        self._profile_id = self.get_parameter('profile').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._seed = self.get_parameter('seed').value
        self._min_depth = self.get_parameter('min_depth').value
        self._max_depth = self.get_parameter('max_depth').value

        if self._profile_id not in PROFILES:
            self.get_logger().error(
                f'Invalid profile {self._profile_id}. Valid: {list(PROFILES.keys())}')
            raise ValueError(f'Invalid profile: {self._profile_id}')

        self._profile = PROFILES[self._profile_id]
        self._rng = np.random.default_rng(self._seed)
        self._bridge = CvBridge()
        self._frame_count = 0

        self.add_on_set_parameters_callback(self._on_param_change)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub = self.create_subscription(
            Image, input_topic, self._on_depth, qos)
        self._pub = self.create_publisher(Image, output_topic, qos)

        self.get_logger().info(
            f'Depth Error Injector: profile={self._profile_id} '
            f'({self._profile.name})')
        self.get_logger().info(f'  {self._profile.description}')
        self.get_logger().info(f'  {input_topic} -> {output_topic}')

    # ------------------------------------------------------------------
    # Runtime parameter change
    # ------------------------------------------------------------------
    def _on_param_change(self, params):
        for param in params:
            if param.name == 'profile':
                new_id = param.value
                if new_id not in PROFILES:
                    return SetParametersResult(
                        successful=False,
                        reason=f'Invalid profile {new_id}. '
                               f'Valid: {list(PROFILES.keys())}')
                self._profile_id = new_id
                self._profile = PROFILES[new_id]
                self._rng = np.random.default_rng(self._seed)
                self._frame_count = 0
                self.get_logger().info(
                    f'Switched to profile {new_id} ({self._profile.name})')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Depth callback
    # ------------------------------------------------------------------
    def _on_depth(self, msg: Image):
        if self._profile_id == 0:
            self._pub.publish(msg)
            self._frame_count += 1
            return

        depth = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough').astype(np.float32)
        injected = self._inject_error(depth)

        out_msg = self._bridge.cv2_to_imgmsg(injected, encoding='32FC1')
        out_msg.header = msg.header
        self._pub.publish(out_msg)
        self._frame_count += 1

    # ------------------------------------------------------------------
    # Error injection core
    # ------------------------------------------------------------------
    def _inject_error(self, depth: np.ndarray) -> np.ndarray:
        """Apply depth-bin-specific Gaussian noise + dead pixel mask."""
        result = depth.copy()
        p = self._profile

        valid = (np.isfinite(depth)
                 & (depth >= self._min_depth)
                 & (depth <= self._max_depth))

        has_noise = (p.near_rmse > 0 or p.mid_rmse > 0
                     or p.far_rmse > 0 or p.beyond_rmse > 0)

        if valid.any() and has_noise:
            noise_std = np.zeros_like(depth)

            d_near, d_mid, d_far = DEPTH_BIN_EDGES[0], DEPTH_BIN_EDGES[1], DEPTH_BIN_EDGES[2]
            d_beyond = DEPTH_BIN_EDGES[3]

            very_near = valid & (depth < d_near)
            near  = valid & (depth >= d_near)  & (depth < d_mid)
            mid   = valid & (depth >= d_mid)   & (depth < d_far)
            far   = valid & (depth >= d_far)   & (depth < d_beyond)
            beynd = valid & (depth >= d_beyond)

            # Very-near pixels (<0.3 m) get half the near-range noise
            noise_std[very_near] = p.near_rmse * 0.5
            noise_std[near]  = p.near_rmse
            noise_std[mid]   = p.mid_rmse
            noise_std[far]   = p.far_rmse
            noise_std[beynd] = p.beyond_rmse

            noise = self._rng.standard_normal(
                size=depth.shape).astype(np.float32)
            result[valid] = depth[valid] + noise[valid] * noise_std[valid]
            result = np.clip(result, self._min_depth, self._max_depth)

        if p.dead_pixel_ratio > 0:
            dead = self._make_dead_mask(
                depth.shape[0], depth.shape[1], p.dead_pixel_ratio)
            result[dead] = np.inf

        # Preserve originally invalid pixels
        originally_invalid = ~np.isfinite(depth)
        result[originally_invalid] = np.inf

        return result

    def _make_dead_mask(self, h: int, w: int, ratio: float) -> np.ndarray:
        """Spatially correlated dead-pixel mask.

        Real Femto Bolt dead pixels cluster spatially (glass panels create
        vertical bands, specular surfaces create patches).  We approximate
        this with 8x8 block-level Bernoulli sampling.
        """
        bh = (h + 7) // 8
        bw = (w + 7) // 8
        block_dead = self._rng.random((bh, bw)) < ratio
        mask = np.repeat(np.repeat(block_dead, 8, axis=0), 8, axis=1)
        return mask[:h, :w]


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DepthErrorInjector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
