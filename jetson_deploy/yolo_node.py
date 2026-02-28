#!/usr/bin/env python3
"""
ROS2 node wrapping YOLOv8-nano TensorRT for real-time object detection.

Subscribes to RGB images, publishes detections as Detection2DArray
and annotated images for visualization.

Designed for Jetson Orin Nano 8GB — runs at 60+ FPS standalone,
~20 Hz when staggered with DA3.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge


CLASSES_NAV = {
    0: "person",
    56: "chair",
    57: "couch",
    58: "potted_plant",
    59: "bed",
    60: "dining_table",
    62: "tv",
    63: "laptop",
}


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("engine_path", "~/yolov8n.engine")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("run_every_n", 3)

        engine_path = self.get_parameter("engine_path").value
        self.conf_thresh = self.get_parameter("confidence_threshold").value
        image_topic = self.get_parameter("image_topic").value
        self.run_every_n = self.get_parameter("run_every_n").value

        self.get_logger().info(f"Loading YOLO engine: {engine_path}")
        from ultralytics import YOLO
        self.model = YOLO(engine_path, task="detect")
        self.get_logger().info("YOLO loaded")

        self.bridge = CvBridge()
        self.frame_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos)

        self.det_pub = self.create_publisher(
            Detection2DArray, "~/detections", 10)
        self.img_pub = self.create_publisher(
            Image, "~/detection_image", qos)

        self.get_logger().info(
            f"YoloNode ready — subscribed to {image_topic}, "
            f"running every {self.run_every_n} frames")

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.run_every_n != 0:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)

        det_array = Detection2DArray()
        det_array.header = msg.header

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float((x1 + x2) / 2)
                det.bbox.center.position.y = float((y1 + y2) / 2)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls_id)
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array.detections.append(det)

        self.det_pub.publish(det_array)

        if self.img_pub.get_subscription_count() > 0:
            annotated = results[0].plot()
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = msg.header
            self.img_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
