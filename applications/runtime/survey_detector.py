"""Score the newest distinct camera frame on the mission CPU."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import os
from pathlib import Path

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from stop_sign import StopSignDetector, inspection_point
from mission_demo.msg import SurveyObservation


class SurveyDetector(Node):
    def __init__(self, model, worker):
        super().__init__("survey_detector")
        self.detector = StopSignDetector(model)
        self.intrinsics = None
        self.get_logger().info(f"Loaded survey ONNX model {model} sha256={hashlib.sha256(model.read_bytes()).hexdigest()}")
        self.bridge = CvBridge()
        self.worker = worker
        self.job = None
        self.latest = None
        self.last_seen = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(SurveyObservation, "/mission/survey_observation", qos)
        self.preview = self.create_publisher(CompressedImage, "/mission/survey_preview", 5)
        self.camera = self.create_subscription(Image, "/camera", self.receive, qos)
        self.create_subscription(CameraInfo, "/camera_info", self.camera_info, qos)
        self.create_timer(0.01, self.poll)

    def camera_info(self, info):
        if info.k[0] > 0 and info.k[4] > 0:
            self.intrinsics = np.array(info.k, dtype=np.float64).reshape(3, 3)

    def receive(self, image):
        stamp = image.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if ns > self.last_seen:
            self.last_seen = ns
            self.latest = image

    def score(self, image):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        score, box = self.detector.detect(frame)
        pose = inspection_point(frame, box, self.intrinsics)
        decision = SurveyObservation(stamp=image.header.stamp, confidence=score,
                                     sign_detected=box is not None, face_valid=pose is not None)
        if pose is not None:
            decision.box_midpoint_on_sign_m = pose
        if box is not None:
            h, w = frame.shape[:2]
            decision.box_offset = [((box[0]+box[2])/w - 1)/2, ((box[1]+box[3])/h - 1)/2]
        return decision, frame, box, score, image.header

    def publish_preview(self, frame, box, score, header):
        height, width = frame.shape[:2]
        canvas = frame.copy()
        cv2.drawMarker(canvas, (width // 2, height // 2), (255, 255, 255),
                       cv2.MARKER_CROSS, 11, 1, cv2.LINE_AA)
        if box is not None:
            x1, y1, x2, y2 = box
            color = (121, 245, 167)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            midpoint = (round((x1 + x2) / 2), round((y1 + y2) / 2))
            cv2.circle(canvas, midpoint, 3, color, 1, cv2.LINE_AA)
            x = max(0, min(x1, width - 120))
            y = y1 - 4 if y1 >= 18 else min(height - 4, y2 + 16)
            label = f"stop sign {score:.2f}"
            cv2.putText(canvas, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1, cv2.LINE_AA)
        msg = self.bridge.cv2_to_compressed_imgmsg(canvas, dst_format="jpg")
        msg.header = header
        self.preview.publish(msg)

    def poll(self):
        if self.job is not None:
            if not self.job.done():
                return
            try:
                decision, frame, box, score, header = self.job.result()
                self.publisher.publish(decision)
                if self.preview.get_subscription_count():
                    self.publish_preview(frame, box, score, header)
            except Exception as exc:
                self.get_logger().error(f"Survey inference failed: {exc}", throttle_duration_sec=5)
            self.job = None
        if self.latest is not None:
            self.job = self.worker.submit(self.score, self.latest)
            self.latest = None


def main():
    rclpy.init()
    try:
        with ThreadPoolExecutor(max_workers=1) as worker:
            node = SurveyDetector(Path(os.environ["SURVEY_ONNX_PATH"]), worker)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
