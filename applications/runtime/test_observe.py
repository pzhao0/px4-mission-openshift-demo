"""Run via stdin inside the camera-bridge container; exercises its installed monitor."""
import json
import sys
import unittest
from unittest.mock import patch

import rclpy
from sensor_msgs.msg import CameraInfo, Image
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition

sys.path.insert(0, "/opt/mission")
from observe import Observer


class ObservationTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Observer()
        self.camera_info = CameraInfo()
        self.camera_info.k = [270.0, 0.0, 320.0, 0.0, 270.0, 240.0, 0.0, 0.0, 1.0]
        self.clock = Clock()
        self.clock.clock.sec = 1

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def receive_data(self):
        for name, msg in [("camera_info", self.camera_info), ("camera", Image()),
                          ("vehicle_status", VehicleStatus()), ("local_position", VehicleLocalPosition())]:
            self.node.receive(name, msg)

    def report(self):
        self.node.report()
        with open("/tmp/mission-status.json") as stream:
            return json.load(stream)

    def test_ros_calibration_array_produces_json_ready_boolean(self):
        with patch("observe.time.monotonic", return_value=100):
            self.node.receive("clock", self.clock)
            self.receive_data()
            status = self.report()
        self.assertIs(status["ready"], True)
        self.assertEqual(status["values"]["camera_info"]["fx"], 270.0)

    def test_repeated_frozen_clock_expires_despite_other_live_data(self):
        with patch("observe.time.monotonic", return_value=100):
            self.node.receive("clock", self.clock)
            self.receive_data()
        with patch("observe.time.monotonic", return_value=106):
            self.node.receive("clock", self.clock)
            self.receive_data()
            status = self.report()
        self.assertIs(status["ready"], False)
        self.assertEqual(status["age_seconds"]["clock"], 6)


if __name__ == "__main__":
    unittest.main()
