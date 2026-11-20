#!/usr/bin/env python3
"""Read-only, wall-clock freshness monitor; never sends vehicle commands."""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition


class Observer(Node):
    def __init__(self):
        super().__init__("mission_observer")
        self.received = {}
        self.values = {}
        self.clock_value = None
        self.subscriptions_ = []
        # Version suffixes are part of the PX4 message interface.
        status_version = VehicleStatus.MESSAGE_VERSION
        status_topic = "/fmu/out/vehicle_status" + (f"_v{status_version}" if status_version else "")
        local_version = VehicleLocalPosition.MESSAGE_VERSION
        local_topic = "/fmu/out/vehicle_local_position" + (f"_v{local_version}" if local_version else "")
        for name, cls, topic in [
            ("clock", Clock, "/clock"), ("camera", Image, "/camera"),
            ("camera_info", CameraInfo, "/camera_info"),
            ("vehicle_status", VehicleStatus, status_topic),
            ("local_position", VehicleLocalPosition, local_topic),
            ("target_pose", PoseStamped, "/target_pose"),
        ]:
            self.subscriptions_.append(self.create_subscription(
                cls, topic, lambda msg, key=name: self.receive(key, msg), qos_profile_sensor_data))
        self.create_timer(1.0, self.report)

    def receive(self, key, msg):
        if key == "clock":
            stamp = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
            if stamp == self.clock_value:
                return
            self.clock_value = stamp
        self.received[key] = time.monotonic()
        if key == "camera":
            self.values[key] = {"width": msg.width, "height": msg.height, "encoding": msg.encoding}
        elif key == "camera_info":
            # ROS fixed-length arrays are NumPy arrays in this distribution.
            # Convert scalars before comparisons/JSON encoding (numpy.bool_
            # from a focal-length comparison is not JSON serializable).
            self.values[key] = {"fx": float(msg.k[0]), "fy": float(msg.k[4])}
        elif key == "vehicle_status":
            self.values[key] = {"arming_state": msg.arming_state, "nav_state": msg.nav_state}
        elif key == "local_position":
            self.values[key] = {"x": msg.x, "y": msg.y, "z": msg.z,
                                "xy_valid": msg.xy_valid, "z_valid": msg.z_valid}
        elif key == "target_pose":
            self.values[key] = {"x": msg.pose.position.x, "y": msg.pose.position.y,
                                "z": msg.pose.position.z, "frame": msg.header.frame_id}

    def report(self):
        now = time.monotonic()
        ages = {key: round(now - stamp, 3) for key, stamp in self.received.items()}
        required = ("clock", "camera", "camera_info", "vehicle_status", "local_position")
        ready = all(ages.get(key, 999) < 5 for key in required)
        ready = ready and self.values.get("camera_info", {}).get("fx", 0) > 0
        result = {"observed_wall_time": time.time(), "ready": ready,
                  "age_seconds": ages, "values": self.values}
        with open("/tmp/mission-status.json.tmp", "w") as stream:
            json.dump(result, stream)
        os.replace("/tmp/mission-status.json.tmp", "/tmp/mission-status.json")


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(Observer())
