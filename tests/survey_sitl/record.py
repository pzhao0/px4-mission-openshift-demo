"""Record aircraft telemetry and detector output without sending vehicle commands."""
import argparse
import json
import os
from pathlib import Path

import rclpy
from mission_demo.msg import SurveyObservation
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Recorder(Node):
    def __init__(self, stream):
        super().__init__("survey_sitl_recorder")
        self.stream = stream
        self.sim_ns = 0
        self.create_subscription(Clock, "/clock", lambda msg: setattr(self, "sim_ns", ns(msg.clock)),
                                 qos_profile_sensor_data)
        for cls, topic, kind, read in [
            (VehicleLocalPosition, "vehicle_local_position", "position", self.position),
            (VehicleStatus, "vehicle_status", "status",
             lambda m: {"arming_state": m.arming_state, "nav_state": m.nav_state}),
            (VehicleLandDetected, "vehicle_land_detected", "land", lambda m: {"landed": m.landed}),
        ]:
            version = f"_v{cls.MESSAGE_VERSION}" if cls.MESSAGE_VERSION else ""
            self.subscribe(cls, "/fmu/out/" + topic + version, kind, read)
        self.subscribe(SurveyObservation, "/mission/survey_observation", "decision",
                       lambda m: {"frame_ns": ns(m.stamp), "detected": m.sign_detected,
                                  "confidence": m.confidence, "face_valid": m.face_valid,
                                  "box_offset": list(map(float, m.box_offset))})
        self.subscribe(String, "/mission/demo_status", "mission", lambda m: json.loads(m.data))

    def subscribe(self, cls, topic, kind, read):
        def capture(msg):
            if self.sim_ns:
                self.stream.write(json.dumps({"kind": kind, "sim_ns": self.sim_ns, **read(msg)}) + "\n")
        self.create_subscription(cls, topic, capture, qos_profile_sensor_data)

    @staticmethod
    def position(msg):
        return {"px4_time_us": msg.timestamp_sample, "local_ned_m": [msg.x, msg.y, msg.z],
                "velocity_ned_mps": [msg.vx, msg.vy, msg.vz],
                "valid": all((msg.xy_valid, msg.z_valid, msg.v_xy_valid, msg.v_z_valid))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--namespace", required=True)
    args = parser.parse_args()
    if args.namespace != os.environ.get("SITL_NAMESPACE"):
        parser.error("Run inside the mission container of the selected SITL installation")
    stop = args.output.with_suffix(".stop")
    rclpy.init()
    with args.output.open("x", buffering=1) as stream:
        node = Recorder(stream)
        try:
            while not stop.exists() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.25)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
