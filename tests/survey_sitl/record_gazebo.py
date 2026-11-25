"""Record independent Gazebo ENU positions for touchdown scoring."""
import csv
from pathlib import Path
import sys
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def main():
    output = Path(sys.argv[1])
    stop = output.with_suffix(".stop")
    with output.open("x", buffering=1) as stream:
        writer = csv.writer(stream)
        writer.writerow(("sim_ns", "east_m", "north_m", "up_m"))
        last = 0

        def capture(msg):
            nonlocal last
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec
            if stamp - last < 45_000_000:
                return
            for pose in msg.pose:
                if pose.name == "x500_mono_cam_down_0":
                    p = pose.position
                    writer.writerow((stamp, p.x, p.y, p.z))
                    last = stamp
                    return

        node = Node()
        if not node.subscribe(Pose_V, "/world/mission_landing/dynamic_pose/info", capture):
            raise RuntimeError("Failed to subscribe to Gazebo poses")
        while not stop.exists():
            time.sleep(.2)


if __name__ == "__main__":
    main()
