"""Select the Gazebo survey scene and record the sign placement for evaluation."""
import argparse
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node

WORLD_FILES = Path("/etc/mission-sim")
NAME = "inspection_stop_sign"
GROUND = "survey_ground_surface"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", choices=("early", "late", "empty"))
    parser.add_argument("--ground", choices=("plain", "grass-gravel"), required=True)
    args = parser.parse_args()
    world = ET.parse(WORLD_FILES / "mission_landing.sdf")
    sign_model = world.find(f"./world/model[@name='{NAME}']")
    if sign_model is None:
        raise ValueError(f"The default Gazebo world is missing {NAME}")
    sign_pose = sign_model.find("pose")
    east, north, height, _, _, yaw = map(float, sign_pose.text.split())
    target = {"early": (east, north, yaw), "late": (20., 42.6, -63), "empty": None}[args.scene]
    sign_model.remove(sign_pose)  # Gazebo receives the selected pose in EntityFactory.
    terrain_sdf = None
    texture_digest = None
    if args.ground == "grass-gravel":
        terrain_sdf = (WORLD_FILES / "survey-ground.sdf").read_text()
        texture_digest = hashlib.sha256((WORLD_FILES / "grass-gravel.jpg").read_bytes()).hexdigest()
    node = Node()

    def request(endpoint, msg, response=Boolean):
        ok, result = node.request("/world/mission_landing/" + endpoint, msg, type(msg), response, 8000)
        if not ok or (response is Boolean and not result.data):
            raise RuntimeError(f"Gazebo rejected {endpoint}")
        return result

    def find_model(name):
        return next((m for m in request("scene/info", Empty(), Scene).model if m.name == name), None)

    def wait_model(name, present):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            model = find_model(name)
            if (model is not None) == present:
                return model
            time.sleep(.1)
        raise RuntimeError(f"Gazebo scene did not report {name} present={present}")

    for name in (NAME, GROUND):
        if find_model(name) is not None:
            request("remove/blocking", Entity(name=name, type=Entity.MODEL))
            wait_model(name, False)
    if terrain_sdf:
        request("create/blocking", EntityFactory(name=GROUND, sdf=terrain_sdf))
    wait_model(GROUND, terrain_sdf is not None)
    if target:
        east, north, yaw = target
        pose = Pose(name=NAME)
        pose.position.x, pose.position.y, pose.position.z = east, north, height
        pose.orientation.w = math.cos(math.radians(yaw) / 2)
        pose.orientation.z = math.sin(math.radians(yaw) / 2)
        sdf = ET.Element("sdf", version="1.9")
        sdf.append(sign_model)
        spawn = EntityFactory(name=NAME, sdf=ET.tostring(sdf, encoding="unicode"))
        spawn.pose.CopyFrom(pose)
        request("create/blocking", spawn)
    sign = wait_model(NAME, target is not None)
    measured = None
    if sign is not None:
        center = sign.pose.position
        measured = [center.x, center.y]
        if math.dist(measured, target[:2]) > .01:
            raise RuntimeError("Spawned sign differs from requested placement")
    print(json.dumps({"scene": args.scene, "ground": args.ground, "texture_sha256": texture_digest,
                      "placement_enu": target, "gazebo_sign_en_m": measured}))


if __name__ == "__main__":
    main()
