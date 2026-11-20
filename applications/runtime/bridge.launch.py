"""Gazebo access stays in the simulator pod; no detector or flight controller here."""
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    world = os.environ.get("PX4_GZ_WORLD", "mission_landing")
    model = os.environ.get("GZ_MODEL_NAME", "x500_mono_cam_down_0")
    camera = f"/world/{world}/model/{model}/link/camera_link/sensor/camera"
    return LaunchDescription([
        Node(package="ros_gz_image", executable="image_bridge",
             name="camera_image_bridge", arguments=[f"{camera}/image"],
             remappings=[(f"{camera}/image", "/camera")],
             parameters=[{"qos": "sensor_data"}], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="camera_clock_bridge", output="screen", arguments=[
                 f"{camera}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                 "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
             remappings=[(f"{camera}/camera_info", "/camera_info")]),
    ])
