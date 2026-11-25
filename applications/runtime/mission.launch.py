"""Companion-only launch: no Gazebo processes, world spawning, or PX4 checkout."""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    survey_enabled = os.environ.get("SURVEY_ENABLED") == "true"
    nodes = [
        Node(package="aruco_tracker", executable="aruco_tracker", name="aruco_tracker",
             parameters=[{"use_sim_time": True, "aruco_id": 0,
                          "dictionary": 2, "marker_size": 0.5}], output="screen"),
        Node(package="mission_demo", executable="mission_demo",
             name="mission_executor", parameters=[{"use_sim_time": True,
                                                   "survey_enabled": survey_enabled}], output="screen"),
        ExecuteProcess(cmd=["python3", "/opt/mission/observe.py"], output="screen"),
    ]
    # Registration failures must fail the container, rather than leave a detector
    # running beside a silently exited controller.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason="Mission process exited"))]
    )) for node in nodes]
    if survey_enabled:
        nodes.append(ExecuteProcess(
            cmd=["python3", "/opt/mission/survey_detector.py"], output="screen",
            respawn=True, respawn_delay=2.0,
            additional_env={"PYTHONPATH": "/opt/mission/onnx:" + os.environ.get("PYTHONPATH", "")}))
    return LaunchDescription(handlers + nodes)
