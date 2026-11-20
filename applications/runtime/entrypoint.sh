#!/usr/bin/env bash
set -eo pipefail
mkdir -p "$HOME" "$ROS_LOG_DIR"
source /opt/ros/jazzy/setup.bash
source /opt/mission_ws/install/local_setup.bash
case "${1:-}" in
  mission) exec ros2 launch /opt/mission/mission.launch.py ;;
  bridge) exec ros2 launch /opt/mission/bridge.launch.py ;;
  agent) exec MicroXRCEAgent udp4 -p 8888 -v 3 ;;
  *) exec "$@" ;;
esac
