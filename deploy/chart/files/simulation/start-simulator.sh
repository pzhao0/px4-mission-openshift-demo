#!/usr/bin/env bash
set -euo pipefail
runtime=/tmp/px4-demo-runtime
mkdir -p "$runtime/models/mono_cam" "$HOME" "$XDG_DATA_HOME"
cd "$runtime"
cp "${PX4_GZ_MODELS}/mono_cam/model.config" models/mono_cam/
python3 - "${PX4_GZ_MODELS}/mono_cam/model.sdf" models/mono_cam/model.sdf <<'PY'
import sys
import xml.etree.ElementTree as ET
tree = ET.parse(sys.argv[1])
sensor = tree.find(".//sensor[@name='camera']")
# Keep raw frames small for the cluster's capped UDP receive buffers.
sensor.find("camera/image/width").text = "320"
sensor.find("camera/image/height").text = "240"
sensor.find("update_rate").text = "10"
tree.write(sys.argv[2], encoding="utf-8", xml_declaration=True)
PY
export GZ_SIM_RESOURCE_PATH="$runtime/models:${GZ_SIM_RESOURCE_PATH:-}"
# The PX4 client parameter is an IPv4 integer, not a DNS name. Resolve the
# stable Agent Service each startup; never bake a pod IP into the image.
export PX4_PARAM_UXRCE_DDS_AG_IP
PX4_PARAM_UXRCE_DDS_AG_IP=$(python3 - <<'PY'
import ipaddress, os, socket
print(int(ipaddress.IPv4Address(socket.gethostbyname(os.environ['XRCE_AGENT_HOST']))))
PY
)
gazebo_pid=""
px4_pid=""
cleanup() {
    if [[ -n "$px4_pid" ]]; then kill "$px4_pid" 2>/dev/null || true; fi
    if [[ -n "$gazebo_pid" ]]; then kill "$gazebo_pid" 2>/dev/null || true; fi
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
gz sim --headless-rendering -r -s /etc/mission-sim/mission_landing.sdf &
gazebo_pid=$!
/opt/px4-gazebo/bin/px4 -d &
px4_pid=$!
wait -n "$gazebo_pid" "$px4_pid" || exit $?
exit 1
