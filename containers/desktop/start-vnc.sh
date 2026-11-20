#!/usr/bin/env bash
# Reuse the installed desktop tools without the old LiDAR/grid-asset coupling.
set -euo pipefail
export DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR="/tmp/runtime-$(id -u)"
mkdir -p "$HOME" "$XDG_CONFIG_HOME" "$XDG_CACHE_HOME" "$XDG_RUNTIME_DIR"
chmod 0700 "$XDG_RUNTIME_DIR"
cd "$HOME"
sed 's/x500_lidar_2d_0/x500_mono_cam_down_0/g' \
    /usr/local/share/gazebo-vnc/gui.config > "$HOME/gui.config"
pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
Xvfb "$DISPLAY" -screen 0 1600x900x24 -nolisten tcp &
pids+=("$!")
for attempt in $(seq 1 50); do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then break; fi
    sleep 0.2
done
xdpyinfo -display "$DISPLAY" >/dev/null
gz sim -g --gui-config "$HOME/gui.config" &
pids+=("$!")
x11vnc -display "$DISPLAY" -nopw -rfbport 5901 -localhost -forever -shared -noxdamage &
pids+=("$!")
python3 /etc/mission-sim/novnc_server.py &
pids+=("$!")
wait -n "${pids[@]}" || exit $?
exit 1
