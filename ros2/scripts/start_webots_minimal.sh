#!/bin/bash
# =============================================================================
# start_webots_minimal.sh
#
# Starts the minimal Webots backend inside the simulation container.
# If no DISPLAY is present, it boots an Xvfb server on :99 so Webots can run
# headlessly without depending on an external X server. It also mirrors that
# display through x11vnc + noVNC so the Webots UI can be reached from a browser.
# =============================================================================
set -euo pipefail

WEBOTS_MODE="${WEBOTS_MODE:-realtime}"
WEBOTS_WORLD="${WEBOTS_WORLD:-mowgli_garden.wbt}"
XVFB_DISPLAY="${XVFB_DISPLAY:-:99}"
XVFB_SCREEN="${XVFB_SCREEN:-1280x720x24}"
VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
XVFB_PID=""
LAUNCH_PID=""
OPENBOX_PID=""
X11VNC_PID=""
WEBSOCKIFY_PID=""

cleanup() {
    if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        kill "${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
    if [ -n "${WEBSOCKIFY_PID}" ] && kill -0 "${WEBSOCKIFY_PID}" 2>/dev/null; then
        kill "${WEBSOCKIFY_PID}" 2>/dev/null || true
        wait "${WEBSOCKIFY_PID}" 2>/dev/null || true
    fi
    if [ -n "${X11VNC_PID}" ] && kill -0 "${X11VNC_PID}" 2>/dev/null; then
        kill "${X11VNC_PID}" 2>/dev/null || true
        wait "${X11VNC_PID}" 2>/dev/null || true
    fi
    if [ -n "${OPENBOX_PID}" ] && kill -0 "${OPENBOX_PID}" 2>/dev/null; then
        kill "${OPENBOX_PID}" 2>/dev/null || true
        wait "${OPENBOX_PID}" 2>/dev/null || true
    fi
    if [ -n "${XVFB_PID}" ] && kill -0 "${XVFB_PID}" 2>/dev/null; then
        kill "${XVFB_PID}" 2>/dev/null || true
        wait "${XVFB_PID}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

set +u
source /opt/ros/kilted/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi
set -u

export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"
export ROS2_WEBOTS_HOME="${ROS2_WEBOTS_HOME:-${WEBOTS_HOME}}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY="${XVFB_DISPLAY}"
fi

if command -v Xvfb >/dev/null 2>&1 && ! xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
    display_num="${DISPLAY#:}"
    rm -f "/tmp/.X${display_num}-lock"
    mkdir -p /tmp/.X11-unix /tmp/runtime-root
    chmod 1777 /tmp/.X11-unix
    chmod 700 /tmp/runtime-root
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    Xvfb "${DISPLAY}" -screen 0 "${XVFB_SCREEN}" -ac +extension GLX -noreset >/tmp/xvfb-webots.log 2>&1 &
    XVFB_PID=$!
    sleep 1
fi

if command -v openbox >/dev/null 2>&1; then
    DISPLAY="${DISPLAY}" openbox --sm-disable >/tmp/openbox-webots.log 2>&1 &
    OPENBOX_PID=$!
fi

if command -v x11vnc >/dev/null 2>&1; then
    x11vnc \
        -display "${DISPLAY}" \
        -rfbport "${VNC_PORT}" \
        -forever \
        -shared \
        -nopw \
        -listen 0.0.0.0 \
        -xkb \
        >/tmp/x11vnc-webots.log 2>&1 &
    X11VNC_PID=$!
fi

if command -v websockify >/dev/null 2>&1; then
    websockify \
        --web /usr/share/novnc/ \
        "${NOVNC_PORT}" \
        "localhost:${VNC_PORT}" \
        >/tmp/novnc-webots.log 2>&1 &
    WEBSOCKIFY_PID=$!
fi

echo "=== Starting Webots minimal backend ==="
echo "  DISPLAY=${DISPLAY}"
echo "  WEBOTS_HOME=${WEBOTS_HOME}"
echo "  world=${WEBOTS_WORLD}"
echo "  mode=${WEBOTS_MODE}"
echo "  VNC=0.0.0.0:${VNC_PORT}"
echo "  noVNC=http://IP_VM:${NOVNC_PORT}/vnc.html"

ros2 launch mowgli_simulation webots_minimal.launch.py \
    world:=${WEBOTS_WORLD} \
    mode:=${WEBOTS_MODE} &
LAUNCH_PID=$!

wait_for_controller_manager() {
    local attempts=0
    until ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_controllers$'; do
        attempts=$((attempts + 1))
        if [ "${attempts}" -ge 90 ]; then
            echo "Timed out waiting for /controller_manager/list_controllers" >&2
            return 1
        fi
        sleep 1
    done
}

spawn_controller_if_missing() {
    local controller_name="$1"
    if ros2 service call /controller_manager/list_controllers \
        controller_manager_msgs/srv/ListControllers "{}" 2>/dev/null \
        | grep -q "name='${controller_name}'"; then
        return 0
    fi

    ros2 run controller_manager spawner "${controller_name}" --controller-manager-timeout 90
}

wait_for_controller_manager
spawn_controller_if_missing joint_state_broadcaster
spawn_controller_if_missing diffdrive_controller

wait "${LAUNCH_PID}"
