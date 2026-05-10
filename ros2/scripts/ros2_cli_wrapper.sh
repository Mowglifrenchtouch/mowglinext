#!/bin/bash
set -e

# docker exec does not pass through the container entrypoint, so source the
# ROS environment here before delegating to the real ros2 CLI.
source /opt/ros/kilted/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec /opt/ros/kilted/bin/ros2 "$@"
