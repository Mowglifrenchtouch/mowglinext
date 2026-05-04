# =============================================================================
# MAVROS for ROS2 Kilted
#
# Lightweight MAVROS sidecar image for MowgliNext.
# For the full MAVROS backend bridge, use the external mowglimavros image.
# =============================================================================
FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-mavros \
    ros-kilted-mavros-msgs \
    ros-kilted-mavros-extras \
    ros-kilted-rmw-cyclonedds-cpp \
    wget \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/install_geo.sh \
 && chmod +x /tmp/install_geo.sh \
 && /tmp/install_geo.sh \
 && rm /tmp/install_geo.sh

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD []