#!/usr/bin/env bash
# Start the dual-stack Luxonis driver.
#   CAMERA_NAME  node name on both graphs (default: luxonis_camera)
#   CONFIG_FILE  ROS2 params file (default: the tote camera config baked into
#                the image; the top-level key must be /<CAMERA_NAME>)
set -o errexit

CAMERA_NAME="${CAMERA_NAME:-luxonis_camera_source_tote}"
CONFIG_FILE="${CONFIG_FILE:-/config/luxonis_camera_source_tote_ros2.yaml}"

exec ros2 run depthai_ros_driver_v3 driver_node --ros-args \
    -r __node:="${CAMERA_NAME}" \
    --params-file "${CONFIG_FILE}"
