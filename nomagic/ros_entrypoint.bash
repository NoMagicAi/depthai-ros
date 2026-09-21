#!/usr/bin/env bash
# NoMagic dual-stack entrypoint. Source order matters (ROS1_AND_2_SUPPORT.md
# section 4.6): ROS1 first, ROS2 workspace LAST so ROS_VERSION=2 and the ROS2
# PYTHONPATH win, while the roscpp half keeps its environment.
set -o errexit

source /opt/ros/one/setup.bash
source "${WS:-/ws}/install/setup.bash"   # chains to /opt/ros/humble/setup.bash

# roscpp half: flush logs line by line so `docker logs` shows them promptly.
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

exec "$@"
