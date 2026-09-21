# NoMagic dual-stack (ROS1 + ROS2) Luxonis camera driver image.
#
# Build context: the repository root (this depthai-ros fork, branch
# nomagic-ros1-support-v3.3.0):
#
#   docker build -f nomagic/Dockerfile -t camera-driver-luxonis-v3 .
#
# The steps mirror the ones validated in the dev container (see the campaign
# summary): apt deps -> depthai-core v3_humble from source -> colcon build with
# both ROS stacks sourced and -DNOMAGIC_ROS1=ON.

# The only NoMagic image with both ROS One (/opt/ros/one) and ROS2 Humble
# (/opt/ros/humble) preinstalled, plus the CycloneDDS localhost scoping every
# NoMagic ROS2 container shares (see ROS1_AND_2_SUPPORT.md section 8).
FROM europe-west3-docker.pkg.dev/staging-nomagic-ai/images/ros-base-jammy:2026-04-03
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ROS2 build/runtime dependencies of depthai_ros_driver_v3 (rosdep is NOT used:
# the ROS1 side needs no extra packages, roscpp/std_srvs/sensor_msgs ship with
# ros-one-ros-base in the base image) + tools needed by depthai-core's vcpkg.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl zip unzip pkg-config cmake build-essential \
    libusb-1.0-0-dev libopencv-dev libboost-dev nlohmann-json3-dev \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration \
    ros-humble-image-pipeline \
    ros-humble-vision-msgs \
    ros-humble-ffmpeg-image-transport-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-backward-ros \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-message-filters \
    ros-humble-composition-interfaces \
    ros-humble-stereo-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pluginlib \
    && rm -rf /var/lib/apt/lists/*

ENV WS=/ws
RUN mkdir -p ${WS}/src

# depthai-core v3 (the ROS packages require depthai_v3 >= 3.7.1); pin the
# commit that was built and tested against the camera.
ARG DEPTHAI_CORE_REF=98934d38c
RUN cd ${WS}/src \
    && git clone --branch v3_humble https://github.com/luxonis/depthai-core.git \
    && cd depthai-core \
    && git checkout ${DEPTHAI_CORE_REF} \
    && git submodule update --init --recursive

# This fork (with the NOMAGIC_ROS1 feature and the connect-by-IP fix).
COPY ./ ${WS}/src/depthai-ros

# Build. Both stacks are sourced, ROS2 LAST (ROS_VERSION=2 and the ROS2
# PYTHONPATH win for colcon, while PKG_CONFIG_PATH keeps the ROS1 .pc files for
# find_ros1_package()). MAKEFLAGS caps parallelism: depthai-core needs
# ~2 GB RAM per compile job.
ARG BUILD_JOBS=4
RUN cd ${WS} \
    && source /opt/ros/one/setup.bash \
    && source /opt/ros/humble/setup.bash \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build \
        --packages-up-to depthai_ros_driver_v3 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DNOMAGIC_ROS1=ON \
    && rm -rf ${WS}/build ${WS}/log

# Camera config (ROS2 params file translated from the ROS1 v2.11.2 config).
COPY nomagic/config /config

COPY nomagic/ros_entrypoint.bash /ros_entrypoint.bash
COPY nomagic/run_driver.bash ${WS}/run_driver.bash
RUN chmod +x /ros_entrypoint.bash ${WS}/run_driver.bash

WORKDIR ${WS}

# ros2 run (Python) handles SIGINT cleanly but not SIGTERM; this makes
# `docker stop` behave like Ctrl+C and shuts down both ROS stacks.
STOPSIGNAL SIGINT

# The ROS1 side needs ROS_MASTER_URI pointing at the station roscore (host
# network namespace); without a reachable master the driver logs a warning and
# runs ROS2-only. Set NOMAGIC_ROS1_ENABLE=0 to switch the ROS1 side off.
ENTRYPOINT ["/ros_entrypoint.bash"]
CMD ["./run_driver.bash"]
