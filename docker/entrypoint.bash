#!/usr/bin/env bash
set -e

WS="$HOME/gazebo_ws"

# Source ROS 2 Jazzy underlay
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

# Source the Gazebo colcon install overlay
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  export GZ_CONFIG_PATH="$WS/install/share/gz${GZ_CONFIG_PATH:+:${GZ_CONFIG_PATH}}"
  export GZ_VERSION=10
  export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
  export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
  export __VK_LAYER_NV_optimus="${__VK_LAYER_NV_optimus:-NVIDIA_only}"
fi

# Source the ROS 2 workspace overlay
if [ -f "$WS/ros_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/ros_ws/install/setup.bash"
fi

exec "$@"
