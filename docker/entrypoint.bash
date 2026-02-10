#!/usr/bin/env bash
set -e

WS="$HOME/gazebo_ws"

# Source the colcon install if it exists
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi

exec "$@"
