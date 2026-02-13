#!/usr/bin/env bash
#
# Helper script for the dockerized Gazebo dev environment.
#
# Usage:
#   ./dev.bash build-image             # build the Docker image
#   ./dev.bash shell                   # open a shell in the container
#   ./dev.bash sync                    # fetch/update sources in src/
#   ./dev.bash build [pkg ...]         # build all or specific packages
#   ./dev.bash ros-build [pkg ...]     # build the ROS 2 workspace
#   ./dev.bash ros-launch [args ...]   # launch the lidar demo
#   ./dev.bash run [cmd ...]           # run a command in the container
#   ./dev.bash stop                    # stop the container
#   ./dev.bash clean                   # remove build volumes
#
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export USER_NAME="${USER_NAME:-$(id -un)}"
export USER_UID="${USER_UID:-$(id -u)}"
export USER_GID="${USER_GID:-$(id -g)}"

_ensure_dirs() {
  mkdir -p src build install log ros_ws
}

_compose() {
  docker compose "$@"
}

_exec() {
  local opts="-i"
  if [ -t 0 ]; then opts="-it"; fi
  if docker ps --format '{{.Names}}' | grep -q '^gz-dev$'; then
    docker exec $opts gz-dev /entrypoint.bash "$@"
  else
    _compose run --rm gz "$@"
  fi
}

cmd_build_image() {
  _compose build
}

cmd_shell() {
  xhost +local:docker 2>/dev/null || true
  _exec bash
}

cmd_sync() {
  mkdir -p src
  _exec bash -c 'vcs import --recursive --force ~/gazebo_ws/src < ~/gazebo_ws/gz-jetty.repos'
}

cmd_build() {
  _exec bash ~/gazebo_ws/build.bash "$@"
}

cmd_run() {
  xhost +local:docker 2>/dev/null || true
  _exec "$@"
}

cmd_stop() {
  _compose down --remove-orphans

  local run_containers=()
  mapfile -t run_containers < <(docker ps -aq --filter "name=^gazebo_ws-gz-run-")
  if [[ ${#run_containers[@]} -gt 0 ]]; then
    docker rm -f "${run_containers[@]}" >/dev/null
  fi
}

cmd_clean() {
  _compose down
  rm -rf build install log ros_ws/build ros_ws/install ros_ws/log
  echo "Build artifacts removed."
}

cmd_up() {
  _ensure_dirs
  xhost +local:docker 2>/dev/null || true
  _compose up -d
}

cmd_ros_build() {
  mkdir -p ros_ws
  local colcon_args=(
    build
    --merge-install
    --symlink-install
    --cmake-args -DBUILD_TESTING=OFF
  )
  if [[ $# -gt 0 ]]; then
    colcon_args+=(--packages-select "$@")
  fi
  _exec bash -c "cd ~/gazebo_ws/ros_ws && colcon ${colcon_args[*]}"
}

_cleanup() {
  trap - EXIT SIGINT SIGTERM
  echo ""
  # Stop the split containers if they are running
  local split_containers=("gazebo_ws-gz-server" "gazebo_ws-gz-ui")
  for container in "${split_containers[@]}"; do
    if docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
      docker rm -f "$container" >/dev/null 2>&1 || true
    fi
  done
  
  if docker ps --format '{{.Names}}' | grep -q '^gz-dev$'; then
    echo "Cleaning up: stopping containers..."
    cmd_stop
  fi
}

cmd_ros_launch() {
  trap _cleanup EXIT SIGINT SIGTERM
  xhost +local:docker 2>/dev/null || true

  # Parse arguments for special flags (handle both arg:=value and default cases)
  local gz_gui="true"
  local hard_headless="false"
  local rviz="false"
  local lidar_type="cpu"
  local launch_args=""

  for arg in "$@"; do
    # Handle arg:=value format
    if [[ "$arg" == gz_gui:=* ]]; then
      gz_gui="${arg#gz_gui:=}"
    elif [[ "$arg" == hard_headless:=* ]]; then
      hard_headless="${arg#hard_headless:=}"
    elif [[ "$arg" == rviz:=* ]]; then
      rviz="${arg#rviz:=}"
    elif [[ "$arg" == lidar_type:=* ]]; then
      lidar_type="${arg#lidar_type:=}"
    fi
    launch_args="$launch_args $arg"
  done

  # Determine if we need GPU for server container
  local server_nvidia_env=()
  if [[ "$gz_gui" == "false" || "$hard_headless" == "true" ]]; then
    # Strip rendering support for headless server - disable ALL GPU rendering
    server_nvidia_env=(
      "-e" "GZ_DEV_DRI=/dev/null"
      "-e" "NVIDIA_VISIBLE_DEVICES=none"
      "-e" "NVIDIA_DRIVER_CAPABILITIES=none"
      "-e" "__GLX_VENDOR_LIBRARY_NAME="
      "-e" "__NV_PRIME_RENDER_OFFLOAD="
      "-e" "__VK_LAYER_NV_optimus="
      # Unset DISPLAY to force hard failure if rendering is attempted
      "-e" "DISPLAY="
    )
  fi

  # Launch server container (gzserver)
  echo "Starting gzserver container..."
  docker compose run --rm --name gazebo_ws-gz-server \
    -e "container_mode:=server" \
    "${server_nvidia_env[@]}" \
    gz bash -c "cd ~/gazebo_ws/ros_ws && ros2 launch cpu_lidar_demo lidar_demo.launch.py container_mode:=server${launch_args}" &
  local server_pid=$!

  # Wait briefly for server container to start
  sleep 2

  # Launch UI container (gz_gui, rviz, etc.)
  echo "Starting UI container..."
  docker compose run --rm --name gazebo_ws-gz-ui \
    -e "container_mode:=ui" \
    gz bash -c "cd ~/gazebo_ws/ros_ws && ros2 launch cpu_lidar_demo lidar_demo.launch.py container_mode:=ui${launch_args}" &
  local ui_pid=$!

  # Wait for both containers
  wait $server_pid
  local server_exit=$?
  wait $ui_pid
  local ui_exit=$?

  # Exit with error if either container failed
  if [[ $server_exit -ne 0 ]]; then
    echo "Server container exited with code $server_exit"
  fi
  if [[ $ui_exit -ne 0 ]]; then
    echo "UI container exited with code $ui_exit"
  fi
}

case "${1:-help}" in
  build-image) shift; cmd_build_image "$@" ;;
  shell)       shift; cmd_shell "$@" ;;
  sync)        shift; cmd_sync "$@" ;;
  build)       shift; cmd_build "$@" ;;
  ros-build)   shift; cmd_ros_build "$@" ;;
  ros-launch)  shift; cmd_ros_launch "$@" ;;
  run)         shift; cmd_run "$@" ;;
  up)          shift; cmd_up "$@" ;;
  stop)        shift; cmd_stop "$@" ;;
  clean)       shift; cmd_clean "$@" ;;
  *)
    echo "Usage: $0 {build-image|up|shell|sync|build|ros-build|ros-launch|run|stop|clean}"
    echo ""
    echo "  build-image         Build the Docker image"
    echo "  up                  Start the container in background"
    echo "  shell               Open a bash shell in the container"
    echo "  sync                Fetch/update sources via vcstool"
    echo "  build [pkg ...]     Build all or specific packages"
    echo "  ros-build [pkg ...] Build the ROS 2 overlay workspace"
    echo "  ros-launch [args]   Launch the lidar demo (e.g. lidar_type:=gpu)"
    echo "  run <cmd ...>       Run a command in the container"
    echo "  stop                Stop the container"
    echo "  clean               Stop and remove build volumes"
    ;;
esac
