#!/usr/bin/env bash
#
# Helper script for the dockerized Gazebo dev environment.
#
# Usage:
#   ./dev.bash build-image             # build the Docker image
#   ./dev.bash shell                   # open a shell in the container
#   ./dev.bash sync                    # fetch/update sources in src/
#   ./dev.bash build [pkg ...]         # build all or specific packages
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
  mkdir -p src build install log
}

_compose() {
  docker compose "$@"
}

_exec() {
  if docker ps --format '{{.Names}}' | grep -q '^gz-dev$'; then
    docker exec -it gz-dev "$@"
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
  _compose down
}

cmd_clean() {
  _compose down
  rm -rf build install log
  echo "Build artifacts removed."
}

cmd_up() {
  _ensure_dirs
  xhost +local:docker 2>/dev/null || true
  _compose up -d
}

case "${1:-help}" in
  build-image) shift; cmd_build_image "$@" ;;
  shell)       shift; cmd_shell "$@" ;;
  sync)        shift; cmd_sync "$@" ;;
  build)       shift; cmd_build "$@" ;;
  run)         shift; cmd_run "$@" ;;
  up)          shift; cmd_up "$@" ;;
  stop)        shift; cmd_stop "$@" ;;
  clean)       shift; cmd_clean "$@" ;;
  *)
    echo "Usage: $0 {build-image|up|shell|sync|build|run|stop|clean}"
    echo ""
    echo "  build-image         Build the Docker image"
    echo "  up                  Start the container in background"
    echo "  shell               Open a bash shell in the container"
    echo "  sync                Fetch/update sources via vcstool"
    echo "  build [pkg ...]     Build all or specific packages"
    echo "  run <cmd ...>       Run a command in the container"
    echo "  stop                Stop the container"
    echo "  clean               Stop and remove build volumes"
    ;;
esac
