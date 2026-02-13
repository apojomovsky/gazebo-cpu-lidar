#!/usr/bin/env bash
#
# Build the Gazebo Jetty workspace.
#
# Usage:
#   bash build.bash                          # build all packages
#   bash build.bash gz-sim10                 # build only gz-sim10
#   bash build.bash gz-sim10 gz-gui10        # build multiple packages
#   bash build.bash --debug                  # build all with debug symbols
#   bash build.bash --debug gz-sim10         # debug build of one package
#
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Activate tooling venv if present (native builds only; not needed in Docker)
if [ -f .venv/bin/activate ]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

CMAKE_ARGS=(
  -DBUILD_TESTING=OFF
)

BUILD_TYPE="RelWithDebInfo"
PACKAGES=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug)
      BUILD_TYPE="Debug"
      shift
      ;;
    --release)
      BUILD_TYPE="Release"
      shift
      ;;
    -*)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
    *)
      PACKAGES+=("$1")
      shift
      ;;
  esac
done

CMAKE_ARGS+=("-DCMAKE_BUILD_TYPE=$BUILD_TYPE")

COLCON_ARGS=(
  build
  --base-paths src
  --merge-install
  --cmake-args "${CMAKE_ARGS[@]}"
)

if [[ ${#PACKAGES[@]} -gt 0 ]]; then
  COLCON_ARGS+=(--packages-select "${PACKAGES[@]}")
fi

echo ">>> Build type: $BUILD_TYPE"
if [[ ${#PACKAGES[@]} -gt 0 ]]; then
  echo ">>> Packages:   ${PACKAGES[*]}"
else
  echo ">>> Packages:   all"
fi

colcon "${COLCON_ARGS[@]}"
