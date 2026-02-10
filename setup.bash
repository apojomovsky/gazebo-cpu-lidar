#!/usr/bin/env bash
#
# Bootstrap the Gazebo Jetty from-source workspace.
#
# Usage:
#   bash setup.bash          # full setup (tools + sources + deps)
#   bash setup.bash --sync   # just re-fetch / update sources
#
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

REPOS_FILE="gz-jetty.repos"
VENV_DIR=".venv"

# ── 1. Python venv with vcstool + colcon ────────────────────────────────────
ensure_tools() {
  if [ ! -d "$VENV_DIR" ]; then
    echo ">>> Creating Python venv for vcstool & colcon..."
    python3 -m venv "$VENV_DIR"
  fi
  # shellcheck disable=SC1091
  source "$VENV_DIR/bin/activate"
  pip install --quiet --upgrade pip vcstool colcon-common-extensions
}

# ── 2. Fetch / update sources ───────────────────────────────────────────────
sync_sources() {
  mkdir -p src
  echo ">>> Importing sources from $REPOS_FILE ..."
  vcs import --recursive --force src < "$REPOS_FILE"
  echo ">>> Sources ready in src/"
}

# ── 3. OSRF apt repo + package dependencies ────────────────────────────────
install_deps() {
  echo ">>> Adding packages.osrfoundation.org apt repo..."
  sudo curl -s https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt-get update -qq

  echo ">>> Installing apt dependencies from packages.apt files..."
  cd src
  ALL_DEPS=$(sort -u $(find . -iname "packages-$(lsb_release -cs).apt" \
                    -o -iname 'packages.apt' | grep -v '/\.git/') \
      | sed '/gz\|sdf\|^cmake$/d' | tr '\n' ' ')

  # Filter out packages not available in this distro's repos
  INSTALLABLE=""
  SKIPPED=""
  for pkg in $ALL_DEPS; do
    if apt-cache show "$pkg" > /dev/null 2>&1; then
      INSTALLABLE="$INSTALLABLE $pkg"
    else
      SKIPPED="$SKIPPED $pkg"
    fi
  done

  if [ -n "$SKIPPED" ]; then
    echo ">>> Skipping packages not found in apt (install equivalents manually):"
    echo "   $SKIPPED"
  fi

  # shellcheck disable=SC2086
  sudo apt -y install $INSTALLABLE
  cd ..
}

# ── main ────────────────────────────────────────────────────────────────────
if [ "${1:-}" = "--sync" ]; then
  ensure_tools
  sync_sources
else
  ensure_tools
  sync_sources
  install_deps
  echo ""
  echo "=== Setup complete ==="
  echo "Build with:"
  echo "  source .venv/bin/activate"
  echo "  colcon build --merge-install --cmake-args '-DBUILD_TESTING=OFF'"
  echo ""
  echo "Then source the install:"
  echo "  source install/setup.bash"
fi
