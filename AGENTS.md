# Gazebo Jetty – Source Development Workspace

## Overview

This is a colcon/vcstool workspace for building **Gazebo Jetty** (latest LTS) entirely from source, using a Docker-based dev environment on Ubuntu 24.04 with NVIDIA GPU support.

## Workspace layout

```
gazebo_ws/
├── gz-jetty.repos       # vcstool manifest – pins every Gazebo library branch
├── docker/
│   ├── Dockerfile       # Ubuntu 24.04 image with all Gazebo deps pre-installed
│   └── entrypoint.bash  # sources colcon install on shell entry
├── docker-compose.yml   # GPU, X11, volume mounts
├── dev.bash             # helper script for common docker operations
├── build.bash           # colcon build wrapper (runs inside container)
├── setup.bash           # native bootstrap (alternative to Docker)
├── .gitignore           # ignores src/, build/, install/, log/, .venv/
├── src/                 # (git-ignored) cloned sources, bind-mounted into container
├── build/               # (git-ignored / Docker volume) colcon build tree
├── install/             # (git-ignored / Docker volume) colcon merged install
└── log/                 # (git-ignored / Docker volume) colcon logs
```

## Prerequisites (host)

- Docker with `nvidia-container-toolkit` installed
- X11 display server (for GUI forwarding)

## Common commands (Docker workflow)

```bash
# Build the Docker image (first time or after Dockerfile changes)
./dev.bash build-image

# Start container in background
./dev.bash up

# Fetch/update sources
./dev.bash sync

# Build all packages
./dev.bash build

# Build specific packages
./dev.bash build gz-sim10 gz-gui10

# Debug build
./dev.bash build --debug gz-sim10

# Open a shell inside the container
./dev.bash shell

# Run Gazebo
./dev.bash run gz sim

# Stop / clean up
./dev.bash stop
./dev.bash clean    # also removes build volumes
```

## Reproducibility

The repo tracks only `gz-jetty.repos` (the vcstool manifest), Dockerfile, and helper scripts.
All source code under `src/` is git-ignored and fetched deterministically via
`vcs import`. To pin a library to a specific commit instead of a branch, change
its `version:` field in `gz-jetty.repos` to the commit SHA.

## Notes

- Building all packages can use up to 16 GB RAM. Use `export CMAKE_BUILD_PARALLEL_LEVEL=1`
  and `--executor sequential` to reduce memory usage.
- After modifying source in `src/<pkg>/`, rebuild with `./dev.bash build <pkg>`.
- Build artifacts live in Docker named volumes (`gz-build`, `gz-install`, `gz-log`) for performance.
- Source code is bind-mounted so you can edit on the host with your IDE.
