# cpu_lidar_demo

ROS 2 demo comparing CPU-based and GPU-based lidar in Gazebo Jetty.

## Overview

| Mode | Physics | Lidar System Plugin | SDF Sensor Type | Requires GPU for Lidar |
|------|---------|--------------------|-----------------|-----------------------|
| `cpu` | DART + Bullet | `gz-sim-cpu-lidar-system` | `lidar` | No |
| `gpu` | ODE | `gz-sim-sensors-system` (ogre2) | `gpu_lidar` | Yes |

Both modes publish `LaserScan` and `PointCloudPacked` on gz-transport, which
the `ros_gz_bridge` forwards to ROS 2 as `/scan` and `/points`.

## Prerequisites

1. Gazebo Jetty built from source in `~/gazebo_ws`
2. Docker image rebuilt with ROS 2 Jazzy:
   ```bash
   ./dev.bash build-image
   ```
3. `ros_gz` cloned into `ros_ws/src/`:
   ```bash
   cd ~/gazebo_ws/ros_ws/src
   git clone -b ros2 https://github.com/gazebosim/ros_gz.git
   ```

## Build

```bash
# From the host
./dev.bash up
./dev.bash ros-build
```

Or manually inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source ~/gazebo_ws/install/setup.bash
cd ~/gazebo_ws/ros_ws
colcon build --merge-install --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## Run

### Supported Arguments

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `lidar_type` | `cpu`, `gpu` | `cpu` | Lidar sensor implementation |
| `gz_gui` | `true`, `false` | `true` | Launch Gazebo GUI client |
| `rviz` | `true`, `false` | `true` | Launch RViz2 visualization |
| `render_mode` | `software`, `server` | `server` | Rendering mode (only applies when `gz_gui:=true`) |

### Examples

```bash
# CPU lidar (default) — no GPU needed for lidar
./dev.bash ros-launch

# GPU lidar (for comparison)
./dev.bash ros-launch lidar_type:=gpu

# Headless server only (no GUI, no RViz)
./dev.bash ros-launch gz_gui:=false rviz:=false

# Headless comparison: CPU vs GPU (both without GUI)
./dev.bash ros-launch lidar_type:=cpu gz_gui:=false

# GPU mode without GUI but still requires Mesa/EGL for rendering
./dev.bash ros-launch lidar_type:=gpu gz_gui:=false
```

## Headless & Remote Simulation

The CPU lidar sensor is designed for environments without a graphics stack, making it ideal for CI/CD pipelines and cloud-based simulation.

### Benefits

- **CI/CD Automation**: Run lidar sensor tests in containers without GPU or graphics dependencies
- **Cloud Simulation**: Deploy headless simulation servers on infrastructure without local display capabilities
- **Resource Efficiency**: No Mesa/EGL or rendering engine overhead when visualization is not needed

### Graphics Stack Requirements

| Mode | Graphics Dependencies |
|------|----------------------|
| `lidar_type:=cpu` | None — works purely with physics engine (DART + Bullet) |
| `lidar_type:=gpu` | Requires Mesa/EGL even without GUI — the rendering pipeline is still initialized |

This distinction is critical for containerized deployments: CPU lidar runs on minimal base images, while GPU lidar requires the full graphics stack regardless of whether a GUI is displayed.

### Running Headless

```bash
# Launch CPU lidar in headless mode (full server + client decoupling)
./dev.bash ros-launch gz_gui:=false
```

This runs the Gazebo server without the GUI client, allowing the simulation to run entirely headless while still publishing lidar data to ROS 2 topics.

## How to Test

### 1. Build the Demo
First, ensure the ROS 2 workspace is built:
```bash
./dev.bash ros-build
```

### 2. Launch the CPU Lidar Demo
This is the default mode. It will launch Gazebo (headless server + client GUI), the ROS bridge, and RViz.
```bash
./dev.bash ros-launch
```

### 3. Launch the GPU Lidar Demo (for comparison)
If you have a GPU and want to compare the output:
```bash
./dev.bash ros-launch lidar_type:=gpu
```

### 4. Verifying the Data
Once the demo is running, you can verify the data flow in a separate terminal inside the container (using `./dev.bash shell`):

*   **Check ROS 2 Topics**:
    ```bash
    ros2 topic list
    ```
*   **Echo Laser Scan**:
    ```bash
    ros2 topic echo /scan
    ```
*   **Check Frequency**:
    ```bash
    ros2 topic hz /scan
    ```

## Topics

| ROS 2 Topic | Message Type | Description |
|-------------|-------------|-------------|
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D laser scan |
| `/points` | `sensor_msgs/msg/PointCloud2` | 3D point cloud |

