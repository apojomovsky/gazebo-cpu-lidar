# Gazebo Jetty – CPU Lidar Development Workspace

This workspace is a specialized environment for developing and testing **Gazebo Jetty** entirely from source, with a primary focus on the new **CPU-based Lidar** implementation.

## What is CPU Lidar?

Standard Gazebo Lidar (`gpu_lidar`) relies on the **rendering engine (Ogre2)** and a **GPU** to calculate distances. This makes it impossible to use on "True Headless" machines like cloud servers or CI/CD runners that lack graphics hardware.

The **CPU Lidar** uses the **physics engine (DART/Bullet)** to perform raycasting. It has **zero graphics dependencies**, making it perfect for:
*   Headless Cloud Simulation
*   Dockerized CI/CD Pipelines
*   Large-scale simulation clusters

---

## 🛠 Getting Started

### 1. Build the Environment
```bash
./dev.bash build-image  # Rebuilds Docker with ROS 2 Jazzy
./dev.bash sync         # Fetches all Gazebo and ROS sources
./dev.bash build        # Builds the full Gazebo stack (src/)
./dev.bash ros-build    # Builds the ROS bridge and demo (ros_ws/)
```

### 2. Run the Demo
The `cpu_lidar_demo` package is included to show the CPU Lidar in action.

```bash
# Launch with full GUI support (Gazebo + RViz)
./dev.bash ros-launch lidar_type:=cpu
```

---

## 🔬 The Headless Experiment

This workspace uses a unique **Split-Container Architecture**. When you run `ros-launch`, it spawns:
1.  **`gz-server`**: Runs the physics simulation.
2.  **`ros-ui`**: Runs RViz and the bridges.

### The "True Headless" Test
To prove that CPU Lidar is independent of graphics, we can run a "Hard Headless" test which **deletes the rendering engine** from the simulation.

| Command | Sensor | Result |
| :--- | :--- | :--- |
| `./dev.bash ros-launch lidar_type:=cpu hard_headless:=true` | **CPU** | ✅ **SUCCESS**: Data appears in RViz using native physics. |
| `./dev.bash ros-launch lidar_type:=gpu hard_headless:=true` | **GPU** | ❌ **FAILURE**: RViz is empty; sensor cannot run without rendering. |

---

## 📂 Repository Summaries

### Gazebo Source (`src/`)
*   **`src/gz-sensors`**: Home of the `CpuLidarSensor` class, implementing the physics-based range calculation.
*   **`src/gz-sim`**: Contains the `CpuLidar` system plugin that bridges the physics engine and the sensor class.

### ROS Workspace (`ros_ws/`)
*   **`ros_gz`**: Patched for compatibility with newer simulation interfaces.
*   **`cpu_lidar_demo`**: The validation suite for this workspace.

## ⌨️ Common Commands

*   **Stop Everything**: `./dev.bash stop`
*   **Open a Shell**: `./dev.bash shell`
*   **Clean Build Artifacts**: `./dev.bash clean`
