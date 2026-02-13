#!/usr/bin/env bash
# This script runs INSIDE the container

# Helper to check if a topic has data and report frequency
check_topic() {
    local topic=$1
    local timeout=$2
    echo "Checking $topic for $timeout seconds..."
    # Get frequency using ros2 topic hz
    timeout "$timeout" ros2 topic hz "$topic" 2>&1 | grep "average rate" || echo "❌ No data received on $topic"
}

# Cleanup function for background processes
cleanup() {
    echo "Cleaning up..."
    pkill -P $$ || true
}
trap cleanup EXIT

LOG_DIR="./ros_ws/test_logs"
mkdir -p "$LOG_DIR"

echo "=== Headless Comparison Test ==="
echo "Note: Both are running with NVIDIA_VISIBLE_DEVICES=none to force CPU-only environment."
echo ""

echo "--- Test 1: CPU Lidar Headless ---"
export NVIDIA_VISIBLE_DEVICES=none
ros2 launch cpu_lidar_demo lidar_demo.launch.py lidar_type:=cpu gz_gui:=false rviz:=false > "$LOG_DIR/cpu_headless.log" 2>&1 &
LAUNCH_PID=$!
sleep 10
check_topic /scan 10
kill "$LAUNCH_PID" || true
sleep 2

echo ""
echo "--- Test 2: GPU Lidar Headless ---"
export NVIDIA_VISIBLE_DEVICES=none
unset __GLX_VENDOR_LIBRARY_NAME
unset __NV_PRIME_RENDER_OFFLOAD
unset __VK_LAYER_NV_optimus
ros2 launch cpu_lidar_demo lidar_demo.launch.py lidar_type:=gpu gz_gui:=false rviz:=false > "$LOG_DIR/gpu_headless.log" 2>&1 &
LAUNCH_PID=$!
sleep 10
check_topic /scan 10
kill "$LAUNCH_PID" || true
sleep 2

echo ""
echo "=== Dependency Analysis ==="
echo "CPU Lidar renderer load search:"
grep -i "rendering" "$LOG_DIR/cpu_headless.log" || echo "No rendering engine loaded (Correct)"
echo ""
echo "GPU Lidar renderer load search:"
grep -i "rendering" "$LOG_DIR/gpu_headless.log" | head -n 5
