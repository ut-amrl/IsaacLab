#!/bin/bash
# Helper script to activate OpenPI environment with correct settings
# Usage: source /workspace/isaaclab/docker/activate_openpi_env.sh

# Activate the venv
if [ -f "/root/.venv_cobot/bin/activate" ]; then
    source /root/.venv_cobot/bin/activate
    echo "✓ Activated venv: /root/.venv_cobot"
else
    echo "✗ ERROR: venv not found at /root/.venv_cobot"
    echo "  Run setup_venv_cobot.sh first"
    return 1
fi

# Disable PyTorch compilation (avoids Triton errors on some GPUs)
# Note: These are already set by .env.openpi in the container, but we export
# them here for consistency and to support running outside the container
export PYTORCH_JIT=0
export TORCH_COMPILE_DISABLE=1
echo "✓ PyTorch JIT compilation disabled"

# Limit threading to prevent resource exhaustion
# PyTorch, NumPy, and OpenMP can create too many threads causing crashes
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=4
export NUMEXPR_NUM_THREADS=4
export VECLIB_MAXIMUM_THREADS=4
echo "✓ Thread limits set (4 threads per library)"

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS2 Humble"
fi

# Isolate from other ROS2 stacks on this host (ecocar_robot publishes static TFs
# on the default domain 0).
export ROS_DOMAIN_ID=42
echo "✓ ROS_DOMAIN_ID=42"

# Source workspace if it exists
if [ -f "/workspace/isaaclab/cobot_ws/install/setup.bash" ]; then
    source /workspace/isaaclab/cobot_ws/install/setup.bash
    echo "✓ Sourced cobot_ws"
fi

# PYTHONPATH for manipulation pipeline (cobot_pick_v2.py, etc.). Export the whole
# workspace so behavior matches the real robot; include src/cobot so cobot_utils resolves.
if [ -d "/workspace/isaaclab/cobot_ws" ]; then
    export PYTHONPATH="/workspace/isaaclab/cobot_ws:/workspace/isaaclab/cobot_ws/src/cobot:/workspace/isaaclab/cobot_ws/src/ros2_gsam2${PYTHONPATH:+:$PYTHONPATH}"
    echo "✓ PYTHONPATH includes cobot_ws, cobot, and ros2_gsam2"
fi

echo ""
echo "=========================================="
echo "OpenPI Environment Ready!"
echo "=========================================="
echo "You can now run:"
echo "  ros2 run openpi_kinova_ros2 openpi_kinova_direct_bridge.py --ros-args -p simulation_mode:=true ..."
echo ""
echo "Manipulation pipeline (uses system Python so rclpy works):"
echo "  bash /workspace/isaaclab/scripts/openpi_setup/run_cobot_script.sh cobot_pick_v2 --query \"cup\""
echo ""

