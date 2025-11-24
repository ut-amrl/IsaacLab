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

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS2 Humble"
fi

# Source workspace if it exists
if [ -f "/workspace/isaaclab/cobot_ws/install/setup.bash" ]; then
    source /workspace/isaaclab/cobot_ws/install/setup.bash
    echo "✓ Sourced cobot_ws"
fi

echo ""
echo "=========================================="
echo "OpenPI Environment Ready!"
echo "=========================================="
echo "You can now run:"
echo "  ros2 run openpi_kinova_ros2 openpi_kinova_direct_bridge.py --ros-args -p simulation_mode:=true ..."
echo ""

