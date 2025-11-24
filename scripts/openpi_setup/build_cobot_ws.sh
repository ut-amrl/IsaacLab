#!/bin/bash
# Build cobot_ws packages needed for OpenPI Kinova bridge
# This builds only the necessary packages and ignores problematic ones

set -e

echo "=========================================="
echo "Building cobot_ws for OpenPI Bridge"
echo "=========================================="

# Check if we're in the right directory
COBOT_WS="${DOCKER_ISAACLAB_PATH:-/workspace/isaaclab}/cobot_ws"

if [ ! -d "$COBOT_WS" ]; then
    echo "ERROR: cobot_ws not found at $COBOT_WS"
    exit 1
fi

cd "$COBOT_WS"

# Make sure we're using system Python (not venv)
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "WARNING: Virtual environment is active. Deactivating..."
    echo "  Active venv: $VIRTUAL_ENV"
    echo "  Please run: deactivate"
    echo "  Then re-run this script."
    exit 1
fi

# Verify system Python
PYTHON_PATH=$(which python3)
if [[ "$PYTHON_PATH" == *".venv"* ]]; then
    echo "ERROR: Still using venv Python: $PYTHON_PATH"
    echo "Please deactivate venv and try again."
    exit 1
fi

echo "Using Python: $PYTHON_PATH"

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS2 Humble"
else
    echo "ERROR: ROS2 Humble not found"
    exit 1
fi

# Clean build artifacts if requested
if [ "$1" == "--clean" ]; then
    echo "Cleaning build artifacts..."
    rm -rf build/ install/ log/ .colcon_install_layout
    echo "✓ Cleaned workspace"
fi

echo ""
echo "Building packages..."
echo ""

# Build only the packages we need, ignoring problematic ones
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select openpi_kinova_ros2 kortex_bringup robotiq_description cobot_description \
  --packages-ignore \
    joint_limits ros2_control ros2_controllers control_msgs realtime_tools \
    hardware_interface controller_interface controller_manager_msgs \
    ros2_control_test_assets controller_manager gripper_controllers \
    joint_state_broadcaster hardware_interface_testing transmission_interface \
    admittance_controller diff_drive_controller force_torque_sensor_broadcaster \
    forward_command_controller gpio_controllers imu_sensor_broadcaster \
    mecanum_drive_controller pid_controller pose_broadcaster \
    range_sensor_broadcaster ros2controlcli steering_controllers_library \
    tricycle_controller ackermann_steering_controller bicycle_steering_controller \
    effort_controllers position_controllers tricycle_steering_controller \
    velocity_controllers picknik_reset_fault_controller picknik_twist_controller \
    kortex_api kortex_driver kortex_bringup joint_trajectory_controller \
    moveit_common moveit_msgs moveit_configs_utils moveit_resources_panda_description \
    moveit_resources_pr2_description moveit_resources_fanuc_description \
    moveit_resources_prbt_support serial amrl_maps amrl_msgs kinova_vision \
    livox_ros_driver2 azure_kinect_ros_driver

BUILD_STATUS=$?

echo ""
if [ $BUILD_STATUS -eq 0 ]; then
    echo "=========================================="
    echo "✓ Build complete!"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo "  1. Activate OpenPI environment:"
    echo "     source /workspace/isaaclab/docker/activate_openpi_env.sh"
    echo ""
    echo "  2. Run the bridge:"
    echo "     ros2 run openpi_kinova_ros2 openpi_kinova_direct_bridge.py --ros-args -p simulation_mode:=true ..."
    echo ""
else
    echo "=========================================="
    echo "✗ Build failed with status: $BUILD_STATUS"
    echo "=========================================="
    exit $BUILD_STATUS
fi

