#!/bin/bash
# Build cobot_ws packages needed for OpenPI Kinova bridge
# This builds MoveIt from source for compatibility with the Kinova config

set -e

echo "=========================================="
echo "Building cobot_ws for OpenPI Bridge"
echo "(Building MoveIt from source)"
echo "=========================================="

# Install apt dependencies for MoveIt build (idempotent - safe to run multiple times)
echo "Installing build dependencies..."
apt-get update -qq
apt-get install -y -qq \
  libompl-dev \
  liborocos-kdl-dev \
  liburdfdom-dev \
  liboctomap-dev \
  libeigen3-dev \
  libbullet-dev \
  libfcl-dev \
  libassimp-dev \
  ros-humble-generate-parameter-library \
  ros-humble-rsl \
  ros-humble-tl-expected \
  ros-humble-ruckig \
  ros-humble-backward-ros \
  ros-humble-ament-cmake-google-benchmark \
  ros-humble-srdfdom \
  ros-humble-geometric-shapes \
  ros-humble-random-numbers \
  ros-humble-warehouse-ros \
  ros-humble-object-recognition-msgs \
  ros-humble-octomap-msgs \
  ros-humble-test-msgs \
  ros-humble-moveit-msgs \
  ros-humble-shape-msgs \
  ros-humble-ros-testing \
  ros-humble-launch-param-builder \
  ros-humble-moveit-resources \
  ros-humble-moveit-resources-fanuc-moveit-config \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-moveit-resources-prbt-moveit-config \
  ros-humble-moveit-resources-prbt-pg70-support \
  ros-humble-moveit-resources-prbt-ikfast-manipulator-plugin \
  > /dev/null 2>&1
echo "✓ Dependencies installed"

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
echo "Building packages (this may take a while for MoveIt)..."
echo ""

# Build using --packages-up-to to automatically resolve dependencies
# This builds MoveIt from source along with the Kinova MoveIt config
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to \
    openpi_kinova_ros2 \
    kinova_gen3_7dof_robotiq_2f_85_moveit_config \
  --packages-ignore \
    kortex_api kortex_driver kortex_bringup \
    serial amrl_maps amrl_msgs kinova_vision \
    livox_ros_driver2 azure_kinect_ros_driver \
    moveit_resources_fanuc_moveit_config moveit_resources_panda_moveit_config \
    moveit_resources_prbt_ikfast_manipulator_plugin \
    moveit_resources_prbt_moveit_config moveit_resources_prbt_pg70_support \
    moveit_hybrid_planning \
    tricycle_controller tricycle_steering_controller ackermann_steering_controller \
    bicycle_steering_controller mecanum_drive_controller diff_drive_controller \
    steering_controllers_library admittance_controller \
    pilz_industrial_motion_planner pilz_industrial_motion_planner_testutils

BUILD_STATUS=$?

echo ""
if [ $BUILD_STATUS -eq 0 ]; then
    echo "=========================================="
    echo "✓ Build complete!"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo "  1. Activate OpenPI environment:"
    echo "     source /workspace/isaaclab/scripts/openpi_setup/activate_openpi_env.sh"
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
