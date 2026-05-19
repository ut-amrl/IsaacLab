#!/bin/bash
set -e

# Script to set up venv_cobot in IsaacLab container
# This creates /root/.venv_cobot with OpenPI dependencies
#
# Usage:
#   1. Start the IsaacLab container with OpenPI extensions:
#      cd ~/IsaacLab/docker
#      ./container.py start ros2 --files docker-compose.openpi.yaml --env-files .env.openpi
#
#   2. Copy this script into the container or run from mounted location:
#      docker cp setup_venv_cobot.sh isaac-lab-ros2:/tmp/setup_venv_cobot.sh
#      # OR if docker directory is accessible in container, run directly
#
#   3. Enter the container and run the script:
#      docker exec -it isaac-lab-ros2 bash
#      bash /tmp/setup_venv_cobot.sh
#      # OR if accessible: bash /workspace/isaaclab/docker/setup_venv_cobot.sh
#
#   4. Activate the venv:
#      source /root/.venv_cobot/bin/activate
#
# Prerequisites:
#   - cobot_ws must be mounted (via docker-compose.openpi.yaml)
#   - Container must have GPU access (for CUDA-enabled PyTorch)
#   - uv will be auto-installed by this script if not present

echo "=========================================="
echo "Setting up venv for OpenPI Kinova Bridge"
echo "=========================================="

# OpenPI upstream requires Python >=3.11 (see cobot_ws/external/openpi/pyproject.toml).
COBOT_PY="${COBOT_PYTHON_VERSION:-3.11}"

# Detect paths
VENV_DIR="/root/.venv_cobot"
COBOT_WS="${DOCKER_ISAACLAB_PATH:-/workspace/isaaclab}/cobot_ws"
OPENPI_DIR="$COBOT_WS/external/openpi"

# Check if cobot_ws exists
if [ ! -d "$COBOT_WS" ]; then
    echo "ERROR: cobot_ws not found at $COBOT_WS"
    echo "Please ensure cobot_ws is mounted correctly in docker-compose"
    exit 1
fi

if [ ! -d "$COBOT_WS/external/openpi" ]; then
    echo "ERROR: openpi directory not found at $COBOT_WS/external/openpi"
    exit 1
fi

echo "VENV_DIR: $VENV_DIR"
echo "COBOT_WS: $COBOT_WS"
echo "OPENPI_DIR: $OPENPI_DIR"

# Bind-mounted or stale venv dirs may exist without a valid interpreter.
if [ -d "$VENV_DIR" ] && [ ! -x "$VENV_DIR/bin/python3" ]; then
    echo "[INFO] Clearing invalid venv at $VENV_DIR (no python3)."
    rm -rf "${VENV_DIR:?}/"*
fi

# Check if uv is installed, install if missing
if ! command -v uv &> /dev/null; then
    echo "uv not found - installing..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    
    # Add both possible installation paths to PATH
    export PATH="/root/.local/bin:/root/.cargo/bin:$PATH"
    
    # Add to bashrc for future sessions
    if ! grep -q '/root/.local/bin' ~/.bashrc; then
        echo 'export PATH="/root/.local/bin:/root/.cargo/bin:$PATH"' >> ~/.bashrc
    fi
    
    # Verify installation (check both possible locations)
    if ! command -v uv &> /dev/null; then
        # Sometimes uv installs to .local/bin instead of .cargo/bin
        if [ -f "/root/.local/bin/uv" ]; then
            echo "✓ uv installed to /root/.local/bin"
        elif [ -f "/root/.cargo/bin/uv" ]; then
            echo "✓ uv installed to /root/.cargo/bin"
        else
            echo "ERROR: uv installation failed - not found in expected locations"
            exit 1
        fi
    else
        echo "✓ uv installed successfully"
    fi
else
    echo "✓ uv is already installed"
fi

# Configure uv for bytecode compilation
export UV_COMPILE_BYTECODE=1
export UV_PROJECT_ENVIRONMENT="$VENV_DIR"
export UV_LINK_MODE=copy
export GIT_LFS_SKIP_SMUDGE=1

# Do not mkdir "$VENV_DIR" before `uv sync` — an empty directory makes uv treat it as a broken env.

# Change to openpi directory
cd "$OPENPI_DIR"

echo ""
echo "Installing OpenPI dependencies with uv sync..."
UV_SYNC_SUCCESS=true
if ! uv sync --no-install-project --python "$COBOT_PY" --no-dev; then
    UV_SYNC_SUCCESS=false
    echo "WARNING: uv sync encountered errors. This may be due to torch==2.7.1 compatibility issues."
    echo "Attempting to continue with manual installation..."
    # Try to create venv manually if uv sync failed
    if [ ! -f "$VENV_DIR/bin/python3" ]; then
        echo "Creating venv manually..."
        UV_VENV_CLEAR=1 uv venv "$VENV_DIR" --python "$COBOT_PY"
    fi
    # Install core dependencies manually (excluding torch which we'll install separately)
    echo "Installing core dependencies manually..."
    "$VENV_DIR/bin/pip" install --upgrade pip setuptools wheel
    # Install most dependencies except torch (we'll install CUDA torch separately)
    "$VENV_DIR/bin/pip" install \
        "augmax>=0.3.4" \
        "dm-tree>=0.1.8" \
        "einops>=0.8.0" \
        "numpy>=1.22.4,<2.0.0" \
        "opencv-python>=4.10.0.84" \
        "pillow>=11.0.0" \
        "tqdm" \
        "typing-extensions>=4.12.2" \
        "transformers==4.53.2" \
        "protobuf<=3.20" \
        "pytest" \
        "deprecated" \
        || echo "Some packages failed to install, continuing..."
fi

# Get Python version from venv (after it's created)
VENV_PYTHON="$VENV_DIR/bin/python3"
if [ -f "$VENV_PYTHON" ]; then
    PYV=$("$VENV_PYTHON" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
else
    # Fallback if venv python doesn't exist yet
    PYV="$COBOT_PY"
fi

echo ""
if [ "$UV_SYNC_SUCCESS" = false ]; then
    echo "Skipping additional package installation (already installed manually)"
else
    echo "Installing additional packages..."
    uv pip install --python "$VENV_PYTHON" pytest deprecated 'protobuf<=3.20' 'transformers==4.53.2' || echo "Some additional packages failed, continuing..."
fi

echo ""
echo "Replacing PyTorch with CUDA-enabled version..."
# Uninstall existing torch (if installed)
"$VENV_PYTHON" -m pip uninstall -y torch torchvision torchaudio 2>/dev/null || true
# Install PyTorch with CUDA 12.1 support
if command -v uv &> /dev/null; then
    uv pip install --python "$VENV_PYTHON" torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121 || \
    "$VENV_PYTHON" -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
else
    "$VENV_PYTHON" -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
fi

echo ""
echo "Copying transformers_replace..."
TRANSFORMERS_REPLACE_DIR="$OPENPI_DIR/src/openpi/models_pytorch/transformers_replace"
if [ -d "$TRANSFORMERS_REPLACE_DIR" ]; then
    SITE_PACKAGES="$VENV_DIR/lib/python${PYV}/site-packages/transformers"
    mkdir -p "$SITE_PACKAGES"
    cp -r "$TRANSFORMERS_REPLACE_DIR"/* "$SITE_PACKAGES/"
    echo "Copied transformers_replace to $SITE_PACKAGES"
else
    echo "WARNING: transformers_replace directory not found at $TRANSFORMERS_REPLACE_DIR"
fi

echo ""
echo "Installing Kortex SDK Python API..."
KORTEX_WHEEL="kortex_api-2.5.0.post6-py3-none-any.whl"
KORTEX_URL="https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/$KORTEX_WHEEL"
TMP_DIR=$(mktemp -d)
cd "$TMP_DIR"
wget "$KORTEX_URL" || {
    echo "WARNING: Failed to download Kortex API. Skipping..."
    cd "$OPENPI_DIR"
    rm -rf "$TMP_DIR"
}
if [ -f "$KORTEX_WHEEL" ]; then
    if command -v uv &> /dev/null; then
        uv pip install --python "$VENV_PYTHON" --no-deps "$KORTEX_WHEEL" || \
        "$VENV_PYTHON" -m pip install --no-deps "$KORTEX_WHEEL"
    else
        "$VENV_PYTHON" -m pip install --no-deps "$KORTEX_WHEEL"
    fi
    rm "$KORTEX_WHEEL"
fi
cd "$OPENPI_DIR"
rm -rf "$TMP_DIR"

echo ""
echo "Installing openpi and openpi-client as editable..."
if command -v uv &> /dev/null; then
    uv pip install --python "$VENV_PYTHON" -e "$OPENPI_DIR" -e "$OPENPI_DIR/packages/openpi-client" || \
    "$VENV_PYTHON" -m pip install -e "$OPENPI_DIR" -e "$OPENPI_DIR/packages/openpi-client"
else
    "$VENV_PYTHON" -m pip install -e "$OPENPI_DIR" -e "$OPENPI_DIR/packages/openpi-client"
fi

echo ""
echo "Installing roboticstoolbox for kinematics (FK/IK)..."
if [ "${SKIP_ROBOTICSTOOLBOX:-0}" = "1" ]; then
    echo "[WARN] SKIP_ROBOTICSTOOLBOX=1 — skipping roboticstoolbox-python / spatialmath-python."
    echo "  Install on a machine with a C toolchain (e.g. full Docker image) or use a prebuilt wheel."
elif command -v uv &> /dev/null; then
    if ! uv pip install --python "$VENV_PYTHON" roboticstoolbox-python spatialmath-python; then
        echo "ERROR: roboticstoolbox install failed (often missing gcc). Set SKIP_ROBOTICSTOOLBOX=1 on Apptainer/read-only images or install build-essential."
        exit 1
    fi
else
    "$VENV_PYTHON" -m pip install roboticstoolbox-python spatialmath-python
fi

echo ""
echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Build the ROS2 workspace (if not already built):"
echo "   bash /workspace/isaaclab/scripts/openpi_setup/build_cobot_ws.sh"
echo "   (Use --clean flag to rebuild from scratch)"
echo ""
echo "2. Activate OpenPI environment:"
echo "   source /workspace/isaaclab/scripts/openpi_setup/activate_openpi_env.sh"
echo ""
echo "This will:"
echo "  - Activate the venv"
echo "  - Set PyTorch environment variables (disable JIT compilation)"
echo "  - Source ROS2 and workspace"
echo ""
echo "Or manually activate:"
echo "  source $VENV_DIR/bin/activate"
echo "  export PYTORCH_JIT=0"
echo "  export TORCH_COMPILE_DISABLE=1"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /workspace/isaaclab/cobot_ws/install/setup.bash"
echo ""
echo "To test installation:"
echo "  $VENV_PYTHON -c 'import torch; print(f\"PyTorch version: {torch.__version__}\"); print(f\"CUDA available: {torch.cuda.is_available()}\")'"
echo "  $VENV_PYTHON -c 'import openpi; print(\"OpenPI imported successfully\")'"
echo ""

