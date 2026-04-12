# OpenPI Kinova Bridge Setup with Isaac Lab Container

This directory contains the configuration and scripts for running the OpenPI Kinova ROS2 bridge in Isaac Lab containers with Podman.

## Quick Start

```bash
cd ~/IsaacLab/docker

# First time: Build and start ros2 container with OpenPI extensions
./container.py start ros2 --files docker-compose.openpi.yaml --env-files .env.openpi

# After first build: Restart without rebuilding (much faster!)`
./container.py restart ros2 --files docker-compose.openpi.yaml --env-files .env.openpi
```

The `start` command:
- Builds the image and starts the `isaac-lab-ros2` container
- Extends it with OpenPI-specific mounts
- Loads `.env.openpi` for PyTorch environment variables
- Mounts `~/cobot_ws` → `/workspace/isaaclab/cobot_ws`
- Mounts `~/.venv_cobot` → `/root/.venv_cobot` (venv lives on host!)

The `restart` command starts an existing stopped container without rebuilding (use this for daily work).

### 2. Enter Container

```bash
cd ~/IsaacLab/docker
./container.py enter ros2
```

### 3. One-Time Setup (Inside Container)

```bash
# Setup Python virtual environment with OpenPI (auto-installs uv if missing)
bash /workspace/isaaclab/scripts/openpi_setup/setup_venv_cobot.sh

# Build ROS2 packages
bash /workspace/isaaclab/scripts/openpi_setup/build_cobot_ws.sh
```

**Note**: The `setup_venv_cobot.sh` script will automatically install `uv` if it's not already present in the container.

### 4. Running the Bridge (Every Session)

```bash
# Activate environment
source /workspace/isaaclab/scripts/openpi_setup/activate_openpi_env.sh

# Run OpenPI bridge
ros2 run openpi_kinova_ros2 openpi_kinova_direct_bridge.py \
  --ros-args \
  -p simulation_mode:=true \
  -p openpi_config:=pi05_cobot \
  -p openpi_checkpoint_dir:=/path/to/checkpoint
```

### 5. Running IsaacSim

Isaac Sim runs inside the container as a headless process with WebRTC livestream. Do **NOT** activate the venv — Isaac Sim uses its own Python environment.

**Pick an available GPU** first (check `nvidia-smi`), since GPU 0 is often in use:

```bash
# Check GPU usage on host
nvidia-smi
```

**Interactive launch** (enter container first):

```bash
cd ~/IsaacLab/docker
./container.py enter ros2
# Inside container:
CUDA_VISIBLE_DEVICES=5 /workspace/isaaclab/_isaac_sim/runheadless.sh \
  --/app/livestream/publicEndpointAddress=10.0.0.201 \
  --/app/livestream/port=49101
```

**Non-interactive / background launch** (for coding agents, or running without holding a shell):

```bash
cd ~/IsaacLab/docker
./container.py exec ros2 --cmd "nohup bash -c 'CUDA_VISIBLE_DEVICES=5 /workspace/isaaclab/_isaac_sim/runheadless.sh --/app/livestream/publicEndpointAddress=10.0.0.201 --/app/livestream/port=49101 > /tmp/isaac_sim.log 2>&1' &"
```

**Important:** When using `container.py exec` without `nohup ... &`, the Isaac Sim process will die when the exec connection closes. Always background it.

**Connect via web viewer:** Point your local [NVIDIA Omniverse Web Viewer](https://github.com/NVIDIA-Omniverse/web-viewer-sample) at `10.0.0.201:49101`.

**Check if Isaac Sim is running:**

```bash
./container.py exec ros2 --cmd "pgrep -fa kit | head -3"
# Or tail the log
./container.py exec ros2 --cmd "tail -20 /tmp/isaac_sim.log"
```

**If Isaac Sim freezes or becomes unresponsive:**

```bash
# From host machine, kill all Isaac Sim processes in container
cd ~/IsaacLab/docker
./container.py exec ros2 --cmd "pkill -9 -f isaac-sim"
# Or more broadly:
./container.py exec ros2 --cmd "pkill -9 -f kit"
```

### 6. Setting up Isaac Sim Scene

In Isaac Sim UI:

1. **Load the cobot stage**: `File` > `Open` > `/workspace/isaaclab/content/stage-19.usd`
   - This loads the cobot with mounted Kinova arm and all action graphs for ROS2 pub/sub (joint states, camera topics)

2. **Add environment** (optional): In Content browser > `Environments` > `Simple_Room` > drag `simple_room.usd` to viewport

3. **Add props** (optional): In Content browser > `Props` > `Mugs` > drag a mug to the scene

## Architecture

```
Host Machine                           Container
───────────────                        ──────────────────────────────
~/IsaacLab/docker/                    /workspace/isaaclab/docker/
├── docker-compose.yaml               ├── [compose config]
├── docker-compose.openpi.yaml        └── [env vars from .env.openpi]
├── .env.openpi                       
└── Dockerfile.ros2                   

~/IsaacLab/scripts/openpi_setup/ →   /workspace/isaaclab/scripts/openpi_setup/
├── setup_venv_cobot.sh               ├── setup_venv_cobot.sh
├── build_cobot_ws.sh                 ├── build_cobot_ws.sh
└── activate_openpi_env.sh            └── activate_openpi_env.sh

~/cobot_ws/                      →   /workspace/isaaclab/cobot_ws/
  └── src/openpi_kinova_ros2/         └── src/openpi_kinova_ros2/

~/.venv_cobot/                   →   /root/.venv_cobot/
  (PERSISTS ON HOST!)                 (bind-mounted, not in image)
```

## Docker/Podman Files

### docker-compose.yaml (Base)
- Base Isaac Lab configuration
- Isaac Sim installation
- GPU access via nvidia runtime
- Volume mounts for caches

### .env.openpi (NEW)
**OpenPI-specific environment variables**

Automatically sourced by `container.py` when using the `openpi` profile or `--files` mode.

Contains:
- `PYTORCH_JIT=0` - Disables PyTorch JIT compilation
- `TORCH_COMPILE_DISABLE=1` - Disables torch.compile()
- `OPENPI_VENV=/root/.venv_cobot` - Venv location
- `UV_COMPILE_BYTECODE=1` - UV optimization
- `UV_PROJECT_ENVIRONMENT=/root/.venv_cobot` - UV venv path
- `UV_LINK_MODE=copy` - UV package installation mode
- `GIT_LFS_SKIP_SMUDGE=1` - Skip Git LFS downloads

### docker-compose.openpi.yaml (Extension File)
**REPLACES docker-compose.ros-bridge.yaml**

A simple extension file that adds OpenPI-specific configuration to `isaac-lab-ros2`:

Features:
- Extends the `ros2` service (not a standalone service)
- Adds `~/cobot_ws` workspace mount
- Adds `~/.venv_cobot` directory mount (bind-mount from host!)
- Works with `--env-files .env.openpi` to load environment variables

Usage pattern: `--files docker-compose.openpi.yaml --env-files .env.openpi`


## Scripts

### setup_venv_cobot.sh
**Purpose**: One-time setup of Python virtual environment

**What it does**:
- **Auto-installs `uv`** if not present in container
- Creates `/root/.venv_cobot` with uv
- Installs OpenPI dependencies
- Installs PyTorch with CUDA 12.1
- Replaces transformers (OpenPI custom version)
- Installs Kortex SDK API (Mock for simulation)
- Installs roboticstoolbox-python and spatialmath-python for FK/IK
- Installs OpenPI packages as editable

**When to run**: Once after first container start, or when dependencies change

**Handles**: All setup automatically, including `uv` installation

### build_cobot_ws.sh
**Purpose**: Build ROS2 workspace packages

**What it does**:
- Checks you're not in venv (prevents build errors)
- Sources ROS2 Humble
- Builds only necessary packages
- Ignores problematic packages

**Options**:
- `--clean`: Clean rebuild from scratch

**When to run**: After pulling code changes, or when ROS2 packages change

### activate_openpi_env.sh
**Purpose**: Setup environment for running OpenPI bridge

**What it does**:
- Activates Python venv
- Sets PyTorch environment variables
- Sources ROS2 Humble
- Sources cobot workspace

**When to run**: Every time you enter the container (before running the bridge)

## Environment Variables

Set automatically by `.env.openpi` (sourced by `container.py`):

- `PYTORCH_JIT=0` - Disables PyTorch JIT compilation
- `TORCH_COMPILE_DISABLE=1` - Disables torch.compile()
- `OPENPI_VENV=/root/.venv_cobot` - Path to venv
- `UV_COMPILE_BYTECODE=1` - UV optimization
- `UV_PROJECT_ENVIRONMENT=/root/.venv_cobot` - UV venv path
- `UV_LINK_MODE=copy` - UV package installation mode
- `GIT_LFS_SKIP_SMUDGE=1` - Skip Git LFS downloads

These are automatically available in the container and don't need to be manually exported.

## Troubleshooting

### "ModuleNotFoundError: No module named 'catkin_pkg'" during build
**Solution**: Deactivate venv before building
```bash
deactivate
bash /workspace/isaaclab/scripts/openpi_setup/build_cobot_ws.sh
```

### "Triton compilation failed" errors
**Solution**: Environment variables are set correctly in docker-compose.openpi.yaml.
If still seeing errors, manually export:
```bash
export PYTORCH_JIT=0
export TORCH_COMPILE_DISABLE=1
```

### Venv not persisting between container restarts
**Solution**: Check that `docker-compose.ros-bridge.yaml` is being used when starting container

### "Failed to find the following files" during colcon build
**Solution**: Clean build with `--clean` flag:
```bash
bash /workspace/isaaclab/scripts/openpi_setup/build_cobot_ws.sh --clean
```

## Advanced Usage

### Custom Container Suffix
```bash
./container.py start ros2 --suffix openpi \
  --files docker-compose.openpi.yaml \
  --env-files .env.openpi
```
Creates: `isaac-lab-ros2-openpi` container

### Additional Environment Files
```bash
./container.py start ros2 \
  --files docker-compose.openpi.yaml \
  --env-files .env.openpi .env.custom
```

### Generate Final Compose Configuration
```bash
./container.py config ros2 \
  --files docker-compose.openpi.yaml \
  --env-files .env.openpi \
  --output-yaml docker-compose.final.yaml
```

## Container Management Commands

```bash
# Start container with OpenPI extensions (builds image if needed)
./container.py start ros2 --files docker-compose.openpi.yaml --env-files .env.openpi

# Restart stopped container (no rebuild - fast!)
./container.py restart ros2 --files docker-compose.openpi.yaml --env-files .env.openpi

# Execute command in container (non-interactive, for automation/LLMs/coding-agents)
./container.py exec ros2 --cmd "ls -la /workspace/isaaclab"
./container.py exec ros2 --workdir /workspace/isaaclab/cobot_ws --cmd "colcon list"

# Enter container (interactive shell)
./container.py enter ros2

# Stop container
./container.py stop ros2

# Copy artifacts from container to host
./container.py copy ros2
```

## What's New / Consolidated

### ✅ Changes Made:

1. **Created `.env.openpi`** (NEW!)
   - Contains all OpenPI-specific environment variables
   - Automatically sourced by `container.py`
   - Clean separation of configuration from code

2. **Simplified `docker-compose.openpi.yaml`**
   - **REPLACES** `docker-compose.ros-bridge.yaml` (can delete old file)
   - Simple extension file that adds OpenPI mounts to `ros2` service
   - No standalone profile needed - keeps it simple!
   - Used with `--files` and `--env-files` flags

3. **Created Shell Scripts** (in `scripts/openpi_setup/`)
   - `setup_venv_cobot.sh` - Creates venv on HOST (bind-mounted)
   - `build_cobot_ws.sh` - Builds ROS2 workspace (bind-mounted source)
   - `activate_openpi_env.sh` - Activates environment each session

### Why Hybrid Approach?

| Component | Location | Why |
|-----------|----------|-----|
| `uv` tool | Dockerfile | Static, same for everyone |
| Env vars | `.env.openpi` | Configuration, follows Isaac Lab pattern |
| Venv creation | Script | Lives on HOST, persists across rebuilds |
| ROS2 build | Script | Source code changes frequently |

### Migration from Old Setup:

If you were using `docker-compose.ros-bridge.yaml`:

```bash
# OLD WAY
./container.py start ros2 --files docker-compose.ros-bridge.yaml

# NEW WAY
./container.py start ros2 --files docker-compose.openpi.yaml --env-files .env.openpi
```

You can now **delete** `docker-compose.ros-bridge.yaml` - its functionality is merged into `docker-compose.openpi.yaml` + `.env.openpi`.

## Files Reference

| File | Location | Purpose | When Modified |
|------|----------|---------|---------------|
| `.env.openpi` | `docker/` | Environment variables | When adding new env vars |
| `setup_venv_cobot.sh` | `scripts/openpi_setup/` | Create venv on host | Once or when deps change |
| `build_cobot_ws.sh` | `scripts/openpi_setup/` | Build ROS2 packages | When code changes |
| `activate_openpi_env.sh` | `scripts/openpi_setup/` | Activate environment | Every session |
| `docker-compose.openpi.yaml` | `docker/` | Extension config | When changing mounts/volumes |

## Integration with IsaacSim

The bridge is designed to work with Isaac Sim's ROS2 interface:

1. **Isaac Sim publishes**:
   - `/joint_states` - Current joint positions
   - Camera topics (optional)

2. **Bridge publishes**:
   - `/joint_command` - Target joint positions (JointState messages)

3. **Required ActionGraph in Isaac Sim**:
   - Joint State Publisher (reads from articulation)
   - Joint State Subscriber (writes to `/joint_command`)
   - Articulation Controller (applies commands)

### Camera settings (Isaac Sim GUI)
Set these on the camera prim in the Stage tree (Properties > Camera). Then set the Render Product resolution accordingly.

- Kinova wrist camera (matches recorded data; ~52.5° × 31.0° FOV)
  - Projection: Perspective (pinhole)
  - Horizontal Aperture: 20.955 mm
  - Vertical Aperture: 11.79 mm    (keeps 16:9 aspect with HA 20.955)
  - Focal Length: 21.24 mm         (yields ~52.5° horizontal FOV)
  - Resolution: 1280 × 720         (Render Product or sensor resolution)
  - Why: Matches the original capture intrinsics/resolution used to train/evaluate; resizing during preprocessing does not change FOV.

- Azure Kinect (WFOV 120° × 120° depth mode)
  - Projection: Perspective (pinhole)
  - Horizontal Aperture: 20.955 mm
  - Vertical Aperture: 20.955 mm   (square aperture → equal HFOV and VFOV)
  - Focal Length: 6.05 mm          (yields ~120° horizontal FOV)
  - Resolution: set per your pipeline (FOV is aperture+focal-length defined)
  - Why: WFOV is isotropic; equal apertures ensure HFOV ≈ VFOV, and 6.05 mm focal length matches ~120° with the default aperture.

## Support

For issues:
- **OpenPI**: https://github.com/Physical-Intelligence/openpi
- **Isaac Lab**: https://github.com/isaac-sim/IsaacLab
- **This bridge**: Check `~/cobot_ws/src/openpi_kinova_ros2/README.md`

