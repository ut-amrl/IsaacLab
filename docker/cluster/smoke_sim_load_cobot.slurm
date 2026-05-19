#!/bin/bash
# Logs: absolute paths under IsaacLab/docker/logs (same style as pull_ngc_isaac_lab.slurm).
# With --record-cobot-video, also writes IsaacLab/docker/logs/cobot_stage_videos/<JOBID>/cobot_stage.mp4
# (PNG frames in .../frames/). Drop --record-cobot-video and --enable_cameras in the apptainer line for a faster non-render smoke.
# Delta Apptainer layout: holosoma-kick `isaacsim_t1_ball_container_test.slurm` (see smoke_delta_holosoma_env.sh).
#SBATCH -A bger-delta-gpu
#SBATCH -p gpuA100x4
#SBATCH --gpus-per-node=1
#SBATCH --mem=64G
#SBATCH -t 00:30:00
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -J smoke-sim-cobot
#SBATCH --export=NONE,ISAACLAB_SIF,COBOT_USD,COBOT_LOAD_MODE,COBOT_URDF_HOST
#SBATCH -o /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-cobot-%j.out
#SBATCH -e /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-cobot-%j.err

set -euo pipefail

if [[ -n "${SLURM_SUBMIT_DIR:-}" ]]; then
  CLUSTER_DIR="$(cd "${SLURM_SUBMIT_DIR}" && pwd)"
else
  CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
REPO_ROOT="$(cd "${CLUSTER_DIR}/../.." && pwd)"
mkdir -p "${REPO_ROOT}/docker/logs"
# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

: "${ISAACLAB_SIF:=${REPO_ROOT}/../.sifs/isaac-lab_test.sif}"
: "${COBOT_USD:=/workspace/isaaclab/content/stage-19.usd}"
: "${COBOT_LOAD_MODE:=urdf}"
COBOT2_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
: "${COBOT_URDF_HOST:=${COBOT2_ROOT}/ros2_kortex/kortex_description/robots/gen3_2f85.urdf}"

echo "[$(date -Is)] REPO_ROOT=${REPO_ROOT}" | tee /dev/stderr
echo "[$(date -Is)] COBOT2_ROOT=${COBOT2_ROOT}" | tee /dev/stderr
echo "[$(date -Is)] SCRATCH=${SCRATCH} HOST_CACHE_DIR=${HOST_CACHE_DIR}" | tee /dev/stderr
echo "[$(date -Is)] APPTAINER_TMPDIR=${APPTAINER_TMPDIR}" | tee /dev/stderr
echo "[$(date -Is)] SIF=${ISAACLAB_SIF}" | tee /dev/stderr
echo "[$(date -Is)] which apptainer: $(command -v apptainer)" | tee /dev/stderr
echo "[$(date -Is)] COBOT_LOAD_MODE=${COBOT_LOAD_MODE}" | tee /dev/stderr
echo "[$(date -Is)] COBOT_USD=${COBOT_USD}" | tee /dev/stderr
echo "[$(date -Is)] COBOT_URDF_HOST=${COBOT_URDF_HOST}" | tee /dev/stderr
test -f "${ISAACLAB_SIF}"

if [[ "${COBOT_LOAD_MODE}" == "usd" ]]; then
  host_usd="${COBOT_USD#/workspace/isaaclab/}"
  if [[ "${COBOT_USD}" == /workspace/isaaclab/* ]]; then
    if [[ ! -f "${REPO_ROOT}/${host_usd}" ]]; then
      echo "[$(date -Is)] ERROR: missing host USD ${REPO_ROOT}/${host_usd} (from COBOT_USD=${COBOT_USD})." >&2
      exit 2
    fi
  fi
else
  case "${COBOT_URDF_HOST}" in
    "${COBOT2_ROOT}/ros2_kortex"/*) ;;
    *)
      echo "[$(date -Is)] ERROR: COBOT_URDF_HOST must be under ${COBOT2_ROOT}/ros2_kortex (got ${COBOT_URDF_HOST})." >&2
      exit 2
      ;;
  esac
  if [[ ! -f "${COBOT_URDF_HOST}" ]]; then
    echo "[$(date -Is)] ERROR: missing URDF ${COBOT_URDF_HOST}. Clone https://github.com/ut-amrl/ros2_kortex and checkout branch cobot-urdf (or point COBOT_URDF_HOST at a resolved .urdf under ros2_kortex/). Install git-lfs, run 'git lfs install' once, then 'git lfs pull' in the clone so meshes are not LFS pointer stubs." >&2
    exit 2
  fi
  mkdir -p "${REPO_ROOT}/content/cobot_runtime"
  COBOT_URDF_IN_CONTAINER="/workspace/ros2_kortex/${COBOT_URDF_HOST#${COBOT2_ROOT}/ros2_kortex/}"
  export COBOT_URDF_IN_CONTAINER
fi

# Editable cuRobo (Isaac Sim pip) uses .pth under ~/.local and sources under external/curobo_isaac_build;
# PYTHONNOUSERSITE=1 (smoke_delta_holosoma_env.sh) would hide user-site; drop it for this job.
mkdir -p "${COBOT2_ROOT}/external/curobo_isaac_build"
unset APPTAINERENV_PYTHONNOUSERSITE 2>/dev/null || true
export HOME="${HOME:-/u/${USER:-}}"

apptainer exec --nv --cleanenv \
  --env ISAACSIM_PATH=/workspace/isaaclab/_isaac_sim \
  --env OMNI_KIT_ALLOW_ROOT=1 \
  --env COBOT_USD="${COBOT_USD}" \
  --env COBOT_LOAD_MODE="${COBOT_LOAD_MODE}" \
  --env COBOT_URDF_IN_CONTAINER="${COBOT_URDF_IN_CONTAINER:-}" \
  -B "${COBOT2_ROOT}/external/curobo_isaac_build:/mnt/curobo_build" \
  -B "${HOME}/.local:${HOME}/.local" \
  -B "${COBOT2_ROOT}/ros2_kortex:/workspace/ros2_kortex:ro" \
  -B "${SCRATCH}:${SCRATCH}" \
  -B "${HOST_CACHE_DIR}/data:/isaac-sim/kit/data" \
  -B "${HOST_CACHE_DIR}/cache:/isaac-sim/kit/cache" \
  -B "${HOST_CACHE_DIR}/logs:/isaac-sim/kit/logs" \
  -B "${HOST_CACHE_DIR}/ov_data:/root/.nvidia-omniverse" \
  -B "${HOST_CACHE_DIR}/ov:/ov" \
  -B "${REPO_ROOT}/source:/workspace/isaaclab/source" \
  -B "${REPO_ROOT}/scripts:/workspace/isaaclab/scripts" \
  -B "${REPO_ROOT}/docs:/workspace/isaaclab/docs" \
  -B "${REPO_ROOT}/tools:/workspace/isaaclab/tools" \
  -B "${REPO_ROOT}/docker:/workspace/isaaclab/docker" \
  -B "${REPO_ROOT}/isaaclab.sh:/workspace/isaaclab/isaaclab.sh" \
  -B "${REPO_ROOT}/VERSION:/workspace/isaaclab/VERSION" \
  -B "${REPO_ROOT}/pyproject.toml:/workspace/isaaclab/pyproject.toml" \
  -B "${REPO_ROOT}/README.md:/workspace/isaaclab/README.md" \
  -B "${REPO_ROOT}/content:/workspace/isaaclab/content" \
  "${ISAACLAB_SIF}" bash -lc 'cd /workspace/isaaclab && if [[ "${COBOT_LOAD_MODE}" == "usd" ]]; then ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_stage.py --headless --enable_cameras --record-cobot-video --usd "${COBOT_USD}"; else ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_stage.py --headless --enable_cameras --record-cobot-video --urdf "${COBOT_URDF_IN_CONTAINER}" --usd-out /workspace/isaaclab/content/cobot_runtime/cobot.usd; fi'

echo "[$(date -Is)] Done."
