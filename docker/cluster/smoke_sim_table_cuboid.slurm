#!/bin/bash
# Logs: absolute paths under IsaacLab/docker/logs (same style as pull_ngc_isaac_lab.slurm).
# Delta Apptainer layout: holosoma-kick `isaacsim_t1_ball_container_test.slurm` (see smoke_delta_holosoma_env.sh).
#SBATCH -A bger-delta-gpu
#SBATCH -p gpuA100x4
#SBATCH --gpus-per-node=1
#SBATCH --mem=64G
#SBATCH -t 00:30:00
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -J smoke-sim-table
#SBATCH --export=NONE,ISAACLAB_SIF,COBOT_USD
#SBATCH -o /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-table-%j.out
#SBATCH -e /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-table-%j.err

set -euo pipefail

if [[ -n "${SLURM_SUBMIT_DIR:-}" ]]; then
  CLUSTER_DIR="$(cd "${SLURM_SUBMIT_DIR}" && pwd)"
else
  CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
REPO_ROOT="$(cd "${CLUSTER_DIR}/../.." && pwd)"
COBOT2_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
mkdir -p "${REPO_ROOT}/docker/logs"
# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

: "${ISAACLAB_SIF:=${REPO_ROOT}/../.sifs/isaac-lab_test.sif}"

echo "[$(date -Is)] REPO_ROOT=${REPO_ROOT}" | tee /dev/stderr
echo "[$(date -Is)] SCRATCH=${SCRATCH} HOST_CACHE_DIR=${HOST_CACHE_DIR}" | tee /dev/stderr
echo "[$(date -Is)] APPTAINER_TMPDIR=${APPTAINER_TMPDIR}" | tee /dev/stderr
echo "[$(date -Is)] SIF=${ISAACLAB_SIF}" | tee /dev/stderr
echo "[$(date -Is)] which apptainer: $(command -v apptainer)" | tee /dev/stderr
test -f "${ISAACLAB_SIF}"

mkdir -p "${COBOT2_ROOT}/external/curobo_isaac_build"
unset APPTAINERENV_PYTHONNOUSERSITE 2>/dev/null || true
export HOME="${HOME:-/u/${USER:-}}"

apptainer exec --nv --cleanenv \
  --env ISAACSIM_PATH=/workspace/isaaclab/_isaac_sim \
  --env OMNI_KIT_ALLOW_ROOT=1 \
  -B "${COBOT2_ROOT}/external/curobo_isaac_build:/mnt/curobo_build" \
  -B "${HOME}/.local:${HOME}/.local" \
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
  "${ISAACLAB_SIF}" bash -lc 'cd /workspace/isaaclab && ./isaaclab.sh -p scripts/tutorials/00_sim/spawn_table_blue_cuboid.py --headless'

echo "[$(date -Is)] Done."
