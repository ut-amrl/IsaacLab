#!/usr/bin/env bash
# Apptainer + partial IsaacLab binds: run cuRobo FK demo (Isaac Sim python + GPU).
# Invoked by srun_curobo_fk_gen3_model.sh on a GPU node.
set -euo pipefail

export USER="${USER:-$(id -un)}"

CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${CLUSTER_DIR}/../.." && pwd)"
COBOT2_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
mkdir -p "${REPO_ROOT}/docker/logs"

module purge 2>/dev/null || true
module load cuda/12.8
HOST_CUDA="${CUDA_HOME:-/opt/nvidia/hpc_sdk/Linux_x86_64/25.3/cuda/12.8}"
test -x "${HOST_CUDA}/bin/nvcc"
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.0+PTX}"

# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

: "${ISAACLAB_SIF:=${REPO_ROOT}/../.sifs/isaac-lab_test.sif}"
export HOME="${HOME:-/u/${USER:-}}"
unset APPTAINERENV_PYTHONNOUSERSITE 2>/dev/null || true
unset APPTAINERENV_HOME 2>/dev/null || true

echo "[$(date -Is)] FK demo REPO_ROOT=${REPO_ROOT} COBOT2_ROOT=${COBOT2_ROOT} SIF=${ISAACLAB_SIF}" >&2
test -f "${ISAACLAB_SIF}"

FK_INNER='cd /workspace/isaaclab && exec "${ISAACSIM_PATH}/python.sh" scripts/tutorials/00_sim/curobo_forward_kinematics_gen3_demo.py'
if [[ "$#" -eq 0 ]]; then
  apptainer_cmd=(bash -lc "${FK_INNER}")
else
  QUOTE_ARGS=$(printf '%q ' "$@")
  apptainer_cmd=(bash -lc "${FK_INNER} ${QUOTE_ARGS}")
fi

apptainer exec --nv --cleanenv \
  --env ISAACSIM_PATH=/workspace/isaaclab/_isaac_sim \
  --env OMNI_KIT_ALLOW_ROOT=1 \
  --env CUDA_HOME=/mnt/host_cuda12.8 \
  --env "PATH=/mnt/host_cuda12.8/bin:${PATH:-}" \
  --env "LD_LIBRARY_PATH=/mnt/host_cuda12.8/lib64:${LD_LIBRARY_PATH:-}" \
  --env TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}" \
  -B "${HOST_CUDA}:/mnt/host_cuda12.8" \
  -B "${COBOT2_ROOT}/external/curobo_isaac_build:/mnt/curobo_build" \
  -B "${HOME}/.local:/root/.local" \
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
  "${ISAACLAB_SIF}" "${apptainer_cmd[@]}"

echo "[$(date -Is)] FK demo done."
