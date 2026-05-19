#!/bin/bash
# Smoke: Isaac Lab + cuRobo MPC cuboid tracking (load_cobot_mpc_cuboid_track.py).
# Logs under IsaacLab/docker/logs (same layout as smoke_sim_load_cobot.slurm).
# Optional: COBOT_MPC_MAX_SIM_S (default 22) so the job exits after one cuboid move (10 s) + margin.
#SBATCH -A bger-delta-gpu
#SBATCH -p gpuA40x4
#SBATCH --gpus-per-node=1
#SBATCH --mem=64G
#SBATCH -t 00:30:00
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -J smoke-sim-mpc-cuboid
#SBATCH --export=NONE,ISAACLAB_SIF,COBOT_USD,COBOT_LOAD_MODE,COBOT_URDF_HOST,COBOT_MPC_MAX_SIM_S,MPC_EXTRA_ARGS
#SBATCH -o /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-mpc-cuboid-%j.out
#SBATCH -e /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-mpc-cuboid-%j.err

set -euo pipefail

# Slurm copies this script to /var/spool/slurmd/...; dirname(BASH_SOURCE) is then NOT the repo.
# Use SLURM_SUBMIT_DIR when running under Slurm from a sensible cwd (see smoke_sim_empty.slurm).
# Otherwise use this file's directory (local: bash path/to/smoke_*.slurm from docker/cluster).
_script_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Under Slurm the batch file is often copied away from the repo; smoke_delta is not beside it.
if [[ -n "${SLURM_JOB_ID:-}" ]] && [[ ! -f "${_script_here}/smoke_delta_holosoma_env.sh" ]]; then
  if [[ -z "${SLURM_SUBMIT_DIR:-}" ]]; then
    echo "[smoke_mpc_cuboid] ERROR: Slurm job but SLURM_SUBMIT_DIR is empty." >&2
    exit 2
  fi
  CLUSTER_DIR="$(cd "${SLURM_SUBMIT_DIR}" && pwd)"
else
  CLUSTER_DIR="${_script_here}"
fi
if [[ ! -f "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh" ]]; then
  echo "[smoke_mpc_cuboid] ERROR: CLUSTER_DIR=${CLUSTER_DIR} is not IsaacLab/docker/cluster." >&2
  echo "[smoke_mpc_cuboid] Fix: cd .../IsaacLab/docker/cluster && sbatch smoke_sim_load_cobot_mpc_cuboid.slurm" >&2
  exit 2
fi
REPO_ROOT="$(cd "${CLUSTER_DIR}/../.." && pwd)"
mkdir -p "${REPO_ROOT}/docker/logs"

# Avoid Apptainer/NVIDIA hooks trying mkdir /var/spool/docker (login-node env leakage).
unset DOCKER_HOST DOCKER_CONTEXT DOCKER_CONTENT_TRUST DOCKER_CONFIG DOCKER_TMPDIR CONTAINERS_CONFIG 2>/dev/null || true
# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

: "${ISAACLAB_SIF:=${REPO_ROOT}/../.sifs/isaac-lab_test.sif}"
: "${COBOT_USD:=/workspace/isaaclab/content/stage-19.usd}"
: "${COBOT_LOAD_MODE:=urdf}"
: "${COBOT_MPC_MAX_SIM_S:=22}"
: "${MPC_EXTRA_ARGS:=--simple-cuboid-mpc --no-mpc-self-collision --diag-extended-csv --log-mpc-every-n-steps 200}"
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
echo "[$(date -Is)] COBOT_MPC_MAX_SIM_S=${COBOT_MPC_MAX_SIM_S}" | tee /dev/stderr
echo "[$(date -Is)] MPC_EXTRA_ARGS=${MPC_EXTRA_ARGS:-}" | tee /dev/stderr
test -f "${ISAACLAB_SIF}"

if [[ "${COBOT_LOAD_MODE}" == "usd" ]]; then
  host_usd="${COBOT_USD#/workspace/isaaclab/}"
  if [[ "${COBOT_USD}" == /workspace/isaaclab/* ]]; then
    if [[ ! -f "${REPO_ROOT}/${host_usd}" ]]; then
      echo "[$(date -Is)] ERROR: missing host USD ${REPO_ROOT}/${host_usd} (from COBOT_USD=${COBOT_USD})." >&2
      exit 2
    fi
  fi
  COBOT_URDF_IN_CONTAINER=""
else
  case "${COBOT_URDF_HOST}" in
    "${COBOT2_ROOT}/ros2_kortex"/*) ;;
    *)
      echo "[$(date -Is)] ERROR: COBOT_URDF_HOST must be under ${COBOT2_ROOT}/ros2_kortex (got ${COBOT_URDF_HOST})." >&2
      exit 2
      ;;
  esac
  if [[ ! -f "${COBOT_URDF_HOST}" ]]; then
    echo "[$(date -Is)] ERROR: missing URDF ${COBOT_URDF_HOST}. Clone ros2_kortex (cobot-urdf), git lfs pull for meshes." >&2
    exit 2
  fi
  mkdir -p "${REPO_ROOT}/content/cobot_runtime"
  COBOT_URDF_IN_CONTAINER="/workspace/ros2_kortex/${COBOT_URDF_HOST#${COBOT2_ROOT}/ros2_kortex/}"
  export COBOT_URDF_IN_CONTAINER
fi

mkdir -p "${COBOT2_ROOT}/external/curobo_isaac_build"
unset APPTAINERENV_PYTHONNOUSERSITE 2>/dev/null || true
export HOME="${HOME:-/u/${USER:-}}"

apptainer exec --nv --cleanenv \
  --env ISAACSIM_PATH=/workspace/isaaclab/_isaac_sim \
  --env OMNI_KIT_ALLOW_ROOT=1 \
  --env COBOT_USD="${COBOT_USD}" \
  --env COBOT_LOAD_MODE="${COBOT_LOAD_MODE}" \
  --env COBOT_URDF_IN_CONTAINER="${COBOT_URDF_IN_CONTAINER:-}" \
  --env COBOT_MPC_MAX_SIM_S="${COBOT_MPC_MAX_SIM_S}" \
  --env MPC_EXTRA_ARGS="${MPC_EXTRA_ARGS:-}" \
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
  "${ISAACLAB_SIF}" bash -lc 'cd /workspace/isaaclab || exit 1; _ec=0; if [[ "${COBOT_LOAD_MODE}" == "usd" ]]; then ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py --headless --usd "${COBOT_USD}" --robot-yml /workspace/isaaclab/content/cobot_runtime/gen3_curobo_robot.yml --max-sim-time-s "${COBOT_MPC_MAX_SIM_S}" ${MPC_EXTRA_ARGS} || _ec=$?; else ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py --headless --fix-base --force-urdf-conversion --urdf "${COBOT_URDF_IN_CONTAINER}" --usd-out /workspace/isaaclab/content/cobot_runtime/cobot.usd --robot-yml /workspace/isaaclab/content/cobot_runtime/gen3_curobo_robot.yml --max-sim-time-s "${COBOT_MPC_MAX_SIM_S}" ${MPC_EXTRA_ARGS} || _ec=$?; fi; echo "[smoke_mpc_cuboid] isaaclab_py_exit=${_ec}"; exit "${_ec}"'

echo "[$(date -Is)] Done."
