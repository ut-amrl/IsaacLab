#!/bin/bash
# MPC cuboid smoke + RGB recording -> IsaacLab/docker/logs/mpc_cuboid_track_videos/<JOBID>/mpc_cuboid_track.mp4
# Requires load_cobot_mpc_cuboid_track.py with --record-cobot-video support (see INSTRUCTIONS in docker/logs).
#SBATCH -A bger-delta-gpu
#SBATCH -p gpuA100x4
#SBATCH --gpus-per-node=1
#SBATCH --mem=64G
#SBATCH -t 00:30:00
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -J smoke-mpc-cuboid-vid
#SBATCH --export=NONE,ISAACLAB_SIF,COBOT_USD,COBOT_LOAD_MODE,COBOT_URDF_HOST,COBOT_MPC_MAX_SIM_S,RECORD_FPS,MPC_EXTRA_ARGS
#SBATCH -o /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-mpc-cuboid-vid-%j.out
#SBATCH -e /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-mpc-cuboid-vid-%j.err

set -euo pipefail

_script_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${SLURM_JOB_ID:-}" ]] && [[ ! -f "${_script_here}/smoke_delta_holosoma_env.sh" ]]; then
  if [[ -z "${SLURM_SUBMIT_DIR:-}" ]]; then
    echo "[smoke_mpc_cuboid_vid] ERROR: Slurm job but SLURM_SUBMIT_DIR is empty." >&2
    exit 2
  fi
  CLUSTER_DIR="$(cd "${SLURM_SUBMIT_DIR}" && pwd)"
else
  CLUSTER_DIR="${_script_here}"
fi
if [[ ! -f "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh" ]]; then
  echo "[smoke_mpc_cuboid_vid] ERROR: CLUSTER_DIR=${CLUSTER_DIR} is not IsaacLab/docker/cluster." >&2
  exit 2
fi
REPO_ROOT="$(cd "${CLUSTER_DIR}/../.." && pwd)"
mkdir -p "${REPO_ROOT}/docker/logs/mpc_cuboid_track_videos"

unset DOCKER_HOST DOCKER_CONTEXT DOCKER_CONTENT_TRUST DOCKER_CONFIG DOCKER_TMPDIR CONTAINERS_CONFIG 2>/dev/null || true
# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

: "${MPC_EXTRA_ARGS:=--simple-cuboid-mpc --no-mpc-self-collision --diag-extended-csv --log-mpc-every-n-steps 200 --cuboid-displace-interval-s 9 --randomize-scene-positions --viz-mpc-usd --mpc-viz-horizon-steps 80}"
# Different A/B + sphere layout per job, reproducible for a given Slurm job id.
if [[ -n "${SLURM_JOB_ID:-}" ]] && [[ "${MPC_EXTRA_ARGS}" != *"--random-scene-seed"* ]]; then
  MPC_EXTRA_ARGS="${MPC_EXTRA_ARGS} --random-scene-seed ${SLURM_JOB_ID}"
fi

: "${ISAACLAB_SIF:=${REPO_ROOT}/../.sifs/isaac-lab_test.sif}"
: "${COBOT_USD:=/workspace/isaaclab/content/stage-19.usd}"
: "${COBOT_LOAD_MODE:=urdf}"
: "${COBOT_MPC_MAX_SIM_S:=15}"
: "${RECORD_FPS:=24}"
COBOT2_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
: "${COBOT_URDF_HOST:=${COBOT2_ROOT}/ros2_kortex/kortex_description/robots/gen3_2f85.urdf}"

echo "[$(date -Is)] COBOT_MPC_MAX_SIM_S=${COBOT_MPC_MAX_SIM_S} RECORD_FPS=${RECORD_FPS} MPC_EXTRA_ARGS=${MPC_EXTRA_ARGS:-}" | tee /dev/stderr
test -f "${ISAACLAB_SIF}"

if [[ "${COBOT_LOAD_MODE}" == "usd" ]]; then
  host_usd="${COBOT_USD#/workspace/isaaclab/}"
  if [[ "${COBOT_USD}" == /workspace/isaaclab/* ]]; then
    if [[ ! -f "${REPO_ROOT}/${host_usd}" ]]; then
      echo "[$(date -Is)] ERROR: missing host USD ${REPO_ROOT}/${host_usd}" >&2
      exit 2
    fi
  fi
  COBOT_URDF_IN_CONTAINER=""
else
  case "${COBOT_URDF_HOST}" in
    "${COBOT2_ROOT}/ros2_kortex"/*) ;;
    *)
      echo "[$(date -Is)] ERROR: COBOT_URDF_HOST must be under ${COBOT2_ROOT}/ros2_kortex" >&2
      exit 2
      ;;
  esac
  if [[ ! -f "${COBOT_URDF_HOST}" ]]; then
    echo "[$(date -Is)] ERROR: missing URDF ${COBOT_URDF_HOST}" >&2
    exit 2
  fi
  mkdir -p "${REPO_ROOT}/content/cobot_runtime"
  COBOT_URDF_IN_CONTAINER="/workspace/ros2_kortex/${COBOT_URDF_HOST#${COBOT2_ROOT}/ros2_kortex/}"
  export COBOT_URDF_IN_CONTAINER
fi

mkdir -p "${COBOT2_ROOT}/external/curobo_isaac_build"
unset APPTAINERENV_PYTHONNOUSERSITE 2>/dev/null || true
export HOME="${HOME:-/u/${USER:-}}"

ISAAC_EC=0
apptainer exec --nv --cleanenv \
  --env ISAACSIM_PATH=/workspace/isaaclab/_isaac_sim \
  --env OMNI_KIT_ALLOW_ROOT=1 \
  --env SLURM_JOB_ID="${SLURM_JOB_ID:-}" \
  --env COBOT_USD="${COBOT_USD}" \
  --env COBOT_LOAD_MODE="${COBOT_LOAD_MODE}" \
  --env COBOT_URDF_IN_CONTAINER="${COBOT_URDF_IN_CONTAINER:-}" \
  --env COBOT_MPC_MAX_SIM_S="${COBOT_MPC_MAX_SIM_S}" \
  --env RECORD_FPS="${RECORD_FPS}" \
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
  "${ISAACLAB_SIF}" bash -lc 'cd /workspace/isaaclab || exit 1; _ec=0; if [[ "${COBOT_LOAD_MODE}" == "usd" ]]; then ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py --headless --enable_cameras --record-cobot-video --record-duration-s "${COBOT_MPC_MAX_SIM_S}" --record-fps "${RECORD_FPS}" --record-video-out-dir /workspace/isaaclab/docker/logs/mpc_cuboid_track_videos --record-camera-eye-m -4 0 4 --record-camera-target-m 0.6 0.4 1.1 --max-sim-time-s "${COBOT_MPC_MAX_SIM_S}" --usd "${COBOT_USD}" --robot-yml /workspace/isaaclab/content/cobot_runtime/gen3_curobo_robot.yml ${MPC_EXTRA_ARGS} || _ec=$?; else ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py --headless --fix-base --force-urdf-conversion --enable_cameras --record-cobot-video --record-duration-s "${COBOT_MPC_MAX_SIM_S}" --record-fps "${RECORD_FPS}" --record-video-out-dir /workspace/isaaclab/docker/logs/mpc_cuboid_track_videos --record-camera-eye-m -4 0 4 --record-camera-target-m 0.6 0.4 1.1 --max-sim-time-s "${COBOT_MPC_MAX_SIM_S}" --urdf "${COBOT_URDF_IN_CONTAINER}" --usd-out /workspace/isaaclab/content/cobot_runtime/cobot.usd --robot-yml /workspace/isaaclab/content/cobot_runtime/gen3_curobo_robot.yml ${MPC_EXTRA_ARGS} || _ec=$?; fi; echo "[smoke_mpc_cuboid_vid] isaaclab_py_exit=${_ec}"; exit "${_ec}"' \
  || ISAAC_EC=$?

# Host-side MP4 (container often has no ffmpeg); uses SLURM_JOB_ID for run folder when passed into the job.
VID_ROOT="${REPO_ROOT}/docker/logs/mpc_cuboid_track_videos"
if [[ -n "${SLURM_JOB_ID:-}" ]]; then
  _vdir="${VID_ROOT}/${SLURM_JOB_ID}"
  if [[ -d "${_vdir}/frames" ]] && [[ ! -f "${_vdir}/mpc_cuboid_track.mp4" ]]; then
    _ff=""
    if command -v ffmpeg >/dev/null 2>&1; then
      _ff="$(command -v ffmpeg)"
    else
      module load ffmpeg 2>/dev/null || module load FFmpeg 2>/dev/null || true
      command -v ffmpeg >/dev/null 2>&1 && _ff="$(command -v ffmpeg)"
    fi
    if [[ -n "${_ff}" ]]; then
      echo "[$(date -Is)] host ffmpeg encoding ${_vdir}/frames -> ${_vdir}/mpc_cuboid_track.mp4" | tee /dev/stderr
      if "${_ff}" -y -hide_banner -loglevel error -framerate "${RECORD_FPS}" -i "${_vdir}/frames/frame_%06d.png" -c:v libx264 -pix_fmt yuv420p "${_vdir}/mpc_cuboid_track.mp4"; then
        echo "[$(date -Is)] host ffmpeg OK: ${_vdir}/mpc_cuboid_track.mp4" | tee /dev/stderr
      else
        echo "[$(date -Is)] host ffmpeg failed (exit $?); PNGs still in ${_vdir}/frames" | tee /dev/stderr
      fi
    else
      echo "[$(date -Is)] host ffmpeg not found; PNGs in ${_vdir}/frames" | tee /dev/stderr
    fi
  fi
fi

echo "[$(date -Is)] Done. Video under ${REPO_ROOT}/docker/logs/mpc_cuboid_track_videos/<jobid>/ (host ffmpeg may add mpc_cuboid_track.mp4)"
exit "${ISAAC_EC}"
