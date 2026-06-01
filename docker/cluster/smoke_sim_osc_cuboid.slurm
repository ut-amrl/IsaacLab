#!/bin/bash
# Smoke: OSC cuboid tracking (osc_cuboid_track.py) — replaces cuRobo MPC with Cartesian impedance OSC.
# Logs: IsaacLab/docker/logs/smoke-osc-cuboid-<JOBID>.out / .err
#SBATCH -A bger-delta-gpu
#SBATCH -p gpuA40x4
#SBATCH --gpus-per-node=1
#SBATCH --mem=64G
#SBATCH -t 00:30:00
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -J smoke-osc-cuboid
#SBATCH --export=NONE,ISAACLAB_SIF,COBOT_USD,COBOT_LOAD_MODE,COBOT_URDF_HOST,OSC_MAX_SIM_S,OSC_EXTRA_ARGS,RECORD_FPS,OSC_RECORD_VIDEO
#SBATCH -o /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-osc-cuboid-%j.out
#SBATCH -e /u/ssebastian/cobot2/IsaacLab/docker/logs/smoke-osc-cuboid-%j.err

set -euo pipefail

_script_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${SLURM_JOB_ID:-}" ]] && [[ ! -f "${_script_here}/smoke_delta_holosoma_env.sh" ]]; then
  if [[ -z "${SLURM_SUBMIT_DIR:-}" ]]; then
    echo "[smoke_osc_cuboid] ERROR: Slurm job but SLURM_SUBMIT_DIR is empty." >&2
    exit 2
  fi
  CLUSTER_DIR="$(cd "${SLURM_SUBMIT_DIR}" && pwd)"
else
  CLUSTER_DIR="${_script_here}"
fi
if [[ ! -f "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh" ]]; then
  echo "[smoke_osc_cuboid] ERROR: CLUSTER_DIR=${CLUSTER_DIR} is not IsaacLab/docker/cluster." >&2
  exit 2
fi
REPO_ROOT="$(cd "${CLUSTER_DIR}/../.." && pwd)"
mkdir -p "${REPO_ROOT}/docker/logs"

unset DOCKER_HOST DOCKER_CONTEXT DOCKER_CONTENT_TRUST DOCKER_CONFIG DOCKER_TMPDIR CONTAINERS_CONFIG 2>/dev/null || true
# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

: "${ISAACLAB_SIF:=${REPO_ROOT}/../.sifs/isaac-lab_test.sif}"
: "${COBOT_USD:=/workspace/isaaclab/content/cobot_runtime/cobot.usd}"
: "${COBOT_LOAD_MODE:=urdf}"
: "${OSC_MAX_SIM_S:=25}"
: "${OSC_EXTRA_ARGS:=--no-table-cuboid --osc-kp 200 --cuboid-displace-interval-s 10}"
: "${RECORD_FPS:=24}"
: "${OSC_RECORD_VIDEO:=1}"

COBOT2_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
: "${COBOT_URDF_HOST:=${COBOT2_ROOT}/ros2_kortex/kortex_description/robots/gen3_2f85.urdf}"

echo "[$(date -Is)] REPO_ROOT=${REPO_ROOT}" | tee /dev/stderr
echo "[$(date -Is)] COBOT2_ROOT=${COBOT2_ROOT}" | tee /dev/stderr
echo "[$(date -Is)] SCRATCH=${SCRATCH} HOST_CACHE_DIR=${HOST_CACHE_DIR}" | tee /dev/stderr
echo "[$(date -Is)] SIF=${ISAACLAB_SIF}" | tee /dev/stderr
echo "[$(date -Is)] COBOT_LOAD_MODE=${COBOT_LOAD_MODE}" | tee /dev/stderr
echo "[$(date -Is)] COBOT_USD=${COBOT_USD}" | tee /dev/stderr
echo "[$(date -Is)] OSC_MAX_SIM_S=${OSC_MAX_SIM_S}" | tee /dev/stderr
echo "[$(date -Is)] OSC_EXTRA_ARGS=${OSC_EXTRA_ARGS:-}" | tee /dev/stderr
echo "[$(date -Is)] RECORD_FPS=${RECORD_FPS} OSC_RECORD_VIDEO=${OSC_RECORD_VIDEO}" | tee /dev/stderr
test -f "${ISAACLAB_SIF}"

if [[ "${COBOT_LOAD_MODE}" == "usd" ]]; then
  COBOT_URDF_IN_CONTAINER=""
else
  case "${COBOT_URDF_HOST}" in
    "${COBOT2_ROOT}/ros2_kortex"/*) ;;
    *)
      echo "[smoke_osc_cuboid] ERROR: COBOT_URDF_HOST must be under ${COBOT2_ROOT}/ros2_kortex" >&2
      exit 2
      ;;
  esac
  if [[ ! -f "${COBOT_URDF_HOST}" ]]; then
    echo "[smoke_osc_cuboid] ERROR: missing URDF ${COBOT_URDF_HOST}" >&2
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
  --env OSC_MAX_SIM_S="${OSC_MAX_SIM_S}" \
  --env OSC_EXTRA_ARGS="${OSC_EXTRA_ARGS:-}" \
  --env RECORD_FPS="${RECORD_FPS}" \
  --env OSC_RECORD_VIDEO="${OSC_RECORD_VIDEO}" \
  --env SLURM_JOB_ID="${SLURM_JOB_ID:-}" \
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
  "${ISAACLAB_SIF}" bash -lc '
    cd /workspace/isaaclab || exit 1
    _ec=0
    _record_args=""
    if [[ "${OSC_RECORD_VIDEO:-0}" == "1" ]]; then
      _record_args="--enable_cameras --record-cobot-video --record-duration-s ${OSC_MAX_SIM_S} --record-fps ${RECORD_FPS} --record-video-out-dir /workspace/isaaclab/docker/logs/osc_cuboid_track_videos --record-camera-eye-m -4 0 4 --record-camera-target-m 0.6 0.4 1.1"
    fi
    if [[ "${COBOT_LOAD_MODE}" == "usd" ]]; then
      ./isaaclab.sh -p scripts/tutorials/00_sim/osc_cuboid_track.py \
        --headless \
        --usd "${COBOT_USD}" \
        --max-sim-time-s "${OSC_MAX_SIM_S}" \
        ${OSC_EXTRA_ARGS} ${_record_args} \
        || _ec=$?
    else
      ./isaaclab.sh -p scripts/tutorials/00_sim/osc_cuboid_track.py \
        --headless \
        --fix-base --force-urdf-conversion \
        --urdf "${COBOT_URDF_IN_CONTAINER}" \
        --usd-out /workspace/isaaclab/content/cobot_runtime/cobot.usd \
        --max-sim-time-s "${OSC_MAX_SIM_S}" \
        ${OSC_EXTRA_ARGS} ${_record_args} \
        || _ec=$?
    fi
    echo "[smoke_osc_cuboid] isaaclab_py_exit=${_ec}"
    exit "${_ec}"
  '
ISAAC_EC=$?

# Host-side MP4 encoding (mirrors smoke_sim_load_cobot_mpc_cuboid_video.slurm)
mkdir -p "${REPO_ROOT}/docker/logs/osc_cuboid_track_videos"
if [[ "${OSC_RECORD_VIDEO:-0}" == "1" ]] && [[ -n "${SLURM_JOB_ID:-}" ]]; then
  _vdir="${REPO_ROOT}/docker/logs/osc_cuboid_track_videos/${SLURM_JOB_ID}"
  if [[ -d "${_vdir}/frames" ]] && [[ ! -f "${_vdir}/osc_cuboid_track.mp4" ]]; then
    _ff=""
    if command -v ffmpeg >/dev/null 2>&1; then
      _ff="$(command -v ffmpeg)"
    else
      module load ffmpeg 2>/dev/null || module load FFmpeg 2>/dev/null || true
      command -v ffmpeg >/dev/null 2>&1 && _ff="$(command -v ffmpeg)"
    fi
    if [[ -n "${_ff}" ]]; then
      echo "[$(date -Is)] host ffmpeg encoding -> ${_vdir}/osc_cuboid_track.mp4" | tee /dev/stderr
      if "${_ff}" -y -hide_banner -loglevel error -framerate "${RECORD_FPS}" -i "${_vdir}/frames/frame_%06d.png" -c:v libx264 -pix_fmt yuv420p "${_vdir}/osc_cuboid_track.mp4"; then
        echo "[$(date -Is)] host ffmpeg OK" | tee /dev/stderr
      else
        echo "[$(date -Is)] host ffmpeg failed; PNGs in ${_vdir}/frames" | tee /dev/stderr
      fi
    else
      echo "[$(date -Is)] host ffmpeg not found; PNGs in ${_vdir}/frames" | tee /dev/stderr
    fi
  fi
fi

echo "[$(date -Is)] Done. Video (if recorded) under ${REPO_ROOT}/docker/logs/osc_cuboid_track_videos/${SLURM_JOB_ID:-}/"
exit "${ISAAC_EC}"
