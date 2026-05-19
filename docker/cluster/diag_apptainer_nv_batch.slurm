#!/bin/bash
# Minimal batch repro: Apptainer + GPU on Delta (holosoma-kick-style env).
#SBATCH -A bger-delta-gpu
#SBATCH -p gpuA100x4
#SBATCH --gpus-per-node=1
#SBATCH --mem=8G
#SBATCH -t 00:05:00
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -J diag-appt-nv
#SBATCH --export=NONE
#SBATCH -o /u/ssebastian/cobot2/IsaacLab/docker/logs/diag-appt-nv-%j.out
#SBATCH -e /u/ssebastian/cobot2/IsaacLab/docker/logs/diag-appt-nv-%j.err

set -euo pipefail
SIF=/u/ssebastian/cobot2/.sifs/isaac-lab_test.sif

if [[ -n "${SLURM_SUBMIT_DIR:-}" ]]; then
  CLUSTER_DIR="$(cd "${SLURM_SUBMIT_DIR}" && pwd)"
else
  CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

echo "[$(date -Is)] host=$(hostname) SCRATCH=${SCRATCH} HOST_CACHE_DIR=${HOST_CACHE_DIR}" | tee /dev/stderr
test -f "$SIF"

echo "[$(date -Is)] apptainer true (no --nv)" | tee /dev/stderr
apptainer exec --cleanenv \
  -B "${SCRATCH}:${SCRATCH}" \
  -B "${HOST_CACHE_DIR}/data:/isaac-sim/kit/data" \
  -B "${HOST_CACHE_DIR}/cache:/isaac-sim/kit/cache" \
  -B "${HOST_CACHE_DIR}/logs:/isaac-sim/kit/logs" \
  -B "${HOST_CACHE_DIR}/ov_data:/root/.nvidia-omniverse" \
  -B "${HOST_CACHE_DIR}/ov:/ov" \
  "$SIF" true
echo "[$(date -Is)] ok no-nv" | tee /dev/stderr

echo "[$(date -Is)] apptainer true (--nv --cleanenv)" | tee /dev/stderr
apptainer exec --nv --cleanenv \
  -B "${SCRATCH}:${SCRATCH}" \
  -B "${HOST_CACHE_DIR}/data:/isaac-sim/kit/data" \
  -B "${HOST_CACHE_DIR}/cache:/isaac-sim/kit/cache" \
  -B "${HOST_CACHE_DIR}/logs:/isaac-sim/kit/logs" \
  -B "${HOST_CACHE_DIR}/ov_data:/root/.nvidia-omniverse" \
  -B "${HOST_CACHE_DIR}/ov:/ov" \
  "$SIF" true
echo "[$(date -Is)] ok nv" | tee /dev/stderr

echo "[$(date -Is)] done" | tee /dev/stderr
