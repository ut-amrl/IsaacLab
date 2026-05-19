#!/bin/bash
#SBATCH -A bger-delta-gpu
#SBATCH -p gpuA100x4
#SBATCH --gpus-per-node=1
# Slurm requires a walltime; use the *batch* partition max so large pulls/SIF builds can finish.
# Do not use gpuA100x4-interactive for long pulls — its MaxTime is 01:00:00 on Delta.
#SBATCH -t 2-00:00:00
# mksquashfs during `apptainer pull` can OOM with the default few-GB limit — request plenty of RAM.
#SBATCH --mem=128G
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -J ngc-isaac-lab-pull
#SBATCH -o /u/ssebastian/cobot2/IsaacLab/docker/logs/pull-%j.out
#SBATCH -e /u/ssebastian/cobot2/IsaacLab/docker/logs/pull-%j.err

set -euo pipefail
REPO_ROOT="/u/ssebastian/cobot2/IsaacLab"
SIF_DIR="${REPO_ROOT}/../.sifs"
mkdir -p "$SIF_DIR" "${REPO_ROOT}/docker/logs"

export APPTAINER_CACHEDIR="/tmp/${USER}-${SLURM_JOB_ID}-appt-oci"
export APPTAINER_TMPDIR="/tmp/${USER}-${SLURM_JOB_ID}-appt-tmp"
mkdir -p "$APPTAINER_CACHEDIR" "$APPTAINER_TMPDIR"

PRIMARY_TAG="2.3.2"
[[ -f "${REPO_ROOT}/VERSION" ]] && PRIMARY_TAG="$(tr -d '[:space:]' <"${REPO_ROOT}/VERSION")"

pull_one() {
  local tag="$1"
  local uri="docker://nvcr.io/nvidia/isaac-lab:${tag}"
  local out="${SIF_DIR}/isaac-lab_${tag}.sif"
  echo "[$(date -Is)] ${uri} -> ${out}"
  rm -f "${out}.partial" 2>/dev/null || true
  apptainer pull "${out}.partial" "$uri"
  mv -f "${out}.partial" "$out"
  ls -lh "$out"
}

set +e
pull_one "$PRIMARY_TAG" || pull_one "2.3.1"
set -e
echo "[$(date -Is)] Done."