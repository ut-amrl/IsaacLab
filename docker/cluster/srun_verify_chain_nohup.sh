#!/bin/bash
# Detached GPU checks (survives SSH drop / IDE): logs to IsaacLab/docker/logs/srun-verify-chain.log
set -euo pipefail
LOG=/u/ssebastian/cobot2/IsaacLab/docker/logs/srun-verify-chain.log
cd /u/ssebastian/cobot2/IsaacLab/docker/cluster
export VERIFY_TIMEOUT_SEC="${VERIFY_TIMEOUT_SEC:-1200}"
{
  echo "[$(date -Is)] start empty"
  srun -A bger-delta-gpu -p gpuA100x4-interactive --gpus-per-node=1 --cpus-per-task=8 \
    -t 00:50:00 -n 1 --mem=64G ./srun_verify_headless_smoke.sh empty
  echo "[$(date -Is)] empty_exit=$?"
  echo "[$(date -Is)] start table"
  srun -A bger-delta-gpu -p gpuA100x4-interactive --gpus-per-node=1 --cpus-per-task=8 \
    -t 00:50:00 -n 1 --mem=64G ./srun_verify_headless_smoke.sh table
  echo "[$(date -Is)] table_exit=$?"
  echo "[$(date -Is)] done"
} >>"$LOG" 2>&1
