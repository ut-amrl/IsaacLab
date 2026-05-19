#!/usr/bin/env bash
# Interactive GPU allocation + same Apptainer build as build_curobo_gen3_model.slurm.
# Run from a login node (queues like sbatch, but streams logs to your terminal).
#
# Usage:
#   cd /path/to/cobot2/IsaacLab/docker/cluster
#   bash srun_build_curobo_gen3_model.sh
#
# Optional overrides (exported before the command):
#   export ISAACLAB_SIF=/path/to/image.sif
set -euo pipefail

export USER="${USER:-$(id -un)}"

CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec srun -A bger-delta-gpu -p gpuA100x4 --gpus-per-node=1 --mem=48G -t 00:45:00 -N 1 -n 1 \
  bash "${CLUSTER_DIR}/build_curobo_gen3_apptainer_worker.sh"
