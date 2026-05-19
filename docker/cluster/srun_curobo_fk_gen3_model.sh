#!/usr/bin/env bash
# Interactive GPU allocation: cuRobo forward kinematics demo (Gen3 YAML) inside Apptainer.
#
# Usage:
#   cd /path/to/cobot2/IsaacLab/docker/cluster
#   bash srun_curobo_fk_gen3_model.sh
#   bash srun_curobo_fk_gen3_model.sh -- --robot-yml /workspace/isaaclab/content/cobot_runtime/gen3_curobo_robot.yml
set -euo pipefail

export USER="${USER:-$(id -un)}"

CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec srun -A bger-delta-gpu -p gpuA100x4 --gpus-per-node=1 --mem=48G -t 00:45:00 -N 1 -n 1 \
  -- bash "${CLUSTER_DIR}/curobo_fk_gen3_apptainer_worker.sh" "$@"
