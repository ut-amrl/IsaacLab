#!/bin/bash
# One-shot: cuRobo RobotBuilder for Kinova Gen3 arm (see scripts/tutorials/00_sim/build_curobo_gen3_model.sh).
# Uses partial IsaacLab binds so /workspace/isaaclab/_isaac_sim from the SIF is NOT hidden.
# For interactive runs without batch queue: bash docker/cluster/srun_build_curobo_gen3_model.sh
#SBATCH -A bger-delta-gpu
#SBATCH -p gpuA100x4
#SBATCH --gpus-per-node=1
#SBATCH --mem=48G
#SBATCH -t 00:45:00
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -J build-curobo-gen3
#SBATCH --export=NONE,ISAACLAB_SIF,VISUALIZE,FULL_ROBOT
#SBATCH -o /u/ssebastian/cobot2/IsaacLab/docker/logs/build-curobo-gen3-%j.out
#SBATCH -e /u/ssebastian/cobot2/IsaacLab/docker/logs/build-curobo-gen3-%j.err

set -euo pipefail

export USER="${USER:-$(id -un)}"

CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bash "${CLUSTER_DIR}/build_curobo_gen3_apptainer_worker.sh"
