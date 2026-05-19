#!/usr/bin/env bash
# Submit several MPC+cuboid smoke jobs with different diagnostic flags (MPC_EXTRA_ARGS).
# Compare logs under docker/logs/smoke-mpc-cuboid-<jobid>.out
#
# Usage (from repo login node):
#   bash docker/cluster/submit_mpc_diag_runs.sh
#
# Optional overrides before calling:
#   export COBOT_URDF_HOST=/path/to/gen3_2f85.urdf
#   export ISAACLAB_SIF=/path/to/isaac-lab_test.sif

set -euo pipefail

CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${CLUSTER_DIR}/../.." && pwd)"
# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

: "${ISAACLAB_SIF:=${REPO_ROOT}/../.sifs/isaac-lab_test.sif}"
COBOT2_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
: "${COBOT_URDF_HOST:=${COBOT2_ROOT}/ros2_kortex/kortex_description/robots/gen3_2f85.urdf}"
: "${COBOT_MPC_MAX_SIM_S:=15}"

SLURM="${CLUSTER_DIR}/smoke_sim_load_cobot_mpc_cuboid.slurm"

submit_one() {
  local job_name="$1"
  local extra="$2"
  ( cd "${CLUSTER_DIR}" && sbatch --job-name="${job_name}" \
    --export=NONE,ISAACLAB_SIF="${ISAACLAB_SIF}",COBOT_USD=/workspace/isaaclab/content/stage-19.usd,COBOT_LOAD_MODE=urdf,COBOT_URDF_HOST="${COBOT_URDF_HOST}",COBOT_MPC_MAX_SIM_S="${COBOT_MPC_MAX_SIM_S}",MPC_EXTRA_ARGS="${extra}" \
    "${SLURM}" )
}

echo "[submit_mpc_diag] ISAACLAB_SIF=${ISAACLAB_SIF}"
echo "[submit_mpc_diag] COBOT_URDF_HOST=${COBOT_URDF_HOST}"
echo "[submit_mpc_diag] COBOT_MPC_MAX_SIM_S=${COBOT_MPC_MAX_SIM_S}"

submit_one "mpc-diag-extcsv" "--diag-extended-csv --diag-debug-first-steps 25 --curobo-log-level info"
submit_one "mpc-diag-periodic" "--diag-extended-csv --diag-every-n-steps 150 --curobo-log-level info"
submit_one "mpc-diag-noselfcol" "--diag-extended-csv --diag-debug-first-steps 15 --no-mpc-self-collision --curobo-log-level info"
submit_one "mpc-diag-debug" "--diag-extended-csv --diag-debug-first-steps 40 --diag-every-n-steps 100 --curobo-log-level debug"

echo "[submit_mpc_diag] Submitted 4 jobs. Watch: ls -t ${REPO_ROOT}/docker/logs/smoke-mpc-cuboid-*.out | head"
