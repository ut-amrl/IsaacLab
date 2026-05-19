# shellcheck shell=bash
# Delta + Apptainer host setup aligned with holosoma-kick
# `slurm_scripts/isaacsim_t1_ball_container_test.slurm` (NVMe scratch, Apptainer cache,
# scratch-backed Kit / Omniverse dirs). Source from smoke_*.slurm after `set -euo pipefail`.
#
# Batch jobs must use `#SBATCH --export=NONE` so Slurm does not inherit the login-node
# environment (otherwise some sites hit: mkdir /var/spool/docker: Permission denied).
#
# Override before sbatch if your allocation does not use /work/nvme/bger:
#   export SCRATCH=/scratch/bger/$USER

export PATH="/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin${PATH:+:$PATH}"

export SCRATCH="${SCRATCH:-/work/nvme/bger/${USER}}"
export PROJECT="${PROJECT:-/u/${USER}}"
mkdir -p "${SCRATCH}/apptainer_tmp" "${SCRATCH}/apptainer_cache"
export APPTAINER_TMPDIR="${SCRATCH}/apptainer_tmp"
export APPTAINER_CACHEDIR="${SCRATCH}/apptainer_cache"
export APPTAINERENV_TMPDIR="${SCRATCH}/apptainer_tmp"

export HOST_CACHE_DIR="${SCRATCH}/isaac_cache"
mkdir -p "${HOST_CACHE_DIR}/data" "${HOST_CACHE_DIR}/cache" "${HOST_CACHE_DIR}/logs" \
  "${HOST_CACHE_DIR}/ov_data" "${HOST_CACHE_DIR}/ov"

export APPTAINERENV_OMNI_KIT_ACCEPT_EULA=y
export APPTAINERENV_PYTHONNOUSERSITE=1

unset DOCKER_HOST DOCKER_CONTEXT 2>/dev/null || true
# Drop other common login-node / container client variables that can confuse GPU hooks.
unset DOCKER_CONTENT_TRUST DOCKER_CONFIG DOCKER_TMPDIR CONTAINERS_CONFIG 2>/dev/null || true
