#!/bin/bash
# Run from a GPU srun/salloc on Delta: verifies headless Isaac Lab tutorials reach "[INFO]: Setup complete".
# Usage (from this directory):  bash srun_verify_headless_smoke.sh empty
#                                 bash srun_verify_headless_smoke.sh table
set -euo pipefail

# srun without --pty sets TERM=dumb; Kit/isaaclab.sh misbehaves. Do not preserve dumb/empty.
if [[ -z "${TERM:-}" || "${TERM}" == "dumb" ]]; then
  export TERM=xterm
fi
export LC_ALL="${LC_ALL:-C.UTF-8}"

MODE="${1:-empty}"
if [[ "$MODE" != "empty" && "$MODE" != "table" ]]; then
  echo "usage: $0 empty|table" >&2
  exit 2
fi

CLUSTER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$CLUSTER_DIR"
# shellcheck source=smoke_delta_holosoma_env.sh
source "${CLUSTER_DIR}/smoke_delta_holosoma_env.sh"

REPO_ROOT="$(cd "${CLUSTER_DIR}/../.." && pwd)"
COBOT2_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
ISAACLAB_SIF="${ISAACLAB_SIF:-${REPO_ROOT}/../.sifs/isaac-lab_test.sif}"
if [[ "$MODE" == "empty" ]]; then
  PY="scripts/tutorials/00_sim/create_empty.py"
  NEEDLE="[INFO]: Setup complete..."
else
  PY="scripts/tutorials/00_sim/spawn_table_blue_cuboid.py"
  NEEDLE="[INFO]: Setup complete (table + blue cuboid)..."
fi

LOG="${SCRATCH}/isaac_cache/logs/srun-verify-${MODE}-$$.log"
mkdir -p "$(dirname "$LOG")"
echo "[$(date -Is)] MODE=${MODE} REPO_ROOT=${REPO_ROOT} SIF=${ISAACLAB_SIF} log=${LOG}"

test -f "${ISAACLAB_SIF}"

# Host repo may override image apps/ (experience files); bind when present.
APPS_BIND=()
if [[ -d "${REPO_ROOT}/apps" ]]; then
  APPS_BIND=( -B "${REPO_ROOT}/apps:/workspace/isaaclab/apps" )
fi

# First Kit + sim load can take many minutes on cold cache; table scene pulls Nucleus USD.
TIME_SEC="${VERIFY_TIMEOUT_SEC:-1800}"
mkdir -p "${COBOT2_ROOT}/external/curobo_isaac_build"
unset APPTAINERENV_PYTHONNOUSERSITE 2>/dev/null || true
export HOME="${HOME:-/u/${USER:-}}"

set +e
timeout "${TIME_SEC}"s apptainer exec --nv --cleanenv \
  --env ISAACSIM_PATH=/workspace/isaaclab/_isaac_sim \
  --env OMNI_KIT_ALLOW_ROOT=1 \
  --env TERM=xterm \
  -B "${COBOT2_ROOT}/external/curobo_isaac_build:/mnt/curobo_build" \
  -B "${HOME}/.local:${HOME}/.local" \
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
  "${APPS_BIND[@]}" \
  "${ISAACLAB_SIF}" bash -lc "cd /workspace/isaaclab && ./isaaclab.sh -p ${PY} --headless" 2>&1 | tee "$LOG"
ec="${PIPESTATUS[0]}"
set -e

echo "[$(date -Is)] timeout/apptainer exit code: ${ec} (124 = timed out after ${TIME_SEC}s, expected for infinite sim loop)"

if grep -Fq "$NEEDLE" "$LOG"; then
  echo "[$(date -Is)] OK: found success line: ${NEEDLE}"
  exit 0
fi

# timeout(1) uses 124; treat as soft-fail only if we never saw setup (e.g. still loading).
if [[ "${ec}" == "124" ]]; then
  echo "[$(date -Is)] WARN: hit ${TIME_SEC}s cap before success line; check log for slow Nucleus/network." >&2
fi

echo "[$(date -Is)] FAIL: did not find: ${NEEDLE}" >&2
tail -80 "$LOG" >&2 || true
exit 1
