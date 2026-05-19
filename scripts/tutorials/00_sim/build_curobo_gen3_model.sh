#!/usr/bin/env bash
# Build a cuRobo robot YAML from the Kinova Gen3 (arm-only) URDF in ros2_kortex.
#
# Prerequisites:
#   - cuRobo importable from the same Python as Isaac Sim (see COBOT_CUROBO_REFERENCE.md
#     and docker/cluster smoke scripts for Apptainer binds).
#   - ros2_kortex clone at ${COBOT2_ROOT}/ros2_kortex with meshes under kortex_description/.
#
# Optional (full Gen3 + Robotiq URDF): not supported by the preset YAML writer yet; use arm-only.
#
# Usage (from Isaac Lab repo root, host or inside Apptainer):
#   bash scripts/tutorials/00_sim/build_curobo_gen3_model.sh
#
# Interactive GPU (no sbatch queue): from docker/cluster run
#   bash srun_build_curobo_gen3_model.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACLAB_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
COBOT2_ROOT="$(cd "${ISAACLAB_ROOT}/.." && pwd)"

# Prefer Isaac Sim Kit Python (works inside Apptainer when _isaac_sim is from the image, not hidden by a full-repo bind).
if [[ -n "${ISAACSIM_PATH:-}" && -x "${ISAACSIM_PATH}/python.sh" ]]; then
  ISAAC_PYTHON="${ISAACSIM_PATH}/python.sh"
else
  ISAAC_PYTHON="${ISAAC_PYTHON:-${ISAACLAB_ROOT}/_isaac_sim/python.sh}"
fi
if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  ISAAC_PYTHON="$(command -v python3 || true)"
fi
if [[ -z "${ISAAC_PYTHON}" ]]; then
  echo "ERROR: no Python found (set ISAAC_PYTHON or install python3)." >&2
  exit 127
fi

KORTEX_URDF_IN="${COBOT2_ROOT}/ros2_kortex/kortex_description/robots/gen3_2f85.urdf"
OUT_URDF="${ISAACLAB_ROOT}/content/cobot_runtime/gen3_curobo_input.urdf"
OUT_YAML="${ISAACLAB_ROOT}/content/cobot_runtime/gen3_curobo_robot.yml"

if [[ ! -f "${KORTEX_URDF_IN}" ]]; then
  echo "ERROR: missing URDF: ${KORTEX_URDF_IN}" >&2
  exit 2
fi

if [[ -n "${FULL_ROBOT:-}" ]]; then
  echo "ERROR: FULL_ROBOT=1 is not supported by curobo_write_gen3_arm_robot_yaml.py (ros2 vs stock link names)." >&2
  exit 2
fi

if [[ -n "${VISUALIZE:-}" ]]; then
  echo "[build_curobo_gen3_model] WARN: VISUALIZE is ignored (preset YAML; no Viser/MorphIt step)." >&2
fi

"${ISAAC_PYTHON}" "${SCRIPT_DIR}/curobo_prepare_gen3_urdf.py" \
  --input "${KORTEX_URDF_IN}" \
  --output "${OUT_URDF}"
ASSET_PATH="${COBOT2_ROOT}/ros2_kortex"

echo "[build_curobo_gen3_model] URDF=${OUT_URDF}"
echo "[build_curobo_gen3_model] ASSET_PATH=${ASSET_PATH}"
echo "[build_curobo_gen3_model] OUT_YAML=${OUT_YAML}"

"${ISAAC_PYTHON}" "${SCRIPT_DIR}/curobo_write_gen3_arm_robot_yaml.py" \
  --urdf "${OUT_URDF}" \
  --asset-root "${ASSET_PATH}" \
  --output "${OUT_YAML}"

echo "[build_curobo_gen3_model] done."
