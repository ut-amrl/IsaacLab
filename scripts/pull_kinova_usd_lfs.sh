#!/usr/bin/env bash
# Materialize Kinova Gen3 + 2F-85 USD (and other LFS-tracked content) for this Isaac Lab checkout.
#
# Same idea as UT-AMRL cobot `scripts/pull_kinova_usd_lfs.sh`: after clone or submodule sync, run
# from anywhere; the script cds to the Isaac Lab repo root and invokes `git lfs pull` with a
# narrow include so login nodes stay fast. Isaac Lab tracks `*.usd` / meshes via Git LFS
# (see `.gitattributes`); until payloads are fetched, files can be pointer stubs and Kit will fail.
#
# Usage:
#   bash scripts/pull_kinova_usd_lfs.sh
# Optional broader pull (large download):
#   ISAACLAB_LFS_PULL_CONTENT='1' bash scripts/pull_kinova_usd_lfs.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACLAB_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ISAACLAB_DIR}"
git rev-parse --git-dir >/dev/null 2>&1 || {
  echo "ERROR: ${ISAACLAB_DIR} is not a git checkout; cannot run git lfs pull." >&2
  exit 1
}
command -v git-lfs >/dev/null 2>&1 || {
  echo "ERROR: git-lfs is not installed (e.g. dnf install git-lfs). Run: git lfs install" >&2
  exit 1
}

git lfs pull --include="content/osu_gen3_assets/kinova_gen3_2f85.usd"
if [[ "${ISAACLAB_LFS_PULL_CONTENT:-}" == "1" ]]; then
  git lfs pull --include="content/**/*.usd"
fi
