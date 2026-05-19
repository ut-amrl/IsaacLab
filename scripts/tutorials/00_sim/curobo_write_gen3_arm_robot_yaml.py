# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Write a cuRobo ``robot_cfg`` YAML for the arm-only ros2_kortex Gen3 URDF.

cuRobo no longer ships ``curobo.examples.getting_started.build_robot_model``. This script emits a
valid config using the same kinematic layout as NVIDIA's stock ``kinova_gen3`` sphere presets,
renamed to match ``gen3_*`` link names from ``curobo_prepare_gen3_urdf.py`` (arm-only slice).
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict

import yaml

# Arm-only collision spheres: derived from cuRobo ``spheres/kinova_gen3.yml`` (NVIDIA Gen3 arm),
# with link keys rewritten to ros2_kortex ``gen3_*`` names.
_COLLISION_SPHERES: Dict[str, Any] = {
    "gen3_base_link": [
        {"center": [0.0, 0.0, 0.1], "radius": 0.05},
    ],
    "gen3_shoulder_link": [
        {"center": [0.0, 0.0, -0.1], "radius": 0.06},
        {"center": [0.0, 0.0, -0.15], "radius": 0.05},
    ],
    "gen3_half_arm_1_link": [
        {"center": [0.0, -0.0, 0.0], "radius": 0.055},
        {"center": [0.0, -0.07, 0.0], "radius": 0.055},
        {"center": [0.0, -0.15, 0.0], "radius": 0.055},
    ],
    "gen3_half_arm_2_link": [
        {"center": [0.0, -0.0, 0.0], "radius": 0.055},
        {"center": [0.0, -0.0, -0.07], "radius": 0.055},
        {"center": [0.0, -0.0, -0.15], "radius": 0.055},
        {"center": [0.0, -0.0, -0.21], "radius": 0.055},
    ],
    "gen3_forearm_link": [
        {"center": [0.0, -0.0, 0.0], "radius": 0.055},
        {"center": [0.0, -0.07, -0.0], "radius": 0.055},
        {"center": [0.0, -0.17, -0.0], "radius": 0.055},
    ],
    "gen3_spherical_wrist_1_link": [
        {"center": [0.0, -0.0, 0.0], "radius": 0.055},
        {"center": [0.0, -0.0, -0.085], "radius": 0.055},
    ],
    "gen3_spherical_wrist_2_link": [
        {"center": [0.0, -0.0, 0.0], "radius": 0.05},
        {"center": [0.0, -0.085, -0.0], "radius": 0.05},
    ],
    "gen3_bracelet_link": [
        {"center": [0.0, -0.0, -0.05], "radius": 0.04},
        {"center": [0.0, -0.05, -0.05], "radius": 0.04},
    ],
}


def _robot_cfg(urdf_abs: Path, asset_root_abs: Path) -> Dict[str, Any]:
    arm_links = list(_COLLISION_SPHERES.keys())
    return {
        "kinematics": {
            "base_link": "gen3_base_link",
            "ee_link": "gen3_end_effector_link",
            "urdf_path": str(urdf_abs.resolve()),
            "asset_root_path": str(asset_root_abs.resolve()),
            "mesh_link_names": arm_links,
            "collision_link_names": arm_links,
            "collision_sphere_buffer": 0.005,
            "collision_spheres": _COLLISION_SPHERES,
            "self_collision_buffer": {ln: 0.0 for ln in arm_links},
            "self_collision_ignore": {
                "gen3_base_link": ["gen3_shoulder_link"],
                "gen3_shoulder_link": ["gen3_half_arm_1_link"],
                "gen3_half_arm_1_link": ["gen3_half_arm_2_link"],
                "gen3_half_arm_2_link": ["gen3_forearm_link"],
                "gen3_forearm_link": ["gen3_spherical_wrist_1_link"],
                "gen3_spherical_wrist_1_link": ["gen3_spherical_wrist_2_link"],
                "gen3_spherical_wrist_2_link": ["gen3_bracelet_link"],
            },
            "cspace": {
                "joint_names": [f"gen3_joint_{i}" for i in range(1, 8)],
                "cspace_distance_weight": [1.0] * 7,
                "null_space_weight": [1.0] * 7,
                "retract_config": [0.0, -0.8, 0.0, 1.5, 0.0, 0.4, 0.0],
                "max_acceleration": 25.0,
                "max_jerk": 500.0,
            },
        }
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--urdf", type=Path, required=True, help="Absolute path to prepared Gen3 URDF.")
    p.add_argument(
        "--asset-root",
        type=Path,
        required=True,
        help="Directory that contains ``kortex_description`` (package:// root for meshes).",
    )
    p.add_argument("--output", type=Path, required=True, help="Path to write gen3_curobo_robot.yml.")
    args = p.parse_args()

    urdf = args.urdf
    asset_root = args.asset_root
    if not urdf.is_file():
        raise SystemExit(f"URDF not found: {urdf}")
    if not asset_root.is_dir():
        raise SystemExit(f"asset-root not a directory: {asset_root}")

    out = {"robot_cfg": _robot_cfg(urdf, asset_root)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(out, sort_keys=False), encoding="utf-8")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
