# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Emit ``gen3_curobo_robot_2f85.yml`` for cuRobo: Gen3 arm + **gripper proxy** collision.

Why not list every Isaac gripper DOF in ``cspace``?
  Isaac's articulation often exposes **mimic follower** joints as separate DOFs, while cuRobo MPPI
  typically optimizes **one** gripper driver plus arm (see ``joint_map``: ``sim_joints_not_in_mpc``).
  Driving mismatched DOF counts causes poor tracking.

This config therefore uses:
  - **7-DOF** ``cspace`` (arm only), matching the original arm YAML control dimensionality.
  - **No** ``lock_joints`` on the gripper: ``ee_link`` is ``gen3_end_effector_link``, which is **upstream**
    of the Robotiq chain in the URDF, so cuRobo never builds finger joints in its tree; locking a
    knuckle joint would KeyError in ``_get_joint_links``. Keep the hand open in Isaac; MPC plans the arm only.
  - **Proxy collision spheres** on ``gen3_end_effector_link`` that conservatively cover an open Robotiq
    2F-85 envelope (no per-finger spheres).

Uses ``gen3_2f85_curobo.urdf`` (limits on continuous mimic joints for cuRobo's parser).

Paths are **container** paths for Slurm smoke binds.
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path
from typing import Any, Dict, List

import yaml

_ARM_LINKS: List[str] = [
    "gen3_base_link",
    "gen3_shoulder_link",
    "gen3_half_arm_1_link",
    "gen3_half_arm_2_link",
    "gen3_forearm_link",
    "gen3_spherical_wrist_1_link",
    "gen3_spherical_wrist_2_link",
    "gen3_bracelet_link",
]

# Tool link between bracelet and Robotiq base; used only for proxy gripper envelope spheres.
_EE_PROXY_LINK = "gen3_end_effector_link"

_ARM_SPHERES: Dict[str, Any] = {
    "gen3_base_link": [{"center": [0.0, 0.0, 0.1], "radius": 0.05}],
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

# Open 2F-85: ~90 mm opening + ~12–14 cm reach from EE origin along tool; conservative overlap.
_EE_PROXY_SPHERES: Dict[str, Any] = {
    _EE_PROXY_LINK: [
        {"center": [0.0, 0.0, 0.06], "radius": 0.085},
        {"center": [0.0, 0.0, 0.12], "radius": 0.075},
    ],
}


def _self_collision_ignore() -> Dict[str, List[str]]:
    return {
        "gen3_base_link": ["gen3_shoulder_link"],
        "gen3_shoulder_link": ["gen3_half_arm_1_link"],
        "gen3_half_arm_1_link": ["gen3_half_arm_2_link"],
        "gen3_half_arm_2_link": ["gen3_forearm_link"],
        "gen3_forearm_link": ["gen3_spherical_wrist_1_link"],
        "gen3_spherical_wrist_1_link": ["gen3_spherical_wrist_2_link"],
        "gen3_spherical_wrist_2_link": ["gen3_bracelet_link"],
        "gen3_bracelet_link": [_EE_PROXY_LINK],
        _EE_PROXY_LINK: ["gen3_bracelet_link"],
    }


def build_robot_cfg() -> Dict[str, Any]:
    arm_joints = [f"gen3_joint_{i}" for i in range(1, 8)]
    n = len(arm_joints)
    mesh_links = _ARM_LINKS + [_EE_PROXY_LINK]
    collision_spheres = {**copy.deepcopy(_ARM_SPHERES), **copy.deepcopy(_EE_PROXY_SPHERES)}
    self_buf = {ln: 0.0 for ln in mesh_links}
    self_ignore = _self_collision_ignore()
    retract = [0.0, -0.8, 0.0, 1.5, 0.0, 0.4, 0.0]
    return {
        "kinematics": {
            "base_link": "gen3_base_link",
            "ee_link": "gen3_end_effector_link",
            "urdf_path": "/workspace/ros2_kortex/kortex_description/robots/gen3_2f85_curobo.urdf",
            "asset_root_path": "/workspace/ros2_kortex/kortex_description/robots",
            "mesh_link_names": mesh_links,
            "collision_link_names": list(mesh_links),
            "collision_sphere_buffer": 0.005,
            "collision_spheres": collision_spheres,
            "self_collision_buffer": self_buf,
            "self_collision_ignore": self_ignore,
            "cspace": {
                "joint_names": arm_joints,
                "cspace_distance_weight": [1.0] * n,
                "null_space_weight": [1.0] * n,
                "retract_config": retract,
                "max_acceleration": 25.0,
                "max_jerk": 500.0,
            },
        }
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Write path (default: IsaacLab/content/cobot_runtime/gen3_curobo_robot_2f85.yml).",
    )
    args = p.parse_args()
    script_dir = Path(__file__).resolve().parent
    isaaclab_root = script_dir.parent.parent.parent
    default_out = isaaclab_root / "content" / "cobot_runtime" / "gen3_curobo_robot_2f85.yml"
    out = (args.output or default_out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = {"robot_cfg": build_robot_cfg()}
    out.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
