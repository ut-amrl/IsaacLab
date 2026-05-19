# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Prepare Kinova Gen3 URDF from ros2_kortex for cuRobo (``build_curobo_gen3_model.sh`` / YAML writer).

The shipped ``gen3_2f85.urdf`` uses absolute ``file://`` paths from another workspace.
This tool rewrites Kinova meshes to ``package://kortex_description/...`` and optionally
strips the Robotiq + wrist camera subtree so only ``kortex_description`` is required
(``--arm-only``, default).

Full gripper meshes live in the ``robotiq_description`` ROS package (not in this repo);
use ``--full-robot`` only after you clone that package and pass a combined ``--asset-path``
(see ``build_curobo_gen3_model.sh``).
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


KORTEX_PREFIX = re.compile(
    r"file:///root/c3pzero_ws/install/kortex_description/share/kortex_description/"
)
ROBOTIQ_PREFIX = re.compile(
    r"file:///root/c3pzero_ws/install/robotiq_description/share/robotiq_description/"
)


def rewrite_mesh_paths(text: str) -> str:
    text = KORTEX_PREFIX.sub("package://kortex_description/", text)
    text = ROBOTIQ_PREFIX.sub("package://robotiq_description/", text)
    return text


def strip_ros2_control(text: str) -> str:
    """Remove ``ros2_control`` blocks (not used by cuRobo; may reference stripped joints)."""
    return re.sub(r"<ros2_control\b[^>]*>.*?</ros2_control>\s*", "", text, flags=re.DOTALL)


def strip_after_end_effector(text: str) -> str:
    """Keep ``<robot>`` through ``gen3_end_effector`` fixed joint; drop gripper and cameras."""
    marker = '<link name="gen3_robotiq_85_base_link">'
    i = text.find(marker)
    if i == -1:
        raise ValueError(f"Expected marker {marker!r} not found (wrong URDF version?)")
    head = text[:i].rstrip()
    head = strip_ros2_control(head)
    if not head.rstrip().endswith("</robot>"):
        head = head + "\n</robot>\n"
    return head


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--input",
        type=Path,
        required=True,
        help="Path to gen3_2f85.urdf (or compatible Kinova+Robotiq URDF with the same file:// prefix).",
    )
    p.add_argument("--output", type=Path, required=True, help="Path to write prepared URDF.")
    p.add_argument(
        "--arm-only",
        action="store_true",
        default=True,
        help="Truncate after gen3_end_effector (no Robotiq / wrist camera). Default: true.",
    )
    p.add_argument(
        "--full-robot",
        action="store_true",
        help="Keep full URDF including Robotiq (requires robotiq_description on disk for mesh resolve).",
    )
    args = p.parse_args()
    arm_only = args.arm_only and not args.full_robot

    text = args.input.read_text(encoding="utf-8")
    text = rewrite_mesh_paths(text)
    if arm_only:
        text = strip_after_end_effector(text)
    else:
        text = strip_ros2_control(text)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="utf-8")
    print(f"Wrote {args.output} (arm_only={arm_only})")


if __name__ == "__main__":
    main()
