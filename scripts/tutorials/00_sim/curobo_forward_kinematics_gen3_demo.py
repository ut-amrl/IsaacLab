# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""cuRobo forward kinematics demo for the Gen3 arm YAML (GPU + autograd).

Replaces the removed ``python -m curobo.examples.getting_started.forward_kinematics`` tutorial
for this repo's ``content/cobot_runtime/gen3_curobo_robot.yml`` (build via
``build_curobo_gen3_model.sh``).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import load_yaml


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    isaaclab_root = script_dir.parent.parent.parent
    default_yml = isaaclab_root / "content" / "cobot_runtime" / "gen3_curobo_robot.yml"

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--robot-yml",
        type=Path,
        default=default_yml,
        help="Path to cuRobo-format robot YAML (default: content/cobot_runtime/gen3_curobo_robot.yml).",
    )
    args = p.parse_args()
    robot_yml = args.robot_yml.resolve()

    if not robot_yml.is_file():
        print(
            f"ERROR: robot YAML not found: {robot_yml}\n"
            "Build it first: bash scripts/tutorials/00_sim/build_curobo_gen3_model.sh\n"
            "or: bash docker/cluster/srun_build_curobo_gen3_model.sh",
            file=sys.stderr,
        )
        sys.exit(2)

    if not torch.cuda.is_available():
        print("ERROR: CUDA is required (run on a GPU node inside Apptainer with --nv).", file=sys.stderr)
        sys.exit(2)

    tensor_args = TensorDeviceType()
    raw = load_yaml(str(robot_yml))
    if "robot_cfg" not in raw:
        print("ERROR: YAML must contain top-level 'robot_cfg'.", file=sys.stderr)
        sys.exit(2)

    robot_cfg = RobotConfig.from_dict(raw["robot_cfg"], tensor_args)
    kin_model = CudaRobotModel(robot_cfg.kinematics)
    dof = kin_model.get_dof()
    print(f"[curobo_fk_gen3] Loaded {robot_yml}  dof={dof}  device={tensor_args.device}")

    q1 = torch.zeros((1, dof), **tensor_args.as_torch_dict())
    state1 = kin_model.get_state(q1)
    print(f"[curobo_fk_gen3] single q=0  ee_position={state1.ee_position.detach().cpu().numpy()}")
    print(f"[curobo_fk_gen3] single q=0  ee_quaternion(wxyz)={state1.ee_quaternion.detach().cpu().numpy()}")

    q_batch = torch.rand((1000, dof), **tensor_args.as_torch_dict())
    state_b = kin_model.get_state(q_batch)
    pos = state_b.ee_position
    print(f"[curobo_fk_gen3] batch FK  shape={tuple(pos.shape)}  sample||p||={float(torch.linalg.norm(pos[0]).item()):.4f}m")

    q_grad = torch.rand((1, dof), **tensor_args.as_torch_dict(), requires_grad=True)
    state_g = kin_model.get_state(q_grad)
    target = state_g.ee_position.detach() + 0.05
    loss = ((state_g.ee_position - target) ** 2).sum()
    loss.backward()
    gnorm = float(torch.linalg.norm(q_grad.grad).item()) if q_grad.grad is not None else 0.0
    print(f"[curobo_fk_gen3] autograd  loss={float(loss.detach().item()):.6f}  ||dloss/dq||={gnorm:.6f}")
    print("[curobo_fk_gen3] done.")


if __name__ == "__main__":
    main()
