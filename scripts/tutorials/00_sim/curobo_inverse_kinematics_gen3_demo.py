# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""cuRobo inverse kinematics demo for the Gen3 arm YAML (GPU, batch, collision world, update_world).

Headless substitute for ``python -m curobo.examples.getting_started.inverse_kinematics`` using
``IKSolver`` (see upstream ``examples/ik_example.py``). Viser / differential IK / reachability map
are not run here (interactive or heavy); use a desktop Isaac+cuRobo setup for those modes.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import torch

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.util_file import load_yaml
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

torch.backends.cudnn.benchmark = True
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


def _floor_world() -> WorldConfig:
    """Thin floor under the base (base frame); primitive collision only."""
    return WorldConfig.from_dict(
        {
            "cuboid": {
                "floor": {
                    "dims": [4.0, 4.0, 0.05],
                    "pose": [0.0, 0.0, -0.3, 1.0, 0.0, 0.0, 0.0],
                },
            },
        }
    )


def _floor_plus_near_obstacle_world() -> WorldConfig:
    """Floor plus a small cube in the workspace for ``update_world`` demo."""
    return WorldConfig.from_dict(
        {
            "cuboid": {
                "floor": {
                    "dims": [4.0, 4.0, 0.05],
                    "pose": [0.0, 0.0, -0.3, 1.0, 0.0, 0.0, 0.0],
                },
                "near_box": {
                    "dims": [0.15, 0.15, 0.15],
                    "pose": [0.55, 0.0, 0.45, 1.0, 0.0, 0.0, 0.0],
                },
            },
        }
    )


def _make_ik_solver(
    robot_cfg: RobotConfig,
    tensor_args: TensorDeviceType,
    world_model: Optional[WorldConfig],
    *,
    num_seeds: int,
    self_collision_check: bool,
    self_collision_opt: bool,
) -> IKSolver:
    ik_config = IKSolverConfig.load_from_robot_config(
        robot_cfg,
        world_model,
        rotation_threshold=0.05,
        position_threshold=0.005,
        num_seeds=num_seeds,
        self_collision_check=self_collision_check,
        self_collision_opt=self_collision_opt,
        tensor_args=tensor_args,
        use_cuda_graph=False,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
    )
    return IKSolver(ik_config)


def _print_fk_verify(ik: IKSolver, goal: Pose, q_sol: torch.Tensor, label: str) -> None:
    """``q_sol`` must be ``[batch, dof]`` (active joints), not ``[batch, return_seeds, dof]``."""
    if q_sol.dim() == 3:
        q_sol = q_sol[:, 0, :].contiguous()
    if q_sol.dim() == 1:
        q_sol = q_sol.unsqueeze(0)
    fk = ik.fk(q_sol)
    pe = torch.linalg.norm(fk.ee_position - goal.position).item()
    print(f"[curobo_ik_gen3] {label} FK verify  ||p_ee - p_goal||={pe:.5f}m")


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    isaaclab_root = script_dir.parent.parent.parent
    default_yml = isaaclab_root / "content" / "cobot_runtime" / "gen3_curobo_robot.yml"

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--robot-yml",
        type=Path,
        default=default_yml,
        help="cuRobo robot YAML (default: content/cobot_runtime/gen3_curobo_robot.yml).",
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
        print("ERROR: CUDA is required (GPU node, Apptainer --nv).", file=sys.stderr)
        sys.exit(2)

    tensor_args = TensorDeviceType()
    raw = load_yaml(str(robot_yml))
    if "robot_cfg" not in raw:
        print("ERROR: YAML must contain top-level 'robot_cfg'.", file=sys.stderr)
        sys.exit(2)
    robot_cfg = RobotConfig.from_dict(raw["robot_cfg"], tensor_args)
    print(f"[curobo_ik_gen3] Loaded {robot_yml}  device={tensor_args.device}")

    # --- Single IK + FK check ---
    ik = _make_ik_solver(
        robot_cfg,
        tensor_args,
        None,
        num_seeds=48,
        self_collision_check=False,
        self_collision_opt=False,
    )
    q0 = ik.sample_configs(1)
    kin0 = ik.fk(q0)
    goal0 = Pose(kin0.ee_position, kin0.ee_quaternion)
    t0 = time.time()
    r0 = ik.solve_batch(goal0)
    torch.cuda.synchronize()
    print(
        f"[curobo_ik_gen3] single IK  success={bool(r0.success[0].item())}  "
        f"solve_time={r0.solve_time:.4f}s  pos_err={float(r0.position_error[0].item()):.5f}m"
    )
    if bool(r0.success[0].item()):
        _print_fk_verify(ik, goal0, r0.solution[0:1, 0, :], "single")

    # --- Batched IK (100 goals) ---
    n_batch = 100
    ik_b = _make_ik_solver(
        robot_cfg,
        tensor_args,
        None,
        num_seeds=32,
        self_collision_check=False,
        self_collision_opt=False,
    )
    qb = ik_b.sample_configs(n_batch)
    kinb = ik_b.fk(qb)
    goalb = Pose(kinb.ee_position, kinb.ee_quaternion)
    t1 = time.time()
    rb = ik_b.solve_batch(goalb)
    torch.cuda.synchronize()
    dt = time.time() - t1
    n_ok = int(torch.count_nonzero(rb.success).item())
    print(
        f"[curobo_ik_gen3] batch IK  n={n_batch}  successes={n_ok}  wall={dt:.3f}s  "
        f"mean_pos_err={float(torch.mean(rb.position_error).item()):.5f}m"
    )

    # --- Collision-aware IK (world + self-collision) ---
    world0 = _floor_world()
    ik_c = _make_ik_solver(
        robot_cfg,
        tensor_args,
        world0,
        num_seeds=48,
        self_collision_check=True,
        self_collision_opt=True,
    )
    qc = ik_c.sample_configs(1)
    kinc = ik_c.fk(qc)
    goalc = Pose(kinc.ee_position, kinc.ee_quaternion)
    rc = ik_c.solve_batch(goalc)
    torch.cuda.synchronize()
    print(
        f"[curobo_ik_gen3] collision IK (floor)  success={bool(rc.success[0].item())}  "
        f"pos_err={float(rc.position_error[0].item()):.5f}m"
    )
    if bool(rc.success[0].item()):
        _print_fk_verify(ik_c, goalc, rc.solution[0:1, 0, :], "collision")

    # --- Runtime world update ---
    world1 = _floor_plus_near_obstacle_world()
    ik_c.update_world(world1)
    qc2 = ik_c.sample_configs(1)
    kin2 = ik_c.fk(qc2)
    goal2 = Pose(kin2.ee_position, kin2.ee_quaternion)
    r2 = ik_c.solve_batch(goal2)
    torch.cuda.synchronize()
    print(
        f"[curobo_ik_gen3] after update_world  success={bool(r2.success[0].item())}  "
        f"pos_err={float(r2.position_error[0].item()):.5f}m"
    )

    print("[curobo_ik_gen3] done. (Viser / --differential / --reachability not run in this script.)")


if __name__ == "__main__":
    main()
