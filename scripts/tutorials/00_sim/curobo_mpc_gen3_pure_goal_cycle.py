# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Pure cuRobo MPC on Gen3 (no Isaac Sim, no USD robot, no cuboid).

Runs ``MpcSolver`` in **software only**: integrate ``current_state <- result.action`` each step,
same pattern as upstream ``examples/mpc_example.py``. Every ``--goal-interval-s`` of **simulated**
time (``step_dt * steps``), the Cartesian goal is replaced by the FK of a randomly perturbed joint
configuration so the target is kinematically realizable.

Use this to inspect ``feasible``, ``pose_error``, constraint/cost, NaNs, and solve time without
physics or joint-command mapping confounding the picture.

Example::

    python scripts/tutorials/00_sim/curobo_mpc_gen3_pure_goal_cycle.py \\
        --robot-yml content/cobot_runtime/gen3_curobo_robot.yml \\
        --goal-interval-s 10 --cycles 3 --curobo-log-level debug

Why a real Isaac arm might not move even when this script looks healthy
-----------------------------------------------------------------------
- **Sim vs model mismatch**: URDF in Isaac differs from ``gen3_curobo_robot.yml`` / mesh used by
  MPC (link frames, mimic joints, finger DOFs excluded from MPC but driven in sim).
- **Wrong joint index map**: ``set_joint_position_target`` must align **names** and indices between
  ``js_action`` and the articulation; silent skips or wrong DOFs freeze part of the chain.
- **Command write path**: Targets must reach PhysX via ``write_data_to_sim`` before ``sim.step``;
  wrong order or missing write yields zero motion with a happy MPC.
- **Stiffness / limits**: Low PD gains or position targets clamped by limits look like “no move”.
- **Frame errors**: Goals built in world but passed as base-frame poses (or wrong root quat)
  make the optimizer chase an inconsistent target; EE may barely drift.
- **Collision model**: Extra proxies (table/block) can make MPC **infeasible** or hover at a
  constraint; check ``feasible`` and ``constraint`` here vs in the full scene.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from pathlib import Path
from typing import Any

import torch

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util.logger import setup_curobo_logger
from curobo.util_file import load_yaml
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig


def _floor_world() -> WorldConfig:
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


def _scalar(x: torch.Tensor | None) -> float | None:
    if x is None:
        return None
    return float(x.detach().float().reshape(-1)[0].item())


def _metrics_dict(m: Any) -> dict[str, float | bool | None]:
    out: dict[str, float | bool | None] = {}
    if m is None:
        return out
    for name in (
        "cost",
        "constraint",
        "feasible",
        "cspace_error",
        "position_error",
        "rotation_error",
        "pose_error",
        "null_space_error",
    ):
        if not hasattr(m, name):
            continue
        v = getattr(m, name)
        if v is None:
            out[name] = None
        elif v.dtype == torch.bool:
            out[name] = bool(v.reshape(-1)[0].item())
        else:
            out[name] = _scalar(v)
    return out


def _log_verbose(
    *,
    tag: str,
    step_i: int,
    sim_t: float,
    result: Any,
    current_state: JointState,
    goal_pose: Pose,
    mpc: MpcSolver,
    joint_names: list[str],
) -> None:
    m = result.metrics
    md = _metrics_dict(m)
    act = result.action
    pos = act.position
    nan_pos = bool(torch.isnan(pos).any().item()) if pos is not None else False
    inf_pos = bool(torch.isinf(pos).any().item()) if pos is not None else False
    dq = None
    if pos is not None and current_state.position is not None:
        dq = float(torch.norm(pos - current_state.position).item())

    kin = mpc.rollout_fn.compute_kinematics(current_state)
    ee_p = kin.ee_pos_seq
    g_p = goal_pose.position
    cart_err = None
    if ee_p is not None and g_p is not None:
        cart_err = float(torch.norm(ee_p.view(-1, 3) - g_p.view(-1, 3)).item())

    print(
        f"[{tag}] step={step_i} sim_t={sim_t:.4f}s solve_t={result.solve_time:.4f}s "
        f"metrics={md} cart_pos_err_m={cart_err} |dq|={dq} nan_pos={nan_pos} inf_pos={inf_pos}",
        flush=True,
    )
    if md.get("feasible") is False or nan_pos or inf_pos:
        print(
            f"[{tag}] ANOMALY detail: action.pos min/max= "
            f"{float(torch.nanmin(pos).item()) if pos is not None and not nan_pos else 'nan'}/"
            f"{float(torch.nanmax(pos).item()) if pos is not None and not nan_pos else 'nan'}",
            flush=True,
        )


def _sample_joint_goal(
    retract: torch.Tensor, mpc: MpcSolver, tensor_args: TensorDeviceType, scale: float
) -> Pose:
    """Random joint vector near retract; return FK EE pose (base frame)."""
    r = retract.clone()
    noise = torch.clamp(scale * torch.randn_like(r), -0.9, 0.9)
    q = r + noise
    jn = mpc.rollout_fn.joint_names
    js = JointState.from_position(q, joint_names=jn).to(tensor_args)
    kin = mpc.rollout_fn.compute_kinematics(js)
    return Pose(
        position=kin.ee_pos_seq,
        quaternion=kin.ee_quat_seq,
        normalize_rotation=False,
    )


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    isaaclab_root = script_dir.parent.parent.parent
    default_yml = isaaclab_root / "content" / "cobot_runtime" / "gen3_curobo_robot.yml"

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot-yml", type=Path, default=default_yml, help="cuRobo robot YAML (robot_cfg).")
    p.add_argument("--step-dt", type=float, default=0.01, help="MPC model dt (matches control rate).")
    p.add_argument("--goal-interval-s", type=float, default=10.0, help="Simulated seconds between new goals.")
    p.add_argument("--cycles", type=int, default=3, help="How many goal intervals to run.")
    p.add_argument("--joint-noise-scale", type=float, default=0.35, help="Scale for random joint goal sampling.")
    p.add_argument("--log-every-steps", type=int, default=100, help="Periodic verbose line every N steps.")
    p.add_argument(
        "--curobo-log-level",
        type=str,
        default="debug",
        choices=["debug", "info", "warn", "warning", "error"],
        help="Python logging level for the 'curobo' logger.",
    )
    p.add_argument("--max-attempts", type=int, default=2, help="mpc.step(max_attempts=...).")
    args = p.parse_args()
    robot_yml = args.robot_yml.resolve()

    if not robot_yml.is_file():
        print(f"ERROR: robot YAML not found: {robot_yml}", file=sys.stderr)
        sys.exit(2)
    if not torch.cuda.is_available():
        print("ERROR: CUDA required.", file=sys.stderr)
        sys.exit(2)

    setup_curobo_logger(args.curobo_log_level)
    lvl_name = args.curobo_log_level.upper()
    if lvl_name in ("WARN", "WARNING"):
        lvl = logging.WARNING
    else:
        lvl = getattr(logging, lvl_name)
    logging.getLogger("curobo").setLevel(lvl)

    tensor_args = TensorDeviceType()
    raw = load_yaml(str(robot_yml))
    if "robot_cfg" not in raw:
        print("ERROR: YAML must contain top-level 'robot_cfg'.", file=sys.stderr)
        sys.exit(2)
    robot_cfg = RobotConfig.from_dict(raw["robot_cfg"], tensor_args)
    print(f"[pure_mpc_gen3] robot_yml={robot_yml} step_dt={args.step_dt} goal_interval_s={args.goal_interval_s}", flush=True)

    world_model = _floor_world()
    mpc_config = MpcSolverConfig.load_from_robot_config(
        robot_cfg,
        world_model,
        tensor_args=tensor_args,
        use_cuda_graph=False,
        use_cuda_graph_metrics=False,
        self_collision_check=True,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
        collision_cache={"obb": 16},
        store_rollouts=False,
        step_dt=float(args.step_dt),
    )
    mpc = MpcSolver(mpc_config)
    joint_names = mpc.rollout_fn.joint_names
    retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)

    start_state = JointState.from_position(retract_cfg, joint_names=joint_names)
    state0 = mpc.rollout_fn.compute_kinematics(start_state)
    retract_pose = Pose(state0.ee_pos_seq, quaternion=state0.ee_quat_seq, normalize_rotation=False)
    goal = Goal(
        current_state=start_state,
        goal_state=JointState.from_position(retract_cfg, joint_names=joint_names),
        goal_pose=retract_pose,
    )
    goal_buffer = mpc.setup_solve_single(goal, 1)

    current_state = start_state.clone()
    steps_per_goal = max(1, int(math.floor(args.goal_interval_s / args.step_dt + 1e-9)))
    total_steps = steps_per_goal * args.cycles
    print(
        f"[pure_mpc_gen3] steps_per_goal={steps_per_goal} cycles={args.cycles} total_steps={total_steps}",
        flush=True,
    )

    for cycle in range(args.cycles):
        if cycle == 0:
            gpose = retract_pose.clone()
        else:
            gpose = _sample_joint_goal(retract_cfg, mpc, tensor_args, args.joint_noise_scale)
        goal_buffer.goal_pose.copy_(gpose)
        mpc.update_goal(goal_buffer)
        gp = gpose.position.view(-1, 3).mean(dim=0).tolist() if gpose.position is not None else None
        print(f"[pure_mpc_gen3] cycle={cycle}/{args.cycles} NEW goal ee_pos(base)_m ~ {gp}", flush=True)

        for s in range(steps_per_goal):
            step_i = cycle * steps_per_goal + s
            sim_t = step_i * args.step_dt

            result = mpc.step(current_state, max_attempts=args.max_attempts)
            anomaly = False
            if result.metrics is not None and getattr(result.metrics, "feasible", None) is not None:
                if not bool(result.metrics.feasible.reshape(-1)[0].item()):
                    anomaly = True
            if result.action.position is not None:
                if torch.isnan(result.action.position).any() or torch.isinf(result.action.position).any():
                    anomaly = True

            if anomaly or (args.log_every_steps > 0 and s % args.log_every_steps == 0):
                _log_verbose(
                    tag="VERBOSE",
                    step_i=step_i,
                    sim_t=sim_t,
                    result=result,
                    current_state=current_state,
                    goal_pose=goal_buffer.goal_pose,
                    mpc=mpc,
                    joint_names=joint_names,
                )

            current_state.copy_(result.action)

        kin_f = mpc.rollout_fn.compute_kinematics(current_state)
        pe = _scalar(getattr(result.metrics, "pose_error", None)) if result.metrics else None
        ce = None
        if kin_f.ee_pos_seq is not None and goal_buffer.goal_pose.position is not None:
            ce = float(
                torch.norm(kin_f.ee_pos_seq.view(-1, 3) - goal_buffer.goal_pose.position.view(-1, 3)).item()
            )
        print(
            f"[pure_mpc_gen3] end_cycle={cycle} pose_error={pe} cart_pos_err_to_goal_m={ce}",
            flush=True,
        )

    print("[pure_mpc_gen3] done.", flush=True)


if __name__ == "__main__":
    main()
