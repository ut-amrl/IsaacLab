# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Sample random EE positions (robot base frame) and estimate IK feasibility with cuRobo.

No Isaac Sim — only CUDA + ``gen3_curobo_robot.yml``. Uses ``IKSolver.solve_batch`` in chunks.

**What you get**

- Overall IK **success rate** for poses drawn uniformly in an axis-aligned box around the
  retract-configuration EE pose (same orientation for all samples).
- **Axis-aligned bounds** (min/max per X,Y,Z in base frame) over **successful** samples only.

This is a *Monte Carlo* outer approximation of where collision-aware IK tends to succeed for the
given world model — not a certified workspace (depends on ``--num-seeds``, thresholds, and
sampling volume).

**Cluster (example)**

    srun --gres=gpu:1 --pty bash -lc '
      cd /workspace/isaaclab && ./isaaclab.sh -p \\
        scripts/tutorials/00_sim/curobo_ik_gen3_feasible_ee_sample.py \\
        --samples 8192 --batch-size 512 --world mpc_proxies --num-seeds 48
    '
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Optional

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

# Same primitive proxies as load_cobot_mpc_cuboid_track.py when base ≈ world at scene load.
_TABLE_TRANSLATION = (0.55, 0.40, 1.05)
_CUBOID_TRANSLATION = (0.68, 0.40, 1.22)
_CUBOID_MESH_DIMS = (0.2, 0.5, 0.2)
_PROXY_INFLATE = 0.02


def _world_config(mode: str) -> Optional[WorldConfig]:
    if mode == "none":
        return None
    if mode == "floor":
        return WorldConfig.from_dict(
            {
                "cuboid": {
                    "floor": {"dims": [4.0, 4.0, 0.05], "pose": [0.0, 0.0, -0.3, 1.0, 0.0, 0.0, 0.0]},
                },
            }
        )
    if mode == "mpc_proxies":
        bx, by, bz = _CUBOID_TRANSLATION
        td = _PROXY_INFLATE
        return WorldConfig.from_dict(
            {
                "cuboid": {
                    "floor": {"dims": [4.0, 4.0, 0.05], "pose": [0.0, 0.0, -0.3, 1.0, 0.0, 0.0, 0.0]},
                    "table_proxy": {
                        "dims": [1.45, 0.78, 0.1],
                        "pose": [
                            _TABLE_TRANSLATION[0],
                            _TABLE_TRANSLATION[1],
                            _TABLE_TRANSLATION[2],
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                    },
                    "block_proxy": {
                        "dims": [
                            _CUBOID_MESH_DIMS[0] + td,
                            _CUBOID_MESH_DIMS[1] + td,
                            _CUBOID_MESH_DIMS[2] + td,
                        ],
                        "pose": [bx, by, bz, 1.0, 0.0, 0.0, 0.0],
                    },
                },
            }
        )
    raise ValueError(f"unknown --world {mode!r}")


def _make_ik(
    robot_cfg: RobotConfig,
    tensor_args: TensorDeviceType,
    world_model: Optional[WorldConfig],
    *,
    num_seeds: int,
    self_collision: bool,
) -> IKSolver:
    ik_config = IKSolverConfig.load_from_robot_config(
        robot_cfg,
        world_model,
        tensor_args=tensor_args,
        num_seeds=num_seeds,
        position_threshold=0.005,
        rotation_threshold=0.05,
        use_cuda_graph=False,
        self_collision_check=self_collision,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
        collision_cache={"obb": 32},
    )
    return IKSolver(ik_config)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    isaaclab_root = script_dir.parent.parent.parent
    default_yml = isaaclab_root / "content" / "cobot_runtime" / "gen3_curobo_robot.yml"

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--robot-yml", type=Path, default=default_yml)
    p.add_argument(
        "--world",
        type=str,
        default="none",
        choices=["none", "floor", "mpc_proxies"],
        help="Collision world: none, thin floor, or floor+table+cuboid proxies (approx. MPC scene).",
    )
    p.add_argument("--samples", type=int, default=4096, help="Total random EE positions to try.")
    p.add_argument("--batch-size", type=int, default=512, help="Chunk size for solve_batch.")
    p.add_argument("--half-extent-m", type=float, nargs=3, default=(0.35, 0.35, 0.35), metavar=("X", "Y", "Z"))
    p.add_argument("--num-seeds", type=int, default=32, help="IK seeds per pose (higher = slower, may help).")
    p.add_argument(
        "--no-self-collision",
        action="store_true",
        help="Disable self-collision checking in IK (isolate world / pose issues).",
    )
    p.add_argument("--seed", type=int, default=0, help="RNG seed for reproducible samples.")
    p.add_argument("--csv-out", type=Path, default=None, help="If set, write successful goal xyz (base frame) CSV.")
    p.add_argument("--json-out", type=Path, default=None, help="If set, write summary JSON to this path.")
    args = p.parse_args()

    robot_yml = args.robot_yml.resolve()
    if not robot_yml.is_file():
        print(f"ERROR: robot YAML not found: {robot_yml}", file=sys.stderr)
        sys.exit(2)
    if not torch.cuda.is_available():
        print("ERROR: CUDA is required (GPU node, Apptainer --nv).", file=sys.stderr)
        sys.exit(2)

    tensor_args = TensorDeviceType()
    dev = tensor_args.device
    dt = tensor_args.dtype

    raw = load_yaml(str(robot_yml))
    if "robot_cfg" not in raw:
        print("ERROR: YAML must contain top-level 'robot_cfg'.", file=sys.stderr)
        sys.exit(2)
    robot_cfg = RobotConfig.from_dict(raw["robot_cfg"], tensor_args)

    world_model = _world_config(args.world)
    self_col = not args.no_self_collision
    ik = _make_ik(robot_cfg, tensor_args, world_model, num_seeds=args.num_seeds, self_collision=self_col)

    retract = ik.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
    kin0 = ik.fk(retract)
    p0 = kin0.ee_position[0].to(dtype=dt)  # [3]
    q0 = kin0.ee_quaternion[0].to(dtype=dt)  # [4]

    hx, hy, hz = (float(args.half_extent_m[0]), float(args.half_extent_m[1]), float(args.half_extent_m[2]))
    rng = torch.Generator(device=dev)
    rng.manual_seed(int(args.seed))

    n = int(args.samples)
    bs = max(1, int(args.batch_size))
    half = torch.tensor([hx, hy, hz], device=dev, dtype=dt)

    ok_positions: list[torch.Tensor] = []
    n_ok = 0
    t_wall0 = time.perf_counter()

    for start in range(0, n, bs):
        b = min(bs, n - start)
        u = torch.rand((b, 3), device=dev, dtype=dt, generator=rng)
        pos = p0.unsqueeze(0) + (u * 2.0 - 1.0) * half.unsqueeze(0)
        quat = q0.unsqueeze(0).repeat(b, 1)
        goal = Pose(position=pos, quaternion=quat, normalize_rotation=False)
        res = ik.solve_batch(goal)
        torch.cuda.synchronize()
        ok = res.success.reshape(-1).to(torch.bool)
        n_ok += int(torch.count_nonzero(ok).item())
        if torch.any(ok):
            ok_positions.append(pos[ok].detach())

    t_wall = time.perf_counter() - t_wall0

    summary: dict[str, Any] = {
        "robot_yml": str(robot_yml),
        "world": args.world,
        "self_collision": self_col,
        "samples": n,
        "batch_size": bs,
        "half_extent_m": [hx, hy, hz],
        "num_seeds": int(args.num_seeds),
        "rng_seed": int(args.seed),
        "retract_ee_position_base_m": [float(p0[i].item()) for i in range(3)],
        "success_count": n_ok,
        "success_rate": float(n_ok) / float(max(1, n)),
        "wall_time_s": round(t_wall, 3),
    }

    if ok_positions:
        all_ok = torch.cat(ok_positions, dim=0)
        mins = all_ok.min(dim=0).values
        maxs = all_ok.max(dim=0).values
        summary["success_xyz_min_base_m"] = [float(mins[i].item()) for i in range(3)]
        summary["success_xyz_max_base_m"] = [float(maxs[i].item()) for i in range(3)]
        summary["success_xyz_mean_base_m"] = [float(all_ok[:, i].mean().item()) for i in range(3)]
    else:
        summary["success_xyz_min_base_m"] = None
        summary["success_xyz_max_base_m"] = None
        summary["success_xyz_mean_base_m"] = None

    print("[curobo_ik_feasible_ee]", json.dumps(summary, indent=2))

    if args.csv_out is not None and ok_positions:
        args.csv_out.parent.mkdir(parents=True, exist_ok=True)
        all_ok = torch.cat(ok_positions, dim=0).cpu()
        with open(args.csv_out, "w", encoding="utf-8") as f:
            f.write("x_base_m,y_base_m,z_base_m\n")
            for i in range(all_ok.shape[0]):
                f.write(f"{all_ok[i,0].item():.6f},{all_ok[i,1].item():.6f},{all_ok[i,2].item():.6f}\n")
        print(f"[curobo_ik_feasible_ee] wrote {all_ok.shape[0]} rows to {args.csv_out}")

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
        print(f"[curobo_ik_feasible_ee] wrote {args.json_out}")

    # Non-zero exit if nothing succeeded (useful in CI / smoke wrappers).
    if n_ok == 0:
        sys.exit(3)


if __name__ == "__main__":
    main()
