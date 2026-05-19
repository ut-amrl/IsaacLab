# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Isaac Lab + cuRobo MPC: track a point above a blue visual marker centroid while the marker oscillates A/B."""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--usd",
    type=str,
    default="/workspace/isaaclab/content/stage-19.usd",
    help="Root USD when --urdf is not set.",
)
parser.add_argument("--urdf", type=str, default="", help="Optional URDF to convert to --usd-out then load.")
parser.add_argument(
    "--usd-out",
    type=str,
    default="/workspace/isaaclab/content/cobot_runtime/cobot.usd",
    help="Output USD when using --urdf.",
)
parser.add_argument("--force-urdf-conversion", action="store_true")
parser.add_argument("--merge-joints", action="store_true", default=False)
parser.add_argument(
    "--fix-base",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="URDF import: weld robot base to world (default: True). Use --no-fix-base for a floating base.",
)
parser.add_argument("--joint-stiffness", type=float, default=400.0)
parser.add_argument("--joint-damping", type=float, default=40.0)
parser.add_argument(
    "--joint-target-type",
    type=str,
    default="position",
    choices=["position", "velocity", "none"],
)
parser.add_argument("--no-table-cuboid", action="store_true")
parser.add_argument(
    "--simple-cuboid-mpc",
    action="store_true",
    help=(
        "Phase-A-style diagnostic: spawn blue visual cuboid marker only (no Seattle table); "
        "MPC world is floor slab only (no table proxies); "
        "goal uses marker centroid with zero --above-z-m offset."
    ),
)
parser.add_argument("--cuboid-centroid-prim", type=str, default="/World/Objects/CuboidMarker")
parser.add_argument("--distance-link-regex", type=str, default="gen3_end_effector_link")
parser.add_argument(
    "--cuboid-displace-interval-s",
    type=float,
    default=10.0,
    help="Sim seconds between A/B marker goal switches (oscillation period).",
)
parser.add_argument("--cuboid-displace-range-m", type=float, default=0.3)
parser.add_argument("--log-interval-s", type=float, default=0.3)
parser.add_argument("--max-sim-time-s", type=float, default=0.0, help="Stop after this sim time (0 = run until app closes).")
parser.add_argument(
    "--robot-yml",
    type=Path,
    default=None,
    help="cuRobo robot YAML (default: <IsaacLab>/content/cobot_runtime/gen3_curobo_robot.yml).",
)
parser.add_argument(
    "--above-z-m",
    type=float,
    default=0.0,
    help="Goal point = marker centroid world position + this offset along world +Z (meters).",
)
parser.add_argument(
    "--first-goal-ee-world-dz-m",
    type=float,
    default=None,
    help=(
        "If set (e.g. -0.1), the first MPC goal uses current EE world position + (0,0,dz) m in world frame, "
        "with EE quaternion fixed as downward in base frame (wxyz); after one control step the goal reverts to marker."
    ),
)
parser.add_argument(
    "--diag-extended-csv",
    action="store_true",
    help="CSV adds dist_goal_w_m, MPC pose_err, feasible, cost, constraint, |dq_cmd|, solve_ms.",
)
parser.add_argument(
    "--diag-every-n-steps",
    type=int,
    default=0,
    help="If >0, print a [DIAG] line every N sim steps (MPC metrics + joint map + root pose).",
)
parser.add_argument(
    "--diag-debug-first-steps",
    type=int,
    default=0,
    help="If >0, print detailed [DIAG] for the first N control steps.",
)
parser.add_argument(
    "--no-mpc-self-collision",
    action="store_true",
    help="Build MpcSolver with self_collision_check=False (isolate feasibility vs self-collision).",
)
parser.add_argument(
    "--curobo-log-level",
    type=str,
    default="warning",
    choices=["debug", "info", "warn", "warning", "error"],
    help="cuRobo Python logger level (debug is very noisy).",
)
parser.add_argument(
    "--cuboid-in-ik-success-aabb",
    action="store_true",
    help=(
        "After the robot is initialized, sample a point (uniform) inside a fixed EE IK-success "
        "AABB in robot base frame, convert to world, and move the cuboid centroid there."
    ),
)
parser.add_argument(
    "--cuboid-success-place-seed",
    type=int,
    default=0,
    help="RNG seed for --cuboid-in-ik-success-aabb sampling (default 0 = reproducible).",
)
parser.add_argument(
    "--random-scene-seed",
    type=int,
    default=0,
    help="Global RNG seed for scene randomization (auto-set to SLURM_JOB_ID by video slurm script).",
)
parser.add_argument(
    "--log-mpc-every-n-steps",
    type=int,
    default=0,
    help="If >0, print [mpc_cuboid_track][MPC] feasible/constraint/|dq_cmd| every N sim steps.",
)
parser.add_argument(
    "--mpc-update-every-n-steps",
    type=int,
    default=1,
    help="Update MPC plan every N sim steps (default 1 = every step). Use >1 for receding horizon with coarser updates.",
)
parser.add_argument(
    "--mpc-collision-activation-m",
    type=float,
    default=None,
    help="If set, passed to MpcSolverConfig as collision_activation_distance (m); tune obstacle cost.",
)
parser.add_argument(
    "--record-cobot-video",
    action="store_true",
    help=(
        "Record RGB from a world-frame camera (requires --enable_cameras). Writes "
        "mpc_cuboid_track.mp4 under --record-video-out-dir/<run_id>/ (ffmpeg if available)."
    ),
)
parser.add_argument(
    "--record-duration-s",
    type=float,
    default=15.0,
    help="Sim time (seconds) to capture when --record-cobot-video is set (default 15).",
)
parser.add_argument(
    "--record-fps",
    type=float,
    default=24.0,
    help="Target capture rate (frames per sim second) for --record-cobot-video (8 = one PNG every 0.125 s).",
)
parser.add_argument(
    "--record-video-out-dir",
    type=str,
    default="/workspace/isaaclab/docker/logs/mpc_cuboid_track_videos",
    help="Directory for frames + mpc_cuboid_track.mp4 (bind IsaacLab/docker/logs on cluster).",
)
parser.add_argument(
    "--record-camera-eye-m",
    type=float,
    nargs=3,
    default=[-4.0, 0.0, 4.0],
    metavar=("X", "Y", "Z"),
    help="World-frame camera eye (m) for --record-cobot-video.",
)
parser.add_argument(
    "--record-camera-target-m",
    type=float,
    nargs=3,
    default=[0.6, 0.4, 1.1],
    metavar=("X", "Y", "Z"),
    help="World-frame look-at (m) for --record-cobot-video (default near cuboid workspace).",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

GIT_LFS_POINTER_PREFIX = b"version https://git-lfs.github.com/spec/v1"


def _v(stage: str, msg: str) -> None:
    print(f"[mpc_cuboid_track][{stage}] {msg}", flush=True)


def _is_git_lfs_pointer_file(path: str) -> bool:
    try:
        with open(path, "rb") as f:
            return f.read(len(GIT_LFS_POINTER_PREFIX)) == GIT_LFS_POINTER_PREFIX
    except OSError:
        return False


def _preflight_usd_not_lfs_stub() -> None:
    if args_cli.urdf:
        return
    usd_path = os.path.abspath(os.path.expanduser(args_cli.usd))
    if not os.path.isfile(usd_path):
        return
    if _is_git_lfs_pointer_file(usd_path):
        raise RuntimeError(
            f"USD is a Git LFS pointer stub: {usd_path}\n"
            "Run: bash scripts/pull_kinova_usd_lfs.sh  or use --urdf ... --usd-out ..."
        )


_preflight_usd_not_lfs_stub()

_v("kit", "starting AppLauncher")
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.timeline
import shutil
import subprocess
import time
import torch
from pxr import Gf, UsdGeom

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, check_file_path
from isaaclab.utils.dict import print_dict
from isaaclab.utils.math import quat_apply, quat_inv, quat_mul

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

# One-time warnings for MPC joint names not present on the Isaac articulation.
_MPC_JS_MISSING_JOINTS_WARNED: set[str] = set()

COBOT_STAGE_PRIM = "/World/CobotStage"
_TABLE_TRANSLATION = (0.55, 0.40, 1.05)
_CUBOID_MESH_DIMS = (0.1, 0.1, 0.1)
# Known-reachable EE goal positions in world frame (robot base at world origin).
# A/B (and reflected C/D) use position vectors halved vs the prior ABCD exploration pair.
def _oscillate_xy_reflect_world(p: tuple[float, float, float]) -> tuple[float, float, float]:
    return (-float(p[0]), -float(p[1]), float(p[2]))


_OSCILLATE_POS_A_WORLD = (0.26, -0.05, 0.17)
_OSCILLATE_POS_B_WORLD = (0.10, 0.21, 0.24)
_OSCILLATE_POS_C_WORLD = _oscillate_xy_reflect_world(_OSCILLATE_POS_A_WORLD)
_OSCILLATE_POS_D_WORLD = _oscillate_xy_reflect_world(_OSCILLATE_POS_B_WORLD)
_OSCILLATE_POS_WAYPOINTS: tuple[tuple[float, float, float], ...] = (
    _OSCILLATE_POS_A_WORLD,
    _OSCILLATE_POS_B_WORLD,
    _OSCILLATE_POS_C_WORLD,
    _OSCILLATE_POS_D_WORLD,
)
# 10 cm visual marker; spawn at oscillation Pos A (matches initial MPC goal).
_CUBOID_TRANSLATION = _OSCILLATE_POS_A_WORLD

# Obstacle spheres (world-frame positions) requested by user: 10 cm diameter -> 0.05 m radius
_OBST_SPHERES_WORLD: tuple[tuple[float, float, float], ...] = (
    (0.2, 0.1, 0.4),
    (-0.2, 0.1, 0.3),
    (-0.2, -0.1, 0.2),
)
_OBST_SPHERE_RADIUS_M = 0.05

# EE goal positions (robot base frame) that passed batch IK in curobo_ik_gen3_feasible_ee_sample.py:
# world=mpc_proxies, half_extent 1m, 4096 samples, self-collision on (Slurm job 18150069).
_IK_FEASIBLE_EE_SUCCESS_AABB_BASE_MIN = (-0.533193826675415, -0.6688550114631653, -0.10550636053085327)
_IK_FEASIBLE_EE_SUCCESS_AABB_BASE_MAX = (0.8736165165901184, 0.7221959233283997, 1.003830909729004)


def _ensure_camera_sensor_initialized(camera: Camera) -> None:
    timeline_iface = omni.timeline.get_timeline_interface()
    playing = timeline_iface.is_playing()
    if playing and not camera.is_initialized:
        camera._initialize_callback(None)
    if not camera.is_initialized:
        raise RuntimeError(
            "Camera sensor failed to initialize. Use --enable_cameras with --record-cobot-video."
        )


def _create_mpc_cuboid_record_camera(sim: sim_utils.SimulationContext, camera_id: str = "A") -> Camera:
    """Create a record camera with an optional ID suffix for multi-camera setups."""
    prim_base = f"/World/MpcCuboidRecordCamera_{camera_id}"
    sim_utils.create_prim(prim_base, "Xform")
    camera_cfg = CameraCfg(
        prim_path=f"{prim_base}/CameraSensor",
        update_period=0.0,
        height=720,
        width=1280,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.05, 1.0e5),
        ),
    )
    return Camera(cfg=camera_cfg)


def _save_record_rgb_png(out_path: str, rgb: torch.Tensor) -> None:
    from PIL import Image

    img = rgb.detach().cpu()
    if img.dim() == 4:
        img = img[0]
    if img.dtype != torch.uint8:
        if float(img.max()) <= 1.0 + 1e-5:
            img = (img.clamp(0.0, 1.0) * 255.0).to(torch.uint8)
        else:
            img = img.clamp(0.0, 255.0).to(torch.uint8)
    arr = img.numpy()
    if arr.shape[-1] >= 4:
        arr = arr[..., :3]
    Image.fromarray(arr).save(out_path)


def _finalize_record_mp4(frames_dir: str, fps: float, out_mp4: str) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        ep = os.environ.get("FFMPEG_PATH")
        if ep and os.path.isfile(ep):
            ffmpeg = ep
    if not ffmpeg:
        _v(
            "record",
            f"ffmpeg not in PATH; PNG sequence in {frames_dir!r}. On login node: "
            f"ffmpeg -y -framerate {fps} -i frame_%06d.png -c:v libx264 -pix_fmt yuv420p mpc_cuboid_track.mp4",
        )
        return
    cmd = [
        ffmpeg,
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-framerate",
        str(fps),
        "-i",
        os.path.join(frames_dir, "frame_%06d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        out_mp4,
    ]
    r = subprocess.run(cmd, check=False)
    if r.returncode != 0:
        _v("record", f"ffmpeg exited {r.returncode}; cmd={repr(' '.join(cmd))}")
    if os.path.isfile(out_mp4):
        _v("record", f"wrote {out_mp4!r} ({os.path.getsize(out_mp4) / 1e6:.2f} MB)")
    else:
        _v("record", f"expected mp4 missing at {out_mp4!r}")


def _set_cuboid_root_world_position_preserve_orientation(
    prim_path: str, world_xyz: tuple[float, float, float]
) -> None:
    """Set marker root translation (parent-local). Assumes parent ``/World/Objects`` is at identity."""
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid prim path for cuboid displacement: {prim_path}")
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2]))
    )


def _cuboid_centroid_world_from_prim(prim_path: str) -> tuple[float, float, float]:
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid prim for cuboid centroid: {prim_path}")
    pos, _quat = sim_utils.resolve_prim_pose(prim)
    return (float(pos[0]), float(pos[1]), float(pos[2]))


def _robot_link_world_pos_m(robot: Articulation, link_regex: str) -> tuple[float, float, float, str]:
    idxs, names = robot.find_bodies(link_regex)
    if not idxs:
        raise RuntimeError(f"No body matched regex {link_regex!r}. Example bodies: {robot.body_names[:8]}...")
    i = idxs[0]
    name = names[0]
    p = robot.data.body_link_pos_w[0, i].detach().cpu().tolist()
    return (float(p[0]), float(p[1]), float(p[2]), name)


def _spawn_scene_ground_plane() -> None:
    """Flat ground for scene contact (stage USD may not include a colliding floor)."""
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)


def _spawn_blue_cuboid_only() -> None:
    if not sim_utils.get_current_stage().GetPrimAtPath("/World/Objects").IsValid():
        sim_utils.create_prim("/World/Objects", "Xform")
    cfg_marker = sim_utils.MeshCuboidCfg(
        size=_CUBOID_MESH_DIMS,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
    )
    cfg_marker.func("/World/Objects/CuboidMarker", cfg_marker, translation=_CUBOID_TRANSLATION)


def _spawn_table_and_blue_cuboid() -> None:
    _spawn_blue_cuboid_only()
    cfg_table = sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
    )
    cfg_table.func("/World/Objects/Table", cfg_table, translation=_TABLE_TRANSLATION)


def _attach_articulation_for_mpc(*, stiffness: float, damping: float) -> Articulation:
    articulation_cfg = ArticulationCfg(
        prim_path=COBOT_STAGE_PRIM,
        spawn=None,
        actuators={
            "mpc": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness=stiffness,
                damping=damping,
            ),
        },
    )
    robot = Articulation(articulation_cfg)
    timeline_iface = omni.timeline.get_timeline_interface()
    playing = timeline_iface.is_playing()
    if playing and not robot.is_initialized:
        robot._initialize_callback(None)
    if not robot.is_initialized:
        raise RuntimeError(
            "Articulation not initialized after sim.reset(). "
            f"timeline.is_playing()={playing}. Check prim {COBOT_STAGE_PRIM}."
        )
    return robot


def _resolve_usd_path_to_load() -> str:
    if not args_cli.urdf:
        usd_path = os.path.abspath(os.path.expanduser(args_cli.usd))
        if not os.path.isfile(usd_path):
            raise FileNotFoundError(f"USD not found: {usd_path}")
        return usd_path

    urdf_path = os.path.abspath(os.path.expanduser(args_cli.urdf))
    if not check_file_path(urdf_path):
        raise ValueError(f"Invalid or missing URDF: {urdf_path}")
    dest_path = os.path.abspath(os.path.expanduser(args_cli.usd_out))
    os.makedirs(os.path.dirname(dest_path), exist_ok=True)
    urdf_converter_cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=os.path.dirname(dest_path),
        usd_file_name=os.path.basename(dest_path),
        fix_base=args_cli.fix_base,
        merge_fixed_joints=args_cli.merge_joints,
        force_usd_conversion=args_cli.force_urdf_conversion,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=args_cli.joint_stiffness,
                damping=args_cli.joint_damping,
            ),
            target_type=args_cli.joint_target_type,
        ),
    )
    print("-" * 80, flush=True)
    print(f"Input URDF: {urdf_path}", flush=True)
    print_dict(urdf_converter_cfg.to_dict(), nesting=0)
    print("-" * 80, flush=True)
    urdf_converter = UrdfConverter(urdf_converter_cfg)
    out = urdf_converter.usd_path
    print(f"[INFO]: URDF import complete; loading USD: {out}", flush=True)
    if not os.path.isfile(out):
        raise FileNotFoundError(f"Expected USD after conversion but missing: {out}")
    if _is_git_lfs_pointer_file(out):
        raise RuntimeError(f"Converted output unexpectedly looks like an LFS stub: {out}")
    return out


def _spawn_scene_lights() -> None:
    """Spawn lighting for scene visualization and rendering."""
    # Distant light for general illumination
    cfg_light_distant = sim_utils.DistantLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1.0, 0.0, 10.0))


def _spawn_scene_spheres() -> None:
    """Spawn visual obstacle spheres for MPC collision testing and scene visualization."""
    if not sim_utils.get_current_stage().GetPrimAtPath("/World/Objects").IsValid():
        sim_utils.create_prim("/World/Objects", "Xform")
    # Spawn spheres at predefined world positions
    for i, pos_w in enumerate(_OBST_SPHERES_WORLD, start=1):
        sphere_name = f"obs_sphere_{i}"
        cfg_sphere = sim_utils.SphereCfg(
            radius=_OBST_SPHERE_RADIUS_M,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.5, 0.0)),
        )
        cfg_sphere.func(
            f"/World/Objects/{sphere_name}",
            cfg_sphere,
            translation=pos_w,
        )


def _world_point_to_base(
    robot: Articulation, p_w: tuple[float, float, float], tensor_args: TensorDeviceType
) -> torch.Tensor:
    """Express a world-frame point in the articulation root (base) frame; Isaac Lab quat is wxyz."""
    root = robot.data.root_link_pose_w[0].float()
    t_wb = tensor_args.to_device(root[:3])
    q_wb = tensor_args.to_device(root[3:7])
    p = tensor_args.to_device(torch.tensor(p_w, dtype=torch.float32))
    rel = (p - t_wb).unsqueeze(0)
    q_inv = quat_inv(q_wb.unsqueeze(0))
    return quat_apply(q_inv, rel).squeeze(0)


def _world_pose_to_base_list(
    robot: Articulation, pos_w: tuple[float, float, float], tensor_args: TensorDeviceType
) -> list[float]:
    """World position + world-axis-aligned identity orientation -> robot base frame (cuRobo pose list)."""
    root = robot.data.root_link_pose_w[0].float()
    t_wb = tensor_args.to_device(root[:3])
    q_wb = tensor_args.to_device(root[3:7])
    p_w = tensor_args.to_device(torch.tensor(pos_w, dtype=torch.float32))
    q_identity = tensor_args.to_device(torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32))
    rel = (p_w - t_wb).unsqueeze(0)
    q_inv = quat_inv(q_wb.unsqueeze(0))
    p_b = quat_apply(q_inv, rel).squeeze(0)
    q_b = quat_mul(q_inv, q_identity.unsqueeze(0)).squeeze(0)
    return [float(p_b[0]), float(p_b[1]), float(p_b[2]), float(q_b[0]), float(q_b[1]), float(q_b[2]), float(q_b[3])]


def _base_point_to_world(
    robot: Articulation, p_b: tuple[float, float, float], tensor_args: TensorDeviceType
) -> tuple[float, float, float]:
    """Inverse of ``_world_point_to_base`` (Isaac root quat wxyz)."""
    root = robot.data.root_link_pose_w[0].float()
    t_wb = tensor_args.to_device(root[:3])
    q_wb = tensor_args.to_device(root[3:7])
    rel = tensor_args.to_device(torch.tensor(p_b, dtype=torch.float32)).unsqueeze(0)
    p_w = t_wb + quat_apply(q_wb.unsqueeze(0), rel).squeeze(0)
    return (float(p_w[0]), float(p_w[1]), float(p_w[2]))


def _build_mpc_world(
    robot: Articulation,
    *,
    tensor_args: TensorDeviceType,
    floor_only: bool = False,
) -> WorldConfig:
    """Primitive obstacles in robot base frame (cuRobo convention): floor; optional table proxy only."""
    floor_pose = [0.0, 0.0, -0.3, 1.0, 0.0, 0.0, 0.0]
    cuboid_entries: dict = {
        "floor": {"dims": [4.0, 4.0, 0.05], "pose": floor_pose},
    }
    if not floor_only:
        table_pose = _world_pose_to_base_list(robot, _TABLE_TRANSLATION, tensor_args)
        cuboid_entries["table_proxy"] = {"dims": [1.45, 0.78, 0.1], "pose": table_pose}
    # Build sphere entries (convert world-frame sphere centers -> robot base frame poses)
    sphere_entries: dict = {}
    for i, p_w in enumerate(_OBST_SPHERES_WORLD, start=1):
        name = f"obs_sphere_{i}"
        pose_b = _world_pose_to_base_list(robot, p_w, tensor_args)
        sphere_entries[name] = {"radius": float(_OBST_SPHERE_RADIUS_M), "pose": pose_b}

    world_dict: dict = {"cuboid": cuboid_entries}
    if sphere_entries:
        world_dict["sphere"] = sphere_entries
    return WorldConfig.from_dict(world_dict)


def _ee_to_cuboid_distance_m(robot: Articulation, prim_path: str, link_regex: str) -> float:
    cx, cy, cz = _cuboid_centroid_world_from_prim(prim_path)
    lx, ly, lz, _ = _robot_link_world_pos_m(robot, link_regex)
    return float(math.sqrt((cx - lx) ** 2 + (cy - ly) ** 2 + (cz - lz) ** 2))


def _ee_to_goal_world_distance_m(
    robot: Articulation,
    prim_path: str,
    link_regex: str,
    *,
    above_z_m: float,
) -> float:
    """Distance EE link -> world goal point (marker centroid + world +Z offset)."""
    cx, cy, cz = _cuboid_centroid_world_from_prim(prim_path)
    gx, gy, gz = cx, cy, cz + above_z_m
    lx, ly, lz, _ = _robot_link_world_pos_m(robot, link_regex)
    return float(math.sqrt((gx - lx) ** 2 + (gy - ly) ** 2 + (gz - lz) ** 2))


def _metric_float(m, name: str) -> float | None:
    if m is None or not hasattr(m, name):
        return None
    v = getattr(m, name)
    if v is None:
        return None
    return float(v.reshape(-1)[0].detach().cpu())


def _metric_bool(m, name: str) -> bool | None:
    if m is None or not hasattr(m, name):
        return None
    v = getattr(m, name)
    if v is None:
        return None
    return bool(v.reshape(-1)[0].detach().cpu().item())


def _dq_cmd_norm(
    cu_js_pre: JointState, mpc_result, joint_names: list[str], tensor_args: TensorDeviceType
) -> float:
    o_pre = cu_js_pre.get_ordered_joint_state(joint_names)
    o_cmd = mpc_result.js_action.get_ordered_joint_state(joint_names)
    d = o_cmd.position - o_pre.position
    return float(torch.norm(d).item())


def _setup_curobo_logging(level: str) -> None:
    import logging

    setup_curobo_logger(level)
    lvl_name = level.upper()
    if lvl_name in ("WARN", "WARNING"):
        py_lvl = logging.WARNING
    else:
        py_lvl = getattr(logging, lvl_name)
    logging.getLogger("curobo").setLevel(py_lvl)


def _log_joint_alignment(robot: Articulation, mpc_joint_names: list[str]) -> None:
    sim_set = set(robot.joint_names)
    mpc_set = set(mpc_joint_names)
    missing = [n for n in mpc_joint_names if n not in sim_set]
    extra = [n for n in sim_set if n not in mpc_set]
    _v("joint_map", f"mpc_joint_count={len(mpc_joint_names)} sim_joint_count={len(robot.joint_names)}")
    _v("joint_map", f"mpc_joints_missing_in_sim ({len(missing)}): {missing}")
    _v("joint_map", f"sim_joints_not_in_mpc ({len(extra)}), first 50: {extra[:50]}")


def _log_diag_step(
    *,
    tag: str,
    sim_step: int,
    sim_elapsed: float,
    robot: Articulation,
    mpc_result,
    cu_js_pre: JointState,
    joint_names: list[str],
    tensor_args: TensorDeviceType,
) -> None:
    m = mpc_result.metrics
    root = robot.data.root_link_pose_w[0].detach().cpu().tolist()
    _v(
        tag,
        f"step={sim_step} sim_t={sim_elapsed:.4f}s solve_s={mpc_result.solve_time:.4f} "
        f"feasible={_metric_bool(m, 'feasible')} pose_err={_metric_float(m, 'pose_error')} "
        f"pos_err={_metric_float(m, 'position_error')} rot_err={_metric_float(m, 'rotation_error')} "
        f"cost={_metric_float(m, 'cost')} constraint={_metric_float(m, 'constraint')} "
        f"cspace_err={_metric_float(m, 'cspace_error')} |dq_cmd|={_dq_cmd_norm(cu_js_pre, mpc_result, joint_names, tensor_args):.6f}",
    )
    _v(tag, f"root_link_pose_w[pos_xyz_quat_wxyz]={root}")
    act = mpc_result.js_action.position
    if act is not None:
        nan_a = bool(torch.isnan(act).any().item())
        inf_a = bool(torch.isinf(act).any().item())
        _v(tag, f"js_action.position nan={nan_a} inf={inf_a} min={float(act.min().cpu()):.4f} max={float(act.max().cpu()):.4f}")


def _sim_joint_state_ordered(
    robot: Articulation, mpc_joint_names: list[str], tensor_args: TensorDeviceType
) -> JointState:
    pos = robot.data.joint_pos[0:1].to(device=tensor_args.device, dtype=torch.float32)
    js = JointState.from_position(pos, joint_names=list(robot.joint_names))
    js = js.to(tensor_args)
    return js.get_ordered_joint_state(mpc_joint_names)


def _apply_mpc_js_action(robot: Articulation, mpc_result, mpc_joint_names: list[str]) -> None:
    pos_all = robot.data.joint_pos[0].clone()
    js_cmd = mpc_result.js_action.get_ordered_joint_state(mpc_joint_names)
    cmd_pos = js_cmd.position
    if cmd_pos is None:
        _v("mpc_guard", "js_action.position is None; holding last commanded position")
        return
    if torch.isnan(cmd_pos).any().item() or torch.isinf(cmd_pos).any().item():
        _v("mpc_guard", "js_action.position has NaN/Inf; holding last commanded position")
        return
    name_to_i = {n: i for i, n in enumerate(robot.joint_names)}
    for j, name in enumerate(js_cmd.joint_names):
        if name in name_to_i:
            pos_all[name_to_i[name]] = cmd_pos[0, j].to(pos_all.device)
        elif name not in _MPC_JS_MISSING_JOINTS_WARNED:
            _v("mpc_guard", f"MPC joint name not found on articulation (skipping): {name!r}")
            _MPC_JS_MISSING_JOINTS_WARNED.add(name)
    robot.set_joint_position_target(pos_all.unsqueeze(0))


def _goal_ee_quat_down_wxyz(tensor_args: TensorDeviceType, batch: int) -> torch.Tensor:
    """Fixed EE orientation: pointing straight down in robot base frame (quaternion wxyz)."""
    q = tensor_args.to_device(torch.tensor([0.0, 1.0, 0.0, 0.0], dtype=torch.float32))
    return q.unsqueeze(0).expand(int(batch), -1).contiguous()


def _goal_pose_above_cuboid(
    robot: Articulation,
    *,
    cuboid_centroid_w: tuple[float, float, float],
    above_z_m: float,
    tensor_args: TensorDeviceType,
) -> Pose:
    p_w = (cuboid_centroid_w[0], cuboid_centroid_w[1], cuboid_centroid_w[2] + above_z_m)
    p_b = _world_point_to_base(robot, p_w, tensor_args).unsqueeze(0)
    q_b = _goal_ee_quat_down_wxyz(tensor_args, p_b.shape[0])
    return Pose(position=p_b, quaternion=q_b, normalize_rotation=False)


def _goal_pose_ee_world_dz(
    robot: Articulation,
    *,
    link_regex: str,
    world_dz_m: float,
    tensor_args: TensorDeviceType,
) -> Pose:
    """World-frame EE position + (0,0,world_dz_m); fixed downward EE orientation in base frame (wxyz)."""
    lx, ly, lz, name = _robot_link_world_pos_m(robot, link_regex)
    p_w = (lx, ly, lz + float(world_dz_m))
    p_b = _world_point_to_base(robot, p_w, tensor_args).unsqueeze(0)
    q_b = _goal_ee_quat_down_wxyz(tensor_args, p_b.shape[0])
    _v(
        "goal_ee_dz",
        f"first goal from EE {name!r} world ({lx:.4f},{ly:.4f},{lz:.4f}) + dz={world_dz_m:+.4f} m -> "
        f"target world ({p_w[0]:.4f},{p_w[1]:.4f},{p_w[2]:.4f})",
    )
    return Pose(position=p_b, quaternion=q_b, normalize_rotation=False)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    isaaclab_root = script_dir.parent.parent.parent
    default_yml = isaaclab_root / "content" / "cobot_runtime" / "gen3_curobo_robot.yml"
    robot_yml = (args_cli.robot_yml or default_yml).resolve()

    if not robot_yml.is_file():
        print(f"ERROR: robot YAML not found: {robot_yml}", file=sys.stderr)
        sys.exit(2)
    if not torch.cuda.is_available():
        print("ERROR: CUDA is required for cuRobo MPC.", file=sys.stderr)
        sys.exit(2)

    tensor_args = TensorDeviceType()
    raw = load_yaml(str(robot_yml))
    if "robot_cfg" not in raw:
        print("ERROR: YAML must contain top-level 'robot_cfg'.", file=sys.stderr)
        sys.exit(2)
    robot_cfg = RobotConfig.from_dict(raw["robot_cfg"], tensor_args)
    _setup_curobo_logging(args_cli.curobo_log_level)
    _v("cfg", f"Loaded robot YAML: {robot_yml}")
    # Logged in cluster .out so smoke runs prove the bound script revision (not self_collision_opt).
    _v("script", "mpc_cuboid_track build=2026-05-10 ABCD_waypoints marker_visual mpc_order cuda_graph hold_infeas")

    record_enabled = bool(args_cli.record_cobot_video)
    if record_enabled and not getattr(args_cli, "enable_cameras", False):
        raise RuntimeError("--record-cobot-video requires --enable_cameras (Isaac Lab AppLauncher flag).")

    record_cam: Camera | None = None
    record_frames_dir = ""
    record_run_dir = ""
    record_frame_idx = 0
    next_capture_sim_t = 0.0
    record_fps_val = max(1e-3, float(args_cli.record_fps))
    record_duration_val = max(0.01, float(args_cli.record_duration_s))

    reach_feasible_n = 0
    reach_sum_pose_err = 0.0
    reach_sum_dist_goal = 0.0
    reach_mpc_steps_total = 0
    reach_robot_for_summary: Articulation | None = None
    reach_prim_for_summary = ""

    try:
        usd_path = _resolve_usd_path_to_load()
        sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
        sim = sim_utils.SimulationContext(sim_cfg)
        if record_enabled:
            sim.set_camera_view(
                list(args_cli.record_camera_eye_m),
                list(args_cli.record_camera_target_m),
            )
        else:
            sim.set_camera_view([2.0, 0.0, 2.5], [0.0, 0.0, 0.5])
        cfg = sim_utils.UsdFileCfg(usd_path=usd_path)
        cfg.func("/World/CobotStage", cfg)

        _spawn_scene_ground_plane()
        # Lighting and visual obstacles for better visualization and MPC collision testing
        _spawn_scene_lights()
        _spawn_scene_spheres()

        if args_cli.simple_cuboid_mpc:
            _spawn_blue_cuboid_only()
            _v("scene", "simple cuboid MPC: cuboid only (no Seattle table)")
        elif not args_cli.no_table_cuboid:
            _spawn_table_and_blue_cuboid()
        else:
            _v("scene", "skip table+cuboid (--no-table-cuboid)")

        sim.reset()
        robot = _attach_articulation_for_mpc(
            stiffness=args_cli.joint_stiffness,
            damping=args_cli.joint_damping,
        )
        robot.update(0.0)

        if args_cli.no_table_cuboid and not args_cli.simple_cuboid_mpc:
            raise RuntimeError("This script expects the blue marker cuboid; omit --no-table-cuboid or use --simple-cuboid-mpc.")

        prim_path = args_cli.cuboid_centroid_prim
        stage = sim_utils.get_current_stage()
        if not stage.GetPrimAtPath(prim_path).IsValid():
            raise RuntimeError(f"Missing marker cuboid prim at {prim_path!r}")

        above_z_effective = 0.0 if args_cli.simple_cuboid_mpc else float(args_cli.above_z_m)
        mpc_floor_only = bool(args_cli.simple_cuboid_mpc)
        reach_robot_for_summary = robot
        reach_prim_for_summary = prim_path

        if args_cli.cuboid_in_ik_success_aabb:
            rng = random.Random(int(args_cli.cuboid_success_place_seed))
            mn, mx = _IK_FEASIBLE_EE_SUCCESS_AABB_BASE_MIN, _IK_FEASIBLE_EE_SUCCESS_AABB_BASE_MAX
            bx = rng.uniform(mn[0], mx[0])
            by = rng.uniform(mn[1], mx[1])
            bz = rng.uniform(mn[2], mx[2])
            p_w = _base_point_to_world(robot, (bx, by, bz), tensor_args)
            _set_cuboid_root_world_position_preserve_orientation(prim_path, p_w)
            robot.update(0.0)
            _v(
                "cuboid_ik_aabb",
                f"sample_base_m=({bx:.4f},{by:.4f},{bz:.4f}) -> centroid_world_m="
                f"({p_w[0]:.4f},{p_w[1]:.4f},{p_w[2]:.4f}) seed={int(args_cli.cuboid_success_place_seed)}",
            )

        cuboid_displace_enabled = True
        oscillate_interval_s = max(float(args_cli.cuboid_displace_interval_s), float(sim_cfg.dt))
        # Marker starts at waypoint 0 (A); each scheduled tick advances A→B→C→D→A (index in Z_4).
        cuboid_waypoint_idx = 0
        next_displace_sim_t = oscillate_interval_s
        sim_elapsed = 0.0
        next_log_sim_t = 0.0

        # Marker spawned at _CUBOID_TRANSLATION == Pos A; centroid must match for first MPC goal.
        centroid_w = _cuboid_centroid_world_from_prim(prim_path)
        world_model = _build_mpc_world(robot, tensor_args=tensor_args, floor_only=mpc_floor_only)

        mpc_self = not args_cli.no_mpc_self_collision
        mpc_load_kw: dict = dict(
            tensor_args=tensor_args,
            # CUDA graphs improve per-step latency; set both False if graph capture fails during debug.
            use_cuda_graph=True,
            use_cuda_graph_metrics=True,
            self_collision_check=mpc_self,
            collision_checker_type=CollisionCheckerType.PRIMITIVE,
            collision_cache={"obb": 32},
            store_rollouts=False,
            step_dt=float(sim_cfg.dt),
        )
        if args_cli.mpc_collision_activation_m is not None:
            mpc_load_kw["collision_activation_distance"] = float(args_cli.mpc_collision_activation_m)
        mpc_config = MpcSolverConfig.load_from_robot_config(robot_cfg, world_model, **mpc_load_kw)
        mpc = MpcSolver(mpc_config)
        joint_names = mpc.rollout_fn.joint_names
        _log_joint_alignment(robot, joint_names)
        retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
        current_state = JointState.from_position(retract_cfg, joint_names=joint_names)
        state0 = mpc.rollout_fn.compute_kinematics(current_state)
        retract_pose = Pose(state0.ee_pos_seq, quaternion=state0.ee_quat_seq, normalize_rotation=False)
        goal = Goal(
            current_state=current_state,
            goal_state=JointState.from_position(retract_cfg, joint_names=joint_names),
            goal_pose=retract_pose,
        )
        goal_buffer = mpc.setup_solve_single(goal, 1)

        use_first_ee_dz = args_cli.first_goal_ee_world_dz_m is not None
        first_ee_dz_m = float(args_cli.first_goal_ee_world_dz_m) if use_first_ee_dz else 0.0
        switched_from_first_ee_goal = not use_first_ee_dz

        if use_first_ee_dz:
            gpose = _goal_pose_ee_world_dz(
                robot,
                link_regex=args_cli.distance_link_regex,
                world_dz_m=first_ee_dz_m,
                tensor_args=tensor_args,
            )
        else:
            gpose = _goal_pose_above_cuboid(
                robot,
                cuboid_centroid_w=centroid_w,
                above_z_m=above_z_effective,
                tensor_args=tensor_args,
            )
        goal_buffer.goal_pose.copy_(gpose)
        mpc.update_goal(goal_buffer)

        record_duration_val = max(float(sim_cfg.dt), float(args_cli.record_duration_s))
        record_cam = None  # will hold dict of cameras
        if record_enabled:
            run_id = os.environ.get("SLURM_JOB_ID", f"local_{int(time.time())}")
            base_out = os.path.abspath(os.path.expanduser(args_cli.record_video_out_dir))
            os.makedirs(base_out, exist_ok=True)
            record_run_dir = os.path.join(base_out, str(run_id))
            
            # Record only the B camera view for this run with a lower eye height.
            camera_configs = {
                "view_b": {"eye": [-3.0, -1.5, 1.0], "target": [0.6, 0.4, 0.0]},
            }
            
            record_cam = {}  # Dict to hold multiple cameras
            for view_name, view_cfg in camera_configs.items():
                frames_dir = os.path.join(record_run_dir, f"frames_{view_name}")
                os.makedirs(frames_dir, exist_ok=True)
                
                cam = _create_mpc_cuboid_record_camera(sim, camera_id=view_name)
                _ensure_camera_sensor_initialized(cam)
                cam.reset()
                
                dev = sim.device
                eye = torch.tensor([view_cfg["eye"]], device=dev, dtype=torch.float32)
                tgt = torch.tensor([view_cfg["target"]], device=dev, dtype=torch.float32)
                cam.set_world_poses_from_view(eye, tgt)
                
                record_cam[view_name] = {
                    "camera": cam,
                    "frames_dir": frames_dir,
                    "eye": view_cfg["eye"],
                    "target": view_cfg["target"],
                }
                
                _v(
                    "record",
                    f"{view_name} camera ready: duration_sim_s={record_duration_val} fps={record_fps_val} "
                    f"run_dir={record_run_dir!r} eye_m={tuple(view_cfg['eye'])} target_m={tuple(view_cfg['target'])}",
                )
        if args_cli.diag_extended_csv:
            print(
                "t_sim_s,dist_centroid_m,dist_goal_w_m,pose_err,feasible,cost,constraint,dq_cmd_norm,solve_ms",
                flush=True,
            )
        else:
            print("t_sim_s,dist_m", flush=True)
        _v("loop", "starting sim + MPC loop")
        _v(
            "mpc_note",
            "Batch IK feasibility of a marker centroid does not imply per-step MPC feasible=True; "
            "if motion stalls, check [MPC] lines (feasible, constraint, |dq_cmd|) or use "
            "--diag-every-n-steps / --no-mpc-self-collision.",
        )

        sim_step_k = 0
        mpc_result = None
        while simulation_app.is_running():
            stop_sim_t = float("inf")
            if args_cli.max_sim_time_s > 0.0:
                stop_sim_t = min(stop_sim_t, float(args_cli.max_sim_time_s))
            if record_enabled:
                stop_sim_t = min(stop_sim_t, float(record_duration_val))
            if stop_sim_t < float("inf") and sim_elapsed >= stop_sim_t - 1e-9:
                _v("loop", f"sim stop at sim_t={sim_elapsed:.4f}s (cap={stop_sim_t})")
                break

            if use_first_ee_dz and not switched_from_first_ee_goal and sim_step_k >= 1:
                gpose = _goal_pose_above_cuboid(
                    robot,
                    cuboid_centroid_w=centroid_w,
                    above_z_m=above_z_effective,
                    tensor_args=tensor_args,
                )
                goal_buffer.goal_pose.copy_(gpose)
                mpc.update_goal(goal_buffer)
                switched_from_first_ee_goal = True
                _v("goal", "reverted to marker goal after first-goal-ee-world-dz-m step")

            robot.update(float(sim_cfg.dt))
            cu_js_pre = _sim_joint_state_ordered(robot, joint_names, tensor_args)
            # Only update MPC plan every N steps; reuse trajectory for intermediate steps
            if sim_step_k % int(args_cli.mpc_update_every_n_steps) == 0:
                mpc_result = mpc.step(cu_js_pre, max_attempts=2)
                reach_mpc_steps_total += 1
            m_fe = mpc_result.metrics
            fe_step = _metric_bool(m_fe, "feasible")
            infeas_warn_stride = (
                int(args_cli.diag_every_n_steps) if int(args_cli.diag_every_n_steps) > 0 else 50
            )
            if fe_step is False and sim_step_k % infeas_warn_stride == 0:
                _v(
                    "mpc_guard",
                    f"MPC infeasible at step={sim_step_k} sim_t={sim_elapsed:.4f}s; holding joint targets",
                )
            if fe_step is True:
                reach_feasible_n += 1
                pe_step = _metric_float(m_fe, "pose_error")
                if pe_step is not None:
                    reach_sum_pose_err += pe_step
                reach_sum_dist_goal += _ee_to_goal_world_distance_m(
                    robot,
                    prim_path,
                    args_cli.distance_link_regex,
                    above_z_m=above_z_effective,
                )

            diag_mpc = args_cli.diag_debug_first_steps > 0 or args_cli.diag_every_n_steps > 0
            log_mpc_period = int(args_cli.log_mpc_every_n_steps) > 0 and (
                sim_step_k % int(args_cli.log_mpc_every_n_steps) == 0
            )
            if (not diag_mpc and sim_step_k < 5) or log_mpc_period:
                m = mpc_result.metrics
                fe = _metric_bool(m, "feasible")
                con = _metric_float(m, "constraint")
                pe = _metric_float(m, "pose_error")
                dq = _dq_cmd_norm(cu_js_pre, mpc_result, joint_names, tensor_args)
                _v(
                    "MPC",
                    f"step={sim_step_k} sim_t={sim_elapsed:.4f}s feasible={fe} pose_err={pe} "
                    f"constraint={con} |dq_cmd|={dq:.6f}",
                )

            if args_cli.diag_debug_first_steps > 0 and sim_step_k < args_cli.diag_debug_first_steps:
                _log_diag_step(
                    tag="DIAG_FIRST",
                    sim_step=sim_step_k,
                    sim_elapsed=sim_elapsed,
                    robot=robot,
                    mpc_result=mpc_result,
                    cu_js_pre=cu_js_pre,
                    joint_names=joint_names,
                    tensor_args=tensor_args,
                )
            if args_cli.diag_every_n_steps > 0 and sim_step_k % args_cli.diag_every_n_steps == 0:
                _log_diag_step(
                    tag="DIAG",
                    sim_step=sim_step_k,
                    sim_elapsed=sim_elapsed,
                    robot=robot,
                    mpc_result=mpc_result,
                    cu_js_pre=cu_js_pre,
                    joint_names=joint_names,
                    tensor_args=tensor_args,
                )

            if fe_step is True:
                _apply_mpc_js_action(robot, mpc_result, joint_names)
            robot.write_data_to_sim()
            sim.step()
            sim_elapsed += float(sim_cfg.dt)
            if record_cam is not None:
                dt_cap = float(sim_cfg.dt)
                for view_name, view_data in record_cam.items():
                    cam = view_data["camera"]
                    cam.update(dt_cap)
                    while sim_elapsed + 1e-9 >= next_capture_sim_t and sim_elapsed <= record_duration_val + 1e-6:
                        out_png = os.path.join(view_data["frames_dir"], f"frame_{record_frame_idx:06d}.png")
                        _save_record_rgb_png(out_png, cam.data.output["rgb"])
                        record_frame_idx += 1
                        next_capture_sim_t += 1.0 / record_fps_val

            if cuboid_displace_enabled and sim_elapsed + 1e-9 >= next_displace_sim_t:
                cuboid_waypoint_idx = (cuboid_waypoint_idx + 1) % len(_OSCILLATE_POS_WAYPOINTS)
                new_world = _OSCILLATE_POS_WAYPOINTS[cuboid_waypoint_idx]
                wname = ("A", "B", "C", "D")[cuboid_waypoint_idx]
                _v(
                    "cuboid_displace",
                    f"sim_t={sim_elapsed:.4f}s oscillating to pos_{wname} world_m={new_world}",
                )
                _set_cuboid_root_world_position_preserve_orientation(prim_path, new_world)
                centroid_w = _cuboid_centroid_world_from_prim(prim_path)
                world_model = _build_mpc_world(robot, tensor_args=tensor_args, floor_only=mpc_floor_only)
                mpc.update_world(world_model)
                gpose = _goal_pose_above_cuboid(
                    robot,
                    cuboid_centroid_w=centroid_w,
                    above_z_m=above_z_effective,
                    tensor_args=tensor_args,
                )
                goal_buffer.goal_pose.copy_(gpose)
                mpc.update_goal(goal_buffer)
                next_displace_sim_t += oscillate_interval_s

            sim_step_k += 1

            if sim_elapsed + 1e-9 >= next_log_sim_t:
                dist_c = _ee_to_cuboid_distance_m(robot, prim_path, args_cli.distance_link_regex)
                if args_cli.diag_extended_csv and mpc_result is not None:
                    dist_g = _ee_to_goal_world_distance_m(
                        robot,
                        prim_path,
                        args_cli.distance_link_regex,
                        above_z_m=above_z_effective,
                    )
                    m = mpc_result.metrics
                    fe = _metric_bool(m, "feasible")
                    fe_s = "" if fe is None else ("1" if fe else "0")
                    pe = _metric_float(m, "pose_error")
                    co = _metric_float(m, "cost")
                    con = _metric_float(m, "constraint")
                    dq = _dq_cmd_norm(cu_js_pre, mpc_result, joint_names, tensor_args)
                    sol_ms = mpc_result.solve_time * 1000.0
                    print(
                        f"{sim_elapsed:.6f},{dist_c:.6f},{dist_g:.6f},"
                        f"{'' if pe is None else f'{pe:.6f}'},{fe_s},"
                        f"{'' if co is None else f'{co:.6f}'},{'' if con is None else f'{con:.6f}'},"
                        f"{dq:.6f},{sol_ms:.3f}",
                        flush=True,
                    )
                else:
                    print(f"{sim_elapsed:.6f},{dist_c:.6f}", flush=True)
                next_log_sim_t += float(args_cli.log_interval_s)

    except Exception:
        _v("main", "EXCEPTION; traceback follows")
        traceback.print_exc()
        raise
    finally:
        if reach_robot_for_summary is not None and reach_prim_for_summary:
            try:
                n_mpc = int(reach_mpc_steps_total)
                frac = (float(reach_feasible_n) / float(n_mpc)) if n_mpc > 0 else 0.0
                if reach_feasible_n > 0:
                    mean_pe = reach_sum_pose_err / float(reach_feasible_n)
                    mean_dg = reach_sum_dist_goal / float(reach_feasible_n)
                else:
                    mean_pe = float("nan")
                    mean_dg = float("nan")
                final_dc = _ee_to_cuboid_distance_m(
                    reach_robot_for_summary,
                    reach_prim_for_summary,
                    args_cli.distance_link_regex,
                )
                _v(
                    "reach_summary",
                    f"steps={n_mpc} feasible_frac={frac:.6f} "
                    f"mean_pose_err_feasible={mean_pe:.6f} mean_dist_goal_feasible={mean_dg:.6f} "
                    f"final_dist_centroid_m={final_dc:.6f}",
                )
            except Exception as ex:
                _v("reach_summary", f"skipped ({type(ex).__name__}: {ex})")
        if record_cam is not None and record_frames_dir and record_frame_idx > 0:
            mp4_path = os.path.join(record_run_dir, "mpc_cuboid_track.mp4")
            _v("record", f"encoding {record_frame_idx} frames with ffmpeg -> {mp4_path!r}")
            _finalize_record_mp4(record_frames_dir, record_fps_val, mp4_path)


if __name__ == "__main__":
    main()
    simulation_app.close()
