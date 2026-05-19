# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Isaac Lab + cuRobo MPC: track a blue marker through a cubby/shelf environment.

Uses NVIDIA's ``collision_cubby.yml`` world (three walls/shelves + floor) as cuRobo MPC collision
obstacles, with matching visual cuboids spawned in Isaac.  The EE goal oscillates between waypoints
A/B/C/D (A near one cubby compartment, B near the opposite side; C/D are x,y reflections).

All controller logic (MPPI MPC, ImplicitActuator PD, gravity compensation, recording) is
identical to ``load_cobot_mpc_cuboid_track.py``; only the scene geometry and obstacle
registration differ.

Example (container paths)::

    ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_mpc_cubby_track.py --headless \\
        --robot-yml /workspace/isaaclab/content/cobot_runtime/gen3_curobo_robot_arm_only.yml \\
        --urdf /workspace/ros2_kortex/kortex_description/robots/gen3_arm_only.urdf \\
        --usd-out /workspace/isaaclab/content/cobot_runtime/cobot.usd \\
        --max-sim-time-s 45
"""

from __future__ import annotations

import argparse
import math
import os
import tempfile
import sys
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", type=str, default="/workspace/isaaclab/content/stage-19.usd")
parser.add_argument("--urdf", type=str, default="")
parser.add_argument("--usd-out", type=str, default="/workspace/isaaclab/content/cobot_runtime/cobot.usd")
parser.add_argument("--force-urdf-conversion", action="store_true")
parser.add_argument("--merge-joints", action="store_true", default=False)
parser.add_argument("--fix-base", action=argparse.BooleanOptionalAction, default=True)
parser.add_argument("--joint-stiffness", type=float, default=400.0)
parser.add_argument("--joint-damping", type=float, default=40.0)
parser.add_argument("--enable-gravity-compensation", action=argparse.BooleanOptionalAction, default=True)
parser.add_argument("--joint-target-type", type=str, default="position", choices=["position", "velocity", "none"])
parser.add_argument("--cuboid-centroid-prim", type=str, default="/World/Objects/CuboidMarker")
parser.add_argument("--distance-link-regex", type=str, default="gen3_end_effector_link")
parser.add_argument("--cuboid-displace-interval-s", type=float, default=10.0)
parser.add_argument("--log-interval-s", type=float, default=0.3)
parser.add_argument("--max-sim-time-s", type=float, default=0.0)
parser.add_argument("--robot-yml", type=Path, default=None)
parser.add_argument("--above-z-m", type=float, default=0.0)
parser.add_argument("--diag-extended-csv", action="store_true")
parser.add_argument("--diag-every-n-steps", type=int, default=0)
parser.add_argument("--diag-debug-first-steps", type=int, default=0)
parser.add_argument("--no-mpc-self-collision", action="store_true")
parser.add_argument("--curobo-log-level", type=str, default="warning", choices=["debug", "info", "warn", "warning", "error"])
parser.add_argument("--log-mpc-every-n-steps", type=int, default=0)
parser.add_argument("--mpc-update-every-n-steps", type=int, default=1)
parser.add_argument("--mpc-collision-activation-m", type=float, default=None)
parser.add_argument("--record-cobot-video", action="store_true")
parser.add_argument("--record-duration-s", type=float, default=15.0)
parser.add_argument("--record-fps", type=float, default=24.0)
parser.add_argument("--record-video-out-dir", type=str, default="/workspace/isaaclab/docker/logs/mpc_cubby_track_videos")
parser.add_argument("--record-camera-eye-m", type=float, nargs=3, default=[-2.0, -2.0, 1.5])
parser.add_argument("--record-camera-target-m", type=float, nargs=3, default=[0.0, 0.0, 0.4])
parser.add_argument("--viz-mpc-usd", action="store_true")
parser.add_argument("--mpc-viz-horizon-steps", type=int, default=30)
parser.add_argument(
    "--world-yml", type=str, default="",
    help="cuRobo world YAML (default: <IsaacLab>/content/cobot_runtime/collision_cubby.yml).",
)
parser.add_argument(
    "--pos-a", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
    help="World-frame waypoint A (m). Default chosen for the cubby layout.",
)
parser.add_argument(
    "--pos-b", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
    help="World-frame waypoint B (m). Default chosen for the cubby layout.",
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

GIT_LFS_POINTER_PREFIX = b"version https://git-lfs.github.com/spec/v1"


def _safe_print(*args, **kwargs) -> None:
    try:
        print(*args, **kwargs)
    except (OSError, BrokenPipeError, BlockingIOError):
        pass


def _v(stage: str, msg: str) -> None:
    _safe_print(f"[mpc_cubby_track][{stage}] {msg}", flush=True)


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
        raise RuntimeError(f"USD is a Git LFS pointer stub: {usd_path}\nRun: bash scripts/pull_kinova_usd_lfs.sh  or use --urdf ... --usd-out ...")


_preflight_usd_not_lfs_stub()

_v("kit", "starting AppLauncher")
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.timeline
import shutil
import subprocess
import time
import torch
import yaml as pyyaml
from pxr import Gf, UsdGeom, UsdPhysics, Vt

import isaaclab.sim as sim_utils
from isaaclab.sim.schemas import ArticulationRootPropertiesCfg, modify_articulation_root_properties
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

_MPC_JS_MISSING_JOINTS_WARNED: set[str] = set()

COBOT_STAGE_PRIM = "/World/CobotStage"
_CUBOID_MESH_DIMS = (0.1, 0.1, 0.1)

# Waypoint defaults: positions inside/near the cubby compartments that are IK-reachable.
_DEFAULT_POS_A = (-0.30, 0.20, 0.50)
_DEFAULT_POS_B = (0.30, -0.30, 0.50)


def _oscillate_xy_reflect_world(p: tuple[float, float, float]) -> tuple[float, float, float]:
    return (-float(p[0]), -float(p[1]), float(p[2]))


_POS_A = tuple(args_cli.pos_a) if args_cli.pos_a else _DEFAULT_POS_A
_POS_B = tuple(args_cli.pos_b) if args_cli.pos_b else _DEFAULT_POS_B
_POS_C = _oscillate_xy_reflect_world(_POS_A)
_POS_D = _oscillate_xy_reflect_world(_POS_B)
_WAYPOINTS: tuple[tuple[float, float, float], ...] = (_POS_A, _POS_B, _POS_C, _POS_D)
_CUBOID_TRANSLATION = _POS_A

_v("scene", f"waypoints A={_POS_A} B={_POS_B} C={_POS_C} D={_POS_D}")

# ---------------------------------------------------------------------------
# Resolve world YAML path
# ---------------------------------------------------------------------------
_script_dir = Path(__file__).resolve().parent
_isaaclab_root = _script_dir.parent.parent.parent
_default_world_yml = _isaaclab_root / "content" / "cobot_runtime" / "collision_cubby.yml"
_world_yml_path = args_cli.world_yml if args_cli.world_yml else str(_default_world_yml)

# ---------------------------------------------------------------------------
# Load cubby world dict (used for both MPC WorldConfig and Isaac visual spawn)
# ---------------------------------------------------------------------------
with open(_world_yml_path, "r") as _f:
    _CUBBY_WORLD_RAW: dict = pyyaml.safe_load(_f)
_v("world", f"loaded cubby world from {_world_yml_path}: {list((_CUBBY_WORLD_RAW.get('cuboid') or {}).keys())}")

# IK feasible region (from cuboid_track script batch IK sampling)
_IK_FEASIBLE_EE_SUCCESS_AABB_BASE_MIN = (-0.533193826675415, -0.6688550114631653, -0.10550636053085327)
_IK_FEASIBLE_EE_SUCCESS_AABB_BASE_MAX = (0.8736165165901184, 0.7221959233283997, 1.003830909729004)

# ---------------------------------------------------------------------------
# Helpers carried over from load_cobot_mpc_cuboid_track.py (unchanged)
# ---------------------------------------------------------------------------

def _ensure_camera_sensor_initialized(camera: Camera) -> None:
    timeline_iface = omni.timeline.get_timeline_interface()
    playing = timeline_iface.is_playing()
    if playing and not camera.is_initialized:
        camera._initialize_callback(None)
    if not camera.is_initialized:
        raise RuntimeError("Camera sensor failed to initialize. Use --enable_cameras with --record-cobot-video.")


def _create_record_camera(sim: sim_utils.SimulationContext, camera_id: str = "A") -> Camera:
    prim_base = f"/World/MpcCubbyRecordCamera_{camera_id}"
    sim_utils.create_prim(prim_base, "Xform")
    camera_cfg = CameraCfg(
        prim_path=f"{prim_base}/CameraSensor",
        update_period=0.0,
        height=720,
        width=1280,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.05, 1.0e5),
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
        _v("record", f"ffmpeg not in PATH; PNG sequence in {frames_dir!r}.")
        return
    cmd = [ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-framerate", str(fps),
           "-i", os.path.join(frames_dir, "frame_%06d.png"),
           "-c:v", "libx264", "-pix_fmt", "yuv420p", out_mp4]
    r = subprocess.run(cmd, check=False)
    if r.returncode != 0:
        _v("record", f"ffmpeg exited {r.returncode}")
    if os.path.isfile(out_mp4):
        _v("record", f"wrote {out_mp4!r} ({os.path.getsize(out_mp4) / 1e6:.2f} MB)")


def _set_cuboid_root_world_position_preserve_orientation(
    prim_path: str, world_xyz: tuple[float, float, float]
) -> None:
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid prim path: {prim_path}")
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2]))
    )


def _ensure_mpc_viz_prims(prim_base: str, count: int, radius: float = 0.005) -> None:
    stage = sim_utils.get_current_stage()
    if not stage.GetPrimAtPath(prim_base).IsValid():
        sim_utils.create_prim(prim_base, "Xform")
    for i in range(count):
        p = f"{prim_base}/point_{i}"
        if stage.GetPrimAtPath(p).IsValid():
            continue
        cfg = sim_utils.SphereCfg(radius=radius, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 1.0)))
        cfg.func(p, cfg, translation=(0.0, 0.0, -1.0))


def _write_mpc_viz_override_yaml(horizon_steps: int) -> str:
    override_text = f"model:\n  horizon: {int(horizon_steps)}\n"
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix="_particle_mpc_override.yml", delete=False)
    try:
        tmp.write(override_text)
        tmp.flush()
        return tmp.name
    finally:
        tmp.close()


def _get_mpc_ee_world_positions(
    current_state: JointState, mpc_result, robot: Articulation,
    tensor_args: TensorDeviceType, rollout_fn,
) -> list[tuple[float, float, float]]:
    out: list[tuple[float, float, float]] = []
    if mpc_result is None or mpc_result.raw_action is None:
        return out
    root_pose = robot.data.root_link_pose_w[0].float()
    t_wb = tensor_args.to_device(root_pose[:3])
    q_wb = tensor_args.to_device(root_pose[3:7])
    try:
        raw = mpc_result.raw_action
        if raw.position is None:
            return out
        state_traj = rollout_fn.get_full_dof_from_solution(raw)
        if state_traj.position is None:
            return out
        for step_idx in range(state_traj.position.shape[0]):
            q_step = state_traj.position[step_idx : step_idx + 1]
            step_js = JointState.from_position(q_step, joint_names=rollout_fn.joint_names)
            step_kin = rollout_fn.compute_kinematics(step_js)
            ee_pos_base = step_kin.ee_pos_seq[0]
            rel = ee_pos_base.unsqueeze(0)
            ee_pos_world = t_wb + quat_apply(q_wb.unsqueeze(0), rel).squeeze(0)
            out.append(tuple(float(x) for x in ee_pos_world.detach().cpu().tolist()))
    except Exception as ex:
        _v("traj_viz", f"USD trajectory reconstruction failed: {type(ex).__name__}: {ex}")
        return []
    return out


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
        raise RuntimeError(f"No body matched regex {link_regex!r}.")
    i = idxs[0]
    p = robot.data.body_link_pos_w[0, i].detach().cpu().tolist()
    return (float(p[0]), float(p[1]), float(p[2]), names[0])


def _spawn_scene_ground_plane() -> None:
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)


def _articulation_root_prim_paths_under(stage, start_path: str) -> list[str]:
    root = stage.GetPrimAtPath(start_path)
    if not root.IsValid():
        return []
    out: list[str] = []
    stack = [root]
    while stack:
        prim = stack.pop()
        if UsdPhysics.ArticulationRootAPI(prim):
            out.append(str(prim.GetPath()))
        stack.extend(prim.GetChildren())
    return out


def _apply_physx_disable_self_collision_if_no_mpc_self_collision(stage) -> None:
    if not args_cli.no_mpc_self_collision:
        return
    paths = _articulation_root_prim_paths_under(stage, COBOT_STAGE_PRIM)
    for p in paths:
        ok = modify_articulation_root_properties(p, ArticulationRootPropertiesCfg(enabled_self_collisions=False), stage)
        _v("physx", f"PhysXArticulation: enabled_self_collisions=False at {p!r} ok={ok}")


def _spawn_blue_cuboid_marker() -> None:
    stage = sim_utils.get_current_stage()
    if not stage.GetPrimAtPath("/World/Objects").IsValid():
        sim_utils.create_prim("/World/Objects", "Xform")
    cfg_marker = sim_utils.MeshCuboidCfg(
        size=_CUBOID_MESH_DIMS,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
    )
    cfg_marker.func("/World/Objects/CuboidMarker", cfg_marker, translation=_CUBOID_TRANSLATION)


def _spawn_cubby_visuals(world_dict: dict) -> None:
    """Spawn visual-only cuboids in Isaac matching the cuRobo cubby world YAML."""
    stage = sim_utils.get_current_stage()
    if not stage.GetPrimAtPath("/World/Objects").IsValid():
        sim_utils.create_prim("/World/Objects", "Xform")

    cuboids = world_dict.get("cuboid", {})
    for name, entry in cuboids.items():
        if name == "table":
            continue
        dims = entry.get("dims", [0.1, 0.1, 0.1])
        pose = entry.get("pose", [0, 0, 0, 1, 0, 0, 0])
        translation = (float(pose[0]), float(pose[1]), float(pose[2]))
        # cuRobo pose: [x, y, z, qw, qx, qy, qz]; Isaac orientation: (w, x, y, z)
        orientation = (float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]))
        cfg = sim_utils.MeshCuboidCfg(
            size=tuple(float(d) for d in dims),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.45, 0.50)),
        )
        prim_path = f"/World/Objects/cubby_{name}"
        cfg.func(prim_path, cfg, translation=translation, orientation=orientation)
        _v("cubby_spawn", f"{prim_path} dims={dims} pos=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})")


def _set_cuboid_color(prim_path: str, rgb: tuple[float, float, float]) -> None:
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    col = Vt.Vec3fArray([Gf.Vec3f(float(rgb[0]), float(rgb[1]), float(rgb[2]))])
    try:
        img = UsdGeom.Imageable(prim)
        if img.GetDisplayColorAttr().IsValid():
            img.GetDisplayColorAttr().Set(col)
            return
    except Exception:
        pass
    try:
        for child in prim.GetAllChildren():
            try:
                cimg = UsdGeom.Imageable(child)
                if cimg.GetDisplayColorAttr().IsValid():
                    cimg.GetDisplayColorAttr().Set(col)
            except Exception:
                continue
    except Exception:
        return


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
    return out


def _world_point_to_base(
    robot: Articulation, p_w: tuple[float, float, float], tensor_args: TensorDeviceType
) -> torch.Tensor:
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


def _build_mpc_world() -> WorldConfig:
    """Load the cubby world directly from the YAML dict (already in robot base frame)."""
    return WorldConfig.from_dict(_CUBBY_WORLD_RAW)


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
        raise RuntimeError(f"Articulation not initialized. timeline.is_playing()={playing}.")
    return robot


def _spawn_scene_lights() -> None:
    cfg_light_distant = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1.0, 0.0, 10.0))


def _ee_to_cuboid_distance_m(robot: Articulation, prim_path: str, link_regex: str) -> float:
    cx, cy, cz = _cuboid_centroid_world_from_prim(prim_path)
    lx, ly, lz, _ = _robot_link_world_pos_m(robot, link_regex)
    return float(math.sqrt((cx - lx) ** 2 + (cy - ly) ** 2 + (cz - lz) ** 2))


def _ee_to_goal_world_distance_m(robot: Articulation, prim_path: str, link_regex: str, *, above_z_m: float) -> float:
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


def _dq_cmd_norm(cu_js_pre: JointState, mpc_result, joint_names: list[str], tensor_args: TensorDeviceType) -> float:
    o_pre = cu_js_pre.get_ordered_joint_state(joint_names)
    o_cmd = mpc_result.js_action.get_ordered_joint_state(joint_names)
    return float(torch.norm(o_cmd.position - o_pre.position).item())


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
    _v("joint_map", f"mpc_joint_count={len(mpc_joint_names)} sim_joint_count={len(robot.joint_names)}")
    _v("joint_map", f"mpc_joints_missing_in_sim ({len([n for n in mpc_joint_names if n not in sim_set])})")
    _v("joint_map", f"sim_joints_not_in_mpc ({len([n for n in sim_set if n not in mpc_set])})")


def _log_diag_step(*, tag, sim_step, sim_elapsed, robot, mpc_result, cu_js_pre, joint_names, tensor_args) -> None:
    if mpc_result is None:
        _v(tag, f"step={sim_step} sim_t={sim_elapsed:.4f}s mpc_result=None")
        return
    m = mpc_result.metrics
    _v(tag, f"step={sim_step} sim_t={sim_elapsed:.4f}s solve_s={mpc_result.solve_time:.4f} "
       f"feasible={_metric_bool(m, 'feasible')} pose_err={_metric_float(m, 'pose_error')} "
       f"constraint={_metric_float(m, 'constraint')} |dq_cmd|={_dq_cmd_norm(cu_js_pre, mpc_result, joint_names, tensor_args):.6f}")


def _sim_joint_state_ordered(robot: Articulation, mpc_joint_names: list[str], tensor_args: TensorDeviceType) -> JointState:
    pos = robot.data.joint_pos[0:1].to(device=tensor_args.device, dtype=torch.float32)
    js = JointState.from_position(pos, joint_names=list(robot.joint_names))
    js = js.to(tensor_args)
    return js.get_ordered_joint_state(mpc_joint_names)


def _apply_joint_state(robot: Articulation, joint_state: JointState) -> None:
    pos_all = robot.data.joint_pos[0].clone()
    cmd_pos = joint_state.position
    if cmd_pos is None or torch.isnan(cmd_pos).any().item() or torch.isinf(cmd_pos).any().item():
        return
    if cmd_pos.ndim == 1:
        cmd_values = cmd_pos
    elif cmd_pos.ndim == 2 and cmd_pos.shape[0] == 1:
        cmd_values = cmd_pos[0]
    else:
        cmd_values = cmd_pos.reshape(-1)
    name_to_i = {n: i for i, n in enumerate(robot.joint_names)}
    for j, name in enumerate(joint_state.joint_names or []):
        if name in name_to_i:
            pos_all[name_to_i[name]] = cmd_values[j].to(pos_all.device)
        elif name not in _MPC_JS_MISSING_JOINTS_WARNED:
            _v("mpc_guard", f"MPC joint not on articulation: {name!r}")
            _MPC_JS_MISSING_JOINTS_WARNED.add(name)
    robot.set_joint_position_target(pos_all.unsqueeze(0))


def _apply_mpc_js_action(robot: Articulation, mpc_result, mpc_joint_names: list[str]) -> None:
    js_cmd = mpc_result.js_action.get_ordered_joint_state(mpc_joint_names)
    _apply_joint_state(robot, js_cmd)


def _arm_joint_indices_for_mpc(robot: Articulation, mpc_joint_names: list[str]) -> list[int]:
    name_to_i = {n: i for i, n in enumerate(robot.joint_names)}
    return [name_to_i[n] for n in mpc_joint_names]


def _apply_sim_commands_after_mpc(robot, mpc_result, mpc_joint_names, *, fe_step, gravity_compensation, arm_joint_ids):
    num_envs = robot.data.joint_pos.shape[0]
    num_j = robot.data.joint_pos.shape[1]
    device = robot.device
    dtype = robot.data.joint_pos.dtype
    efforts = torch.zeros(num_envs, num_j, device=device, dtype=dtype)
    if gravity_compensation and arm_joint_ids:
        physx_view = getattr(robot, "root_physx_view", None)
        if physx_view is not None:
            try:
                g_all = physx_view.get_gravity_compensation_forces()
                idx = torch.as_tensor(arm_joint_ids, device=device, dtype=torch.long)
                efforts[:, idx] = g_all[:, idx].to(dtype=dtype)
            except Exception as ex:
                _v("grav_ff", f"get_gravity_compensation_forces failed: {ex}")
    if fe_step and mpc_result is not None:
        _apply_mpc_js_action(robot, mpc_result, mpc_joint_names)
    robot.set_joint_effort_target(efforts)


def _goal_ee_quat_down_wxyz(tensor_args: TensorDeviceType, batch: int) -> torch.Tensor:
    q = tensor_args.to_device(torch.tensor([0.0, 1.0, 0.0, 0.0], dtype=torch.float32))
    return q.unsqueeze(0).expand(int(batch), -1).contiguous()


def _goal_pose_above_cuboid(robot, *, cuboid_centroid_w, above_z_m, tensor_args, position_only=True) -> Pose:
    p_w = (cuboid_centroid_w[0], cuboid_centroid_w[1], cuboid_centroid_w[2] + above_z_m)
    p_b = _world_point_to_base(robot, p_w, tensor_args).unsqueeze(0)
    if position_only:
        q_b = tensor_args.to_device(torch.tensor([[1.0, 0.0, 0.0, 0.0]], dtype=torch.float32))
    else:
        q_b = _goal_ee_quat_down_wxyz(tensor_args, p_b.shape[0])
    return Pose(position=p_b, quaternion=q_b, normalize_rotation=False)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

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
    _v("script", "mpc_cubby_track build=2026-05-13 cubby_world ABCD_waypoints")

    record_enabled = bool(args_cli.record_cobot_video)
    if record_enabled and not getattr(args_cli, "enable_cameras", False):
        raise RuntimeError("--record-cobot-video requires --enable_cameras.")

    record_cam: dict | None = None
    record_frames_dir = ""
    record_run_dir = ""
    record_frame_idx = 0
    next_capture_sim_t = 0.0
    record_fps_val = max(1e-3, float(args_cli.record_fps))
    record_duration_val = max(0.01, float(args_cli.record_duration_s))
    cube_touched = False
    mpc_viz_override_path = ""

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
            sim.set_camera_view(list(args_cli.record_camera_eye_m), list(args_cli.record_camera_target_m))
        else:
            sim.set_camera_view([2.0, 0.0, 2.5], [0.0, 0.0, 0.5])
        cfg = sim_utils.UsdFileCfg(usd_path=usd_path)
        cfg.func("/World/CobotStage", cfg)

        _spawn_scene_ground_plane()
        _spawn_scene_lights()
        _spawn_cubby_visuals(_CUBBY_WORLD_RAW)
        _spawn_blue_cuboid_marker()
        _v("scene", "cubby + blue marker spawned")

        _apply_physx_disable_self_collision_if_no_mpc_self_collision(sim_utils.get_current_stage())

        sim.reset()
        robot = _attach_articulation_for_mpc(stiffness=args_cli.joint_stiffness, damping=args_cli.joint_damping)
        robot.update(0.0)

        prim_path = args_cli.cuboid_centroid_prim
        stage = sim_utils.get_current_stage()
        if not stage.GetPrimAtPath(prim_path).IsValid():
            raise RuntimeError(f"Missing marker cuboid prim at {prim_path!r}")

        above_z_effective = float(args_cli.above_z_m)
        reach_robot_for_summary = robot
        reach_prim_for_summary = prim_path

        cuboid_displace_enabled = True
        oscillate_interval_s = max(float(args_cli.cuboid_displace_interval_s), float(sim_cfg.dt))
        cuboid_waypoint_idx = 0
        next_displace_sim_t = oscillate_interval_s
        sim_elapsed = 0.0
        next_log_sim_t = 0.0

        centroid_w = _cuboid_centroid_world_from_prim(prim_path)
        world_model = _build_mpc_world()

        mpc_self = not args_cli.no_mpc_self_collision
        _v("mpc", f"self_collision_check={mpc_self}")
        mpc_load_kw: dict = dict(
            tensor_args=tensor_args,
            use_cuda_graph=True,
            use_cuda_graph_metrics=True,
            self_collision_check=mpc_self,
            collision_checker_type=CollisionCheckerType.PRIMITIVE,
            collision_cache={"obb": 32, "sphere": 10},
            store_rollouts=True,
            step_dt=float(sim_cfg.dt),
        )
        if args_cli.mpc_collision_activation_m is not None:
            mpc_load_kw["collision_activation_distance"] = float(args_cli.mpc_collision_activation_m)
        if args_cli.viz_mpc_usd:
            _curobo_horizon_cap = 99
            _h_req = int(args_cli.mpc_viz_horizon_steps)
            if _h_req >= 100:
                args_cli.mpc_viz_horizon_steps = _curobo_horizon_cap
            mpc_viz_override_path = _write_mpc_viz_override_yaml(int(args_cli.mpc_viz_horizon_steps))
            mpc_load_kw["override_particle_file"] = mpc_viz_override_path
            _v("mpc_cfg", f"USD trajectory viz; horizon={int(args_cli.mpc_viz_horizon_steps)} steps")
        mpc_config = MpcSolverConfig.load_from_robot_config(robot_cfg, world_model, **mpc_load_kw)

        try:
            pcost = mpc_config.rollout_fn.primitive_collision_cost
        except Exception:
            pcost = None
        if pcost is not None:
            try:
                pcost.update_weight(10000000.0)
            except Exception:
                pcost.weight = pcost.tensor_args.to_device([10000.0])
            pcost.classify = True
            if hasattr(pcost, "world_coll_checker") and pcost.world_coll_checker is not None:
                pcost.coll_check_fn = pcost.world_coll_checker.get_sphere_collision
                pcost.sweep_check_fn = pcost.world_coll_checker.get_swept_sphere_collision
            _act_m = float(args_cli.mpc_collision_activation_m) if args_cli.mpc_collision_activation_m is not None else 0.15
            try:
                pcost.activation_distance[:] = pcost.tensor_args.to_device([_act_m])
            except Exception:
                pcost.activation_distance = pcost.tensor_args.to_device([_act_m])
            _v("mpc_cfg", f"Collision cost tuned: weight=10M, classify=True, activation={_act_m}m")

        mpc = MpcSolver(mpc_config)
        mpc.update_world(world_model)
        joint_names = mpc.rollout_fn.joint_names
        arm_joint_ids = _arm_joint_indices_for_mpc(robot, joint_names)
        _log_joint_alignment(robot, joint_names)
        retract_cfg = mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0)
        current_state = JointState.from_position(retract_cfg, joint_names=joint_names)
        state0 = mpc.rollout_fn.compute_kinematics(current_state)
        retract_pose = Pose(state0.ee_pos_seq, quaternion=state0.ee_quat_seq, normalize_rotation=False)
        goal = Goal(current_state=current_state, goal_state=JointState.from_position(retract_cfg, joint_names=joint_names), goal_pose=retract_pose)
        goal_buffer = mpc.setup_solve_single(goal, 1)

        if args_cli.viz_mpc_usd:
            _ensure_mpc_viz_prims("/World/MpcViz", int(mpc.rollout_fn.horizon), radius=0.005)
            mpc_viz_count = int(mpc.rollout_fn.horizon)
        else:
            mpc_viz_count = 0

        gpose = _goal_pose_above_cuboid(robot, cuboid_centroid_w=centroid_w, above_z_m=above_z_effective, tensor_args=tensor_args)
        goal_buffer.goal_pose.copy_(gpose)
        mpc.update_goal(goal_buffer)

        record_duration_val = max(float(sim_cfg.dt), float(args_cli.record_duration_s))
        if record_enabled:
            run_id = os.environ.get("SLURM_JOB_ID", f"local_{int(time.time())}")
            base_out = os.path.abspath(os.path.expanduser(args_cli.record_video_out_dir))
            os.makedirs(base_out, exist_ok=True)
            record_run_dir = os.path.join(base_out, str(run_id))
            camera_configs = {
                "view_b": {"eye": list(args_cli.record_camera_eye_m), "target": list(args_cli.record_camera_target_m)},
            }
            record_cam = {}
            for view_name, view_cfg in camera_configs.items():
                frames_dir = os.path.join(record_run_dir, f"frames_{view_name}")
                os.makedirs(frames_dir, exist_ok=True)
                cam = _create_record_camera(sim, camera_id=view_name)
                _ensure_camera_sensor_initialized(cam)
                cam.reset()
                dev = sim.device
                eye = torch.tensor([view_cfg["eye"]], device=dev, dtype=torch.float32)
                tgt = torch.tensor([view_cfg["target"]], device=dev, dtype=torch.float32)
                cam.set_world_poses_from_view(eye, tgt)
                record_cam[view_name] = {"camera": cam, "frames_dir": frames_dir, "eye": view_cfg["eye"], "target": view_cfg["target"]}
                _v("record", f"{view_name} camera ready: duration_sim_s={record_duration_val} fps={record_fps_val} run_dir={record_run_dir!r}")

        if args_cli.diag_extended_csv:
            _safe_print("t_sim_s,dist_centroid_m,dist_goal_w_m,pose_err,feasible,cost,constraint,dq_cmd_norm,solve_ms", flush=True)
        else:
            _safe_print("t_sim_s,dist_m", flush=True)
        _v("loop", "starting sim + MPC loop")

        sim_step_k = 0
        mpc_result = None
        mpc_last_solve = None
        mpc_viz_enabled = bool(args_cli.viz_mpc_usd)
        mpc_viz_prim_base = "/World/MpcViz"
        mpc_update_every_n_steps = max(1, int(args_cli.mpc_update_every_n_steps))
        while simulation_app.is_running():
            stop_sim_t = float("inf")
            if args_cli.max_sim_time_s > 0.0:
                stop_sim_t = min(stop_sim_t, float(args_cli.max_sim_time_s))
            if record_enabled:
                stop_sim_t = min(stop_sim_t, float(record_duration_val))
            if stop_sim_t < float("inf") and sim_elapsed >= stop_sim_t - 1e-9:
                _v("loop", f"sim stop at sim_t={sim_elapsed:.4f}s")
                break

            robot.update(float(sim_cfg.dt))
            cu_js_pre = _sim_joint_state_ordered(robot, joint_names, tensor_args)
            if sim_step_k % mpc_update_every_n_steps == 0 or mpc_result is None:
                new_mpc_result = mpc.step(cu_js_pre, max_attempts=2)
                mpc_last_solve = new_mpc_result
                reach_mpc_steps_total += 1
                if _metric_bool(new_mpc_result.metrics, "feasible") is True:
                    mpc_result = new_mpc_result
                if mpc_viz_enabled:
                    ee_positions = _get_mpc_ee_world_positions(cu_js_pre, new_mpc_result, robot, tensor_args, mpc.rollout_fn)
                    for i, p in enumerate(ee_positions[:mpc_viz_count]):
                        try:
                            _set_cuboid_root_world_position_preserve_orientation(f"{mpc_viz_prim_base}/point_{i}", p)
                        except Exception:
                            pass

            m_log = mpc_result if mpc_result is not None else mpc_last_solve
            m_fe = m_log.metrics if m_log is not None else None
            fe_step = mpc_result is not None

            if fe_step:
                reach_feasible_n += 1
                pe_step = _metric_float(m_fe, "pose_error")
                if pe_step is not None:
                    reach_sum_pose_err += pe_step
                reach_sum_dist_goal += _ee_to_goal_world_distance_m(robot, prim_path, args_cli.distance_link_regex, above_z_m=above_z_effective)

            log_mpc_period = int(args_cli.log_mpc_every_n_steps) > 0 and (sim_step_k % int(args_cli.log_mpc_every_n_steps) == 0)
            if (sim_step_k < 5) or log_mpc_period:
                cmd_src = mpc_result if mpc_result is not None else mpc_last_solve
                if cmd_src is not None:
                    m = cmd_src.metrics
                    fe = _metric_bool(m, "feasible")
                    con = _metric_float(m, "constraint")
                    pe = _metric_float(m, "pose_error")
                    dq = _dq_cmd_norm(cu_js_pre, cmd_src, joint_names, tensor_args)
                    _v("MPC", f"step={sim_step_k} sim_t={sim_elapsed:.4f}s feasible={fe} pose_err={pe} constraint={con} |dq_cmd|={dq:.6f}")

            if args_cli.diag_every_n_steps > 0 and sim_step_k % args_cli.diag_every_n_steps == 0:
                _log_diag_step(tag="DIAG", sim_step=sim_step_k, sim_elapsed=sim_elapsed, robot=robot,
                               mpc_result=mpc_last_solve, cu_js_pre=cu_js_pre, joint_names=joint_names, tensor_args=tensor_args)

            _apply_sim_commands_after_mpc(robot, mpc_result, joint_names, fe_step=(fe_step is True),
                                         gravity_compensation=bool(args_cli.enable_gravity_compensation), arm_joint_ids=arm_joint_ids)
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
                cuboid_waypoint_idx = (cuboid_waypoint_idx + 1) % len(_WAYPOINTS)
                new_world = _WAYPOINTS[cuboid_waypoint_idx]
                wname = ("A", "B", "C", "D")[cuboid_waypoint_idx]
                _v("cuboid_displace", f"sim_t={sim_elapsed:.4f}s -> pos_{wname} world_m={new_world}")
                _set_cuboid_root_world_position_preserve_orientation(prim_path, new_world)
                centroid_w = _cuboid_centroid_world_from_prim(prim_path)
                gpose = _goal_pose_above_cuboid(robot, cuboid_centroid_w=centroid_w, above_z_m=above_z_effective, tensor_args=tensor_args)
                goal_buffer.goal_pose.copy_(gpose)
                mpc.update_goal(goal_buffer)
                next_displace_sim_t += oscillate_interval_s

            sim_step_k += 1

            if sim_elapsed + 1e-9 >= next_log_sim_t:
                dist_c = _ee_to_cuboid_distance_m(robot, prim_path, args_cli.distance_link_regex)
                try:
                    if not cube_touched and dist_c <= 0.16:
                        _set_cuboid_color(prim_path, (0.0, 1.0, 0.0))
                        cube_touched = True
                        _v("cube_touch", f"cuboid touched; recolored green")
                except Exception:
                    pass
                if args_cli.diag_extended_csv and mpc_result is not None:
                    dist_g = _ee_to_goal_world_distance_m(robot, prim_path, args_cli.distance_link_regex, above_z_m=above_z_effective)
                    m = mpc_result.metrics
                    fe = _metric_bool(m, "feasible")
                    fe_s = "" if fe is None else ("1" if fe else "0")
                    pe = _metric_float(m, "pose_error")
                    co = _metric_float(m, "cost")
                    con = _metric_float(m, "constraint")
                    dq = _dq_cmd_norm(cu_js_pre, mpc_result, joint_names, tensor_args)
                    sol_ms = mpc_result.solve_time * 1000.0
                    _safe_print(f"{sim_elapsed:.6f},{dist_c:.6f},{dist_g:.6f},{'' if pe is None else f'{pe:.6f}'},{fe_s},{'' if co is None else f'{co:.6f}'},{'' if con is None else f'{con:.6f}'},{dq:.6f},{sol_ms:.3f}", flush=True)
                else:
                    _safe_print(f"{sim_elapsed:.6f},{dist_c:.6f}", flush=True)
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
                mean_pe = reach_sum_pose_err / float(reach_feasible_n) if reach_feasible_n > 0 else float("nan")
                mean_dg = reach_sum_dist_goal / float(reach_feasible_n) if reach_feasible_n > 0 else float("nan")
                final_dc = _ee_to_cuboid_distance_m(reach_robot_for_summary, reach_prim_for_summary, args_cli.distance_link_regex)
                _v("reach_summary", f"steps={n_mpc} feasible_frac={frac:.6f} mean_pose_err={mean_pe:.6f} mean_dist_goal={mean_dg:.6f} final_dist={final_dc:.6f}")
            except Exception as ex:
                _v("reach_summary", f"skipped ({type(ex).__name__}: {ex})")
        if record_cam is not None and record_run_dir and record_frame_idx > 0:
            for view_name, view_data in record_cam.items():
                mp4_path = os.path.join(record_run_dir, f"mpc_cubby_track_{view_name}.mp4")
                _v("record", f"encoding {record_frame_idx} frames -> {mp4_path!r}")
                _finalize_record_mp4(view_data["frames_dir"], record_fps_val, mp4_path)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
    finally:
        try:
            simulation_app.close()
        except Exception:
            pass
