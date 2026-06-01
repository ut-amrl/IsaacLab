# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Operational Space Controller (OSC) tracking a blue cuboid marker that oscillates A/B/C/D.

Replaces the cuRobo MPC pipeline from load_cobot_mpc_cuboid_track.py with an OSC that drives
the Gen3 arm directly via joint torques (stiffness/damping zeroed on arm joints, OSC owns all effort).
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--usd",
    type=str,
    default="/workspace/isaaclab/content/cobot_runtime/cobot.usd",
    help="Root USD containing the Gen3 arm (loaded at /World/CobotStage).",
)
parser.add_argument("--urdf", type=str, default="", help="Optional URDF; converted to --usd-out then used.")
parser.add_argument(
    "--usd-out",
    type=str,
    default="/workspace/isaaclab/content/cobot_runtime/cobot.usd",
    help="Output USD path when --urdf is set.",
)
parser.add_argument("--fix-base", action=argparse.BooleanOptionalAction, default=True)
parser.add_argument("--merge-joints", action="store_true", default=False)
parser.add_argument("--force-urdf-conversion", action="store_true")
parser.add_argument("--cuboid-centroid-prim", type=str, default="/World/Objects/CuboidMarker")
parser.add_argument(
    "--distance-link-regex",
    type=str,
    default="gen3_end_effector_link",
    help="Body name regex for the end-effector (also used as OSC EE frame).",
)
parser.add_argument(
    "--cuboid-displace-interval-s",
    type=float,
    default=10.0,
    help="Sim seconds between A/B/C/D marker goal switches.",
)
parser.add_argument("--cuboid-displace-range-m", type=float, default=0.3)
parser.add_argument("--no-table-cuboid", action="store_true")
parser.add_argument("--log-interval-s", type=float, default=0.3)
parser.add_argument("--max-sim-time-s", type=float, default=0.0)
parser.add_argument(
    "--above-z-m",
    type=float,
    default=0.0,
    help="Goal = marker centroid + this offset along world +Z.",
)
parser.add_argument(
    "--osc-kp",
    type=float,
    default=200.0,
    help="Cartesian stiffness (N/m and N·m/rad) for the OSC (variable_kp mode).",
)
parser.add_argument(
    "--osc-damping-ratio",
    type=float,
    default=1.0,
    help="Damping ratio for critically-damped Cartesian response (1.0 = critically damped).",
)
parser.add_argument(
    "--record-cobot-video",
    action="store_true",
    help="Record RGB video (requires --enable_cameras).",
)
parser.add_argument("--record-duration-s", type=float, default=15.0)
parser.add_argument("--record-fps", type=float, default=24.0)
parser.add_argument(
    "--record-video-out-dir",
    type=str,
    default="/workspace/isaaclab/docker/logs/osc_cuboid_track_videos",
)
parser.add_argument("--record-camera-eye-m", type=float, nargs=3, default=[-4.0, 0.0, 4.0])
parser.add_argument("--record-camera-target-m", type=float, nargs=3, default=[0.6, 0.4, 1.1])
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni.timeline
import torch
from pxr import Gf, UsdGeom

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.controllers import OperationalSpaceController, OperationalSpaceControllerCfg
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, check_file_path
from isaaclab.utils.math import matrix_from_quat, quat_apply, quat_apply_inverse, quat_inv, subtract_frame_transforms

COBOT_STAGE_PRIM = "/World/CobotStage"
_TABLE_TRANSLATION = (0.55, 0.40, 1.05)
_CUBOID_MESH_DIMS = (0.1, 0.1, 0.1)


def _oscillate_xy_reflect_world(p):
    return (-float(p[0]), -float(p[1]), float(p[2]))


_OSCILLATE_POS_A_WORLD = (0.26, -0.05, 0.17)
_OSCILLATE_POS_B_WORLD = (0.10,  0.21, 0.24)
_OSCILLATE_POS_C_WORLD = _oscillate_xy_reflect_world(_OSCILLATE_POS_A_WORLD)
_OSCILLATE_POS_D_WORLD = _oscillate_xy_reflect_world(_OSCILLATE_POS_B_WORLD)
_OSCILLATE_POS_WAYPOINTS = (
    _OSCILLATE_POS_A_WORLD,
    _OSCILLATE_POS_B_WORLD,
    _OSCILLATE_POS_C_WORLD,
    _OSCILLATE_POS_D_WORLD,
)
_CUBOID_TRANSLATION = _OSCILLATE_POS_A_WORLD

_OBST_SPHERES_WORLD = (
    (0.2,  0.1, 0.4),
    (-0.2, 0.1, 0.3),
    (-0.2, -0.1, 0.2),
)
_OBST_SPHERE_RADIUS_M = 0.05


def _v(stage: str, msg: str) -> None:
    print(f"[osc_cuboid_track][{stage}] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Scene helpers (shared with MPC script)
# ---------------------------------------------------------------------------

def _spawn_scene_ground_plane() -> None:
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)


def _spawn_scene_lights() -> None:
    cfg = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/lightDistant", cfg, translation=(1.0, 0.0, 10.0))


def _spawn_scene_spheres() -> None:
    if not sim_utils.get_current_stage().GetPrimAtPath("/World/Objects").IsValid():
        sim_utils.create_prim("/World/Objects", "Xform")
    for i, pos_w in enumerate(_OBST_SPHERES_WORLD, start=1):
        cfg = sim_utils.SphereCfg(
            radius=_OBST_SPHERE_RADIUS_M,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.5, 0.0)),
        )
        cfg.func(f"/World/Objects/obs_sphere_{i}", cfg, translation=pos_w)


def _spawn_blue_cuboid_only() -> None:
    if not sim_utils.get_current_stage().GetPrimAtPath("/World/Objects").IsValid():
        sim_utils.create_prim("/World/Objects", "Xform")
    cfg = sim_utils.MeshCuboidCfg(
        size=_CUBOID_MESH_DIMS,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
    )
    cfg.func("/World/Objects/CuboidMarker", cfg, translation=_CUBOID_TRANSLATION)


def _spawn_table_and_blue_cuboid() -> None:
    _spawn_blue_cuboid_only()
    cfg = sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
    )
    cfg.func("/World/Objects/Table", cfg, translation=_TABLE_TRANSLATION)


def _set_cuboid_world_position(prim_path: str, world_xyz: tuple) -> None:
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2]))
    )


def _cuboid_centroid_world(prim_path: str) -> tuple:
    prim = sim_utils.get_current_stage().GetPrimAtPath(prim_path)
    pos, _ = sim_utils.resolve_prim_pose(prim)
    return (float(pos[0]), float(pos[1]), float(pos[2]))


def _robot_link_world_pos(robot: Articulation, link_regex: str) -> tuple:
    idxs, names = robot.find_bodies(link_regex)
    if not idxs:
        raise RuntimeError(f"No body matched {link_regex!r}. Bodies: {robot.body_names[:8]}")
    p = robot.data.body_pos_w[0, idxs[0]].detach().cpu().tolist()
    return (float(p[0]), float(p[1]), float(p[2]))


def _resolve_ee_body_name(robot: Articulation, requested_regex: str) -> tuple[str, int]:
    # URDF conversion can merge `gen3_end_effector_link` into `gen3_bracelet_link`.
    fallback_candidates = [requested_regex, "gen3_bracelet_link", "gen3_robotiq_85_base_link"]
    for candidate in fallback_candidates:
        idxs, _ = robot.find_bodies(candidate)
        if idxs:
            return candidate, idxs[0]
    raise RuntimeError(
        "Could not resolve an end-effector body. Tried: "
        + ", ".join(repr(name) for name in fallback_candidates)
        + f". Available bodies: {robot.body_names}"
    )


def _ee_to_centroid_dist(robot: Articulation, prim_path: str, link_regex: str) -> tuple:
    """Returns (dist, cx, cy, cz, lx, ly, lz) — components exposed so callers can diagnose NaN."""
    import math
    cx, cy, cz = _cuboid_centroid_world(prim_path)
    lx, ly, lz = _robot_link_world_pos(robot, link_regex)
    dist = float(math.sqrt((cx - lx) ** 2 + (cy - ly) ** 2 + (cz - lz) ** 2))
    return dist, cx, cy, cz, lx, ly, lz


# ---------------------------------------------------------------------------
# Video helpers
# ---------------------------------------------------------------------------

def _ensure_record_camera_initialized(camera: Camera) -> None:
    """Force IsaacLab Camera initialization when created after sim.reset()."""
    timeline_iface = omni.timeline.get_timeline_interface()
    if timeline_iface.is_playing() and not camera.is_initialized:
        camera._initialize_callback(None)
    if not camera.is_initialized:
        raise RuntimeError("Camera sensor failed to initialize. Pass --enable_cameras with --record-cobot-video.")


def _create_record_camera(camera_id: str = "A") -> Camera:
    base = f"/World/OscRecordCamera_{camera_id}"
    sim_utils.create_prim(base, "Xform")
    cfg = CameraCfg(
        prim_path=f"{base}/CameraSensor",
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
    return Camera(cfg=cfg)


def _save_png(out_path: str, rgb: torch.Tensor) -> None:
    from PIL import Image
    img = rgb.detach().cpu()
    if img.dim() == 4:
        img = img[0]
    if img.dtype != torch.uint8:
        img = (img.clamp(0.0, 255.0) if float(img.max()) > 1.0 else img.clamp(0.0, 1.0) * 255).to(torch.uint8)
    arr = img.numpy()
    if arr.shape[-1] >= 4:
        arr = arr[..., :3]
    Image.fromarray(arr).save(out_path)


def _encode_mp4(frames_dir: str, fps: float, out_mp4: str) -> None:
    ffmpeg = shutil.which("ffmpeg") or os.environ.get("FFMPEG_PATH", "")
    if not ffmpeg or not os.path.isfile(ffmpeg):
        _v("record", f"ffmpeg not found; PNG sequence in {frames_dir!r}")
        return
    subprocess.run(
        [ffmpeg, "-y", "-hide_banner", "-loglevel", "error",
         "-framerate", str(fps),
         "-i", os.path.join(frames_dir, "frame_%06d.png"),
         "-c:v", "libx264", "-pix_fmt", "yuv420p", out_mp4],
        check=False,
    )
    if os.path.isfile(out_mp4):
        _v("record", f"wrote {out_mp4!r} ({os.path.getsize(out_mp4)/1e6:.2f} MB)")


# ---------------------------------------------------------------------------
# Articulation attach
# ---------------------------------------------------------------------------

def _attach_articulation_for_osc() -> Articulation:
    """Attach to existing USD prim with arm stiffness/damping zeroed (OSC owns torque).

    Actuator ordering matters: Isaac Lab processes groups in dict order and the last
    group to claim a joint wins.  We put the catch-all FIRST so every joint (including
    any unmerged camera/EE pseudo-joints that survive URDF conversion) gets stable PD
    gains and a non-zero inertia contribution.  The arm group comes SECOND and overrides
    those same seven joints back to zero so the OSC owns all torque there.
    """
    cfg = ArticulationCfg(
        prim_path=COBOT_STAGE_PRIM,
        spawn=None,
        actuators={
            # FIRST: catch-all gives every joint (gripper + any pseudo-DOFs) stable PD
            # gains, preventing near-zero diagonal entries in the mass matrix.
            "all_others": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness=400.0,
                damping=40.0,
            ),
            # SECOND: overrides arm joints to zero so OSC drives via raw torque.
            # effort_limit_sim prevents unbounded torques from blowing up the physics.
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["gen3_joint_[1-7]"],
                effort_limit_sim={"gen3_joint_[1-4]": 80.0, "gen3_joint_[5-7]": 20.0},
                velocity_limit_sim=100.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )
    robot = Articulation(cfg)
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing() and not robot.is_initialized:
        robot._initialize_callback(None)
    if not robot.is_initialized:
        raise RuntimeError(f"Articulation failed to init at {COBOT_STAGE_PRIM}")
    return robot


# ---------------------------------------------------------------------------
# OSC state
# ---------------------------------------------------------------------------

def _update_states(robot: Articulation, ee_frame_idx: int, arm_joint_ids: list[int], device: str):
    """Read kinematic state needed by OSC.compute()."""
    ee_jacobi_idx = ee_frame_idx - 1
    jacobian_w = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, arm_joint_ids]
    mass_matrix = robot.root_physx_view.get_generalized_mass_matrices()[:, arm_joint_ids, :][:, :, arm_joint_ids]
    gravity = robot.root_physx_view.get_gravity_compensation_forces()[:, arm_joint_ids]

    root_rot_matrix = matrix_from_quat(quat_inv(robot.data.root_quat_w))
    jacobian_b = jacobian_w.clone()
    jacobian_b[:, :3, :] = torch.bmm(root_rot_matrix, jacobian_b[:, :3, :])
    jacobian_b[:, 3:, :] = torch.bmm(root_rot_matrix, jacobian_b[:, 3:, :])

    root_pos_w = robot.data.root_pos_w
    root_quat_w = robot.data.root_quat_w
    ee_pos_b, ee_quat_b = subtract_frame_transforms(
        root_pos_w, root_quat_w,
        robot.data.body_pos_w[:, ee_frame_idx],
        robot.data.body_quat_w[:, ee_frame_idx],
    )
    ee_pose_b = torch.cat([ee_pos_b, ee_quat_b], dim=-1)

    rel_vel_w = robot.data.body_vel_w[:, ee_frame_idx, :] - robot.data.root_vel_w
    ee_vel_b = torch.cat([
        quat_apply_inverse(root_quat_w, rel_vel_w[:, :3]),
        quat_apply_inverse(root_quat_w, rel_vel_w[:, 3:6]),
    ], dim=-1)

    joint_pos = robot.data.joint_pos[:, arm_joint_ids]
    joint_vel = robot.data.joint_vel[:, arm_joint_ids]

    return jacobian_b, mass_matrix, gravity, ee_pose_b, ee_vel_b, joint_pos, joint_vel


def _world_pos_to_osc_command(
    robot: Articulation,
    world_xyz: tuple,
    above_z_m: float,
    kp: float,
    device: str,
) -> torch.Tensor:
    """Convert a world-frame goal position into a 13-dim OSC command [pose_abs(7), kp(6)]."""
    root = robot.data.root_link_pose_w[0].float()
    t_wb = root[:3].to(device)
    q_wb = root[3:7].to(device)

    # Goal in world frame (centroid + Z offset)
    p_w = torch.tensor(
        [world_xyz[0], world_xyz[1], world_xyz[2] + above_z_m],
        dtype=torch.float32, device=device,
    )
    # Express in robot base frame
    rel = (p_w - t_wb).unsqueeze(0)
    p_b = quat_apply(quat_inv(q_wb).unsqueeze(0), rel).squeeze(0)

    # Orientation: identity (EE straight ahead in base frame)
    q_b = torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32, device=device)

    pose_cmd = torch.cat([p_b, q_b]).unsqueeze(0)                       # (1, 7)
    kp_cmd = torch.full((1, 6), kp, dtype=torch.float32, device=device)  # (1, 6)
    return torch.cat([pose_cmd, kp_cmd], dim=-1)                         # (1, 13)


# ---------------------------------------------------------------------------
# USD / URDF resolution
# ---------------------------------------------------------------------------

def _resolve_usd_path() -> str:
    """Return the USD to load — either the --usd arg directly, or converted from --urdf."""
    if not args_cli.urdf:
        usd_path = os.path.abspath(os.path.expanduser(args_cli.usd))
        if not os.path.isfile(usd_path):
            print(f"ERROR: USD not found: {usd_path}", file=sys.stderr)
            sys.exit(2)
        return usd_path

    urdf_path = os.path.abspath(os.path.expanduser(args_cli.urdf))
    if not check_file_path(urdf_path):
        print(f"ERROR: URDF not found: {urdf_path}", file=sys.stderr)
        sys.exit(2)
    dest = os.path.abspath(os.path.expanduser(args_cli.usd_out))
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=os.path.dirname(dest),
        usd_file_name=os.path.basename(dest),
        fix_base=args_cli.fix_base,
        merge_fixed_joints=args_cli.merge_joints,  # default False; keeps gen3_end_effector_link as a named body
        force_usd_conversion=args_cli.force_urdf_conversion,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=400.0,
                damping=40.0,
            ),
            target_type="position",
        ),
    )
    out = UrdfConverter(cfg).usd_path
    if not os.path.isfile(out):
        print(f"ERROR: URDF conversion produced no USD at {out}", file=sys.stderr)
        sys.exit(2)
    _v("urdf", f"converted -> {out}")
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    usd_path = _resolve_usd_path()

    record_enabled = bool(args_cli.record_cobot_video)
    if record_enabled and not getattr(args_cli, "enable_cameras", False):
        raise RuntimeError("--record-cobot-video requires --enable_cameras.")

    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    device = sim.device

    if record_enabled:
        sim.set_camera_view(args_cli.record_camera_eye_m, args_cli.record_camera_target_m)
    else:
        sim.set_camera_view([2.0, 0.0, 2.5], [0.0, 0.0, 0.5])

    sim_utils.UsdFileCfg(usd_path=usd_path).func("/World/CobotStage", sim_utils.UsdFileCfg(usd_path=usd_path))
    _spawn_scene_ground_plane()
    _spawn_scene_lights()
    _spawn_scene_spheres()

    if not args_cli.no_table_cuboid:
        _spawn_table_and_blue_cuboid()
    else:
        _spawn_blue_cuboid_only()

    sim.reset()

    robot = _attach_articulation_for_osc()
    robot.update(0.0)

    prim_path = args_cli.cuboid_centroid_prim
    if not sim_utils.get_current_stage().GetPrimAtPath(prim_path).IsValid():
        raise RuntimeError(f"Missing marker cuboid prim at {prim_path!r}")

    # ------------------------------------------------------------------
    # OSC setup
    # ------------------------------------------------------------------
    ee_frame_name, ee_frame_idx = _resolve_ee_body_name(robot, args_cli.distance_link_regex)
    arm_joint_names = ["gen3_joint_[1-7]"]

    arm_joint_ids = robot.find_joints(arm_joint_names)[0]
    _v(
        "osc",
        f"EE frame: {ee_frame_name!r} (idx={ee_frame_idx}), "
        f"requested={args_cli.distance_link_regex!r}, arm joints: {arm_joint_ids}",
    )

    osc_cfg = OperationalSpaceControllerCfg(
        target_types=["pose_abs"],
        impedance_mode="variable_kp",
        inertial_dynamics_decoupling=False,  # gen3_end_effector_link has no URDF mass → near-zero inertia → singular mass matrix
        partial_inertial_dynamics_decoupling=False,
        gravity_compensation=True,  # gravity is active in URDF-loaded scene (not disabled like in run_osc_gen3.py)
        motion_damping_ratio_task=args_cli.osc_damping_ratio,
        motion_control_axes_task=[1, 1, 1, 1, 1, 1],
        contact_wrench_control_axes_task=[0, 0, 0, 0, 0, 0],
        # nullspace_control="none": arm starts in kinematic singularity (fully extended upward).
        # torch.pinverse() in the nullspace path returns NaN at singularity and destroys physics
        # on step 0. With "none", only J^T * f_task is used — safe at any configuration.
        nullspace_control="none",
    )
    osc = OperationalSpaceController(osc_cfg, num_envs=1, device=device)

    joint_centers = torch.mean(robot.data.soft_joint_pos_limits[:, arm_joint_ids, :], dim=-1)

    above_z = float(args_cli.above_z_m)
    kp = float(args_cli.osc_kp)

    # Set initial OSC command
    centroid_w = _cuboid_centroid_world(prim_path)
    robot.update(float(sim_cfg.dt))
    _, _, _, ee_pose_b, _, _, _ = _update_states(robot, ee_frame_idx, arm_joint_ids, device)
    command = _world_pos_to_osc_command(robot, centroid_w, above_z, kp, device)
    osc.reset()
    osc.set_command(command=command, current_ee_pose_b=ee_pose_b)

    # ------------------------------------------------------------------
    # Video recording setup
    # ------------------------------------------------------------------
    record_cam = None
    record_frames_dir = ""
    record_run_dir = ""
    record_frame_idx = 0
    next_capture_sim_t = 0.0
    record_fps_val = max(1e-3, float(args_cli.record_fps))
    record_duration_val = max(float(sim_cfg.dt), float(args_cli.record_duration_s))

    if record_enabled:
        run_id = os.environ.get("SLURM_JOB_ID", f"local_{int(time.time())}")
        base_out = os.path.abspath(os.path.expanduser(args_cli.record_video_out_dir))
        record_run_dir = os.path.join(base_out, str(run_id))
        record_frames_dir = os.path.join(record_run_dir, "frames")
        os.makedirs(record_frames_dir, exist_ok=True)
        record_cam = _create_record_camera()
        _ensure_record_camera_initialized(record_cam)
        record_cam.reset()
        dev_t = torch.tensor([args_cli.record_camera_eye_m], device=device, dtype=torch.float32)
        tgt_t = torch.tensor([args_cli.record_camera_target_m], device=device, dtype=torch.float32)
        record_cam.set_world_poses_from_view(dev_t, tgt_t)
        _v("record", f"camera ready; run_dir={record_run_dir!r}")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    oscillate_interval_s = max(float(args_cli.cuboid_displace_interval_s), float(sim_cfg.dt))
    cuboid_waypoint_idx = 0
    next_displace_sim_t = oscillate_interval_s
    next_log_sim_t = 0.0
    sim_elapsed = 0.0
    sim_step_k = 0
    stop_sim_t = float("inf")
    if args_cli.max_sim_time_s > 0.0:
        stop_sim_t = min(stop_sim_t, float(args_cli.max_sim_time_s))
    if record_enabled:
        stop_sim_t = min(stop_sim_t, record_duration_val)

    print("t_sim_s,dist_centroid_m,cuboid_x,cuboid_y,cuboid_z,ee_x,ee_y,ee_z", flush=True)
    _v("loop", "starting OSC loop")

    try:
        while simulation_app.is_running():
            if stop_sim_t < float("inf") and sim_elapsed >= stop_sim_t - 1e-9:
                _v("loop", f"stopping at sim_t={sim_elapsed:.4f}s")
                break

            robot.update(float(sim_cfg.dt))

            jacobian_b, mass_matrix, gravity, ee_pose_b, ee_vel_b, joint_pos, joint_vel = _update_states(
                robot, ee_frame_idx, arm_joint_ids, device
            )

            if sim_step_k < 20:
                ep = ee_pose_b[0, :3].detach().cpu().tolist()
                bpw = robot.data.body_pos_w[0, ee_frame_idx].detach().cpu().tolist()
                print(
                    f"[diag k={sim_step_k}] ee_pos_b=({ep[0]:.4f},{ep[1]:.4f},{ep[2]:.4f})"
                    f" body_pos_w[{ee_frame_idx}]=({bpw[0]:.4f},{bpw[1]:.4f},{bpw[2]:.4f})",
                    flush=True,
                )

            joint_efforts = osc.compute(
                jacobian_b=jacobian_b,
                current_ee_pose_b=ee_pose_b,
                current_ee_vel_b=ee_vel_b,
                current_ee_force_b=None,
                mass_matrix=mass_matrix,
                gravity=gravity,
                current_joint_pos=joint_pos,
                current_joint_vel=joint_vel,
            )
            if sim_step_k < 20:
                eff = joint_efforts[0].detach().cpu().tolist()
                print(f"[diag k={sim_step_k}] efforts={[f'{v:.2f}' for v in eff]}", flush=True)
            robot.set_joint_effort_target(joint_efforts, joint_ids=arm_joint_ids)
            robot.write_data_to_sim()
            sim.step()
            sim_elapsed += float(sim_cfg.dt)

            if record_cam is not None:
                record_cam.update(float(sim_cfg.dt))
                while sim_elapsed + 1e-9 >= next_capture_sim_t and sim_elapsed <= record_duration_val + 1e-6:
                    out_png = os.path.join(record_frames_dir, f"frame_{record_frame_idx:06d}.png")
                    _save_png(out_png, record_cam.data.output["rgb"])
                    record_frame_idx += 1
                    next_capture_sim_t += 1.0 / record_fps_val

            # Oscillate cuboid marker
            if sim_elapsed + 1e-9 >= next_displace_sim_t:
                cuboid_waypoint_idx = (cuboid_waypoint_idx + 1) % len(_OSCILLATE_POS_WAYPOINTS)
                new_world = _OSCILLATE_POS_WAYPOINTS[cuboid_waypoint_idx]
                wname = ("A", "B", "C", "D")[cuboid_waypoint_idx]
                _v("cuboid", f"sim_t={sim_elapsed:.4f}s -> pos_{wname} {new_world}")
                _set_cuboid_world_position(prim_path, new_world)
                centroid_w = _cuboid_centroid_world(prim_path)
                command = _world_pos_to_osc_command(robot, centroid_w, above_z, kp, device)
                osc.set_command(command=command, current_ee_pose_b=ee_pose_b)
                next_displace_sim_t += oscillate_interval_s

            # Distance logging
            if sim_elapsed + 1e-9 >= next_log_sim_t:
                dist, cx, cy, cz, lx, ly, lz = _ee_to_centroid_dist(robot, prim_path, ee_frame_name)
                print(f"{sim_elapsed:.6f},{dist:.6f},{cx:.6f},{cy:.6f},{cz:.6f},{lx:.6f},{ly:.6f},{lz:.6f}", flush=True)
                next_log_sim_t += float(args_cli.log_interval_s)

            sim_step_k += 1

    except Exception:
        _v("main", "EXCEPTION:")
        traceback.print_exc()
        raise
    finally:
        if record_cam is not None and record_frame_idx > 0:
            mp4_path = os.path.join(record_run_dir, "osc_cuboid_track.mp4")
            _v("record", f"encoding {record_frame_idx} frames -> {mp4_path!r}")
            _encode_mp4(record_frames_dir, record_fps_val, mp4_path)


if __name__ == "__main__":
    main()
    simulation_app.close()
