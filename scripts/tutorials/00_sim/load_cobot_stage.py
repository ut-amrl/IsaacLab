# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Load a cobot USD stage and run the simulation loop (smoke test).

Paths are **inside the container** (match your Apptainer ``-B`` binds).

**USD only** (legacy / pre-baked stage):

.. code-block:: bash

    ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_stage.py --headless \\
        --usd /workspace/isaaclab/content/stage-19.usd

**URDF to USD then load** (Kinova / cobot description from ``ros2_kortex``, e.g. branch
``cobot-urdf`` on `ut-amrl/ros2_kortex <https://github.com/ut-amrl/ros2_kortex/tree/cobot-urdf>`_):

.. code-block:: bash

    ./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_stage.py --headless \\
        --urdf /workspace/ros2_kortex/kortex_description/robots/gen3_2f85.urdf \\
        --usd-out /workspace/isaaclab/content/cobot_runtime/cobot.usd

By default the script also spawns the **Seattle lab table** and **blue deformable cuboid** from
``spawn_table_blue_cuboid.py`` under ``/World/Objects``, offset **+X / +Y** from the robot base. Pass
``--no-table-cuboid`` to skip. When the cuboid exists, the script prints the cuboid root Xform world
position, the world position of a robot **link** (default ``gen3_end_effector_link``), the displacement
vector (cuboid minus link), and the Euclidean distance (override link match with ``--distance-link-regex``).
In sim time, every ``--cuboid-displace-interval-s`` (default 5), the cuboid root is moved to the
anchor spawn position plus independent uniform offsets in ``+/- --cuboid-displace-range-m`` (default 0.3 m)
per axis; after each move the script logs metrics plus ``distance_after_displacement_m``.

With ``--record-cobot-video`` (requires ``--enable_cameras``), spawns an RGB camera at
``--record-camera-eye-m`` looking at ``--record-camera-target-m``, captures at ``--record-fps`` for
``--record-duration-s`` sim seconds under ``--record-video-out-dir/<run_id>/``, then runs ``ffmpeg`` when
available to write ``cobot_stage.mp4`` (paths under ``IsaacLab/docker/logs`` are visible on the login node
when the usual Apptainer bind is used).

CuRobo joint and frame naming for the stock Gen3 + Robotiq URDF is summarized in
``COBOT_CUROBO_REFERENCE.md`` in this directory. Large USD under ``content/`` is Git LFS–tracked;
run ``bash scripts/pull_kinova_usd_lfs.sh`` from the Isaac Lab repo root if ``--usd`` loads fail
with an LFS stub error.

"""

import argparse
import math
import os
import random
import traceback

from isaaclab.app import AppLauncher


def _v(stage: str, msg: str) -> None:
    """Verbose smoke log (always flushed for Slurm / Apptainer)."""
    print(f"[load_cobot_stage][{stage}] {msg}", flush=True)

parser = argparse.ArgumentParser(
    description="Load a cobot USD: either --usd directly or convert --urdf to --usd-out then load."
)
parser.add_argument(
    "--usd",
    type=str,
    default="/workspace/isaaclab/content/stage-19.usd",
    help="Root USD to load when --urdf is not set.",
)
parser.add_argument(
    "--urdf",
    type=str,
    default="",
    help="URDF path inside the container. If set, convert to --usd-out (unless up to date) then load that USD.",
)
parser.add_argument(
    "--usd-out",
    type=str,
    default="/workspace/isaaclab/content/cobot_runtime/cobot.usd",
    help="Output USD path when using --urdf.",
)
parser.add_argument(
    "--force-urdf-conversion",
    action="store_true",
    help="Force URDF re-import even if a matching USD already exists.",
)
parser.add_argument(
    "--merge-joints",
    action="store_true",
    default=False,
    help="Consolidate links connected by fixed joints (passed to UrdfConverter).",
)
parser.add_argument(
    "--fix-base",
    action="store_true",
    default=False,
    help="Fix the robot base in the importer (passed to UrdfConverter).",
)
parser.add_argument(
    "--joint-stiffness",
    type=float,
    default=100.0,
    help="Joint drive stiffness for imported articulation.",
)
parser.add_argument(
    "--joint-damping",
    type=float,
    default=1.0,
    help="Joint drive damping for imported articulation.",
)
parser.add_argument(
    "--joint-target-type",
    type=str,
    default="position",
    choices=["position", "velocity", "none"],
    help="Joint drive target type for imported articulation.",
)
parser.add_argument(
    "--no-table-cuboid",
    action="store_true",
    help="Skip spawning Nucleus Seattle lab table + blue deformable cuboid beside the robot.",
)
parser.add_argument(
    "--cuboid-centroid-prim",
    type=str,
    default="/World/Objects/CuboidDeformable",
    help="USD prim path whose world-frame translation is reported as the cuboid centroid (root Xform of the mesh).",
)
parser.add_argument(
    "--distance-link-regex",
    type=str,
    default="gen3_end_effector_link",
    help="Regex (re.fullmatch per body name) selecting one articulation link for distance-to-cuboid metrics.",
)
parser.add_argument(
    "--skip-cuboid-link-distance",
    action="store_true",
    help="Do not print cuboid vs link centroid and distance (default: print when cuboid prim exists).",
)
parser.add_argument(
    "--cuboid-displace-interval-s",
    type=float,
    default=5.0,
    help="Sim time between random cuboid root displacements (requires table+cuboid; 0 disables).",
)
parser.add_argument(
    "--cuboid-displace-range-m",
    type=float,
    default=0.3,
    help="Per-axis uniform random offset in meters from the anchor cuboid position (inclusive +/- range).",
)
parser.add_argument(
    "--record-cobot-video",
    action="store_true",
    help=(
        "Record RGB video from a world camera (requires --enable_cameras). Stops the sim loop after "
        "--record-duration-s sim time; writes under --record-video-out-dir (bind docker/logs on cluster)."
    ),
)
parser.add_argument(
    "--record-duration-s",
    type=float,
    default=240.0,
    help="Simulation time (seconds) to record when --record-cobot-video is set (default 4 minutes).",
)
parser.add_argument(
    "--record-fps",
    type=float,
    default=30.0,
    help="Target capture rate (frames per simulation second) for --record-cobot-video.",
)
parser.add_argument(
    "--record-video-out-dir",
    type=str,
    default="/workspace/isaaclab/docker/logs/cobot_stage_videos",
    help="Host-visible directory (container path) for PNG frames + cobot_stage.mp4.",
)
parser.add_argument(
    "--record-camera-eye-m",
    type=float,
    nargs=3,
    default=[-2.0, 0.0, 2.0],
    metavar=("X", "Y", "Z"),
    help="World-frame camera eye position (m) for --record-cobot-video.",
)
parser.add_argument(
    "--record-camera-target-m",
    type=float,
    nargs=3,
    default=[0.0, 0.0, 0.5],
    metavar=("X", "Y", "Z"),
    help="World-frame look-at target (m) for --record-cobot-video.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
_v(
    "parse",
    f"argv parsed: urdf={args_cli.urdf!r} usd={args_cli.usd!r} usd_out={args_cli.usd_out!r} "
    f"force_urdf_conversion={args_cli.force_urdf_conversion} no_table_cuboid={args_cli.no_table_cuboid} "
    f"cuboid_prim={args_cli.cuboid_centroid_prim!r} distance_link_regex={args_cli.distance_link_regex!r} "
    f"skip_cuboid_link_distance={args_cli.skip_cuboid_link_distance} "
    f"cuboid_displace_interval_s={args_cli.cuboid_displace_interval_s} "
    f"cuboid_displace_range_m={args_cli.cuboid_displace_range_m} "
    f"record_cobot_video={args_cli.record_cobot_video} enable_cameras={getattr(args_cli, 'enable_cameras', False)} "
    f"device={getattr(args_cli, 'device', '?')}",
)

GIT_LFS_POINTER_PREFIX = b"version https://git-lfs.github.com/spec/v1"


def _is_git_lfs_pointer_file(path: str) -> bool:
    """True if ``path`` looks like a Git LFS pointer stub, not a materialized asset."""
    try:
        with open(path, "rb") as f:
            return f.read(len(GIT_LFS_POINTER_PREFIX)) == GIT_LFS_POINTER_PREFIX
    except OSError:
        return False


def _preflight_usd_not_lfs_stub() -> None:
    """Fail fast before launching Kit if ``--usd`` points at an LFS pointer (common after shallow clone)."""
    _v("preflight", "start: Git LFS pointer check for --usd (skipped if --urdf is set)")
    if args_cli.urdf:
        _v("preflight", "skip (--urdf set; conversion output is not expected to be an LFS stub)")
        return
    usd_path = os.path.abspath(os.path.expanduser(args_cli.usd))
    _v("preflight", f"resolved --usd path: {usd_path}")
    if not os.path.isfile(usd_path):
        _v("preflight", "skip (USD file missing; will fail later in _resolve or main)")
        return
    is_ptr = _is_git_lfs_pointer_file(usd_path)
    _v("preflight", f"Git LFS pointer check: is_pointer={is_ptr}")
    if is_ptr:
        raise RuntimeError(
            f"USD is a Git LFS pointer stub, not a real file: {usd_path}\n"
            "Install git-lfs, run `git lfs install`, then from the Isaac Lab repo root run:\n"
            "  bash scripts/pull_kinova_usd_lfs.sh\n"
            "Or: git lfs pull --include=\"content/**/*.usd\"\n"
            "For URDF-only smoke (no pre-baked Kinova USD), use --urdf /path/to/robot.urdf --usd-out ..."
        )
    _v("preflight", "ok (USD exists and is not an LFS pointer stub)")


_preflight_usd_not_lfs_stub()

_v("kit", "starting AppLauncher (Isaac Sim / Kit); this can take tens of seconds")
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
_v("kit", "AppLauncher finished; simulation_app acquired")

"""Rest everything follows."""

import shutil
import subprocess
import time

import omni.timeline
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, check_file_path
from isaaclab.utils.dict import print_dict

COBOT_STAGE_PRIM = "/World/CobotStage"
# Table + cuboid offsets (meters, world): robot ~origin; place props to +X/+Y side (same assets as spawn_table_blue_cuboid.py).
_TABLE_TRANSLATION = (0.55, 0.40, 1.05)
_CUBOID_TRANSLATION = (0.68, 0.40, 1.12)
_CUBOID_MESH_DIMS = (0.1, 0.1, 0.1)


def _set_cuboid_root_world_position_preserve_orientation(
    prim_path: str, world_xyz: tuple[float, float, float]
) -> None:
    """Move an Xformable prim to ``world_xyz`` while keeping its current world orientation."""
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid prim path for cuboid displacement: {prim_path}")
    _pos_w, quat_w = sim_utils.resolve_prim_pose(prim)
    parent = prim.GetParent()
    if not parent.IsValid():
        raise RuntimeError(f"Prim {prim_path!r} has no valid parent for local transform.")
    local_pos, local_quat = sim_utils.convert_world_pose_to_local(world_xyz, quat_w, parent)
    if local_quat is None:
        raise RuntimeError("convert_world_pose_to_local returned no orientation")
    if not sim_utils.standardize_xform_ops(prim, translation=local_pos, orientation=local_quat):
        raise RuntimeError(f"standardize_xform_ops failed for {prim_path}")


def _cuboid_centroid_world_from_prim(prim_path: str) -> tuple[float, float, float]:
    """World translation of the cuboid asset root Xform (nominal centroid proxy for the spawned mesh)."""
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid prim for cuboid centroid: {prim_path}")
    pos, _quat = sim_utils.resolve_prim_pose(prim)
    return (float(pos[0]), float(pos[1]), float(pos[2]))


def _robot_link_world_pos_m(robot: Articulation, link_regex: str) -> tuple[float, float, float, str]:
    """World link-frame position (m) for the first body matching ``link_regex`` (fullmatch)."""
    try:
        idxs, names = robot.find_bodies(link_regex)
    except ValueError as e:
        raise RuntimeError(
            f"No unique body match for regex {link_regex!r}: {e} "
            f"(example bodies: {robot.body_names[:12]})"
        ) from e
    if not idxs:
        raise RuntimeError(f"No body matched regex {link_regex!r}. Example bodies: {robot.body_names[:8]}...")
    i = idxs[0]
    name = names[0]
    p = robot.data.body_link_pos_w[0, i].detach().cpu().tolist()
    return (float(p[0]), float(p[1]), float(p[2]), name)


def _report_cuboid_link_distance(robot: Articulation, *, stage_label: str, update_dt: float) -> float | None:
    """Log cuboid Xform world position, robot link world position, displacement (cuboid - link), and distance.

    Returns:
        Euclidean distance in meters when metrics are printed, else ``None``.
    """
    if args_cli.skip_cuboid_link_distance:
        _v("metrics", f"{stage_label}: skip (--skip-cuboid-link-distance)")
        return None
    prim_path = args_cli.cuboid_centroid_prim
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        _v("metrics", f"{stage_label}: skip (no prim at {prim_path!r})")
        return None

    robot.update(update_dt)
    try:
        cx, cy, cz = _cuboid_centroid_world_from_prim(prim_path)
    except ValueError as e:
        _v("metrics", f"{stage_label}: skip cuboid centroid ({e})")
        return None

    try:
        lx, ly, lz, link_name = _robot_link_world_pos_m(robot, args_cli.distance_link_regex)
    except RuntimeError as e:
        _v("metrics", f"{stage_label}: skip link position ({e})")
        return None

    dx, dy, dz = cx - lx, cy - ly, cz - lz
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    _v("metrics", f"{stage_label}: cuboid_centroid_world_m ({prim_path}) = ({cx:.6f}, {cy:.6f}, {cz:.6f})")
    _v(
        "metrics",
        f"{stage_label}: link_frame_world_m (body {link_name!r}, regex {args_cli.distance_link_regex!r}) "
        f"= ({lx:.6f}, {ly:.6f}, {lz:.6f})",
    )
    _v(
        "metrics",
        f"{stage_label}: displacement_vector_cuboid_minus_link_m = ({dx:.6f}, {dy:.6f}, {dz:.6f})",
    )
    _v("metrics", f"{stage_label}: euclidean_distance_m = {dist:.6f}")
    return dist


def _spawn_default_ground_plane() -> None:
    """Ground plane for deformable cuboid / scene contact when not in the root USD."""
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)


def _spawn_table_and_blue_cuboid() -> None:
    """Seattle lab table + blue deformable cuboid (see ``spawn_table_blue_cuboid.py``), offset beside robot."""
    _v("scene", "creating /World/Objects Xform for table + cuboid")
    sim_utils.create_prim("/World/Objects", "Xform")

    _v("scene", "spawning blue MeshCuboid (deformable) next to robot")
    cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
        size=_CUBOID_MESH_DIMS,
        deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
        physics_material=sim_utils.DeformableBodyMaterialCfg(poissons_ratio=0.4, youngs_modulus=1.0e5),
    )
    cfg_cuboid_deformable.func(
        "/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=_CUBOID_TRANSLATION
    )

    _v("scene", f"spawning SeattleLabTable from Nucleus (translation={_TABLE_TRANSLATION})")
    cfg_table = sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
    )
    cfg_table.func("/World/Objects/Table", cfg_table, translation=_TABLE_TRANSLATION)
    _v("scene", "table + cuboid spawn calls returned")


def _attach_articulation_for_logging() -> Articulation:
    """Wrap the spawned cobot USD so we can read PhysX DOF names and state (post-reset, timeline playing)."""
    _v("articulation", f"wrapping stage prim as Articulation: prim_path={COBOT_STAGE_PRIM!r}, spawn=None")
    articulation_cfg = ArticulationCfg(
        prim_path=COBOT_STAGE_PRIM,
        spawn=None,
        actuators={
            "diag": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness=10.0,
                damping=1.0,
            ),
        },
    )
    robot = Articulation(articulation_cfg)
    timeline_iface = omni.timeline.get_timeline_interface()
    playing = timeline_iface.is_playing()
    _v("articulation", f"timeline.is_playing()={playing} robot.is_initialized={robot.is_initialized}")
    if playing and not robot.is_initialized:
        _v("articulation", "manually calling _initialize_callback (PLAY already active; subscription missed)")
        robot._initialize_callback(None)
    if not robot.is_initialized:
        raise RuntimeError(
            "Articulation not initialized after sim.reset(). "
            f"timeline.is_playing()={playing}. Try stepping sim once before attach, or check articulation root under "
            f"{COBOT_STAGE_PRIM}."
        )
    _v(
        "articulation",
        f"PhysX view ready: num_joints={robot.num_joints} num_bodies={robot.num_bodies} "
        f"is_fixed_base={robot.is_fixed_base}",
    )
    return robot


def _ensure_camera_sensor_initialized(camera: Camera) -> None:
    """Initialize Replicator-backed camera when timeline is already PLAY (smoke / script order)."""
    timeline_iface = omni.timeline.get_timeline_interface()
    playing = timeline_iface.is_playing()
    if playing and not camera.is_initialized:
        camera._initialize_callback(None)
    if not camera.is_initialized:
        raise RuntimeError(
            "Camera sensor failed to initialize. Use --enable_cameras with --record-cobot-video."
        )


def _create_cobot_record_camera(sim: sim_utils.SimulationContext) -> Camera:
    """RGB camera under /World/SmokeRecordCamera (world pose set after reset)."""
    sim_utils.create_prim("/World/SmokeRecordCamera", "Xform")
    camera_cfg = CameraCfg(
        prim_path="/World/SmokeRecordCamera/CameraSensor",
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
    """Write one RGB PNG from camera ``rgb`` tensor (N,H,W,C) or (H,W,C), uint8 or float in [0,1]."""
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
        _v(
            "record",
            f"ffmpeg not in PATH; PNG sequence in {frames_dir!r}. On login node: "
            f"ffmpeg -y -framerate {fps} -i frame_%06d.png -c:v libx264 -pix_fmt yuv420p cobot_stage.mp4",
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


def _print_all_joints(robot: Articulation, *, stage: str, update_dt: float) -> None:
    """Print every DOF name with position, velocity, and soft limits (instance 0)."""
    robot.update(update_dt)
    names = robot.joint_names
    pos = robot.data.joint_pos[0].detach().cpu().tolist()
    vel = robot.data.joint_vel[0].detach().cpu().tolist()
    lims = robot.data.soft_joint_pos_limits
    lims_row = lims[0].detach().cpu().tolist() if lims is not None else None
    _v("joints", f"======== {stage}: all DOFs (count={len(names)}) ========")
    for i, name in enumerate(names):
        if lims_row is not None and i < len(lims_row):
            lo, hi = lims_row[i][0], lims_row[i][1]
            lim_s = f"  soft_lim=[{lo:+.4f},{hi:+.4f}]"
        else:
            lim_s = ""
        _v(
            "joints",
            f"  [{i:3d}] {name!r}  pos={pos[i]:+.6f}  vel={vel[i]:+.6f}{lim_s}",
        )
    _v("joints", f"======== end {stage} ========")


def _resolve_usd_path_to_load() -> str:
    """Return absolute USD path after optional URDF conversion."""
    _v("resolve", "enter _resolve_usd_path_to_load")
    if not args_cli.urdf:
        _v("resolve", "mode=usd_only (no --urdf)")
        usd_path = os.path.abspath(os.path.expanduser(args_cli.usd))
        _v("resolve", f"resolved usd_path={usd_path} exists={os.path.isfile(usd_path)}")
        if not os.path.isfile(usd_path):
            raise FileNotFoundError(f"USD not found: {usd_path}")
        _v("resolve", "returning existing USD path (no URDF conversion)")
        return usd_path

    _v("resolve", "mode=urdf_then_usd")
    urdf_path = os.path.abspath(os.path.expanduser(args_cli.urdf))
    _v("resolve", f"resolved urdf_path={urdf_path}")
    _v("resolve", "calling check_file_path(urdf)")
    if not check_file_path(urdf_path):
        raise ValueError(f"Invalid or missing URDF: {urdf_path}")
    _v("resolve", "URDF path ok")

    dest_path = os.path.abspath(os.path.expanduser(args_cli.usd_out))
    _v("resolve", f"usd_out dest_path={dest_path}")
    _v("resolve", f"os.makedirs({os.path.dirname(dest_path)!r}, exist_ok=True)")
    os.makedirs(os.path.dirname(dest_path), exist_ok=True)
    _v("resolve", "output directory ready")

    _v("resolve", "building UrdfConverterCfg")
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
    print("URDF importer config:", flush=True)
    print_dict(urdf_converter_cfg.to_dict(), nesting=0)
    print("-" * 80, flush=True)

    _v("resolve", "instantiating UrdfConverter (may run importer / write USD)")
    urdf_converter = UrdfConverter(urdf_converter_cfg)
    _v("resolve", "UrdfConverter constructed")
    out = urdf_converter.usd_path
    _v("resolve", f"UrdfConverter.usd_path -> {out}")
    print(f"[INFO]: URDF import complete; loading USD: {out}", flush=True)
    if not os.path.isfile(out):
        raise FileNotFoundError(f"Expected USD after conversion but missing: {out}")
    _v("resolve", "converted USD file exists on disk")
    if _is_git_lfs_pointer_file(out):
        raise RuntimeError(f"Converted output unexpectedly looks like a Git LFS stub: {out}")
    _v("resolve", "converted USD is not an LFS pointer")
    _v("resolve", "done _resolve_usd_path_to_load (urdf branch)")
    return out


def main() -> None:
    """Main function."""
    _v("main", "enter main()")
    try:
        usd_path = _resolve_usd_path_to_load()
        _v("main", f"will load USD into sim: {usd_path}")

        _v("main", "building SimulationCfg(dt=0.01, device=...)")
        sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
        _v("main", f"SimulationCfg ready device={sim_cfg.device}")

        _v("main", "creating SimulationContext (PhysX / physics scene setup)")
        sim = sim_utils.SimulationContext(sim_cfg)
        _v("main", "SimulationContext constructed")

        _v("main", "set_camera_view([2,0,2.5] -> [0,0,0.5])")
        sim.set_camera_view([2.0, 0.0, 2.5], [0.0, 0.0, 0.5])
        _v("main", "set_camera_view returned")

        record_enabled = bool(args_cli.record_cobot_video)
        if record_enabled and not bool(getattr(args_cli, "enable_cameras", False)):
            raise RuntimeError("--record-cobot-video requires --enable_cameras (Isaac Lab AppLauncher flag).")
        if record_enabled:
            sim.set_camera_view(
                list(args_cli.record_camera_eye_m),
                list(args_cli.record_camera_target_m),
            )
            _v(
                "main",
                f"set_camera_view(record eye -> target) "
                f"{tuple(args_cli.record_camera_eye_m)} -> {tuple(args_cli.record_camera_target_m)}",
            )

        _v("main", f"creating UsdFileCfg(usd_path={usd_path!r})")
        cfg = sim_utils.UsdFileCfg(usd_path=usd_path)
        _v("main", "UsdFileCfg created")

        _v("main", "calling cfg.func('/World/CobotStage', cfg) - spawn USD under /World")
        cfg.func("/World/CobotStage", cfg)
        _v("main", "cfg.func returned (prim spawn / reference should be in stage)")

        _spawn_default_ground_plane()

        if not args_cli.no_table_cuboid:
            _spawn_table_and_blue_cuboid()
        else:
            _v("scene", "skip table+cuboid (--no-table-cuboid)")

        _v("main", "calling sim.reset() - initializes physics for current stage")
        sim.reset()
        _v("main", "sim.reset() returned")
        extra = " + table + blue cuboid" if not args_cli.no_table_cuboid else ""
        print(f"[INFO]: Setup complete (loaded {usd_path}){extra}...", flush=True)

        _v("articulation", "attaching Articulation for joint diagnostics")
        robot = _attach_articulation_for_logging()
        _print_all_joints(robot, stage="stage_0_post_reset (before any sim.step)", update_dt=0.0)
        _report_cuboid_link_distance(robot, stage_label="stage_0_post_reset", update_dt=0.0)

        cuboid_displace_enabled = False
        cuboid_anchor_world = (0.0, 0.0, 0.0)
        sim_elapsed = 0.0
        next_displace_sim_t = float(args_cli.cuboid_displace_interval_s)
        if (
            not args_cli.no_table_cuboid
            and not args_cli.skip_cuboid_link_distance
            and args_cli.cuboid_displace_interval_s > 0.0
        ):
            prim_path_anchor = args_cli.cuboid_centroid_prim
            prim_anchor = sim_utils.get_current_stage().GetPrimAtPath(prim_path_anchor)
            if prim_anchor.IsValid():
                try:
                    cuboid_anchor_world = _cuboid_centroid_world_from_prim(prim_path_anchor)
                    cuboid_displace_enabled = True
                    _v(
                        "cuboid_displace",
                        f"enabled: interval_s={args_cli.cuboid_displace_interval_s} "
                        f"range_m={args_cli.cuboid_displace_range_m} anchor_world_m={cuboid_anchor_world}",
                    )
                except ValueError as e:
                    _v("cuboid_displace", f"disabled (anchor read failed: {e})")
            else:
                _v("cuboid_displace", f"disabled (no prim at {prim_path_anchor!r})")
        else:
            _v("cuboid_displace", "disabled (no table/cuboid, skip metrics, or interval<=0)")

        record_cam = None
        record_frames_dir = ""
        record_run_dir = ""
        record_fps = max(1e-3, float(args_cli.record_fps))
        record_duration = max(float(sim.cfg.dt), float(args_cli.record_duration_s))
        next_capture_sim_t = 0.0
        record_frame_idx = 0

        if record_enabled:
            run_id = os.environ.get("SLURM_JOB_ID", f"local_{int(time.time())}")
            base_out = os.path.abspath(os.path.expanduser(args_cli.record_video_out_dir))
            os.makedirs(base_out, exist_ok=True)
            record_run_dir = os.path.join(base_out, str(run_id))
            record_frames_dir = os.path.join(record_run_dir, "frames")
            os.makedirs(record_frames_dir, exist_ok=True)
            record_cam = _create_cobot_record_camera(sim)
            _ensure_camera_sensor_initialized(record_cam)
            record_cam.reset()
            dev = sim.device
            eye_l = list(args_cli.record_camera_eye_m)
            tgt_l = list(args_cli.record_camera_target_m)
            eye = torch.tensor([eye_l], device=dev, dtype=torch.float32)
            tgt = torch.tensor([tgt_l], device=dev, dtype=torch.float32)
            record_cam.set_world_poses_from_view(eye, tgt)
            _v(
                "record",
                f"camera ready: duration_sim_s={record_duration} fps={record_fps} "
                f"run_dir={record_run_dir!r} eye_m={tuple(eye_l)} target_m={tuple(tgt_l)}",
            )

        if record_enabled:
            _v("loop", f"entering sim loop until record duration {record_duration}s sim time (or app stop)")
        else:
            _v("loop", "entering infinite sim.step() loop (cancel job or walltime stops run)")
        step_i = 0
        while simulation_app.is_running():
            if step_i == 0:
                _v("loop", "about to sim.step() #0")
            sim.step()
            if step_i == 0:
                _v("loop", "sim.step() #0 returned")
                _print_all_joints(
                    robot,
                    stage="stage_1_after_sim_step_0 (one physics step)",
                    update_dt=sim.cfg.dt,
                )
                _report_cuboid_link_distance(robot, stage_label="stage_1_after_sim_step_0", update_dt=sim.cfg.dt)
            if step_i == 1:
                _v("loop", "sim.step() #1 returned (physics stepping ok)")
            step_i += 1

            sim_elapsed += sim.cfg.dt
            if (
                cuboid_displace_enabled
                and args_cli.cuboid_displace_interval_s > 0.0
                and sim_elapsed + 1e-9 >= next_displace_sim_t
            ):
                prim_path = args_cli.cuboid_centroid_prim
                r = float(args_cli.cuboid_displace_range_m)
                ox = random.uniform(-r, r)
                oy = random.uniform(-r, r)
                oz = random.uniform(-r, r)
                new_world = (
                    cuboid_anchor_world[0] + ox,
                    cuboid_anchor_world[1] + oy,
                    cuboid_anchor_world[2] + oz,
                )
                _v(
                    "cuboid_displace",
                    f"sim_t={sim_elapsed:.4f}s applying random offset_m=({ox:+.6f},{oy:+.6f},{oz:+.6f}) "
                    f"-> cuboid_root_world_target_m=({new_world[0]:.6f},{new_world[1]:.6f},{new_world[2]:.6f})",
                )
                try:
                    _set_cuboid_root_world_position_preserve_orientation(prim_path, new_world)
                except (ValueError, RuntimeError) as e:
                    _v("cuboid_displace", f"failed to set cuboid pose: {e}")
                else:
                    dist_m = _report_cuboid_link_distance(
                        robot,
                        stage_label=f"after_cuboid_displace_sim_t={sim_elapsed:.4f}s",
                        update_dt=sim.cfg.dt,
                    )
                    if dist_m is not None:
                        _v(
                            "cuboid_displace",
                            f"distance_after_displacement_m={dist_m:.6f} (euclidean cuboid centroid to link)",
                        )
                next_displace_sim_t += float(args_cli.cuboid_displace_interval_s)

            if record_cam is not None:
                dt = float(sim.cfg.dt)
                robot.update(dt)
                record_cam.update(dt)
                while sim_elapsed + 1e-9 >= next_capture_sim_t and sim_elapsed <= record_duration + 1e-6:
                    out_png = os.path.join(record_frames_dir, f"frame_{record_frame_idx:06d}.png")
                    _save_record_rgb_png(out_png, record_cam.data.output["rgb"])
                    record_frame_idx += 1
                    next_capture_sim_t += 1.0 / record_fps

            if record_cam is not None and sim_elapsed >= record_duration - 1e-6:
                _v("record", f"sim_elapsed={sim_elapsed:.4f}s reached duration {record_duration}s; exiting loop")
                break

        if record_cam is not None:
            mp4_path = os.path.join(record_run_dir, "cobot_stage.mp4")
            _v("record", f"encoding {record_frame_idx} frames with ffmpeg -> {mp4_path!r}")
            _finalize_record_mp4(record_frames_dir, record_fps, mp4_path)
    except Exception:
        _v("main", "EXCEPTION in main(); traceback follows")
        traceback.print_exc()
        raise


if __name__ == "__main__":
    _v("main", "__main__: calling main()")
    main()
    _v("main", "__main__: main() returned (unexpected for infinite loop unless app stopped)")
    simulation_app.close()
    _v("main", "simulation_app.close() done")
