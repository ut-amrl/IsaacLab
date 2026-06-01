# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
Operational Space Controller (OSC) demo for the Kinova Gen3 7-DOF arm + Robotiq 2F-85 gripper.

The arm's built-in joint-space PD gains are zeroed out so the OSC owns all torque commands.
The controller uses impedance_mode="variable_kp", meaning a different Cartesian stiffness value
is injected as part of the command on each goal switch.

Usage:
    ./isaaclab.sh -p scripts/tutorials/05_controllers/run_osc_gen3.py --num_envs 1
"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="OSC demo for Kinova Gen3.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg
from isaaclab.controllers import OperationalSpaceController, OperationalSpaceControllerCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    quat_apply_inverse,
    quat_inv,
    subtract_frame_transforms,
)

# ---------------------------------------------------------------------------
# Gen3 USD path — resolved relative to this script so no external package
# is required.
# ---------------------------------------------------------------------------
_CONTENT_DIR = Path(__file__).resolve().parent.parent.parent.parent / "content"
_GEN3_USD = str(_CONTENT_DIR / "osu_gen3_assets" / "kinova_gen3_2f85.usd")

# Inline ArticulationCfg for the Gen3 + 2F-85.
# Mirrors kinova_gen3_2f85.py but keeps the import self-contained.
# Arm stiffness/damping are zeroed here so the OSC owns all joint torque.
_GEN3_OSC_CFG = ArticulationCfg(
    prim_path="/World/envs/env_.*/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=_GEN3_USD,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint_1": 0.0,
            "joint_2": 0.523599,
            "joint_3": 0.0,
            "joint_4": 1.5708,
            "joint_5": 0.0,
            "joint_6": 0.785398,
            "joint_7": 0.0,
            "robotiq_85_left_knuckle_joint": 0.0,
            "robotiq_85_right_knuckle_joint": 0.0,
            "robotiq_85_left_finger_tip_joint": 0.0,
            "robotiq_85_right_finger_tip_joint": 0.0,
            "robotiq_85_left_inner_knuckle_joint": 0.0,
            "robotiq_85_right_inner_knuckle_joint": 0.0,
        },
    ),
    actuators={
        # Stiffness and damping are zeroed — OSC sends raw joint torques instead.
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint_[1-7]"],
            velocity_limit_sim=100.0,
            effort_limit_sim={
                "joint_[1-4]": 80.0,
                "joint_[5-7]": 20.0,
            },
            stiffness=0.0,
            damping=0.0,
        ),
        # Gripper keeps its PD gains so it holds position passively.
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[
                "robotiq_85_left_knuckle_joint",
                "robotiq_85_right_knuckle_joint",
                "robotiq_85_left_finger_tip_joint",
                "robotiq_85_right_finger_tip_joint",
                "robotiq_85_left_inner_knuckle_joint",
                "robotiq_85_right_inner_knuckle_joint",
            ],
            effort_limit_sim=10.0,
            stiffness=2000.0,
            damping=200.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = _GEN3_OSC_CFG


# ---------------------------------------------------------------------------
# State helper
# ---------------------------------------------------------------------------

def update_states(
    sim: sim_utils.SimulationContext,
    robot: Articulation,
    ee_frame_idx: int,
    arm_joint_ids: list[int],
):
    """Read the current kinematic state needed by the OSC."""
    ee_jacobi_idx = ee_frame_idx - 1  # PhysX Jacobian is offset by 1 vs body index

    jacobian_w = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, arm_joint_ids]
    mass_matrix = robot.root_physx_view.get_generalized_mass_matrices()[:, arm_joint_ids, :][:, :, arm_joint_ids]
    gravity = robot.root_physx_view.get_gravity_compensation_forces()[:, arm_joint_ids]

    # Rotate Jacobian from world frame to root (base) frame.
    root_rot_matrix = matrix_from_quat(quat_inv(robot.data.root_quat_w))
    jacobian_b = jacobian_w.clone()
    jacobian_b[:, :3, :] = torch.bmm(root_rot_matrix, jacobian_b[:, :3, :])
    jacobian_b[:, 3:, :] = torch.bmm(root_rot_matrix, jacobian_b[:, 3:, :])

    # EE pose in root frame.
    root_pos_w = robot.data.root_pos_w
    root_quat_w = robot.data.root_quat_w
    ee_pos_w = robot.data.body_pos_w[:, ee_frame_idx]
    ee_quat_w = robot.data.body_quat_w[:, ee_frame_idx]
    ee_pos_b, ee_quat_b = subtract_frame_transforms(root_pos_w, root_quat_w, ee_pos_w, ee_quat_w)
    ee_pose_b = torch.cat([ee_pos_b, ee_quat_b], dim=-1)

    # EE velocity in root frame (linear + angular).
    ee_vel_w = robot.data.body_vel_w[:, ee_frame_idx, :]
    root_vel_w = robot.data.root_vel_w
    relative_vel_w = ee_vel_w - root_vel_w
    ee_lin_vel_b = quat_apply_inverse(root_quat_w, relative_vel_w[:, :3])
    ee_ang_vel_b = quat_apply_inverse(root_quat_w, relative_vel_w[:, 3:6])
    ee_vel_b = torch.cat([ee_lin_vel_b, ee_ang_vel_b], dim=-1)

    joint_pos = robot.data.joint_pos[:, arm_joint_ids]
    joint_vel = robot.data.joint_vel[:, arm_joint_ids]

    return jacobian_b, mass_matrix, gravity, ee_pose_b, ee_vel_b, joint_pos, joint_vel


# ---------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    robot: Articulation = scene["robot"]
    sim_dt = sim.get_physics_dt()

    ee_frame_name = "gen3_end_effector_link"
    arm_joint_names = ["joint_[1-7]"]

    ee_frame_idx = robot.find_bodies(ee_frame_name)[0][0]
    arm_joint_ids = robot.find_joints(arm_joint_names)[0]

    # ------------------------------------------------------------------
    # OSC — variable_kp: command = [pose_abs(7), kp(6)] = 13 dims
    # ------------------------------------------------------------------
    osc_cfg = OperationalSpaceControllerCfg(
        target_types=["pose_abs"],
        impedance_mode="variable_kp",
        inertial_dynamics_decoupling=True,
        partial_inertial_dynamics_decoupling=False,
        gravity_compensation=False,          # gravity is disabled in the USD
        motion_damping_ratio_task=1.0,       # critically damped
        motion_control_axes_task=[1, 1, 1, 1, 1, 1],
        contact_wrench_control_axes_task=[0, 0, 0, 0, 0, 0],
        nullspace_control="position",        # keeps joints near their centre range
    )
    osc = OperationalSpaceController(osc_cfg, num_envs=scene.num_envs, device=sim.device)

    # ------------------------------------------------------------------
    # Three Cartesian waypoints in the Gen3's reachable workspace.
    # Pose format: [x, y, z, qw, qx, qy, qz] (identity orientation = EE straight ahead).
    # kp values control how stiff the Cartesian spring is for each goal.
    # ------------------------------------------------------------------
    ee_goal_pose_set_b = torch.tensor(
        [
            [0.55,  0.00, 0.35, 1.0, 0.0, 0.0, 0.0],   # forward
            [0.35,  0.35, 0.45, 1.0, 0.0, 0.0, 0.0],   # left-up
            [0.45, -0.25, 0.50, 1.0, 0.0, 0.0, 0.0],   # right-up
        ],
        device=sim.device,
    )
    kp_set_task = torch.tensor(
        [
            [200.0, 200.0, 200.0, 200.0, 200.0, 200.0],
            [300.0, 300.0, 300.0, 300.0, 300.0, 300.0],
            [150.0, 150.0, 150.0, 150.0, 150.0, 150.0],
        ],
        device=sim.device,
    )
    # Concatenate: pose_abs(7) + kp(6) = 13-dim command
    ee_target_set = torch.cat([ee_goal_pose_set_b, kp_set_task], dim=-1)

    # Visualisation markers.
    marker_cfg = FRAME_MARKER_CFG.copy()
    marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(marker_cfg.replace(prim_path="/Visuals/ee_current"))
    goal_marker = VisualizationMarkers(marker_cfg.replace(prim_path="/Visuals/ee_goal"))

    zero_arm_efforts = torch.zeros(scene.num_envs, len(arm_joint_ids), device=sim.device)
    zero_all_efforts = torch.zeros(scene.num_envs, robot.num_joints, device=sim.device)

    current_goal_idx = 0
    command = torch.zeros(scene.num_envs, osc.action_dim, device=sim.device)
    ee_target_pose_b = torch.zeros(scene.num_envs, 7, device=sim.device)

    robot.update(dt=sim_dt)
    joint_centers = torch.mean(robot.data.soft_joint_pos_limits[:, arm_joint_ids, :], dim=-1)

    count = 0
    while simulation_app.is_running():
        if count % 500 == 0:
            # Reset arm to default joint state.
            default_joint_pos = robot.data.default_joint_pos.clone()
            default_joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(default_joint_pos, default_joint_vel)
            robot.set_joint_effort_target(zero_all_efforts)
            robot.write_data_to_sim()
            robot.reset()

            robot.update(sim_dt)
            _, _, _, ee_pose_b, _, _, _ = update_states(sim, robot, ee_frame_idx, arm_joint_ids)

            # Pick next waypoint.
            command[:] = ee_target_set[current_goal_idx]
            ee_target_pose_b[:] = command[:, :7]
            current_goal_idx = (current_goal_idx + 1) % len(ee_target_set)

            osc.reset()
            osc.set_command(command=command, current_ee_pose_b=ee_pose_b)
        else:
            jacobian_b, mass_matrix, gravity, ee_pose_b, ee_vel_b, joint_pos, joint_vel = update_states(
                sim, robot, ee_frame_idx, arm_joint_ids
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
                nullspace_joint_pos_target=joint_centers,
            )
            robot.set_joint_effort_target(joint_efforts, joint_ids=arm_joint_ids)
            robot.write_data_to_sim()

        # Update markers.
        root_pos_w = robot.data.root_pos_w
        root_quat_w = robot.data.root_quat_w
        ee_pos_w = robot.data.body_pos_w[:, ee_frame_idx]
        ee_quat_w = robot.data.body_quat_w[:, ee_frame_idx]
        ee_pose_w = torch.cat([ee_pos_w, ee_quat_w], dim=-1)

        goal_pos_w, goal_quat_w = combine_frame_transforms(
            root_pos_w, root_quat_w, ee_target_pose_b[:, :3], ee_target_pose_b[:, 3:]
        )
        goal_pose_w = torch.cat([goal_pos_w, goal_quat_w], dim=-1)

        ee_marker.visualize(ee_pose_w[:, :3], ee_pose_w[:, 3:])
        goal_marker.visualize(goal_pose_w[:, :3], goal_pose_w[:, 3:])

        sim.step(render=True)
        robot.update(sim_dt)
        scene.update(sim_dt)
        count += 1


def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 2.0, 2.0], [0.0, 0.0, 0.5])

    scene_cfg = SceneCfg(num_envs=args_cli.num_envs, env_spacing=2.5)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Setup complete — running OSC on Kinova Gen3.")
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
