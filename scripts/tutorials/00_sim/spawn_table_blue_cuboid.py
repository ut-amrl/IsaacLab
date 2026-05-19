# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Minimal spawn scene: Seattle lab table + blue deformable cuboid (optional ground plane).

.. code-block:: bash

    # Usage (add --headless on clusters without a display)
    ./isaaclab.sh -p scripts/tutorials/00_sim/spawn_table_blue_cuboid.py
    ./isaaclab.sh -p scripts/tutorials/00_sim/spawn_table_blue_cuboid.py --headless

"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn table and blue cuboid only (tutorial).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


def design_scene() -> None:
    """Spawn parent Xform, ground plane, blue deformable cuboid, and Nucleus table USD."""
    sim_utils.create_prim("/World/Objects", "Xform")

    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
        size=(0.1, 0.1, 0.1),
        deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
        physics_material=sim_utils.DeformableBodyMaterialCfg(poissons_ratio=0.4, youngs_modulus=1.0e5),
    )
    cfg_cuboid_deformable.func("/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=(0.15, 0.0, 1.12))

    cfg_table = sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
    )
    cfg_table.func("/World/Objects/Table", cfg_table, translation=(0.0, 0.0, 1.05))


def main() -> None:
    """Main function."""
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 0.0, 2.5], [-0.5, 0.0, 0.5])
    design_scene()
    sim.reset()
    print("[INFO]: Setup complete (table + blue cuboid)...")

    while simulation_app.is_running():
        sim.step()


if __name__ == "__main__":
    main()
    simulation_app.close()
