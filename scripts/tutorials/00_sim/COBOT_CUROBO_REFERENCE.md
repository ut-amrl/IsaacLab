# Cobot URDF / USD and CuRobo interpretation

This note ties **ROS URDF** names from the Kinova Gen3 + Robotiq 2F-85 stack (as in
[`ut-amrl/ros2_kortex`](https://github.com/ut-amrl/ros2_kortex) branch
[`cobot-urdf`](https://github.com/ut-amrl/ros2_kortex/commits/cobot-urdf/)) to what you need for
**CuRobo** (world frame, base link, actuated joints, tool frame).

After **URDF → USD** with `load_cobot_stage.py` or `scripts/tools/convert_urdf.py`, articulation
joint names in Isaac Sim usually **match URDF joint names** for non-fixed joints. Verify once in
the stage with Isaac Sim’s property panel or a small script that lists `Articulation` DOFs.

## Source of truth

| Item | Location |
|------|----------|
| Upstream fork / cobot branch | [ut-amrl/ros2_kortex `cobot-urdf`](https://github.com/ut-amrl/ros2_kortex/tree/cobot-urdf) |
| Example arm + gripper URDF (stock Gen3 description) | `ros2_kortex/kortex_description/robots/gen3_2f85.urdf` (path in repo checkout) |
| Full cobot base + tower commits | Same branch history ([commits view](https://github.com/ut-amrl/ros2_kortex/commits/cobot-urdf/)); URDF layout may add `cobot_description` or merge into the Kinova xacro—use the **resolved** `.urdf` you pass to the importer |

If your checkout uses **xacro** or `${...}` placeholders, **expand to a single URDF** before Isaac’s
importer runs (same requirement as ROS `check_urdf`).

## Git LFS (Isaac Lab USD and ros2_kortex meshes)

Isaac Lab tracks large assets (including **`content/**/*.usd`**) via [Git LFS](https://git-lfs.com/)
(see **`.gitattributes`**). Until LFS objects are fetched, a **`.usd`** file on disk can be a tiny
**pointer stub** (starts with `version https://git-lfs.github.com/spec/v1`), not a real crate file.
`load_cobot_stage.py` detects that case **before** starting Kit and prints a fix-it message.

**Targeted pull (recommended)** from the Isaac Lab repo root:

```bash
git lfs install   # once per account
bash scripts/pull_kinova_usd_lfs.sh
```

That script mirrors the UT‑AMRL cobot convention (`git lfs pull` with
`--include="content/osu_gen3_assets/kinova_gen3_2f85.usd"`). For a broader download of all USD under
`content/` (large), run:

```bash
ISAACLAB_LFS_PULL_CONTENT=1 bash scripts/pull_kinova_usd_lfs.sh
```

In the full **cobot** stack (separate repo), the same pattern is documented around a vendored
`external/IsaacLab` tree: run the pull there, optionally set **`COBOT_ISAACLAB_CONTENT_DIR`** to that
`content/` tree or **`COBOT_KINOVA_USD`** to a specific materialized file, and treat pointer stubs
as unusable until `git lfs pull` completes (application code may fall back to URDF spawn when USD
is not materialized).

**`ros2_kortex`** (Kinova / forks) may also store **STL meshes** in LFS. After clone:

```bash
cd ros2_kortex && git lfs pull
```

Install the client where you clone (e.g. **`dnf install git-lfs`**, **`apt install git-lfs`**). On
shared clusters, confirm `git-lfs` is available or install per your site policy.

## Link frame table (Gen3 + 2F-85, typical `gen3_2f85.urdf`)

Use these for **CuRobo** `base_link`, `ee_link`, and obstacle / camera frames.

| URDF link | Typical role |
|-----------|----------------|
| `world` | ROS world root; often fixed to `gen3_base_link` |
| `gen3_base_link` | Arm base (good candidate for **robot base** in sim if the stage does not add another base) |
| `gen3_shoulder_link` … `gen3_bracelet_link` | Arm kinematic chain |
| `gen3_end_effector_link` | Kinova wrist/tool flange frame (common **tool0**-class frame before gripper) |
| `gen3_robotiq_85_base_link` | Gripper base mounted on wrist |
| `gen3_robotiq_85_left_finger_tip_link`, `gen3_robotiq_85_right_finger_tip_link` | Finger pads (grasp / contact) |
| `wrist_mounted_camera_*` / `wrist_camera_*` | Camera optical / sensor frames (fixed to wrist) |

CuRobo configs often set **ee_link** to a point between the fingers or the flange; pick the link
whose pose matches your calibration and motion planning convention.

## Actuated joint table (arm + gripper)

Arm (7 DOF) — map 1:1 to CuRobo’s joint list for the manipulator:

| Index (typical) | URDF joint name | Notes |
|-----------------|-----------------|--------|
| 0 | `gen3_joint_1` | Base rotation |
| 1 | `gen3_joint_2` | |
| 2 | `gen3_joint_3` | |
| 3 | `gen3_joint_4` | |
| 4 | `gen3_joint_5` | |
| 5 | `gen3_joint_6` | |
| 6 | `gen3_joint_7` | Wrist |

Robotiq 2F-85 (under-actuated mimic in hardware; URDF may expose several revolute/continuous joints):

| URDF joint name | Role |
|-----------------|------|
| `gen3_robotiq_85_left_knuckle_joint` | Finger drive (one side) |
| `gen3_robotiq_85_right_knuckle_joint` | Finger drive (other side) |
| `gen3_robotiq_85_left_inner_knuckle_joint` | Coupled motion |
| `gen3_robotiq_85_right_inner_knuckle_joint` | Coupled motion |
| `gen3_robotiq_85_left_finger_tip_joint` | Coupled motion |
| `gen3_robotiq_85_right_finger_tip_joint` | Coupled motion |

For **motion planning with a fixed parallel jaw width**, many pipelines **lock gripper joints** and
plan only the 7 arm joints. For **in-hand** or **grasp** planning, include the knuckle pair (and
respect mimic constraints in CuRobo / collision if your config models them).

## USD / Isaac vs CuRobo checklist

1. **World**: Align Isaac `World` origin with the URDF `world` → `gen3_base_link` transform you expect (fixed joint offsets from `cobot-urdf` tower/base geometry).
2. **Articulation root**: After import, confirm which prim is the articulation root and that base is fixed or floating per `--fix-base` in `load_cobot_stage.py`.
3. **Joint order**: CuRobo expects a consistent vector `q`; read joint order from the USD articulation or from your `RobotCfg` — do not assume it matches table index without checking.
4. **Collision meshes**: CuRobo needs collision geometry compatible with its world model; URDF collision tags drive Isaac’s collision; simplify meshes if convex decomposition is required elsewhere.

## Apptainer smoke jobs (Delta) and editable cuRobo

If you installed cuRobo with **pip editable** into Isaac Sim’s Python, the checkout usually lives under
**`${COBOT2_ROOT}/external/curobo_isaac_build`** and the `.pth` points at **`/mnt/curobo_build/...`** inside the container.
Cluster smoke scripts (**`docker/cluster/smoke_sim_*.slurm`**, **`srun_verify_headless_smoke.sh`**) add:

- **`-B "${COBOT2_ROOT}/external/curobo_isaac_build:/mnt/curobo_build"`**
- **`-B "${HOME}/.local:${HOME}/.local"`** so the user-site editable metadata is visible
- **`unset APPTAINERENV_PYTHONNOUSERSITE`** before `apptainer exec`, because `smoke_delta_holosoma_env.sh` sets
  **`APPTAINERENV_PYTHONNOUSERSITE=1`**, which would otherwise hide **`~/.local`** and break **`import curobo`**.

Match these binds in any custom **`apptainer exec`** that should load **`isaaclab_mimic`** / CuRobo.

## cuRobo ``build_robot_model`` (Gen3 arm)

To generate collision spheres + self-collision YAML for the **Kinova Gen3** (arm-only slice of ``gen3_2f85.urdf``, meshes via ``package://kortex_description/``):

1. **Prepare URDF** (rewrites bad ``file://`` prefixes, drops Robotiq + wrist camera so only ``kortex_description`` is required):

   ``python3 scripts/tutorials/00_sim/curobo_prepare_gen3_urdf.py --input /path/to/ros2_kortex/kortex_description/robots/gen3_2f85.urdf --output .../gen3_curobo_input.urdf``

2. **Run the tutorial** (from Isaac Lab repo root; use Kit Python inside Apptainer with partial binds so ``_isaac_sim`` is visible):

   ``bash scripts/tutorials/00_sim/build_curobo_gen3_model.sh``

   Or submit: ``cd docker/cluster && sbatch build_curobo_gen3_model.slurm``

   Output defaults to ``content/cobot_runtime/gen3_curobo_robot.yml``. Optional: ``VISUALIZE=1`` for Viser; ``FULL_ROBOT=1`` only after cloning **``robotiq_description``** under ``${COBOT2_ROOT}/external/robotiq_description`` (see script comments).

## Commands

- Convert and load in one process: `scripts/tutorials/00_sim/load_cobot_stage.py` with `--urdf` and `--usd-out`.
- Convert only: `scripts/tools/convert_urdf.py <urdf> <out.usd>` (same importer options).
