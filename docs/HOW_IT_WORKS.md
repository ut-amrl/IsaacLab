**How the MPC Cuboid Track Works**

Purpose: a comprehensive, textbook-style explanation of how the "MPC Cuboid Track"
experiment is implemented in this workspace and how to reproduce, extend, and
understand each component. It maps theory ⇢ implementation and gives practical
advice for debugging and tuning.

**Learning Objectives:**
- **MPC Fundamentals:** Understand Model Predictive Control (MPC) and the MPPI
  (particle) solver used here.
- **Simulation Integration:** See how IsaacLab and cuRobo interact (scene,
  actuators, sensors) and how the outer/inner control loops are wired.
- **Cost & Dynamics:** Know where cost terms, dynamics models, constraints, and
  collision checks live and how they influence behavior.
- **Reproducibility:** Run the experiment, record video, and reproduce results
  with parameter variants.
- **Tuning & Debugging:** Interpret diagnostic outputs and tune key parameters.

**Quick Repro Steps**
- Example invocation (container / workspace paths):

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py --headless \
    --robot-yml /workspace/isaaclab/content/cobot_runtime/gen3_curobo_robot.yml \
    --urdf /workspace/ros2_kortex/kortex_description/robots/gen3_2f85.urdf \
    --usd-out /workspace/isaaclab/content/cobot_runtime/cobot.usd \
    --max-sim-time-s 25
```

If you want a recorded video of the run add: `--record-cobot-video --record-duration-s 15 --record-fps 8`.

**High-level Architecture**

1. Orchestration (IsaacLab): The script [IsaacLab/scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py](IsaacLab/scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py)
   uses `isaaclab.app.AppLauncher` to create and run a simulation `SimulationContext`.
   The script: spawns a table + blue cuboid marker (or a simple cuboid-only scene),
   reads joint state observations, and calls into cuRobo's MPC solver every control
   cycle. It also optionally records video frames.

2. Outer (High-level) Controller — cuRobo `MpcSolver`:
   - Implemented in the cuRobo package at
     `external/curobo_isaac_build/src/.../curobo/wrap/reacher/mpc.py`.
   - Uses a particle-based MPPI optimizer by default (see
     `particle_mpc.yml` for default hyperparameters).
   - Exposes `MpcSolverConfig.load_from_robot_config(...)` which builds a
     `WrapMpc` optimizer and `ArmReacher` rollout model and binds world collision
     checking and cost definitions.

3. Inner (Low-level) Controller — IsaacLab actuators:
   - The script configures IsaacLab's `ImplicitActuatorCfg` to turn the
     joint-target trajectories produced by MPC into applied joint position
     targets. PhysX then applies PD-style implicit actuation (stiffness/damping).

4. Scene & Marker:
   - The example scene spawns a blue cuboid marker (see
     [IsaacLab/scripts/tutorials/00_sim/spawn_table_blue_cuboid.py](IsaacLab/scripts/tutorials/00_sim/spawn_table_blue_cuboid.py)).
   - The load script moves the marker along a four-position cycle (A→B→C→D)
     at a configurable interval. The MPC target is defined as the cuboid centroid
     (+ optional `--above-z-m` offset).

**Key Concepts and How They Map to Code**

- Model Predictive Control (MPC): at each control step, solve a finite-horizon
  optimization to compute an open-loop control sequence; apply the first slice
  (or a short window) and repeat. Implementation: `MpcSolver` using MPPI.

- MPPI (particle) optimizer: stochastic sampling of action sequences (particles),
  evaluation via rollout/dynamics model + cost, and importance-weighted update
  of the action distribution. Configured by `particle_mpc.yml` (horizon, dt,
  num_particles, n_iters, cov, step_size, etc.). See
  `external/.../curobo/content/configs/task/particle_mpc.yml` for defaults.

- Dynamics / Rollout (ArmReacher): cuRobo's `ArmReacher` simulates forward
  kinematics/kinematic dynamics for each candidate control sequence. The
  rollout must match the control frequency (`dt_traj_params.base_dt`) used by
  the controller.

- Cost terms: pose error (position + orientation), joint-space regularization,
  bound/velocity/accel penalties, collision/obstacle costs, self-collision.
  Costs reside in the `cost` section of `particle_mpc.yml` and `base_cfg.yml`.

- Collision checking: cuRobo supports multiple checker backends (mesh/obb).
  The script can pass `--mpc-collision-activation-m` to tune the distance that
  triggers collision costs (useful for soft obstacles vs hard constraints).

- State filtering and control space: `particle_mpc.yml` contains `state_filter_cfg`
  and `control_space` (here `'ACCELERATION'`) to determine action parametrization.

**Important Files & Roles**
- Orchestrator: [IsaacLab/scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py](IsaacLab/scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py) — main experiment launcher.
- Minimal scene: [IsaacLab/scripts/tutorials/00_sim/spawn_table_blue_cuboid.py](IsaacLab/scripts/tutorials/00_sim/spawn_table_blue_cuboid.py) — spawns table + visual cuboid.
- cuRobo MPC API: `external/curobo_isaac_build/src/nvidia-curobo/src/curobo/wrap/reacher/mpc.py`
- MPC defaults: `external/curobo_isaac_build/src/nvidia-curobo/src/curobo/content/configs/task/particle_mpc.yml`

**How the Script Uses cuRobo MPC (step-by-step)**
1. App/Sim setup: `AppLauncher` creates an Isaac simulation application and a
   `SimulationContext` with numeric `dt` (e.g., 0.01 s).
2. Scene spawn: table and cuboid prims are created (or simple cuboid-only if
   `--simple-cuboid-mpc`). Camera sensors optionally configured for recording.
3. Robot articulation: an `Articulation` is loaded from URDF/USD and configured
   with `ImplicitActuatorCfg` (stiffness/damping). The robot's joint order and
   mapping must match the cuRobo robot YAML used to construct the `RobotConfig`.
4. Build MPC solver: the script loads robot/world configs and calls
   `MpcSolverConfig.load_from_robot_config(...)`. This sets up `WrapMpc`, an
   `ArmReacher` rollout model, and the `ParallelMPPI` optimizer using
   `particle_mpc.yml` settings.
5. Control loop (each sim step or every N frames):
   - Read measured joint positions and optionally velocities.
   - Construct a `Goal` object describing the desired EE pose (cuboid centroid).
   - Call `MpcSolver.setup_solve_single(...)`/`solve` (API call flow in cuRobo).
   - Receive optimized action trajectory slice (joint targets) and apply via
     IsaacLab's actuator `set_joint_position_target`.
6. Logging & Diagnostics: distances, feasibility flags, costs and solver timing
   are optionally logged with `--diag-extended-csv` and `--log-interval-s`.

**Parameters You Will Likely Tune**
- `particle_mpc.yml`:
  - `model.horizon` (e.g., 30): prediction horizon (affects lookahead and compute).
  - `model.dt_traj_params.base_dt` (e.g., 0.01): must match control loop dt.
  - `mppi.num_particles` (e.g., 400): sampling budget — higher improves quality,
    increases compute time; tune with `use_cuda_graph` and GPU availability.
  - `mppi.n_iters` (e.g., 5): number of MPPI optimization iterations.
  - `cost.pose_cfg.weight` and `cost.*`: relative importance of pose vs
    joint/collision costs.

- Script CLI flags:
  - `--above-z-m`: raise target point above the cuboid centroid.
  - `--cuboid-displace-interval-s`: how often the marker moves between
    waypoints (A/B/C/D).
  - `--mpc-collision-activation-m`: radius to activate obstacle costs.
  - `--diag-extended-csv`, `--diag-every-n-steps`: collect richer diagnostics.

**Debugging Checklist**
- If the robot does not move or actions are NaN:
  - Check solver logs for NaNs or infeasible trajectories (cuRobo: solver metrics).
  - Run with `--curobo-log-level debug` to get more verbose output.

- If EE tracking is poor but solver reports feasible:
  - Validate `ImplicitActuatorCfg` stiffness/damping; PhysX may not track
    high-frequency references if gains are too low.

- If collisions occur or solver avoids goal too aggressively:
  - Inspect collision weight in `particle_mpc.yml` and `--mpc-collision-activation-m`.

- To separate solver quality vs actuator tracking problems, use
  `--diag-extended-csv` and examine `feasible`, `cost`, `solve_ms`, and `|dq_cmd|`.

**Exercises and Extensions (learning path)**
1. Reduce `num_particles` and compare solution quality vs compute time.
2. Modify `cost.pose_cfg.weight` to prioritize orientation vs position.
3. Add a mesh obstacle to the world model and tune collision activation distance.
4. Replace MPPI with the gradient/L-BFGS variant (experimental) and compare.

**Reproducibility Notes**
- The experiment relies on a matching robot YAML that maps cuRobo joint names to
  Isaac articulation joint indices: ensure `--robot-yml` points to the correct
  `gen3_curobo_robot.yml` for the loaded URDF.
- The `particle_mpc.yml` defaults are in the cuRobo package; to override create
  a copy and pass an override path into the solver build flow (or edit the
  `load_from_robot_config` call in code).

**Appendix — Key Paths**
- Orchestrator: [IsaacLab/scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py](IsaacLab/scripts/tutorials/00_sim/load_cobot_mpc_cuboid_track.py)
- Scene spawn: [IsaacLab/scripts/tutorials/00_sim/spawn_table_blue_cuboid.py](IsaacLab/scripts/tutorials/00_sim/spawn_table_blue_cuboid.py)
- cuRobo MPC wrapper: `external/curobo_isaac_build/src/nvidia-curobo/src/curobo/wrap/reacher/mpc.py`
- MPC defaults: `external/curobo_isaac_build/src/nvidia-curobo/src/curobo/content/configs/task/particle_mpc.yml`

**Where to go next**
- I can: (A) commit this `HOW_IT_WORKS.md` and update the todo list status; (B)
  expand sections with code snippets and direct line-excerpt references; or
  (C) add runnable example scripts that exercise smaller pieces (e.g., an
  isolated MPPI test harness). Tell me which you'd like next.
