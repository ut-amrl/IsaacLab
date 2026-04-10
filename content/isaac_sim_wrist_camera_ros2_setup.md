# Isaac Sim Wrist Camera → ROS2 (Action Graph)

This guide configures the Omni Action Graph so the wrist camera publishes exactly what the **classical perception pipeline** expects:

| Pipeline expects     | Topic               | Message type   |
|----------------------|---------------------|----------------|
| RGB image            | `/rgb/image_raw`    | `sensor_msgs/Image` |
| RGB camera intrinsics| `/rgb/camera_info`  | `sensor_msgs/CameraInfo` |
| Depth image          | `/depth/image_raw`  | `sensor_msgs/Image` |
| Depth camera intrinsics | `/depth/camera_info` | `sensor_msgs/CameraInfo` |

---

## 1. Topic naming in Isaac Sim

- **ROS2 Camera Helper** “Topic Name” is the **base** name. Isaac Sim typically publishes:
  - Image → `{Topic Name}/image_raw` (e.g. `rgb` → `/rgb/image_raw`)
  - So set **Topic Name** = `rgb` for the RGB helper and `depth` for the depth helper.
- **ROS2 Camera Info Helper** has its own “Topic Name” (default `camera_info`). Set it to `rgb/camera_info` or `depth/camera_info` so the full topic is `/rgb/camera_info` or `/depth/camera_info`.

Use the inspection script from the ROS2 container to confirm what is actually published:

```bash
ros2 run openpi_kinova_ros2 inspect_isaac_camera_topics.py --list
ros2 run openpi_kinova_ros2 inspect_isaac_camera_topics.py
```

---

## 2. Your current RGB setup

You already have:

- **On Playback Tick** → **Isaac Run One Simulation Frame** → **Isaac Create Render Product** (Camera Prim = wrist color camera) → **ROS2 Camera Helper**.

To match the pipeline:

- On **ROS2 Camera Helper** set:
  - **Topic Name** = `rgb` (so the image goes to `/rgb/image_raw`).
  - **Type** = `rgb` (or equivalent for color image).
  - **Frame Id** = your wrist camera optical frame, e.g. `wrist_camera_color_optical_frame` or `camera_color_optical_frame` (must match the frame you use in TF for the wrist camera).

If the image is correct but the topic is wrong (e.g. you see `/rgb` only), either:

- Change **Topic Name** to `rgb` and rely on the helper’s `/image_raw` suffix, or  
- Use the **ROS2 bridge** (see below) to subscribe to whatever topic Isaac publishes and republish to `/rgb/image_raw`.

---

## 3. Add camera_info (required for perception)

The pipeline needs camera intrinsics for back-projecting pixels to 3D.

1. Add a **ROS2 Camera Info Helper** node (search “Camera Info” in the Action Graph).
2. **Inputs:**
   - **Exec In**: same execution flow (e.g. from **Isaac Create Render Product** `Exec Out` or from the same **ROS2 Camera Helper** `Exec Out` if you chain them).
   - **Context**: from **ROS2 Context** (same as your Camera Helper).
   - **Render Product Path**: **same** as the RGB **Isaac Create Render Product** → `Render Product Path` (same render product can drive both image and camera_info).
   - **Topic Name** = `rgb/camera_info` → publishes to `/rgb/camera_info`.
   - **Frame Id** = same as the RGB Camera Helper (e.g. `wrist_camera_color_optical_frame`).
3. Connect **Exec Out** of the existing **ROS2 Camera Helper** to **Exec In** of **ROS2 Camera Info Helper**, or run both from the same **Isaac Create Render Product** if the graph allows.

After pressing Play, check:

```bash
ros2 topic echo /rgb/camera_info --once
```

---

## 4. Add depth (optional for pipeline)

For depth you need a **depth render product** and a separate helper.

1. **Second render product (depth)**  
   - Duplicate your existing **Isaac Create Render Product** (or add another one).  
   - **Camera Prim**: same wrist camera prim (many camera prims in Isaac can output both color and depth).  
   - Set the **output** to depth (in the Create Render Product or the camera prim, depending on Isaac version: e.g. “Depth” or “Linear Depth” pass).  
   - This gives a second **Render Product Path** (e.g. for depth).

2. **ROS2 Camera Helper for depth**  
   - Add another **ROS2 Camera Helper**.  
   - **Render Product Path** = the **depth** render product path.  
   - **Topic Name** = `depth` → `/depth/image_raw`.  
   - **Type** = `depth` (or equivalent).  
   - **Frame Id** = depth optical frame (e.g. `wrist_camera_depth_optical_frame`), consistent with your TF.

3. **ROS2 Camera Info Helper for depth**  
   - Add a **ROS2 Camera Info Helper** for depth.  
   - **Render Product Path** = same depth render product.  
   - **Topic Name** = `depth/camera_info` → `/depth/camera_info`.  
   - **Frame Id** = same as the depth Camera Helper.

If your wrist camera prim does not expose a separate depth pass, you may need to add a second camera (depth) under the same link and create a render product from that.

---

## 5. Fixing / improving RGB data

- **Resolution**: set **Height** and **Width** on **Isaac Create Render Product** (e.g. 640×480 or 960×540).  
- **Frame rate**: **Frame Skip Count** on the Camera Helper (0 = every frame).  
- **Timing**: if you use sim time, run MoveIt and the perception pipeline with `use_sim_time:=true` so TF and message timestamps align.  
- **Encoding**: keep **Type** = `rgb` so the helper publishes a standard color encoding (e.g. `bgr8` or `rgb8`); the pipeline’s `cv_bridge` expects a common encoding.

---

## 6. Alternative: ROS2 bridge

If you prefer not to change topic names in the Action Graph, use the existing **isaac_camera_bridge** and extend it:

- It can subscribe to whatever Isaac publishes (e.g. `/rgb` or `/wrist_camera`) and republish to `/rgb/image_raw` (and optionally `/wrist_camera/color/image_raw`).
- You can add parameters for:
  - `isaac_rgb_topic` (e.g. `/rgb` or `/rgb/image_raw`),
  - `isaac_rgb_camera_info_topic` (if you add Camera Info Helper in the graph, e.g. `/rgb/camera_info`),
  - and the same for depth, then republish to `/depth/image_raw` and `/depth/camera_info`.

Then the Action Graph can keep generic names (e.g. `rgb`, `camera_info`) and the bridge normalizes to the pipeline’s topic names.

---

## 7. Quick checks

In the ROS2 container (with Isaac Sim running and graph playing):

```bash
# List camera-related topics
ros2 run openpi_kinova_ros2 inspect_isaac_camera_topics.py --list

# Inspect rate and one message for RGB
ros2 run openpi_kinova_ros2 inspect_isaac_camera_topics.py --topic /rgb/image_raw

# If you added camera_info
ros2 topic echo /rgb/camera_info --once
```

Confirm:

- `/rgb/image_raw` (and optionally `/depth/image_raw`) exist and have sensible width/height and frame_id.  
- `/rgb/camera_info` (and optionally `/depth/camera_info`) exist and have correct `K` and the same frame_id as the images.  
- TF contains the camera optical frame → `base_link` (or your planning frame) so the perception pipeline can transform 3D points into the robot base.

---

## 8. TF: Isaac publishes one link only

The **ROS2 Publish Transform Tree** (or single-transform) node in Isaac Sim publishes **only the transform from the parent prim to the target prim** — one link (e.g. `gen3n7_instanceable` → `Camera`). It does **not** publish the full chain (base → link1 → … → end_effector → camera). So you see a single transform on `/tf`.

**Perception** needs a path from the camera frame to **`base_link`** (the pipeline uses `target_frame="base_link"`). Right now you have `gen3n7_instanceable` → `Camera`. So either:

- **Option A:** In the Isaac node, if there is a **parent frame id** (or similar) input, set it to **`base_link`** so the published transform is `base_link` → `Camera`. Then perception can look up Camera → base_link.
- **Option B:** Keep `gen3n7_instanceable` → `Camera` and add an identity link so `base_link` exists: run a static transform from `base_link` to `gen3n7_instanceable` (same position/orientation), so the chain is Camera → gen3n7_instanceable → base_link:
  ```bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link gen3n7_instanceable
  ```
  Use `use_sim_time:=true` if the rest of your stack uses sim time.

**MoveIt** needs the **full robot chain** (base_link → link1 → … → end_effector_link) on TF. Isaac does not publish that when you only add one transform node. So you should run **robot_state_publisher** in ROS2 with the same Kinova URDF that MoveIt uses, subscribing to **`/joint_states_urdf`** (from the trajectory bridge). That will publish the full robot tree. Start it in the same launch as MoveIt or in a separate terminal, with `use_sim_time:=true` when using Isaac Sim.

---

## 9. "Timestamp is invalid. Timestamp 0 will be neglected" (flooding Isaac Sim)

**What it means:** The ROS2 transform (and possibly other) nodes are publishing messages with **header.stamp = 0**. ROS 2 treats timestamp 0 as invalid, so downstream nodes (and Isaac’s own bridge) warn that those messages will be neglected.

**Why it happens:** The **ROS2 Publish Transform Tree** (or Publish Raw Transform Tree) node has a **Timestamp** input that defaults to **0**. If it’s not driven, every published transform uses stamp 0 and you get the warning every frame.

**Fix in the Action Graph:**

1. **Feed simulation time into the transform node**  
   - Add an **Isaac Read Simulation Time** node (triggered by **On Playback Tick** or your main exec flow).  
   - Connect its **simulation time** output (seconds) to the **Timestamp** (or **timeStamp**) input of your **ROS2 Publish Transform Tree** (or **ROS2 Publish Raw Transform Tree**) node.  
   - Transforms will then be published with the current simulation time and the warning should stop.

2. **Publish `/clock` (recommended)**  
   - Add **ROS2 Publish Clock**.  
   - Connect **Isaac Read Simulation Time** → **timeStamp** (or **time**) of **ROS2 Publish Clock**.  
   - Connect **ROS2 Context** to **ROS2 Publish Clock**.  
   - This publishes simulation time on `/clock` so ROS 2 nodes using `use_sim_time:=true` (MoveIt, bridge, etc.) stay in sync with sim and don’t reject the stamped TF.
