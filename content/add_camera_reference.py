# Copy-paste this into Isaac Sim Script Editor (run with your stage open).
# Adds a Camera prim by referencing *only the Camera prim* from a USD file, so the rest of
# that file's world is not pulled in. The Camera is placed under your existing camera_color_frame.

from pxr import Sdf

stage = omni.usd.get_context().get_stage()

# Where to create the Camera in *your* stage (under your existing frame).
camera_prim_path = "/World/gen3n7_instanceable/end_effector_link/camera_color_frame/Camera"
# Path of the Camera prim *inside* the referenced USD file (same hierarchy as in that file).
referenced_prim_path = "/World/kinova_arm/bracelet_link/end_effector_link/camera_color_frame/Camera"
camera_usd_path = "/workspace/isaaclab/content/old/stage-19.usd"

if stage.GetPrimAtPath(camera_prim_path).IsValid():
    print(f"[SKIP] Exists: {camera_prim_path}")
else:
    prim = stage.DefinePrim(camera_prim_path, "Camera")
    success = prim.GetReferences().AddReference(camera_usd_path, primPath=Sdf.Path(referenced_prim_path))
    if not success:
        print(f"[ERROR] Failed to add reference to {referenced_prim_path} from {camera_usd_path}")
    else:
        print(f"[ADD] {camera_prim_path} <- {camera_usd_path}</{referenced_prim_path}>")

print("Done.")
