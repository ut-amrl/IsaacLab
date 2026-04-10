# Copy-paste this into Isaac Sim Script Editor and run (with your stage open).
# Loads Simple_Room and SM_Mug_A2 at the transforms below. Edit the TRANSFORMS dict
# to set position (x, y, z) and rotation_rpy (roll, pitch, yaw in degrees).

from pxr import Gf, Sdf, UsdGeom

# --- Asset URLs (Isaac 5.0 content). If references fail, add assets via Content browser and use local paths. ---
SIMPLE_ROOM_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Environments/Simple_Room/simple_room.usd"
MUG_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Props/Mugs/SM_Mug_A2.usd"

# --- Transforms: position (x,y,z) m, rotation_rpy (roll, pitch, yaw) in degrees, scale (x,y,z). ---
TRANSFORMS = {
    "simple_room": {
        "prim_path": "/World/Simple_Room",
        "position": (1.35526, 0.0, 0.46491),
        "rotation_rpy": (0.0, 0.0, 90.0),  # Z=90°
        "scale": (1.0, 1.0, 1.0),
    },
    "mug": {
        "prim_path": "/World/SM_Mug_A2",
        "position": (0.6887, 0.0, 0.463),
        "rotation_rpy": (0.0, 0.0, -90.0),
        "scale": (0.01, 0.01, 0.01),
    },
}


def rpy_to_quat_f(roll_deg, pitch_deg, yaw_deg):
    # Gf.Rotation(axis, angle) uses angle in degrees
    r_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), roll_deg)
    r_y = Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch_deg)
    r_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg)
    q = (r_z * r_y * r_x).GetQuat()
    return Gf.Quatf(q.GetReal(), *q.GetImaginary())


def ensure_xform_with_reference(stage, prim_path, asset_url, position, rotation_rpy_deg, scale=(1.0, 1.0, 1.0)):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        prim = stage.DefinePrim(prim_path, "Xform")
        success = prim.GetReferences().AddReference(asset_url)
        if not success:
            print(f"[ERROR] Failed to add reference: {asset_url}")
            return
        print(f"[ADD] {prim_path} <- {asset_url}")
    else:
        print(f"[SKIP] Prim exists, applying xform only: {prim_path}")
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    q = rpy_to_quat_f(*rotation_rpy_deg)
    xform.AddOrientOp().Set(q)
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))
    print(f"[XFORM] {prim_path} pos={position} rpy={rotation_rpy_deg}° scale={scale}")


def main():
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("[ERROR] No stage. Open a stage first.")
        return
    ensure_xform_with_reference(
        stage,
        TRANSFORMS["simple_room"]["prim_path"],
        SIMPLE_ROOM_URL,
        TRANSFORMS["simple_room"]["position"],
        TRANSFORMS["simple_room"]["rotation_rpy"],
        TRANSFORMS["simple_room"]["scale"],
    )
    ensure_xform_with_reference(
        stage,
        TRANSFORMS["mug"]["prim_path"],
        MUG_URL,
        TRANSFORMS["mug"]["position"],
        TRANSFORMS["mug"]["rotation_rpy"],
        TRANSFORMS["mug"]["scale"],
    )
    print("Done.")


main()
