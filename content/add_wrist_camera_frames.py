# Copy-paste this into Isaac Sim Script Editor (run in the instance where your stage is already open).
# Adds camera_link, camera_depth_frame, camera_color_frame under the arm end-effector (from gen3_macro.xacro vision block).

from pxr import Gf, Usd, UsdGeom

def rpy_to_quat_f(roll, pitch, yaw):
    r_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), roll)
    r_y = Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch)
    r_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw)
    q = (r_z * r_y * r_x).GetQuat()  # Gf.Quatd
    return Gf.Quatf(q.GetReal(), *q.GetImaginary())  # Isaac expects GfQuatf

parent_path = "/World/gen3n7_instanceable/end_effector_link"
stage = omni.usd.get_context().get_stage()

PI = 3.1415926535897931
frames = [
    ("camera_link", (0.0, 0.05639, -0.00305), (PI, PI, 0)),
    ("camera_depth_frame", (0.0275, 0.066, -0.00305), (PI, PI, 0)),
    ("camera_color_frame", (0.0, 0.05639, -0.00305), (PI, PI, 0)),
]

# frames = [
#     ("camera_link", (0.0, 0.05639, -0.00305), (0, 0, -PI)),
#     ("camera_depth_frame", (0.0275, 0.066, -0.00305), (0, 0, -PI)),
#     ("camera_color_frame", (0.0, 0.05639, -0.00305), (0, 0, -PI)),
# ]

for name, xyz, rpy in frames:
    prim_path = f"{parent_path}/{name}"
    if stage.GetPrimAtPath(prim_path).IsValid():
        print(f"[SKIP] Exists: {prim_path}")
        continue
    prim = stage.DefinePrim(prim_path, "Xform")
    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    xform.AddOrientOp().Set(rpy_to_quat_f(*rpy))
    print(f"[ADD] {prim_path}")

print("Done.")
