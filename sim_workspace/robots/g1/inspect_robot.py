from pathlib import Path
import mujoco

root = Path(__file__).resolve().parents[3]
xml_path = (
    root / "third_party/unitree_mujoco"
    / "unitree_robots/g1/scene.xml"
)

model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

print("=== Unitree G1 ===")
print("Joints:", model.njnt)
print("Actuators:", model.nu)
print("Generalized coordinates:", model.nq)

print("\n=== Right Arm ===")
for i in range(model.njnt):
    name = mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_JOINT, i
    )
    if name and name.startswith("right_") and any(
        part in name for part in ("shoulder", "elbow", "wrist")
    ):
        print(name, "limits:", model.jnt_range[i])

print("\n=== Right Arm Actuators ===")
for i in range(model.nu):
    name = mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
    )
    if name and name.startswith("right_") and any(
        part in name for part in ("shoulder", "elbow", "wrist")
    ):
        print(name, "control range:", model.actuator_ctrlrange[i])

print("\n=== Right Wrist Position ===")
body_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "right_wrist_yaw_link"
)
print("World position:", data.xpos[body_id])
