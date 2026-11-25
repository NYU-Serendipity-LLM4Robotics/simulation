
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import mujoco
import viser
from mjviser import Viewer

ROOT = Path(__file__).resolve().parents[3]

XML = ROOT / (
    "third_party/unitree_ros/robots/g1_description/"
    "g1_29dof_with_hand_rev_1_0.xml"
)

# Build a fixed-body test model in memory.
# The official XML file is never modified.
tree = ET.parse(XML)
root = tree.getroot()

hand_names = {
    j.get("name")
    for j in root.iter("joint")
    if (j.get("name") or "").startswith("right_hand_")
}

assert len(hand_names) == 7

# Lock every joint except the seven right-hand joints.
for parent in root.iter():
    for child in list(parent):
        if child.tag in ("joint", "freejoint"):
            if child.get("name") not in hand_names:
                parent.remove(child)

# Remove motors that no longer have movable joints.
actuators = root.find("actuator")

for motor in list(actuators):
    if motor.get("joint") not in hand_names:
        actuators.remove(motor)

# Remove incompatible saved poses, if present.
keyframes = root.find("keyframe")
if keyframes is not None:
    root.remove(keyframes)
# Exclude artificial parent-child collisions caused by
# fixing the robot body for this hand-only experiment.

contact = root.find("contact")

if contact is None:
    contact = ET.SubElement(root, "contact")

for finger_body in [
    "right_hand_thumb_0_link",
    "right_hand_index_0_link",
    "right_hand_middle_0_link",
]:
    ET.SubElement(
        contact,
        "exclude",
        body1="right_wrist_yaw_link",
        body2=finger_body,
    )

# Make mesh paths absolute for in-memory loading.
compiler = root.find("compiler")
compiler.set("meshdir", str(XML.parent / "meshes"))

# ============================================================
# Grasp object: small cylinder
# ============================================================

worldbody = root.find("worldbody")

obj = ET.SubElement(
    worldbody,
    "body",
    name="grasp_cylinder",
)

ET.SubElement(
    obj,
    "freejoint",
    name="grasp_cylinder_free",
)

ET.SubElement(
    obj,
    "geom",
    name="grasp_cylinder_geom",
    type="cylinder",
    size="0.015 0.035",
    mass="0.05",
    friction="1.0 0.005 0.0001",
    rgba="1.0 0.4 0.1 1",
)

model = mujoco.MjModel.from_xml_string(
    ET.tostring(root, encoding="unicode")
)

# Smaller physics timestep
model.opt.timestep = 0.001

# More stable integrator
model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

# Debug only: disable gravity and contacts
model.opt.gravity[:] = 0
model.opt.disableflags &= int(
    mujoco.mjtDisableBit.mjDSBL_CONTACT
)

# Add joint damping for all 7 right-hand joints
for j in range(model.njnt):
    name = model.joint(j).name
    if name.startswith("right_hand_"):
        dof_id = model.jnt_dofadr[j]
        model.dof_damping[dof_id] = 0.05

data = mujoco.MjData(model)

hand = []

for j in range(model.njnt):
    name = model.joint(j).name
    if name not in hand_names:
        continue

    aid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
    )
    if aid < 0:
        raise RuntimeError(f"Missing actuator: {name}")

    gear = float(model.actuator_gear[aid, 0])
    if not np.isclose(gear, 1.0):
        raise RuntimeError(f"Unexpected motor gear: {name}")

    if not model.jnt_limited[j]:
        raise RuntimeError(f"Joint has no limits: {name}")

    qadr = model.jnt_qposadr[j]
    vadr = model.jnt_dofadr[j]
    lo, hi = model.jnt_range[j]

    start = lo + 0.25 * (hi - lo)
    # Smaller motion range
    moving_joints = {
        "right_hand_thumb_0_joint",
        "right_hand_thumb_1_joint",
        "right_hand_thumb_2_joint",
        "right_hand_index_0_joint",
        "right_hand_index_1_joint",
        "right_hand_middle_0_joint",
        "right_hand_middle_1_joint",
    }

    if name in moving_joints:
        goal = start + 0.12 * (hi - lo)
    else:
        goal = start

    data.qpos[qadr] = start

    hand.append((name, j, aid, qadr, vadr, start, goal))

assert len(hand) == 7

mujoco.mj_forward(model, data)

# Initial object placement relative to right wrist
wrist_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "right_wrist_yaw_link",
)

object_joint_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_JOINT,
    "grasp_cylinder_free",
)

object_qadr = model.jnt_qposadr[object_joint_id]

# Wrist coordinate frame -> world coordinate frame
wrist_pos = data.xpos[wrist_id].copy()
wrist_rot = data.xmat[wrist_id].reshape(3, 3).copy()

# Candidate position; we will calibrate it visually
local_offset = np.array([0.18, 0.03, 0.0])

object_pos = wrist_pos + wrist_rot @ local_offset

data.qpos[object_qadr:object_qadr + 3] = object_pos

# Align cylinder orientation with wrist frame
quat = np.zeros(4)
mujoco.mju_mat2Quat(quat, data.xmat[wrist_id])
data.qpos[object_qadr + 3:object_qadr + 7] = quat


# Update all body and geom positions
mujoco.mj_forward(model, data)

# Save the complete initial state.
# This prevents Viewer Reset from moving the cylinder
# back to the default world origin.
model.qpos0[:] = data.qpos

# Verify the cylinder exists in the compiled model
cylinder_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_GEOM,
    "grasp_cylinder_geom",
)

if cylinder_id < 0:
    raise RuntimeError("Cylinder was not added to MuJoCo!")

print("\n=== Cylinder Debug ===")
print("Cylinder geom ID:", cylinder_id)
print("Wrist position:", wrist_pos)
print("Local offset:", local_offset)
print("Cylinder target position:", object_pos)
print("Cylinder actual position:", data.geom_xpos[cylinder_id])
print("Cylinder size:", model.geom_size[cylinder_id])
print("Cylinder RGBA:", model.geom_rgba[cylinder_id])
print("Cylinder group:", model.geom_group[cylinder_id])
print("======================\n")


next_log_time = 0.0


def step(model, data):
    global next_log_time

    # Smooth 8-second motion cycle
    phase = 0.5 - 0.5 * np.cos(
        2.0 * np.pi * data.time / 8.0
    )

    # 1. PD control for all seven Dex3-1 joints
    for name, jid, aid, qadr, vadr, start, goal in hand:
        target = start + phase * (goal - start)

        q = data.qpos[qadr]
        dq = data.qvel[vadr]

        kp = 0.3
        kd = 0.002

        torque = kp * (target - q) - kd * dq

        low, high = -0.15, 0.15

        if model.jnt_actfrclimited[jid]:
            joint_low, joint_high = model.jnt_actfrcrange[jid]
            low = max(low, joint_low)
            high = min(high, joint_high)

        data.ctrl[aid] = np.clip(torque, low, high)

    # 2. Advance physics exactly once
    mujoco.mj_step(model, data)

    # 3. Log diagnostics every 0.5 seconds
    if data.time >= next_log_time:
        print(f"\n{'=' * 55}")
        print(f"Time: {data.time:.2f}s")

        print("\n=== Dex3-1 Joint States ===")

        for name, jid, aid, qadr, vadr, start, goal in hand:
            target = start + phase * (goal - start)

            print(
                f"{name}: "
                f"target={target:.4f}, "
                f"actual={data.qpos[qadr]:.4f}, "
                f"velocity={data.qvel[vadr]:.4f}, "
                f"torque={data.ctrl[aid]:.4f}"
            )

        # 4. Contact diagnostics
        print("\n=== Contact Diagnostics ===")
        print(f"Contacts detected: {data.ncon}")

        pairs = {}

        for i in range(data.ncon):
            c = data.contact[i]

            g1 = int(c.geom1)
            g2 = int(c.geom2)

            key = tuple(sorted((g1, g2)))

            if key not in pairs:
                pairs[key] = {
                    "count": 0,
                    "active": 0,
                    "min_dist": float("inf"),
                }

            pairs[key]["count"] += 1
            pairs[key]["active"] += int(c.efc_address >= 0)
            pairs[key]["min_dist"] = min(
                pairs[key]["min_dist"],
                float(c.dist),
            )

        # 5. Print unique collision pairs
        if not pairs:
            print("No contacts")
        else:
            for (g1, g2), info in sorted(
                pairs.items(),
                key=lambda item: item[1]["min_dist"],
            ):
                geom1 = (
                    mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_GEOM,
                        g1
                    ) or f"geom_{g1}"
                )

                geom2 = (
                    mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_GEOM,
                        g2
                    ) or f"geom_{g2}"
                )

                b1 = int(model.geom_bodyid[g1])
                b2 = int(model.geom_bodyid[g2])

                body1 = (
                    mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_BODY,
                        b1
                    ) or f"body_{b1}"
                )

                body2 = (
                    mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_BODY,
                        b2
                    ) or f"body_{b2}"
                )

                print(
                    f"\nGeom {g1} ({geom1}) "
                    f"<-> Geom {g2} ({geom2})"
                )
                print(
                    f"  Bodies: {body1} <-> {body2}"
                )
                print(
                    f"  Contact points: {info['count']}"
                )
                print(
                    f"  Active points: {info['active']}"
                )
                print(
                    f"  Deepest penetration: "
                    f"{info['min_dist']:.6f} m"
                )

        print("=" * 55, flush=True)

        next_log_time += 0.5



print("Fixed-body Dex3-1 physics test ready")
print("Active joints:", model.njnt)
print("Active actuators:", model.nu)

server = viser.ViserServer(
    host="127.0.0.1",
    port=8765,
)

Viewer(
    model,
    data,
    server=server,
    step_fn=step,
).run()
