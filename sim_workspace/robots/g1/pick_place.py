"""Fixed-base G1/Dex3-1 contact benchmark using the Unitree collision meshes."""
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from kinematics import PositionIK
from robots.g1.manipulation import MODEL_XML

ARM = tuple(f'right_{stem}_joint' for stem in (
    'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
    'wrist_roll', 'wrist_pitch', 'wrist_yaw',
))
FINGERS = tuple(f'right_hand_{stem}_joint' for stem in (
    'thumb_0', 'thumb_1', 'thumb_2', 'middle_0', 'middle_1', 'index_0', 'index_1',
))
WRIST = 'right_wrist_yaw_link'
RADIUS = 0.023
LEGACY_PROFILE = 'dex3-contact-v1'
PREVIOUS_CONTACT_PROFILE = 'dex3-contact-v2'
CONTACT_PROFILE = 'dex3-contact-v3'
GRAVITY_M_S2 = (0.0, 0.0, -9.81)
CONTACT_TIMESTEP_S = 0.0005
CONTACT_SOLREF = (0.0025, 1.0)
CONTACT_SOLIMP = (0.97, 0.995, 0.0004, 0.5, 2.0)
HAND_FRICTION = (0.95, 0.003, 0.0001)
RECEIVER_FRICTION = (0.7, 0.008, 0.003)
BASE_SLIDING_FRICTION = 0.55
FINGER_TORQUE_LIMIT_NM = 0.85
MAX_PENETRATION_M = 0.0005
TRACE_INTERVAL_S = 0.02


def make_scene(source, destination, *, profile=CONTACT_PROFILE):
    tree = ET.parse(MODEL_XML)
    root = tree.getroot()
    root.find('compiler').set('meshdir', str(MODEL_XML.parent / 'meshes'))
    for tag in ('keyframe', 'sensor', 'actuator', 'contact', 'equality', 'option'):
        section = root.find(tag)
        if section is not None:
            root.remove(section)
    ET.SubElement(root, 'option', gravity=' '.join(map(str, GRAVITY_M_S2)), timestep='0.001', integrator='implicitfast',
                  cone='elliptic', impratio='20', iterations='75')
    world = root.find('worldbody')
    limits = {}
    for body in world.iter('body'):
        # Keep archived profiles reproducible. In v3 the servos bear the arm's weight.
        body.set('gravcomp', '0' if profile == CONTACT_PROFILE else '1')
        for joint in list(body):
            if joint.tag in ('joint', 'freejoint'):
                name = joint.get('name')
                if name in ARM + FINGERS:
                    limits[name] = joint.get('range')
                    joint.set('armature', '0.025' if name in ARM else '0.0003')
                    joint.set('damping', '0.3' if name in ARM else '0.03')
                else:
                    body.remove(joint)
    # The legacy profile uses robot/world contacts only. The current profile adds finger self-contact.
    for geom in world.iter('geom'):
        if geom.get('name') != 'floor' and geom.get('contype', '1') != '0':
            geom.set('contype', '2')
            geom.set('conaffinity', '1')
            geom.set('friction', '1.4 0.01 0.001')
            geom.set('condim', '4')
    actuators = ET.SubElement(root, 'actuator')
    for name in ARM + FINGERS:
        arm = name in ARM
        ET.SubElement(actuators, 'position', name=name, joint=name, ctrlrange=limits[name],
                      kp='500' if arm else '32', kv='22' if arm else '0.32',
                      forcerange='-35 35' if arm else '-1.6 1.6')
    for name, center in (('source_support', source), ('destination_support', destination)):
        center = np.asarray(center) - [0, 0, RADIUS + 0.012]
        is_source = name == 'source_support'
        ET.SubElement(world, 'geom', name=name, type='cylinder' if is_source else 'box',
                      size='0.013 0.012' if is_source else '0.057 0.048 0.012',
                      pos=' '.join(map(str, center)), rgba='0.35 0.55 0.42 1',
                      contype='1', conaffinity='3', condim='4' if is_source else '6',
                      friction='0.8 0.01 0.001' if is_source else '0.8 0.01 0.003')
        if profile == CONTACT_PROFILE and center[2] > 0.012:
            half_height = (center[2] - 0.012) / 2
            ET.SubElement(world, 'geom', name=f'{name}_post', type='cylinder',
                          size=f'0.009 {half_height}',
                          pos=f'{center[0]} {center[1]} {half_height}',
                          rgba='0.26 0.31 0.32 1', contype='1', conaffinity='3',
                          friction='0.55 0.004 0.0001')
    # The receiver uses sloped sides. The current profile enables robot contact on the ramp geoms.
    for index, angle in enumerate(np.arange(12) * np.pi / 6):
        radial = np.array([np.cos(angle), np.sin(angle), 0.0])
        center = np.asarray(destination) + 0.058 * radial - [0, 0, RADIUS - 0.007]
        yaw = np.array([np.cos(angle / 2), 0, 0, np.sin(angle / 2)])
        pitch = np.array([np.cos(0.24), 0, -np.sin(0.24), 0])
        rotation = np.empty(4)
        mj.mju_mulQuat(rotation, yaw, pitch)
        ET.SubElement(world, 'geom', name=f'destination_ramp_{index}', type='box', size='0.032 0.023 0.005',
                      pos=' '.join(map(str, center)), quat=' '.join(map(str, rotation)),
                      rgba='0.27 0.49 0.36 1', contype='4', conaffinity='1', friction='0.55 0.004 0.0001')
    ET.SubElement(world, 'light', pos='1 -1 2', dir='-0.5 0.5 -1')
    obj = ET.SubElement(world, 'body', name='workpiece', pos=' '.join(map(str, source)))
    ET.SubElement(obj, 'freejoint', name='workpiece_free')
    ET.SubElement(obj, 'geom', name='workpiece', type='sphere', size=str(RADIUS), mass='0.028',
                  friction='1.25 0.012 0.001', condim='4', contype='1', conaffinity='3', rgba='0.86 0.22 0.12 1')
    model = mj.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    if profile in (PREVIOUS_CONTACT_PROFILE, CONTACT_PROFILE):
        configure_contact_model(model)
    elif profile != LEGACY_PROFILE:
        raise ValueError(f'unknown fixed-fixture collision profile: {profile}')
    data = mj.MjData(model)
    return model, data


def configure_contact_model(model):
    """Apply the fixed-fixture contact profile to collidable geoms only."""
    model.opt.timestep = CONTACT_TIMESTEP_S
    model.opt.iterations = 150
    collidable = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
    model.geom_solref[collidable] = CONTACT_SOLREF
    model.geom_solimp[collidable] = CONTACT_SOLIMP
    model.geom_friction[collidable, 0] = BASE_SLIDING_FRICTION
    for geom in np.flatnonzero(collidable):
        name = model.geom(geom).name
        body = model.body(model.geom_bodyid[geom]).name
        if body.startswith('right_hand_'):
            # Bit 8 enables finger-finger collisions without enabling full-body self-contact.
            model.geom_contype[geom] |= 8
            model.geom_conaffinity[geom] |= 8
            model.geom_friction[geom] = HAND_FRICTION
        if name == 'destination_support':
            model.geom_friction[geom] = RECEIVER_FRICTION
        if name.startswith('destination_ramp_'):
            model.geom_conaffinity[geom] |= 2
    for name in FINGERS:
        model.actuator_forcerange[model.actuator(name).id] = [-FINGER_TORQUE_LIMIT_NM, FINGER_TORQUE_LIMIT_NM]


def joint_addresses(model, names):
    return np.array([int(model.joint(name).qposadr[0]) for name in names])

# Grasp calibration in the stock right wrist frame against a 46 mm sphere.
TOOL_OFFSET = (0.131693, 0.035081, 0.001430)
CLOSED = (-0.296695, -0.825995, -0.158903, 0.715306, 0.364739, 0.554417, 0.361386)
SOURCE = (0.3315, -0.1136, 0.890)
DESTINATION = (0.332, -0.310, 0.890)


def contact_summary(model, data, object_id, *, prefix=''):
    fingers = {}
    supports = set()
    force = np.empty(6)
    for i, contact in enumerate(data.contact):
        if object_id not in contact.geom or contact.efc_address < 0:
            continue
        other = int(contact.geom[1] if contact.geom[0] == object_id else contact.geom[0])
        mj.mj_contactForce(model, data, i, force)
        if force[0] < 0.015:
            continue
        body = model.body(model.geom_bodyid[other]).name
        if body.startswith(prefix + 'right_hand_'):
            digit = body.removeprefix(prefix).split('_')[2]
            fingers[digit] = fingers.get(digit, 0.0) + float(force[0])
        elif model.geom(other).name:
            supports.add(model.geom(other).name)
    return fingers, sorted(supports)


def measure_penetration(model, data, object_id, hand_geoms, stage, peaks):
    """Track signed-distance violations at each contact solve, independent of trace sampling."""
    distances = {}
    for contact in data.contact:
        first, second = map(int, contact.geom)
        depth = -float(contact.dist)
        if depth <= 0:
            continue
        in_hand = (first in hand_geoms, second in hand_geoms)
        if object_id in (first, second):
            kind = 'ball_hand' if any(in_hand) else 'ball_environment'
        elif all(in_hand):
            kind = 'hand_self'
        elif any(in_hand) and 0 in (model.geom_bodyid[first], model.geom_bodyid[second]):
            kind = 'hand_environment'
        else:
            continue
        distances[kind] = max(depth, distances.get(kind, 0.0))
        if depth > peaks.get(kind, {}).get('depth_m', 0.0):
            names = [model.geom(g).name or model.body(model.geom_bodyid[g]).name for g in (first, second)]
            peaks[kind] = {'depth_m': depth, 'stage': stage, 'time_s': float(data.time), 'geoms': names}
    return distances


def run_pick_place(source, destination, output, record, *, profile=CONTACT_PROFILE):
    """Apply actuator commands to a fixed base and verify independent object dynamics."""
    source, destination = np.asarray(source, dtype=float), np.asarray(destination, dtype=float)
    if source.shape != (3,) or destination.shape != (3,) or not np.isfinite([source, destination]).all():
        raise ValueError('fixture coordinates must contain three finite XYZ coordinates in meters')
    model, data = make_scene(source, destination, profile=profile)
    planner = mj.MjData(model)
    arm = PositionIK(model, WRIST, ARM, TOOL_OFFSET)
    fingers = joint_addresses(model, FINGERS)
    arm_motors = [model.actuator(name).id for name in ARM]
    finger_motors = [model.actuator(name).id for name in FINGERS]
    object_id = model.geom('workpiece').id
    hand_geoms = {g for g in range(model.ngeom) if model.geom_contype[g] and (
        model.body(model.geom_bodyid[g]).name.startswith('right_hand_')
        or model.body(model.geom_bodyid[g]).name == WRIST
    )}
    sample_steps = round(TRACE_INTERVAL_S / model.opt.timestep)
    penetration_peaks = {}
    record({'stage': 'contact_model', 'profile': profile, 'timestep_s': float(model.opt.timestep),
            'gravity_m_s2': model.opt.gravity.tolist(),
            'gravity_compensated_body_count': int(np.count_nonzero(model.body_gravcomp)), 'max_allowed_penetration_m': MAX_PENETRATION_M})
    object_joint = model.joint('workpiece_free').id
    object_velocity = model.jnt_dofadr[object_joint]
    opened = np.zeros(len(FINGERS))
    closed = np.asarray(CLOSED)
    wrist_rotation = np.eye(3)
    wrist_roll = 0.0
    release_roll = -1.57
    pregrasp = source + [-0.048, 0, 0]
    lifted = source + [0, 0, 0.095]
    above = destination + [0, 0, 0.095]
    release = destination + [0, 0, 0.036]
    retreat = release + [-0.045, -0.06, 0.025]
    start = arm.solve(planner, pregrasp, orientation=wrist_rotation, tolerance=0.0006)
    snapshots, trace = [], []
    status = 'ik_failed'
    baseline_z = source[2]
    evidence = {'lift_height_m': 0.0, 'opposing_contact_steps_while_lifted': 0,
                'lowest_traverse_height_m': None, 'settled_destination_seconds': 0.0}
    if start.converged:
        data.qpos[arm.qpos] = planner.qpos[arm.qpos]
        data.qpos[fingers] = opened
        mj.mj_forward(model, data)
        status = 'placed'
    else:
        record({'stage': 'pregrasp', 'error_m': start.error_m, 'converged': False})
    snapshots.append(data.qpos.copy())
    for name, point, target_fingers, seconds in (
        ('settle_source', pregrasp, opened, 0.45),
        ('approach', source, opened, 1.1),
        ('close', source, closed, 1.25),
        ('lift', lifted, closed, 1.35),
        ('transfer', above, closed, 1.35),
        ('orient_for_release', above, closed, 1.35),
        ('lower', release, closed, 1.25),
        ('release', release, opened, 1.1),
        ('retreat', retreat, opened, 1.0),
        ('settle_destination', retreat, opened, 2.25),
    ):
        if status != 'placed':
            break
        first_point = arm.point(planner).copy()
        first_fingers = data.ctrl[finger_motors].copy()
        first_roll = wrist_roll
        goal_roll = release_roll if name == 'orient_for_release' else first_roll
        force_peak, opposition = 0.0, 0
        minimum_z = float('inf')
        settled_steps = 0
        for t in range(round(seconds / model.opt.timestep)):
            ratio = min(1.0, (t + 1) * model.opt.timestep / (seconds * 0.8))
            blend = ratio * ratio * (3 - 2 * ratio)
            if t % 10 == 0:
                wrist_roll = first_roll + blend * (goal_roll - first_roll)
                c, s = np.cos(wrist_roll), np.sin(wrist_roll)
                wrist_rotation = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
                ik = arm.solve(planner, first_point + blend * (point - first_point),
                               orientation=wrist_rotation, tolerance=0.00025)
                if not ik.converged:
                    status = 'ik_failed'
                    record({'stage': name, 'converged': False, 'error_m': ik.error_m})
                    break
            data.ctrl[arm_motors] = planner.qpos[arm.qpos]
            data.ctrl[finger_motors] = first_fingers + blend * (target_fingers - first_fingers)
            mj.mj_step(model, data)
            xyz = data.geom_xpos[object_id].copy()
            forces, supports = contact_summary(model, data, object_id)
            overlap = measure_penetration(model, data, object_id, hand_geoms, name, penetration_peaks)
            opposing = 'thumb' in forces and any(f in forces for f in ('index', 'middle'))
            force_peak = max(force_peak, sum(forces.values()))
            opposition += int(opposing)
            minimum_z = min(minimum_z, float(xyz[2]))
            evidence['lift_height_m'] = max(evidence['lift_height_m'], float(xyz[2] - baseline_z))
            if opposing and not supports and xyz[2] - baseline_z > 0.045 and name in ('lift', 'transfer'):
                evidence['opposing_contact_steps_while_lifted'] += 1
            settled = (any(support.startswith('destination_') for support in supports) and not forces
                       and np.linalg.norm(data.qvel[object_velocity:object_velocity + 3]) < 0.015)
            settled_steps = settled_steps + 1 if settled else 0
            if t % sample_steps == 0:
                snapshots.append(data.qpos.copy())
                trace.append((float(data.time), name, xyz.tolist(), forces, supports,
                              data.ctrl.copy().tolist(), overlap))
        if status != 'placed':
            break
        if name in ('transfer', 'orient_for_release'):
            previous_height = evidence['lowest_traverse_height_m']
            height = float(minimum_z - baseline_z)
            evidence['lowest_traverse_height_m'] = height if previous_height is None else min(previous_height, height)
        if name == 'settle_destination':
            evidence['settled_destination_seconds'] = settled_steps * model.opt.timestep
        record({'stage': name, 'object_position_m': xyz.tolist(), 'end_finger_forces_n': forces,
                'supports': supports, 'peak_finger_normal_force_n': force_peak,
                'opposing_contact_steps': opposition,
                'tool_tracking_error_m': float(np.linalg.norm(arm.point(data) - point))})
        if name == 'close' and opposition < 100:
            status = 'no_opposing_grasp'
        elif name == 'lift' and (evidence['lift_height_m'] < 0.06
                                 or evidence['opposing_contact_steps_while_lifted'] < 100):
            status = 'lift_failed'
        elif name in ('transfer', 'orient_for_release') and evidence['lowest_traverse_height_m'] < 0.04:
            status = 'dropped_in_transfer'
    final = data.geom_xpos[object_id].copy()
    xy_error = float(np.linalg.norm(final[:2] - destination[:2]))
    if status == 'placed' and (evidence['settled_destination_seconds'] < 0.25 or xy_error > 0.035):
        status = 'placement_failed'
    worst = max((item['depth_m'] for item in penetration_peaks.values()), default=0.0)
    if worst > MAX_PENETRATION_M:
        record({'stage': 'penetration_check', 'converged': False, 'contacts': penetration_peaks})
        if status == 'placed':
            status = 'excess_penetration'
    evidence.update(final_xyz_m=final.tolist(), destination_xy_error_m=xy_error,
                    max_penetration_m=worst, penetration_peaks=penetration_peaks,
                    penetration_limit_m=MAX_PENETRATION_M)
    np.save(output / 'qpos.npy', np.asarray(snapshots), allow_pickle=False)
    return status, evidence, trace
