"""Shared lab scene furniture and free-base dynamics checks."""

import json
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np


TIMESTEP_S = 0.002
TRACE_INTERVAL_S = 0.02
GRAVITY_M_S2 = (0.0, 0.0, -9.81)
G1_TORQUE_MODEL = (Path(__file__).resolve().parents[1] / 'third_party/unitree_mujoco'
                   / 'unitree_robots/g1/g1_29dof.xml')
PUSH_FORCE_N = 8.0
PUSH_DURATION_S = 0.16
OBJECT_MATERIALS = {'apple': (0.18, '0.43 0.006 0.0005'),
                    'block': (0.32, '0.42 0.005 0.0005')}


def furniture(name, x, y, height, half_x, half_y, *, fixture=''):
    """Free standing wooden table made from a top and four legs."""
    top_thickness = 0.035
    leg_height = height - top_thickness
    wood = 'rgba="0.54 0.37 0.24 1" density="650" friction="0.5 0.008 0.0008"'
    parts = [f'<geom name="{name}_top" type="box" {wood} '
             f'pos="0 0 {height - top_thickness/2}" size="{half_x} {half_y} {top_thickness/2}"/>']
    for index, (sx, sy) in enumerate(product((-1, 1), repeat=2)):
        parts.append(f'<geom name="{name}_leg_{index}" type="box" {wood} '
                     f'pos="{sx*(half_x-0.055)} {sy*(half_y-0.055)} {leg_height/2}" '
                     f'size="0.025 0.025 {leg_height/2}"/>')
    return f'<body name="{name}" pos="{x} {y} 0.001"><freejoint/>{"".join(parts)}{fixture}</body>'


def tray(name, x, y, height, color, *, half_x=0.22, half_y=0.19):
    plastic = f'rgba="{color}" density="950" friction="0.35 0.006 0.0006"'
    parts = [f'<geom name="{name}_tray" type="box" {plastic} size="{half_x} {half_y} 0.006"/>']
    for side, sign in (('neg', -1), ('pos', 1)):
        parts.append(f'<geom name="{name}_rim_x_{side}" type="box" {plastic} '
                     f'pos="{sign*(half_x-0.006)} 0 0.018" size="0.006 {half_y} 0.018"/>')
        parts.append(f'<geom name="{name}_rim_y_{side}" type="box" {plastic} '
                     f'pos="0 {sign*(half_y-0.006)} 0.018" size="{half_x-0.012} 0.006 0.018"/>')
    return f'<body name="{name}_tray" pos="{x} {y} {height + 0.01}"><freejoint/>{"".join(parts)}</body>'


WALLS = '''
<geom name="floor_finish" type="box" pos="0 0 -0.06" size="2.4 2.5 0.059" rgba="0.57 0.58 0.56 1" contype="0" conaffinity="0"/>
<geom name="back_wall" type="box" pos="2.4 0 1.2" size="0.06 2.5 1.2" rgba="0.73 0.76 0.79 1"/>
<geom name="side_wall" type="box" pos="0 2.5 1.2" size="2.4 0.06 1.2" rgba="0.82 0.84 0.82 1"/>
'''


def initialize_g1(model, data, base):
    # Start the lowest sole sphere just above the floor so contact loads build through the solver.
    feet = [i for i in range(model.ngeom) if model.geom_contype[i]
            and model.body(model.geom_bodyid[i]).name in
            ('g1/left_ankle_roll_link', 'g1/right_ankle_roll_link')]
    sole_height = min(data.geom_xpos[i, 2] - model.geom_size[i, 0] for i in feet)
    data.qpos[base + 2] += 0.0005 - sole_height


class StandController:
    """Joint-space stand baseline driving the model's torque actuators."""

    def __init__(self, model, data):
        self.joints = model.actuator_trnid[:, 0]
        self.dofs = model.jnt_dofadr[self.joints]
        self.target = data.qpos[model.jnt_qposadr[self.joints]].copy()
        self.qpos = model.jnt_qposadr[self.joints]
        self.gear = model.actuator_gear[:, 0]
        self.lower = np.where(model.actuator_ctrllimited, model.actuator_ctrlrange[:, 0], -np.inf)
        self.upper = np.where(model.actuator_ctrllimited, model.actuator_ctrlrange[:, 1], np.inf)
        names = [model.joint(int(joint)).name for joint in self.joints]
        go2 = np.array([name.startswith('go2/') for name in names])
        fingers = np.array([name.startswith(('g1/left_hand_', 'g1/right_hand_')) for name in names])
        self.kp = np.where(go2, 55, np.where(fingers, 12, 200))
        self.kd = np.where(go2, 2.5, np.where(fingers, 0.6, 3))
        vendor_limits = {motor.attrib['joint']: tuple(map(float, motor.attrib['ctrlrange'].split()))
                         for motor in ET.parse(G1_TORQUE_MODEL).findall('./actuator/motor')}
        for motor, name in enumerate(names):
            if name.startswith('g1/'):
                g1_joint = name.removeprefix('g1/')
                self.lower[motor], self.upper[motor] = (-0.85, 0.85) if fingers[motor] else vendor_limits[g1_joint]
        self.ankle_attitude_gain = 115
        self.pelvis = model.body('g1/pelvis').id
        self.root_dof = int(model.jnt_dofadr[model.body_jntadr[self.pelvis]])
        self.ankle_pitch = [model.actuator(f'g1/{side}_ankle_pitch_joint').id for side in ('left', 'right')]
        self.ankle_roll = [model.actuator(f'g1/{side}_ankle_roll_joint').id for side in ('left', 'right')]

    def apply(self, data):
        torque = self.kp * (self.target - data.qpos[self.qpos]) - self.kd * data.qvel[self.dofs]
        # IMU-like attitude/rate feedback applies corrective torque at the ankles.
        vertical = data.xmat[self.pelvis, [2, 5]]
        roll_rate, pitch_rate = data.qvel[self.root_dof + 3:self.root_dof + 5]
        torque[self.ankle_pitch] += self.ankle_attitude_gain * (vertical[0] + 0.15 * pitch_rate)
        torque[self.ankle_roll] += self.ankle_attitude_gain * (-vertical[1] + 0.15 * roll_rate)
        data.ctrl[:] = np.clip(torque / self.gear, self.lower, self.upper)


def measure(model, data, robots, floor_contacts):
    result = {}
    for robot, body in robots.items():
        tilt = np.degrees(np.arccos(np.clip(data.xmat[body, 8], -1, 1)))
        result[robot] = {'xyz_m': data.xpos[body].tolist(), 'tilt_deg': float(tilt),
                         'floor_contacts': sorted(floor_contacts[robot])}
    return result


def run(model, data, bases, output, record, *, unpowered, disturbance):
    controller = StandController(model, data)
    robots = {name: model.jnt_bodyid[np.flatnonzero(model.jnt_qposadr == adr)[0]]
              for name, adr in bases.items()}
    floor = model.geom('floor').id
    body_ids = {name: model.body(name).id for name in
                ('apple', 'block', 'source_table', 'green_table', 'yellow_table', 'green_tray', 'yellow_tray')}
    start = {name: data.xpos[body].copy() for name, body in body_ids.items()}
    initial_z = {name: float(data.xpos[body, 2]) for name, body in robots.items()}
    floor_contacts = {name: set() for name in robots}
    frames = [data.qpos.copy()]
    first_fall = {}
    dt = model.opt.timestep
    sample_steps = round(TRACE_INTERVAL_S / dt)
    duration_s = 10.0
    record({'stage': 'room_initialized', 'gravity_m_s2': model.opt.gravity.tolist(),
            'timestep_s': float(dt), 'actuators': model.nu, 'unpowered': unpowered,
            'disturbance': disturbance})
    if unpowered:
        model.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_ACTUATION)
    with (output / 'physics_trace.jsonl').open('w') as trace:
        for step in range(round(duration_s / dt)):
            if not unpowered:
                controller.apply(data)
            # A brief horizontal shove demonstrates independent object motion.
            if disturbance:
                data.xfrc_applied[body_ids['block'], 0] = PUSH_FORCE_N if 1.0 <= data.time < 1.0 + PUSH_DURATION_S else 0.0
            mj.mj_step(model, data)
            # xmat and contacts are evaluated at the beginning of this step.
            for contact in data.contact:
                if floor in (contact.geom1, contact.geom2):
                    other = contact.geom2 if contact.geom1 == floor else contact.geom1
                    name = model.body(model.geom_bodyid[other]).name
                    robot = name.split('/')[0]
                    if robot in robots:
                        floor_contacts[robot].add(name)
            for robot, body in robots.items():
                if robot not in first_fall and (data.xmat[body, 8] < np.cos(np.radians(35))
                                                or data.xpos[body, 2] < 0.65 * initial_z[robot]):
                    first_fall[robot] = float(data.time - dt)
            if (step + 1) % sample_steps == 0:
                frames.append(data.qpos.copy())
                trace.write(json.dumps({'time_s': float(data.time - dt),
                                        'robots': measure(model, data, robots, floor_contacts),
                                        'objects_xyz_m': {n: data.xpos[b].tolist() for n, b in body_ids.items()}}) + '\n')
    mj.mj_forward(model, data)
    np.save(output / 'qpos.npy', np.asarray(frames), allow_pickle=False)
    result = measure(model, data, robots, floor_contacts)
    object_motion = {name: float(np.linalg.norm(data.xpos[body] - start[name]))
                     for name, body in body_ids.items()}
    for name in result:
        result[name]['first_fall_s'] = first_fall.get(name)
    expected_contacts = {'go2': 4, 'g1': 2}
    if unpowered:
        robots_ok = all(name in first_fall for name in robots)
    else:
        robots_ok = all(name not in first_fall and len(floor_contacts[name]) >= expected_contacts[name]
                        for name in robots)
    block_height = float(data.xpos[body_ids['block'], 2])
    objects_ok = not disturbance or (object_motion['block'] > 0.10 and block_height < 0.07)
    checks = {'robots': robots_ok, 'objects': objects_ok,
              'solver': bool(not np.any(data.warning.number) and np.isfinite(data.qpos).all())}
    success = all(checks.values())
    record({'stage': 'room_checks', 'checks': checks})
    return {'mode': 'room-physics', 'status': 'physics_checks_passed' if success else 'physics_checks_failed',
            'success': success, 'placement_status': 'not_attempted', 'duration_s': duration_s,
            'trace_interval_s': TRACE_INTERVAL_S, 'unpowered': unpowered, 'disturbance': disturbance,
            'robots': result, 'object_displacement_m': object_motion,
            'block_final_height_m': block_height, 'checks': checks}


def view(model, data, port):
    from queue import Empty, SimpleQueue
    from time import perf_counter, sleep

    import viser
    from mjviser.scene import ViserMujocoScene

    server = viser.ViserServer(host='127.0.0.1', port=port)
    server.initial_camera.position = (-3.45, -4.2, 2.6)
    server.initial_camera.look_at = (0.0, -0.10, 0.48)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.gui.add_markdown('**Live MuJoCo lab** · Joint torques, free bases and contact dynamics')
    powered = server.gui.add_checkbox('Motors enabled', initial_value=True)
    push = server.gui.add_button('Push blue block')
    reset_button = server.gui.add_button('Reset lab')
    commands = SimpleQueue()
    block = model.body('block').id
    initial_qpos = data.qpos.copy()
    controller = StandController(model, data)
    push_end = 0.0

    @push.on_click
    def queue_push(_):
        commands.put('push')

    @reset_button.on_click
    def queue_reset(_):
        commands.put('reset')

    def step(_, state):
        nonlocal push_end
        try:
            command = commands.get_nowait()
        except Empty:
            command = None
        if command == 'reset':
            reset(model, state)
            return
        if command == 'push':
            push_end = state.time + PUSH_DURATION_S
        if powered.value:
            controller.apply(state)
        else:
            state.ctrl[:] = 0.0
        state.xfrc_applied[block, 0] = PUSH_FORCE_N if state.time < push_end else 0.0
        mj.mj_step(model, state)

    def reset(_, state):
        nonlocal push_end
        mj.mj_resetData(model, state)
        state.qpos[:] = initial_qpos
        push_end = 0.0
        mj.mj_forward(model, state)

    scene = ViserMujocoScene(server, model, num_envs=1)
    scene.camera_tracking_enabled = False
    scene.update_from_mjdata(data)
    # Integrate with fixed physics steps and render independently at 60 Hz.
    dt = model.opt.timestep
    frame_interval = 1 / 60
    last_frame = perf_counter()
    accumulated = 0.0
    try:
        while True:
            now = perf_counter()
            wait = frame_interval - (now - last_frame)
            if wait > 0:
                sleep(wait)
                now = perf_counter()
            accumulated = min(accumulated + now - last_frame, 0.05)
            last_frame = now
            while accumulated >= dt:
                step(model, data)
                accumulated -= dt
            scene.update_from_mjdata(data)
    except KeyboardInterrupt:
        server.stop()
