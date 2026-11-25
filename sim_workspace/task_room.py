"""Furniture layout for a shared shelf pick-and-place episode."""
import mujoco as mj
import numpy as np

import room
from robots.g1.pick_place import RADIUS


SCENE_ID = 'serendipity-room-team-v1'
G1_SPAWN = (0.62, -0.56)
TABLE_XY = (1.20, -0.75)
TABLE_HEIGHT = 0.81
OBSERVATION_XY = (-0.62, -1.65)
TRAY_XY = {'green tray': (0.952, -0.87), 'yellow tray': (1.11, -0.87)}


def layout(seed):
    rng = np.random.default_rng(seed)
    x = G1_SPAWN[0] + 0.3315 + rng.uniform(-0.009, 0.009)
    y = G1_SPAWN[1] - 0.1136 + rng.uniform(-0.009, 0.009)
    cx, cy = TABLE_XY
    top = 0.867
    height = top - TABLE_HEIGHT
    cradle = (f'<geom name="apple_rest" type="cylinder" '
              f'pos="{x-cx} {y-cy} {TABLE_HEIGHT+height/2}" '
              f'size="0.013 {height/2}" density="1050" friction="0.55 0.01 0.001" '
              'rgba="0.35 0.42 0.44 1"/>')
    # A shallow circular lip prevents light lateral touches from rolling the sphere off its rest.
    for a, b in zip(np.arange(12)*np.pi/6, np.arange(1, 13)*np.pi/6):
        points = [x-cx+.019*np.cos(a), y-cy+.019*np.sin(a), top+.005,
                  x-cx+.019*np.cos(b), y-cy+.019*np.sin(b), top+.005]
        endpoints = ' '.join(map(str, points))
        cradle += (f'<geom name="apple_lip_{round(a*6/np.pi)}" type="capsule" fromto="{endpoints}" '
                   'size="0.003" density="1050" friction="0.52 0.01 0.001" rgba="0.35 0.42 0.44 1"/>')
    xml = [room.furniture('source_table', cx, cy, TABLE_HEIGHT, 0.30, 0.36, fixture=cradle),
           room.WALLS,
           f'<body name="apple" pos="{x} {y} {top+RADIUS+0.002}"><freejoint/>'
           f'<geom name="apple" type="sphere" size="{RADIUS}" mass="0.028" '
           'rgba="0.96 0.05 0.03 1" friction="0.75 0.006 0.0006" condim="4"/></body>',
           f'<body name="block" pos="1.31 -0.69 {TABLE_HEIGHT+0.023}"><freejoint/>'
           '<geom name="block" type="box" size="0.022 0.022 0.022" mass="0.05" '
           'rgba="0.05 0.2 0.95 1" friction="0.48 0.006 0.0006"/></body>']
    for name, (x, y) in TRAY_XY.items():
        color = '0.15 0.65 0.25 1' if name == 'green tray' else '0.9 0.65 0.06 1'
        xml.append(room.tray(name.split()[0], x, y, TABLE_HEIGHT, color, half_x=0.069, half_y=0.069))
    return ''.join(xml)


def configure_contacts(model, *, tabletop=False):
    model.opt.timestep = .001
    for joint in range(model.njnt):
        name = model.joint(joint).name
        if name.startswith('g1/') and model.jnt_type[joint] == mj.mjtJoint.mjJNT_HINGE:
            dof = model.jnt_dofadr[joint]
            hand = 'hand_' in name
            model.dof_armature[dof] = .0003 if hand else .025
            model.dof_damping[dof] = .03 if hand else .3
    # Keep upstream collision pairs. Apply fixture contact settings to the involved surfaces.
    for i in range(model.ngeom):
        geom = model.geom(i)
        body = model.body(model.geom_bodyid[i]).name
        if body.startswith('g1/right_hand_') and model.geom_contype[i]:
            model.geom_friction[i] = (0.95, 0.003, 0.0001)
            model.geom_condim[i] = 4
            model.geom_solref[i] = (0.002 if tabletop else 0.004, 1)
            if tabletop:
                model.geom_solimp[i] = (0.97, 0.995, .0005, .5, 2)
                model.geom_priority[i] = 1
        elif geom.name.startswith(('green_tray', 'yellow_tray', 'green_rim', 'yellow_rim')):
            model.geom_solref[i] = (0.002, 1)
            model.geom_solimp[i] = (0.97, 0.995, 0.0005, 0.5, 2)
            model.geom_priority[i] = 1
        elif tabletop and geom.name in {item[0] for item in LAB_OBJECTS.values()}:
            model.geom_solref[i] = (.002, 1)
            model.geom_solimp[i] = (.97, .995, .0005, .5, 2)
            model.geom_priority[i] = 1
        elif geom.name.startswith('apple'):
            model.geom_solref[i] = (0.004, 1)


def initialize_arm(model, data):
    from kinematics import PositionIK
    from robots.g1.pick_place import ARM, TOOL_OFFSET, WRIST

    arm = PositionIK(model, 'g1/'+WRIST, ['g1/'+joint for joint in ARM], TOOL_OFFSET)
    parked = np.array([G1_SPAWN[0]+.21, G1_SPAWN[1]-.114, .90])
    result = arm.solve(data, parked, orientation=np.eye(3), tolerance=.002)
    if not result.converged:
        raise RuntimeError(f'G1 initial arm posture is unreachable, {result.error_m:.4f} m')


def configure_g1_spec(spec):
    for side in ('left', 'right'):
        foot = spec.body(f'{side}_ankle_roll_link')
        for geom in foot.geoms:
            if geom.type == mj.mjtGeom.mjGEOM_SPHERE:
                geom.contype = 0
                geom.conaffinity = 0
        # Same length, width and ground plane as the vendor's four sole spheres.
        foot.add_geom(name=f'{side}_sole', type=mj.mjtGeom.mjGEOM_BOX,
                      pos=[.035, 0, -.031], size=[.090, .035, .004],
                      rgba=[.12, .13, .14, 1], mass=.001, friction=[.82, .008, .001], condim=4,
                      priority=2, solref=[.005, 1], solimp=[.95, .99, .001, .5, 2])


def initialize_stance(model, data, base):
    for side, sign in (('left', 1), ('right', -1)):
        data.qpos[model.joint(f'g1/{side}_hip_roll_joint').qposadr[0]] = sign * .13
        data.qpos[model.joint(f'g1/{side}_ankle_roll_joint').qposadr[0]] = -sign * .13
    mj.mj_forward(model, data)
    floor_heights = []
    for side in ('left', 'right'):
        sole = model.geom(f'g1/{side}_sole').id
        vertical = np.abs(data.geom_xmat[sole].reshape(3, 3)[2])
        floor_heights.append(data.geom_xpos[sole, 2] - vertical @ model.geom_size[sole])
    data.qpos[base+2] += .0005 - min(floor_heights)


LAB_SCENE_ID = 'serendipity-flat-table-v1'
LAB_G1_START = (.10, -.56)
LAB_TABLE_XY = (1.08, -.76)
LAB_TRAYS = {'green tray': (.95, -.82), 'yellow tray': (.99, -1.00)}
LAB_OBJECTS = {
    'red apple': ('apple', 'sphere', (RADIUS,), .028, (0.96, .05, .03, 1)),
    'blue block': ('block', 'box', (.018, .018, .021), .05, (.05, .20, .95, 1)),
    'purple cylinder': ('cylinder', 'cylinder', (.021, .025), .045, (.76, .05, .76, 1)),
}


def lab_layout(seed):
    rng = np.random.default_rng(seed)
    x, y = rng.uniform(-.035, .035, size=2) + [.924, -.65]
    centers = np.array([[x, y], [1.03, -.58], [1.14, -.97]])
    centers = np.roll(centers, seed % len(LAB_OBJECTS), axis=0)
    xml = [room.furniture('source_table', *LAB_TABLE_XY, TABLE_HEIGHT, .24, .36), room.WALLS]
    for (name, shape, size, mass, color), (cx, cy) in zip(LAB_OBJECTS.values(), centers):
        size_str = ' '.join(map(str, size))
        rgba = ' '.join(map(str, color))
        xml.append(f'<body name="{name}" pos="{cx} {cy} {TABLE_HEIGHT + size[-1] + .003}">'
                   f'<freejoint/><geom name="{name}" type="{shape}" size="{size_str}" mass="{mass}" '
                   f'rgba="{rgba}" friction=".55 .006 .0006" condim="6"/></body>')
    for name, (tx, ty) in LAB_TRAYS.items():
        tx += rng.uniform(-.019, .019)
        ty += rng.uniform(-.012, .012)
        color = '0.15 0.65 0.25 1' if name == 'green tray' else '0.9 0.65 0.06 1'
        xml.append(room.tray(name.split()[0], tx, ty, TABLE_HEIGHT, color, half_x=.068, half_y=.070))
    return ''.join(xml)
