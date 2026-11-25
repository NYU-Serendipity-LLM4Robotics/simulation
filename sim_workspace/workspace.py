"""Go2/G1 shared MuJoCo scene and reproducible sensor handoff baseline."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import traceback

import mujoco as mj
import numpy as np

import handoff
from robots.g1 import manipulation as g1
from robots.g1 import pick_place
import room
import task_room
from robots.go2 import scout as go2
from robots.go2.scout import localize as localize

ROBOTS = {'go2': go2.MODEL_XML, 'g1': g1.MODEL_XML}
OBJECTS = {
    'red apple': ('apple', 'sphere', '0.045', '0.9 0.1 0.07 1', -0.24),
    'blue block': ('block', 'box', '0.045 0.045 0.045', '0.1 0.35 0.95 1', 0.24),
}
DESTINATIONS = {'green tray': (-0.9, 1.1, 0.48), 'yellow tray': (-0.9, -1.1, 0.48)}


def scene(seed, robot_names=('go2', 'g1'), *, physics_room=False, task_layout=False, lab_layout=False,
          empty_lab=False, extra_world="", extra_meshes="", mesh_files=None, extra_equalities=""):
    task_layout = task_layout or lab_layout
    rng = np.random.default_rng(seed)
    objects = []
    for name, shape, size, color, y in OBJECTS.values():
        x = rng.uniform(1.0, 1.22)
        y += rng.uniform(-0.06, 0.06)
        material = ''
        if physics_room:
            mass, friction = room.OBJECT_MATERIALS[name]
            material = f' mass="{mass}" friction="{friction}"'
        objects.append(f'''<body name="{name}" pos="{x} {y} 0.525">
          <freejoint/><geom name="{name}" type="{shape}" size="{size}" rgba="{color}"{material}/>
        </body>''')
    trays = []
    furniture = '''<geom name="table" type="box" pos="1.15 0 0.455" size="0.42 0.46 0.025" rgba="0.52 0.38 0.26 1"/>
                  <geom name="table_pedestal" type="box" pos="1.15 0 0.22" size="0.08 0.08 0.22" rgba="0.4 0.31 0.23 1"/>'''
    options = '<option cone="elliptic" impratio="100"/>'
    if physics_room:
        gravity = ' '.join(map(str, room.GRAVITY_M_S2))
        options = f'<option gravity="{gravity}" timestep="{room.TIMESTEP_S}" integrator="implicitfast" cone="elliptic" impratio="100"/>'
        furniture = room.furniture('source_table', 1.15, 0, 0.48, 0.42, 0.46) + room.WALLS
    for name, xyz in DESTINATIONS.items():
        color = '0.15 0.65 0.25 1' if name == 'green tray' else '0.9 0.65 0.06 1'
        x, y, z = xyz
        if physics_room:
            prefix = name.split()[0]
            trays.append(room.furniture(f'{prefix}_table', x, y, z, 0.30, 0.27))
            trays.append(room.tray(prefix, x, y, z, color))
            continue
        trays.append(f'''<geom name="{name.split()[0]}_stand" type="box" pos="{x} {y} 0.224" size="0.28 0.26 0.224" rgba="0.35 0.37 0.4 1"/>
          <geom name="{name.split()[0]}_tray" type="box" pos="{x} {y} {z - 0.016}" size="0.25 0.23 0.016" rgba="{color}"/>''')
    width, height = go2.WIDTH, go2.HEIGHT
    if task_layout:
        layout = task_room.lab_layout if lab_layout else task_room.layout
        furniture, objects, trays = (room.WALLS if empty_lab else layout(seed)), [], []
        width, height = 640, 480
    spec = mj.MjSpec.from_string(f'''<mujoco model="serendipity">
{options}
      <visual><global offwidth="{width}" offheight="{height}"/><map zfar="25"/></visual>
      <asset>{extra_meshes}<texture name="tiles" type="2d" builtin="checker" width="128" height="128" rgb1=".24 .28 .31" rgb2=".3 .34 .37"/>
        <material name="floor" texture="tiles" texrepeat="6 6" texuniform="true"/></asset>
      <worldbody>
        <light pos="0 0 4" dir="0 0 -1" diffuse="0.85 0.85 0.85"/>
        <geom name="floor" type="plane" size="4 4 0.1" material="floor"/>
        {furniture}
        {''.join(objects)}{''.join(trays)}{extra_world}
      </worldbody>{extra_equalities}</mujoco>''', assets=mesh_files)
    for robot in robot_names:
        child = mj.MjSpec.from_file(str(ROBOTS[robot]))
        if robot == 'go2':
            go2.configure_camera(child)
        elif task_layout:
            child.delete(child.geom('floor'))
            task_room.configure_g1_spec(child)
        spec.attach(child, prefix=robot + '/', frame=spec.worldbody.add_frame())
    model = spec.compile()
    if task_layout:
        task_room.configure_contacts(model, tabletop=lab_layout)
    data = mj.MjData(model)
    bases = {}
    for joint_id, kind in enumerate(model.jnt_type):
        if kind != mj.mjtJoint.mjJNT_FREE:
            continue
        body = model.body(model.jnt_bodyid[joint_id]).name
        robot = body.split('/')[0]
        if robot in robot_names:
            bases[robot] = int(model.jnt_qposadr[joint_id])
    if set(bases) != set(robot_names):
        raise RuntimeError(f'Expected free bases for {robot_names}, got {bases}')
    if 'go2' in bases:
        go2.initialize_pose(data, bases['go2'])
        place_base(data, bases['go2'], -1.55, 0, 0.29)
    if 'g1' in bases:
        place_base(data, bases['g1'], -1.65, -0.46, 0.80)
    if task_layout:
        data.qpos[bases['go2'] + 1] = task_room.OBSERVATION_XY[1]
    if physics_room:
        if task_layout:
            data.qpos[bases['g1']:bases['g1'] + 2] = (
                task_room.LAB_G1_START if lab_layout else task_room.G1_SPAWN)
        else:
            data.qpos[bases['g1'] + 1] -= 0.29
        g1.g1_stance(model, data, bases['g1'], g1.g1_feet(model))
        if task_layout:
            task_room.initialize_stance(model, data, bases['g1'])
        else:
            room.initialize_g1(model, data, bases['g1'])
    if task_layout and not lab_layout:
        task_room.initialize_arm(model, data)
    mj.mj_forward(model, data)
    return model, data, bases


def place_base(data, address, x, y, height):
    data.qpos[address:address + 7] = [x, y, height, 1, 0, 0, 0]


def run(seed, target, destination, output, *, mode='team'):
    if mode == 'team':
        from team_physics import run as run_physics_team

        def simulate(events, record, _log):
            model, data, _ = scene(seed, physics_room=True, task_layout=True)
            report = run_physics_team(model, data, seed, target, destination, output, record)
            return dict(report, events=events)

        return logged_episode(output, f'team physics seed={seed}', simulate)
    if mode not in ('team-kinematic', 'go2'):
        raise ValueError(f'unknown scene mode: {mode}')
    legacy_mode = 'team' if mode == 'team-kinematic' else mode
    return logged_episode(
        output, f'mode={mode} seed={seed} target={target!r} destination={destination!r}',
        lambda events, record, log: run_episode(seed, target, destination, output, events, record, log, legacy_mode),
    )


def run_lab(seed, target, destination, output, weights):
    from lab_trial import run as run_lab_trial

    def simulate(events, record, _log):
        model, data, _ = scene(seed, physics_room=True, lab_layout=True)
        report = run_lab_trial(model, data, seed, target, destination, output, weights, record)
        return dict(report, events=events)

    return logged_episode(output, f'flat tabletop team seed={seed}', simulate)


def run_fixed(source, destination, output):
    def execute(events, record, log):
        log('building fixed-base G1 and dynamic workpiece')
        status, evidence, trace = pick_place.run_pick_place(source, destination, output, record)
        with (output / 'physics_trace.jsonl').open('x') as stream:
            for time, stage, xyz, fingers, supports, control, overlap in trace:
                row = dict(time_s=time, stage=stage, object_xyz_m=xyz, finger_normal_forces_n=fingers,
                           support_contacts=supports, actuator_targets_rad=control, penetration_m=overlap)
                stream.write(json.dumps(row) + chr(10))
        return {'schema_version': 3, 'mode': 'g1-fixed', 'status': status, 'success': status == 'placed',
                'placement_status': status, 'source_xyz_m': list(source), 'destination_xyz_m': list(destination),
                'frame_id': 'world', 'contact_profile': pick_place.CONTACT_PROFILE,
                'actuators': list(pick_place.ARM + pick_place.FINGERS),
                'evidence': evidence, 'events': events}

    return logged_episode(output, f'mode=g1-fixed source={source} destination={destination}', execute)


def logged_episode(output, description, execute):
    output.mkdir(parents=True, exist_ok=False)
    events = []
    with (output / 'run.log').open('x', encoding='utf-8') as stream:
        def log(message, level='INFO'):
            timestamp = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
            print(f'{timestamp} {level} {message}', file=stream, flush=True)

        def record(event):
            events.append(event)
            failed = event.get('converged') is False or event['stage'] == 'search_failed'
            log(json.dumps(event), 'WARNING' if failed else 'INFO')

        log(f'start {description} '
            f'output={output.resolve()} mujoco={mj.__version__} '
            f'python={sys.executable} MUJOCO_GL={os.environ.get("MUJOCO_GL", "default")}')
        try:
            result = execute(events, record, log)
            (output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
        except Exception:
            log(traceback.format_exc().rstrip(), 'ERROR')
            raise
        status = result['status']
        log(f'finish status={status} placement={result["placement_status"]}',
            'INFO' if result['success'] else 'WARNING')
        return result


def run_episode(seed, target, destination, output, events, record, log, mode):
    log('compiling shared scene and loading robot assets')
    robots = ('go2', 'g1') if mode == 'team' else ('go2',)
    model, data, bases = scene(seed, robots)
    geom = model.geom(OBJECTS[target][0]).id
    if 'g1' in bases:
        g1.g1_stance(model, data, bases['g1'], g1.g1_feet(model))
    snapshots = [data.qpos.copy()]
    record({'stage': 'search', 'target': target, 'destination': destination})
    report, scout_failure = go2.run_scout(
        model, data, bases['go2'], target, geom, output, snapshots, record, log,
    )
    message = handoff.from_scout(seed, target, destination, DESTINATIONS[destination], report, scout_failure)
    (output / 'handoff.json').write_text(json.dumps(message, indent=2) + '\n')
    status = message['status']
    if status != 'localized':
        record({'stage': 'search_failed', 'reason': status})
    elif mode == 'team':
        record({'stage': 'manipulation_ready', 'target': message['object_id'],
                'object_xyz_m': message['object_xyz_m'], 'frame': message['frame_id'],
                'destination': message['destination_id'], 'destination_xyz_m': message['destination_xyz_m']})
        status = g1.approach_with_ik(model, data, bases['g1'], np.array(message['object_xyz_m']), snapshots, record)
    result = {'schema_version': 3, 'mode': mode, 'seed': seed, 'navigation': 'prescribed_base_route_with_foot_ik',
              'status': status, 'success': status in ('localized', 'pregrasp_reached'), 'ik_status': status,
              'localization_succeeded': message['status'] == 'localized', 'placement_status': 'not_run',
              'events': events}
    log('writing replay poses and report')
    np.save(output / 'qpos.npy', np.asarray(snapshots), allow_pickle=False)
    return result


def replay(output, port):
    import viser
    from mjviser import Viewer

    metadata = json.loads((output / 'report.json').read_text())
    mode = metadata.get('mode', 'team')
    if mode == 'g1-fixed':
        profile = metadata.get('contact_profile', pick_place.LEGACY_PROFILE)
        model, data = pick_place.make_scene(metadata['source_xyz_m'], metadata['destination_xyz_m'], profile=profile)
    else:
        robots = ('go2',) if mode == 'go2' else ('go2', 'g1')
        model, data, _ = scene(metadata['seed'], robots, physics_room=mode in ('room-physics', 'team-physics', 'lab-trial'),
                               task_layout=mode == 'team-physics', lab_layout=mode == 'lab-trial')
    if mode in ('team-physics', 'lab-trial'):
        from team_physics import view_recording
        return view_recording(model, data, output, metadata, port)
    poses = np.load(output / 'qpos.npy', allow_pickle=False)
    index = 0
    sample_period_steps = round(pick_place.TRACE_INTERVAL_S / model.opt.timestep) if mode == 'g1-fixed' else 35

    if mode == 'room-physics':
        sample_period_steps = round(metadata['trace_interval_s'] / model.opt.timestep)

    def advance(*_):
        nonlocal index
        data.qpos[:] = poses[(index // sample_period_steps) % len(poses)]
        mj.mj_forward(model, data)
        index += 1

    server = viser.ViserServer(host='127.0.0.1', port=port)
    server.gui.add_markdown('**Recorded trajectory** · Playback of saved frames')
    Viewer(model, data, server=server, step_fn=advance).run()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    team = actions.add_parser('team', aliases=['run'], help='closed-loop physical Go2 and G1 task')
    legacy_team = actions.add_parser('team-kinematic', help='archived kinematic perception/pregrasp baseline')
    solo = actions.add_parser('go2', help='Go2 only, producing calibrated RGB-D and a handoff')
    for command in (team, legacy_team, solo):
        command.add_argument('--seed', type=int, default=0)
        command.add_argument('--object', choices=OBJECTS, default='red apple')
        command.add_argument('--destination', choices=DESTINATIONS, default='green tray')
        command.add_argument('--output', type=Path, required=True)
    lab = actions.add_parser('lab-trial', help='flat tabletop trial with free-base G1 locomotion')
    lab.add_argument('--seed', type=int, default=0)
    lab.add_argument('--object', choices=task_room.LAB_OBJECTS, default='red apple')
    lab.add_argument('--destination', choices=task_room.LAB_TRAYS, default='green tray')
    lab.add_argument('--policy', type=Path, required=True)
    lab.add_argument('--output', type=Path, required=True)
    fixed = actions.add_parser('g1-fixed', help='fixed-base G1 physical grasp, lift and place')
    fixed.add_argument('--source-xyz', type=float, nargs=3, default=pick_place.SOURCE)
    fixed.add_argument('--destination-xyz', type=float, nargs=3, default=pick_place.DESTINATION)
    fixed.add_argument('--output', type=Path, required=True)
    physical = actions.add_parser('room-physics', help='free-base lab dynamics and contact checks')
    physical.add_argument('--seed', type=int, default=0)
    physical.add_argument('--unpowered', action='store_true')
    physical.add_argument('--disturbance', action='store_true')
    physical.add_argument('--output', type=Path, required=True)
    live = actions.add_parser('room-view', help='live free-base lab with a motor toggle and block shove')
    live.add_argument('--port', type=int, default=8767)
    editor = actions.add_parser('scene-editor', help='edit room layouts in a browser')
    editor.add_argument('--port', type=int, default=8793)
    editor.add_argument('--scene', type=Path, help='saved scene JSON to open')
    viewer = actions.add_parser('replay')
    viewer.add_argument('output', type=Path)
    viewer.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    if args.action == 'scene-editor':
        from scene_editor import run as edit_scene
        edit_scene(args.port, args.scene)
        return 0
    if args.action == 'room-view':
        model, data, _ = scene(0, physics_room=True)
        room.view(model, data, args.port)
        return 0
    if args.action == 'replay':
        replay(args.output, args.port)
        return 0
    if args.action == 'room-physics':
        if args.seed < 0:
            parser.error('seed must be nonnegative')

        def simulate(events, record, log):
            model, data, bases = scene(args.seed, physics_room=True)
            result = room.run(model, data, bases, args.output, record,
                              unpowered=args.unpowered, disturbance=args.disturbance)
            return dict(result, seed=args.seed, events=events)

        result = logged_episode(args.output, f'room-physics seed={args.seed}', simulate)
    elif args.action == 'g1-fixed':
        if not np.isfinite([args.source_xyz, args.destination_xyz]).all():
            parser.error('fixed fixture coordinates must be finite')
        result = run_fixed(args.source_xyz, args.destination_xyz, args.output)
    else:
        if args.seed < 0:
            parser.error('seed must be nonnegative')
        mode = 'team' if args.action == 'run' else args.action
        if mode == 'lab-trial':
            result = run_lab(args.seed, args.object, args.destination, args.output, args.policy)
        else:
            result = run(args.seed, args.object, args.destination, args.output, mode=mode)
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
