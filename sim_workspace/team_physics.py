"""Physical Go2-to-G1 episode. Only actuator commands advance the shared state."""
import json

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares

import handoff
import room
import task_room
from robots.g1.pick_place import RADIUS
from robots.g1.room_grasp import GraspController
from robots.go2.trot import Trot


INTERVAL = 0.02
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480


def observe(renderer, model, data, index, output):
    camera = model.camera('go2/rgbd')
    renderer.update_scene(data, camera=camera.id)
    rgb = renderer.render().copy()
    renderer.enable_depth_rendering()
    depth = renderer.render().copy()
    renderer.disable_depth_rendering()
    color = rgb.astype(np.float32)
    red, green, blue = color.transpose(2, 0, 1)
    mask = (red > 48) & (red > 1.75*green) & (red > 1.75*blue) & np.isfinite(depth) & (depth > 0)
    rows, cols = np.nonzero(mask)
    focal = CAMERA_HEIGHT / (2*np.tan(np.deg2rad(float(camera.fovy[0]))*.5))
    calibration = np.array([[focal, 0, (CAMERA_WIDTH-1)/2], [0, focal, (CAMERA_HEIGHT-1)/2], [0, 0, 1]])
    origin = data.cam_xpos[camera.id].copy()
    rotation = data.cam_xmat[camera.id].reshape(3, 3).copy()
    capture = output / 'rgbd'
    capture.mkdir(exist_ok=True)
    np.savez_compressed(capture / f'{index:03d}.npz', rgb=rgb, depth_m=depth, mask=mask,
                        intrinsics=calibration, camera_xyz_m=origin, camera_to_world_rotation=rotation,
                        time_s=data.time)
    if len(rows) < 16:
        return None
    z = depth[rows, cols]
    close = np.abs(z - np.median(z)) < RADIUS
    if np.count_nonzero(close) < 16:
        return None
    optical = np.column_stack(((cols-calibration[0,2])*z/focal,
                               -(rows-calibration[1,2])*z/focal, -z))[close]
    surface = np.median(optical, axis=0)
    guess = surface + RADIUS * surface / np.linalg.norm(surface)
    fit = least_squares(lambda center: np.linalg.norm(optical-center, axis=1)-RADIUS,
                        guess, loss='soft_l1', f_scale=.002)
    if not fit.success or np.median(np.abs(fit.fun)) > .004 or np.linalg.norm(fit.x-guess) > .025:
        return None
    return {'xyz_m': (origin+rotation@fit.x).tolist(), 'capture_index': index,
            'method': 'rgb_color_mask_depth_sphere_fit', 'pixel_count': int(len(optical)),
            'median_residual_m': float(np.median(np.abs(fit.fun)))}


def run(model, data, seed, target, destination, output, record):
    stand = room.StandController(model, data)
    go2 = Trot(model)
    grasp = GraspController(model, data, stand)
    object_geom = model.geom('apple').id
    tray = model.geom(destination.split()[0]+'_tray').id
    goal_xyz = np.r_[task_room.TRAY_XY[destination], task_room.TABLE_HEIGHT+.012]
    dt = float(model.opt.timestep)
    sample_steps = round(INTERVAL/dt)
    command = np.zeros(3)
    samples = [data.qpos.copy()]
    step = 0
    trace = []
    walk_start = data.xpos[go2.root, :2].copy()
    maximum_tilt = 0.0
    floor = model.geom('floor').id
    g1_soles = {model.geom(f'g1/{side}_sole').id for side in ('left', 'right')}
    bilateral_samples = 0

    def tick(stage):
        nonlocal step, maximum_tilt, bilateral_samples
        stand.apply(data)
        grasp.apply()
        go2.apply(model, data, command)
        mj.mj_step(model, data)
        step += 1
        tilt = float(np.degrees(np.arccos(np.clip(data.xmat[stand.pelvis, 8], -1, 1))))
        maximum_tilt = max(maximum_tilt, tilt)
        if step % sample_steps == 0:
            grounded = {int(c.geom2 if c.geom1 == floor else c.geom1) for c in data.contact
                        if floor in c.geom and c.efc_address >= 0}
            grounded_soles = grounded & g1_soles
            bilateral_samples += len(grounded_soles) == 2
            samples.append(data.qpos.copy())
            trace.append({'time_s': float(data.time), 'stage': stage, 'go2_xyz_m': data.xpos[go2.root].tolist(),
                          'g1_xyz_m': data.xpos[stand.pelvis].tolist(), 'g1_tilt_deg': tilt,
                          'object_xyz_m': data.geom_xpos[object_geom].tolist(),
                          'go2_velocity_command': command.tolist(),
                          'g1_grounded_soles': sorted(model.geom(j).name for j in grounded_soles)})

    status = 'target_not_visible'
    result = None
    evidence = {}
    record({'stage': 'start_physics', 'time_s': 0, 'root_control': 'joint_torques', 'target': target})
    if target != 'red apple':
        status = 'unsupported_grasp_object'
    else:
        with mj.Renderer(model, width=CAMERA_WIDTH, height=CAMERA_HEIGHT) as renderer:
            capture_index = 0
            for i in range(round(9.0/dt)):
                position = data.xpos[go2.root]
                rotation = data.xmat[go2.root].reshape(3, 3)
                yaw = np.arctan2(rotation[1,0], rotation[0,0])
                delta = np.asarray(task_room.OBSERVATION_XY) - position[:2]
                local = rotation[:2,:2].T @ delta
                to_table = np.asarray(task_room.TABLE_XY)-position[:2]
                heading = np.arctan2(to_table[1], to_table[0])
                yaw_error = np.arctan2(np.sin(heading-yaw), np.cos(heading-yaw))
                command[:] = np.clip([.9*local[0], 1.2*local[1], 1.7*yaw_error],
                                     [-.16, -.12, -.40], [.26, .12, .40])
                tick('go2_search')
                if data.xmat[go2.root, 8] < .72:
                    status = 'go2_lost_balance'
                    break
                walked = np.linalg.norm(position[:2]-walk_start)
                if i % round(.28/dt) == 0 and walked >= .45 and data.xmat[go2.root, 8] > .90:
                    result = observe(renderer, model, data, capture_index, output)
                    capture_index += 1
                    if result is not None:
                        break
    command[:] = 0
    message = handoff.from_scout(seed, target, destination, goal_xyz.tolist(), result, False,
                                 scene_id=task_room.SCENE_ID, position_kind='object_center', failure_status=status)
    (output/'handoff.json').write_text(json.dumps(message, indent=2)+'\n')
    handoff.validate(message)
    if result is not None:
        record({'stage': 'handoff', 'time_s': float(data.time), 'message': message,
                'rgbd_fit': result, 'center_error_m': float(np.linalg.norm(
                    np.array(message['object_xyz_m'])-data.geom_xpos[object_geom])),
                'go2_displacement_m': float(np.linalg.norm(data.xpos[go2.root, :2]-walk_start))})
        for _ in range(round(.50 / dt)):
            tick('stabilize')
        status, evidence = grasp.follow(np.array(message['object_xyz_m']),
                                        np.array(message['destination_xyz_m']), object_geom, tray, tick, record)
    if status == 'placed' and (np.any(data.warning.number) or not np.isfinite(data.qpos).all()):
        status = 'physics_warning'
    np.save(output/'qpos.npy', np.stack(samples), allow_pickle=False)
    (output/'physics_trace.jsonl').write_text(''.join(json.dumps(sample)+'\n' for sample in trace))
    return {'mode': 'team-physics', 'status': status, 'success': status=='placed',
            'placement_status': status if result is not None else 'not_attempted', 'seed': seed,
            'object': target, 'destination': destination, 'evidence': evidence,
            'maximum_g1_pelvis_tilt_deg': maximum_tilt, 'trace_interval_s': INTERVAL,
            'g1_bilateral_contact_fraction': float(bilateral_samples / max(1, len(trace))),
            'duration_s': float(data.time), 'handoff_status': message['status']}


def view_recording(model, data, output, report, port):
    """Play the recorded physics frames at their simulation timestamps."""
    from time import perf_counter, sleep

    import viser
    from mjviser.scene import ViserMujocoScene

    frames = np.load(output / 'qpos.npy', allow_pickle=False)
    stages = ['start'] + [json.loads(line)['stage'] for line in (output/'physics_trace.jsonl').open()]
    server = viser.ViserServer(host='127.0.0.1', port=port)
    server.initial_camera.position = (-1.8, -3.9, 1.95)
    server.initial_camera.look_at = (-.18, -.83, .62)
    server.initial_camera.up = (0., 0., 1.)
    server.gui.add_markdown(f"**Recorded MuJoCo team run** · {report['status']}")
    status = server.gui.add_markdown('')
    playing = server.gui.add_checkbox('Play', initial_value=True)
    seek = server.gui.add_slider('Time (s)', min=0., max=max(INTERVAL, (len(frames)-1)*INTERVAL),
                                step=INTERVAL, initial_value=0.)
    focus = server.gui.add_button('G1 close-up')
    overview = server.gui.add_button('Overview')

    @focus.on_click
    def zoom(event):
        if event.client is not None:
            event.client.camera.position = (-.2, -2.1, 1.35)
            event.client.camera.look_at = (.90, -.72, .89)

    @overview.on_click
    def wide(event):
        if event.client is not None:
            event.client.camera.position = (-1.8, -3.9, 1.95)
            event.client.camera.look_at = (-.18, -.83, .62)

    scene = ViserMujocoScene(server, model, num_envs=1)
    scene.camera_tracking_enabled = False
    index, last = 0, -1
    clock = perf_counter()
    try:
        while True:
            if playing.value:
                index = (index + 1) % len(frames)
                seek.value = index*INTERVAL
            else:
                index = min(round(seek.value/INTERVAL), len(frames)-1)
            if index != last:
                data.qpos[:] = frames[index]
                mj.mj_forward(model, data)
                scene.update_from_mjdata(data)
                status.content = f'{index*INTERVAL:.2f} s · **{stages[index].replace("_", " ")}**'
                last = index
            clock += INTERVAL
            sleep(max(0., clock-perf_counter()))
    except KeyboardInterrupt:
        server.stop()
