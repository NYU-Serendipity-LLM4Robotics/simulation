"""Flat tabletop trial: RGB-D scan, velocity-policy approach and contact grasp."""
import json

import mujoco as mj
import numpy as np
from scipy.ndimage import label
from scipy.optimize import least_squares

import handoff
import room
import task_room as scene
from robots.g1.locomotion import Locomotion, REVISION
from robots.g1.room_grasp import GraspController, TabletopReach
from robots.g1.pick_place import contact_summary
from robots.go2.trot import Trot
from team_physics import CAMERA_HEIGHT, CAMERA_WIDTH, INTERVAL


def observe(renderer, model, data, target, index, output, *, reacquiring=False):
    camera = model.camera('go2/rgbd')
    renderer.update_scene(data, camera=camera.id)
    rgb = renderer.render().copy()
    renderer.enable_depth_rendering()
    depth = renderer.render().copy()
    renderer.disable_depth_rendering()
    red, green, blue = rgb.astype(float).transpose(2, 0, 1)
    masks = {
        'red apple': (red > 45) & (red > 1.8 * green) & (red > 1.8 * blue),
        'blue block': (blue > 55) & (blue > 1.7 * red) & (blue > 1.6 * green),
        'purple cylinder': (red > 55) & (blue > 55) & (np.maximum(red, blue) < 1.6 * np.minimum(red, blue))
                           & (np.minimum(red, blue) > 2 * green),
    }
    mask = masks[target] & np.isfinite(depth) & (depth > 0)
    if reacquiring:
        components, count = label(mask)
        if not count:
            return None
        areas = np.bincount(components.ravel())
        areas[0] = 0
        mask = components == np.argmax(areas)
    rows, cols = np.nonzero(mask)
    focal = CAMERA_HEIGHT / (2 * np.tan(np.deg2rad(camera.fovy.item()) / 2))
    cx, cy = (CAMERA_WIDTH - 1) / 2, (CAMERA_HEIGHT - 1) / 2
    origin = data.cam_xpos[camera.id].copy()
    rotation = data.cam_xmat[camera.id].reshape(3, 3).copy()
    if len(rows) < 12:
        return None
    z = depth[rows, cols]
    points = np.c_[((cols - cx) * z / focal), (-(rows - cy) * z / focal), -z] @ rotation.T + origin
    keep = (points[:, 2] > scene.TABLE_HEIGHT + .005) & (points[:, 2] < scene.TABLE_HEIGHT + .12)
    if reacquiring:
        keep &= np.all(abs(points[:, :2] - scene.LAB_TABLE_XY) < [.25, .37], axis=1)
    points = points[keep]
    if len(points) < 12:
        return None
    _, shape, size, _, _ = scene.LAB_OBJECTS[target]
    center_height = scene.TABLE_HEIGHT + size[-1]
    surface = np.median(points, axis=0)
    center = surface.copy()
    center[2] = center_height
    view = surface[:2] - origin[:2]
    view /= np.linalg.norm(view)
    if shape in ('sphere', 'cylinder'):
        center[:2] += size[0] * view
        residual = (lambda xy: np.linalg.norm(points - np.r_[xy, center_height], axis=1) - size[0]
                    if shape == 'sphere' else np.linalg.norm(points[:, :2] - xy, axis=1) - size[0])
        fit = least_squares(residual, center[:2], loss='soft_l1', f_scale=.003)
        valid_fit = fit.success and np.linalg.norm(fit.x - center[:2]) < .035
        if reacquiring:
            if not valid_fit or np.median(np.abs(fit.fun)) > .0025:
                return None
        if valid_fit:
            center[:2] = fit.x
    else:
        center[:2] += .012 * view
        if reacquiring:
            # Fit the visible face of the upright box. A narrow fragment leaves its
            # lateral center ambiguous, so keep searching for a less occluded view.
            normal = int(np.argmin(np.ptp(points[:, :2], axis=0)))
            tangent = 1 - normal
            if np.ptp(points[:, tangent]) < 1.3 * size[tangent] or abs(view[normal]) < .35:
                return None
            center[normal] = surface[normal] + np.sign(view[normal]) * size[normal]
            bounds = np.percentile(points[:, tangent], [5, 95])
            center[tangent] = np.mean(bounds)
    capture = output / 'rgbd'
    capture.mkdir(exist_ok=True)
    np.savez_compressed(capture / f'{index:03d}.npz', rgb=rgb, depth_m=depth, mask=mask,
                        intrinsics=[[focal, 0, cx], [0, focal, cy], [0, 0, 1]],
                        camera_xyz_m=origin, camera_to_world_rotation=rotation, time_s=data.time)
    return {'xyz_m': center.tolist(), 'capture_index': index,
            'method': 'rgbd_color_table_shape_prior', 'pixel_count': len(points)}


def run(model, data, seed, target, destination, output, weights, record):
    stand = room.StandController(model, data)
    trot = Trot(model)
    locomotion = Locomotion(stand, model, data, weights)
    grasp = None
    command = np.zeros(3)
    g1_command = np.zeros(3)
    floor = model.geom('floor').id
    soles = {model.geom(f'g1/{side}_sole').id for side in ('left', 'right')}
    dt = model.opt.timestep
    sample_steps = round(INTERVAL / dt)
    step = 0
    poses, trace = [data.qpos.copy()], []
    object_geom = model.geom(scene.LAB_OBJECTS[target][0]).id
    initial_object = data.geom_xpos[object_geom].tolist()
    tray = model.geom(destination.split()[0] + '_tray').id
    # Tray is a registered lab landmark. Its initial pose is known to the task coordinator.
    goal = data.geom_xpos[tray].copy()
    g1_start = data.xpos[stand.pelvis, :2].copy()
    scout_start = data.xpos[trot.root, :2].copy()

    def grounded():
        return {int(c.geom2 if c.geom1 == floor else c.geom1) for c in data.contact
                if floor in c.geom and c.efc_address >= 0} & soles

    def tick(stage):
        nonlocal step
        if grasp is None:
            locomotion.apply(g1_command)
        stand.apply(data)
        if grasp is not None:
            grasp.apply()
        trot.apply(model, data, command)
        mj.mj_step(model, data)
        step += 1
        if step % sample_steps == 0:
            fingers, support = contact_summary(model, data, object_geom, prefix='g1/')
            poses.append(data.qpos.copy())
            trace.append({'time_s': float(data.time), 'stage': stage,
                          'go2_xyz_m': data.xpos[trot.root].tolist(),
                          'g1_xyz_m': data.xpos[stand.pelvis].tolist(),
                          'g1_tilt_deg': float(np.degrees(np.arccos(np.clip(data.xmat[stand.pelvis, 8], -1, 1)))),
                          'object_xyz_m': data.geom_xpos[object_geom].tolist(),
                          'finger_force_n': fingers, 'object_support_contacts': support,
                          'tray_xyz_m': data.geom_xpos[tray].tolist(),
                          'g1_grounded_soles': sorted(model.geom(i).name for i in grounded())})

    def walk_to(stance, phase, *, precise=False):
        stable = 0
        record({'stage': phase, 'time_s': float(data.time), 'goal_xy_m': stance.tolist()})
        status = 'g1_walk_timeout'
        for _ in range(round(13 / dt)):
            rotation = data.xmat[stand.pelvis].reshape(3, 3)
            error = stance - data.xpos[stand.pelvis, :2]
            distance = np.linalg.norm(error)
            horizontal_speed = np.linalg.norm(data.qvel[stand.root_dof:stand.root_dof + 2])
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            local = rotation[:2, :2].T @ error
            g1_command[:] = np.clip([1.6 * local[0], 2 * local[1], -2 * yaw],
                                     [-.18, -.24, -.20], [.37, .24, .20])
            if precise:
                # Small commands fall in the velocity policy's standing regime.
                g1_command[:2] = np.where(abs(local) > [.022, .025],
                                          np.sign(local) * np.maximum(abs(g1_command[:2]), [.14, .27]), 0.)
            elif distance < .065:
                g1_command[:2] = 0
            tick(phase)
            if not g1_standing or rotation[2, 2] < .65 or data.xpos[stand.pelvis, 2] < .55:
                status = 'g1_lost_balance'
                break
            still = distance < (.040 if precise else .075) and abs(yaw) < .16 and horizontal_speed < .12
            stable = stable + 1 if still and grounded() == soles else 0
            if stable * dt >= .12:
                status = 'arrived'
                break
        error = float(np.linalg.norm(stance - data.xpos[stand.pelvis, :2]))
        record({'stage': phase, 'status': status, 'goal_error_m': error,
                'actual_xy_m': data.xpos[stand.pelvis, :2].tolist()})
        return status, error


    stage = 'go2_search'
    status = 'target_not_visible'
    record({'stage': stage, 'target': target})
    result = None
    capture_index = 0
    g1_standing = True
    table_xy = np.asarray(scene.LAB_TABLE_XY)
    with mj.Renderer(model, width=CAMERA_WIDTH, height=CAMERA_HEIGHT) as renderer:
        # Keep the initial survey, then change line of sight using clear aisles.
        viewpoints = (
            ('southwest', (-1.74, -.86), 4),
            ('west_south', (-1.65, -.52), 4),
            ('west', (-1.58, -.26), 4),
            ('northwest_aisle', (-1.55, .65), 9),
            ('north', (-.55, .78), 10),
            ('northeast', (.30, .78), 10),
            ('east', (.73, .15), 10),
        )
        for view_index, (name, offset, timeout) in enumerate(viewpoints):
            waypoint = table_xy + offset
            record({'stage': stage, 'event': 'scout_viewpoint', 'view': name, 'index': view_index,
                    'time_s': float(data.time), 'goal_xy_m': waypoint.tolist()})
            for i in range(round(timeout / dt)):
                position = data.xpos[trot.root]
                rotation = data.xmat[trot.root].reshape(3, 3)
                local = rotation[:2, :2].T @ (waypoint - position[:2])
                look = rotation[:2, :2].T @ (table_xy - position[:2])
                heading_error = np.arctan2(look[1], look[0])
                command[:] = np.clip([.9 * local[0], 1.2 * local[1], 1.7 * heading_error],
                                     [-.16, -.12, -.40], [.26, .12, .40])
                tick(stage)
                g1_standing &= data.xmat[stand.pelvis, 8] > .65
                if data.xmat[trot.root, 8] < .72:
                    status = 'go2_lost_balance'
                    break
                if i % round(.28 / dt) == 0 and np.linalg.norm(position[:2] - scout_start) > .35:
                    result = observe(renderer, model, data, target, capture_index, output,
                                     reacquiring=view_index >= 3)
                    capture_index += 1
                    if result is not None:
                        result['view'] = name
                        break
                    if view_index >= 3 and np.linalg.norm(local) < .12 and abs(heading_error) < .23:
                        break
            if result is not None or status == 'go2_lost_balance':
                break
    command[:] = 0
    message = handoff.from_scout(seed, target, destination, goal.tolist(), result, False,
                                 scene_id=scene.LAB_SCENE_ID, position_kind='object_center', failure_status=status)
    handoff.validate(message)
    (output / 'handoff.json').write_text(json.dumps(message, indent=2) + '\n')
    evidence = {}
    if result is not None:
        source = np.array(message['object_xyz_m'])
        record({'stage': 'handoff', 'center_error_m': float(np.linalg.norm(source - data.geom_xpos[object_geom])),
                'go2_displacement_m': float(np.linalg.norm(data.xpos[trot.root, :2] - scout_start)),
                'object_xyz_m': source.tolist(), 'destination_xyz_m': goal.tolist(), 'view': result['view']})
        stage = 'g1_walk'
        initial_stance = np.array([.62, np.clip(source[1] + .12, -1.00, -.44)])
        status, error = walk_to(initial_stance, stage)
        evidence['g1_goal_error_m'] = error
        reach = TabletopReach(model, stand.pelvis)
        tried = [initial_stance]
        transfer_yaw = 0.
        while status == 'arrived':
            stage = 'g1_stabilize'
            g1_command[:] = 0
            for _ in range(round(.65 / dt)):
                tick(stage)
            if data.xmat[stand.pelvis, 8] < .93 or grounded() != soles:
                status = 'g1_lost_balance'
                break
            stage = 'g1_reach_check'
            actual = data.xpos[stand.pelvis, :2].copy()
            check = reach.check(data, actual, source, goal, transfer_yaw=transfer_yaw)
            record({'stage': stage, 'time_s': float(data.time), 'stance_xy_m': actual.tolist(),
                    'transfer_yaw_rad': transfer_yaw, **check})
            if check['reachable']:
                break

            stage = 'g1_reposition'
            # Search the clear strip along the west edge. Center candidate stances between
            # the two tool locations, with room for the right arm on the south side.
            southward = goal[1] < source[1]
            goal_weight = .22 if southward else .5
            center_y = (1 - goal_weight) * source[1] + goal_weight * goal[1] + .10
            # Keep additional arm clearance for a workpiece close to the near edge.
            near_edge = source[0] < scene.LAB_TABLE_XY[0] - .11
            columns = (.60, .62, .64, .67) if near_edge else (.65, .68, .62, .70)
            lateral_offsets = (0., -.035, .035, -.07, .07, -.105, .105)
            candidates = [np.array([x, center_y + dy]) for dy in lateral_offsets for x in columns]
            candidates = [p for p in candidates if all(np.linalg.norm(p - previous) > .032
                                                       for previous in tried)]
            if len(tried) >= 4:
                candidates = []
            chosen = None
            for candidate in candidates:
                direction = np.arctan2(goal[1] - candidate[1], goal[0] - candidate[0])
                for yaw in (0., .70 * direction, direction):
                    preview = reach.check(data, candidate, source, goal, transfer_yaw=yaw)
                    record({'stage': stage, 'event': 'candidate', 'stance_xy_m': candidate.tolist(),
                            'transfer_yaw_rad': float(yaw), **preview})
                    if preview['reachable']:
                        chosen, transfer_yaw = candidate, float(yaw)
                        break
                if chosen is not None:
                    break
            if chosen is None:
                status = 'g1_no_reachable_stance'
                break
            tried.append(chosen)
            status, error = walk_to(chosen, stage, precise=True)
            evidence['g1_goal_error_m'] = error

        evidence['g1_reposition_attempts'] = len(tried) - 1
        evidence['g1_walk_m'] = float(np.linalg.norm(data.xpos[stand.pelvis, :2] - g1_start))
        if status == 'arrived':
            stand.target[:] = data.qpos[stand.qpos]
            grasp = GraspController(model, data, stand)
            grasp.balance_kp = 140
            grasp.balance_kd = 25

            def record_grasp(event):
                nonlocal stage
                stage = event['stage']
                record(event)

            def tick_grasp(active_stage):
                nonlocal stage
                stage = active_stage
                tick(stage)

            status, metrics = grasp.follow(source, goal, object_geom, tray, tick_grasp,
                                           record_grasp, tabletop=True, transfer_yaw=transfer_yaw)
            evidence.update(metrics)
            stage = metrics.get('failed_stage', stage)
    if status == 'placed' and (np.any(data.warning.number) or not np.isfinite(data.qpos).all()):
        status = 'physics_warning'
    np.save(output / 'qpos.npy', np.stack(poses), allow_pickle=False)
    (output / 'physics_trace.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in trace))
    before_grasp = {'go2_search', 'g1_walk', 'g1_stabilize', 'g1_reach_check', 'g1_reposition'}
    return {'mode': 'lab-trial', 'status': status, 'success': status == 'placed',
            'placement_status': 'not_attempted' if stage in before_grasp else status,
            'failure_stage': None if status == 'placed' else stage,
            'seed': seed, 'object': target, 'destination': destination, 'evidence': evidence,
            'policy_revision': REVISION, 'duration_s': float(data.time), 'trace_interval_s': INTERVAL,
            'handoff_status': message['status'], 'initial_object_xyz_m': initial_object,
            'tray_xyz_m': goal.tolist(), 'final_tray_xyz_m': data.geom_xpos[tray].tolist()}
