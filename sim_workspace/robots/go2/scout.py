"""Unitree Go2 scouting, foot IK and RGB-D localization."""
from pathlib import Path

import mujoco as mj
import numpy as np

from kinematics import PositionIK, walk

MODEL_XML = (Path(__file__).resolve().parents[3] / 'third_party/unitree_mujoco'
             / 'unitree_robots/go2/go2.xml')
WIDTH, HEIGHT = 320, 240


def configure_camera(child):
    # Camera on a simulated mast, pointed slightly down and forward.
    child.worldbody.bodies[0].add_camera(
        name='rgbd', pos=[0.22, 0, 0.51],
        xyaxes=[0, -1, 0, 0.26, 0, 0.9656], fovy=55,
    )


def initialize_pose(data, base):
    data.qpos[base + 7:base + 19] = [0, 0.9, -1.8] * 4


def go2_feet(model):
    return [PositionIK(model, f'go2/{leg}_foot',
                       [f'go2/{leg}_{joint}_joint' for joint in ('hip', 'thigh', 'calf')])
            for leg in ('FL', 'FR', 'RL', 'RR')]


def capture(renderer, data):
    renderer.update_scene(data, camera='go2/rgbd')
    rgb = renderer.render().copy()
    renderer.enable_depth_rendering()
    depth = renderer.render().copy()
    renderer.disable_depth_rendering()
    renderer.enable_segmentation_rendering()
    segmentation = renderer.render().copy()
    renderer.disable_segmentation_rendering()
    return rgb, depth, segmentation


def intrinsics(camera):
    focal = HEIGHT / (2 * np.tan(np.deg2rad(camera.fovy.item()) / 2))
    return np.array([[focal, 0, (WIDTH - 1) / 2],
                     [0, focal, (HEIGHT - 1) / 2], [0, 0, 1]])


def localize(mask, depth, calibration, position, rotation):
    rows, cols = np.nonzero(mask & np.isfinite(depth) & (depth > 0))
    if len(rows) < 12:
        return None
    z = depth[rows, cols]
    x = (cols - calibration[0, 2]) * z / calibration[0, 0]
    y = (rows - calibration[1, 2]) * z / calibration[1, 1]
    # MuJoCo camera axes are right, up, backward. Depth is optical-axis distance.
    camera_points = np.column_stack([x, -y, -z])
    return position + rotation @ np.median(camera_points, axis=0)


def run_scout(model, data, base, target, geom, output, snapshots, record, log):
    camera = model.camera('go2/rgbd')
    calibration = intrinsics(camera)
    report = None
    scout_poses, errors, scout_failure = walk(
        model, data, base, go2_feet(model), (0.08, 0, 0.29), 56, (0, 0.5, 0.5, 0), 0.045,
    )
    record({'stage': 'go2_foot_ik', 'converged': scout_failure is None,
                   'samples': len(scout_poses), 'max_error_m': max(errors, default=None),
                   'failed_error_m': None if scout_failure is None else scout_failure.error_m})
    log('initializing RGB-D renderer')
    with mj.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        for step, pose in enumerate(scout_poses, start=1):
            snapshots.append(pose)
            if step % 4:
                continue
            index = step // 4 - 1
            data.qpos[:] = pose
            mj.mj_forward(model, data)
            log(f'capturing RGB-D frame={index:03d}')
            rgb, depth, segmentation = capture(renderer, data)
            mask = (segmentation[..., 0] == geom) & (segmentation[..., 1] == int(mj.mjtObj.mjOBJ_GEOM))
            position = data.cam_xpos[camera.id].copy()
            rotation = data.cam_xmat[camera.id].reshape(3, 3).copy()
            np.savez_compressed(output / f'frame_{index:03}.npz', rgb=rgb, depth_m=depth,
                                segmentation=segmentation, target_mask=mask, K=calibration,
                                camera_position_world_m=position, camera_rotation_world=rotation)
            # Keep the oracle scan fixed-length for comparable datasets.
            if index < 7 or report is not None:
                continue
            point = localize(mask, depth, calibration, position, rotation)
            if point is not None:
                error = np.linalg.norm(point - data.geom_xpos[geom])
                report = {'stage': 'report', 'frame': 'world', 'capture_index': index,
                               'target': target, 'xyz_m': point.tolist(),
                               'method': 'oracle_mask_rgbd_surface_point', 'center_error_m': float(error)}
                record(report)
    return report, scout_failure
