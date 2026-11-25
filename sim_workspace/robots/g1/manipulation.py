"""Unitree G1 foot IK and Dex3-1 pregrasp approach."""
from pathlib import Path

import mujoco as mj
import numpy as np

from kinematics import PositionIK, walk

MODEL_XML = (Path(__file__).resolve().parents[3] / 'third_party/unitree_ros'
             / 'robots/g1_description/g1_29dof_with_hand_rev_1_0.xml')


def g1_feet(model):
    joints = ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')
    return [PositionIK(model, f'g1/{side}_ankle_roll_link',
                       [f'g1/{side}_{joint}_joint' for joint in joints], (0.03, 0, -0.035))
            for side in ('left', 'right')]


def g1_hand(model, side):
    joints = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
              'wrist_roll', 'wrist_pitch', 'wrist_yaw')
    return PositionIK(model, f'g1/{side}_wrist_yaw_link',
                      [f'g1/{side}_{joint}_joint' for joint in joints], (0.085, 0, 0))


def arm_clearance(model, data, hand):
    """Reject deep contacts of the moving arm against the scene or either robot."""
    shoulder = model.jnt_bodyid[model.dof_jntid[hand.dofs[0]]]
    arm_bodies = {int(shoulder)}
    for body, parent in enumerate(model.body_parentid):
        if body and int(parent) in arm_bodies:
            arm_bodies.add(body)
    for contact in data.contact:
        first, second = (int(model.geom_bodyid[geom]) for geom in contact.geom)
        if (first in arm_bodies or second in arm_bodies) and contact.dist < -0.004:
            return False
    return True


def g1_stance(model, data, base, feet):
    for side in ('left', 'right'):
        for joint, angle in (('hip_pitch', -0.27), ('knee', 0.55), ('ankle_pitch', -0.28)):
            data.qpos[model.joint(f'g1/{side}_{joint}_joint').qposadr[0]] = angle
    mj.mj_forward(model, data)
    data.qpos[base + 2] -= np.mean([foot.point(data)[2] for foot in feet])
    mj.mj_forward(model, data)


def approach_with_ik(model, data, base, point, snapshots, record):
    sign = 1 if point[1] > 0 else -1
    feet = g1_feet(model)
    lane_y = sign * 0.55
    route = [(-1.65, lane_y, data.qpos[base + 2]), (point[0] - 0.29, lane_y, 0.66)]
    for route_goal in route:
        poses, errors, failed = walk(model, data, base, feet, route_goal, 80, (0, 0.5),
                                      0.025, keep_sole_attitude=True)
        snapshots.extend(poses)
        record({'stage': 'g1_foot_ik', 'converged': failed is None,
                       'samples': len(poses), 'max_error_m': max(errors, default=None),
                       'failed_error_m': None if failed is None else failed.error_m})
        if failed is not None:
            return 'g1_foot_ik_failed'

    side = 'right' if sign > 0 else 'left'
    hand = g1_hand(model, side)
    goal = point + [-0.06, sign * 0.04, 0.15]
    tools = np.linspace(hand.point(data), goal, 20)[1:]
    clearance = lambda state: arm_clearance(model, state, hand)
    for waypoint in tools:
        result = hand.solve(data, waypoint, accept=clearance)
        if not result.converged:
            break
        snapshots.append(data.qpos.copy())
    # Point the nominal tool x-axis toward the observed object surface.
    tool_x = point - goal
    tool_x /= np.linalg.norm(tool_x)
    tool_y = np.cross([0, 0, 1], tool_x)
    tool_y /= np.linalg.norm(tool_y)
    orientation = np.column_stack([tool_x, tool_y, np.cross(tool_x, tool_y)])
    if result.converged:
        result = hand.solve(data, goal, accept=clearance, orientation=orientation, trace=snapshots)
    actual_orientation = data.xmat[hand.body].reshape(3, 3)
    attitude_cosine = np.clip((np.trace(actual_orientation.T @ orientation) - 1) / 2, -1, 1)
    record({'stage': 'g1_pregrasp_ik', 'converged': result.converged, 'arm': side,
                   'frame': 'world', 'goal_xyz_m': goal.tolist(),
                   'achieved_xyz_m': hand.point(data).tolist(),
                   'error_m': float(np.linalg.norm(goal - hand.point(data))),
                   'goal_rotation_world': orientation.tolist(),
                   'orientation_error_rad': float(np.arccos(attitude_cosine)),
                   'joint_names': [model.joint(model.dof_jntid[dof]).name for dof in hand.dofs],
                   'joint_angles_rad': data.qpos[hand.qpos].tolist()})
    return 'pregrasp_reached' if result.converged else 'g1_pregrasp_ik_failed'
