"""Free-base Dex3 manipulation using online world-frame IK and torque targets."""
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import PositionIK
from robots.g1.pick_place import ARM, CLOSED, FINGERS, RADIUS, TOOL_OFFSET, WRIST, contact_summary


MAX_OBJECT_PENETRATION_M = .00075


def tabletop_stages(source, destination, stance, *, transfer_yaw=0.):
    """Use the same tabletop tool poses for reach checks and execution."""
    displacement = source[:2] - stance
    grasp_yaw = .42 if displacement[0] < .31 and displacement[1] > -.075 else 0.
    grasp_rotation = Rotation.from_rotvec([0, 0, grasp_yaw]).as_matrix()
    source = source + grasp_rotation @ np.array([-.012, -.018, .018])
    pinch_angle = Rotation.from_euler('xyz', [-.6, .25, 0]).as_matrix()
    angle = grasp_rotation @ pinch_angle
    entry_y = stance[1] - .14 if source[1] > stance[1] - .075 else source[1]
    entry = np.array([stance[0] + .145, entry_y, .915])
    entry_angle = Rotation.from_euler('xyz', [-.6, .15, 0]).as_matrix()
    yaw_rotation = Rotation.from_rotvec([0, 0, transfer_yaw]).as_matrix()
    tray_angle = yaw_rotation @ pinch_angle
    tray_center = destination + yaw_rotation @ np.array([-.012, -.018, 0])
    above = tray_center + [0, 0, .100]
    release = tray_center + [0, 0, .069]
    clear = tray_center + [0, 0, .120]
    return (
        ('raise_arm', entry, entry_angle, False, 2.5),
        ('pregrasp', source + [0, 0, .046], angle, False, 1.6),
        ('approach', source, angle, False, 1.4),
        ('close', source, angle, True, 1.5),
        ('lift', source + [0, 0, .074], angle, True, 1.2),
        ('transfer', above, tray_angle, True, 1.8),
        ('lower', release, tray_angle, True, 1.3),
        ('release', release, tray_angle, False, 1.2),
        ('clear_tray', clear, tray_angle, False, 1.2),
        ('settle', clear, tray_angle, False, 1.0),
    )


def right_arm(model):
    joints = ['g1/' + name for name in ARM]
    return PositionIK(model, 'g1/' + WRIST, joints, TOOL_OFFSET)


class TabletopReach:
    """Check reach at a proposed stance in scratch data without moving the live robot."""

    def __init__(self, model, pelvis):
        self.scratch = mj.MjData(model)
        self.arm = right_arm(model)
        self.base = int(model.jnt_qposadr[model.body_jntadr[pelvis]])

    def check(self, data, stance, source, destination, *, transfer_yaw=0.):
        scratch = self.scratch
        scratch.qpos[:] = data.qpos
        scratch.qpos[self.base:self.base + 2] = stance
        for name, point, rotation, _, _ in tabletop_stages(
                source, destination, stance, transfer_yaw=transfer_yaw):
            result = self.arm.solve(scratch, point, orientation=rotation,
                                    tolerance=.002, max_iterations=60)
            if not result.converged:
                return {'reachable': False, 'blocked_stage': name, 'ik_error_m': result.error_m}
        return {'reachable': True}


def tray_clearance(model, data, object_geom, tray_axes):
    """Project the object shape onto the two tray axes."""
    size = model.geom_size[object_geom]
    relative = tray_axes.T @ data.geom_xmat[object_geom].reshape(3, 3)
    kind = model.geom_type[object_geom]
    if kind == mj.mjtGeom.mjGEOM_BOX:
        return np.abs(relative[:2]) @ size
    if kind == mj.mjtGeom.mjGEOM_CYLINDER:
        alignment = np.clip(np.abs(relative[:2, 2]), 0, 1)
        return size[0] * np.sqrt(1 - alignment**2) + size[1] * alignment
    return size[0]


class GraspController:

    def __init__(self, model, data, stand):
        self.model = model
        self.data = data
        self.stand = stand
        self.planner = mj.MjData(model)
        self.arm = right_arm(model)
        self.arm_motors = np.array([model.actuator('g1/' + name).id for name in ARM])
        self.fingers = np.array([model.actuator('g1/' + name).id for name in FINGERS])
        self.stand.kp[np.array([model.actuator(i).name.startswith('g1/') and 'hand_' not in model.actuator(i).name for i in range(model.nu)])] = 240
        self.stand.ankle_attitude_gain = 60
        self.stand.kp[self.arm_motors] = 260
        self.stand.kd[self.arm_motors] = 6
        self.stand.kp[self.fingers] = 32
        self.stand.kd[self.fingers] = .32
        self.goal = stand.target[self.arm_motors].copy()
        self.gravity_torque = np.zeros(len(self.arm_motors))
        self.root_reference = data.xpos[stand.pelvis, :2].copy()
        self.balance_kp = 90.0
        self.balance_kd = 16.0

    def apply(self):
        offset = self.data.xpos[self.stand.pelvis, :2] - self.root_reference
        velocity = self.data.qvel[self.stand.root_dof:self.stand.root_dof + 2]
        correction = np.clip(self.balance_kp * offset + self.balance_kd * velocity, -9, 9)
        for ids, torque in ((self.stand.ankle_pitch, correction[0]), (self.stand.ankle_roll, -correction[1])):
            self.data.ctrl[ids] = np.clip(self.data.ctrl[ids] + torque, self.stand.lower[ids], self.stand.upper[ids])
        ids = self.arm_motors
        self.data.ctrl[ids] = np.clip(self.data.ctrl[ids] + self.gravity_torque,
                                     self.stand.lower[ids], self.stand.upper[ids])

    def solve(self, target, rotation):
        self.planner.qpos[:] = self.data.qpos
        self.planner.qpos[self.arm.qpos] = self.goal
        result = self.arm.solve(self.planner, target, orientation=rotation,
                                tolerance=0.001, max_iterations=50)
        if result.converged:
            self.goal = self.planner.qpos[self.arm.qpos].copy()
            self.gravity_torque[:] = self.planner.qfrc_bias[self.arm.dofs]
        return result

    def follow(self, source, destination, object_geom, tray, tick, record, *, tabletop=False, transfer_yaw=0.):
        identity = np.eye(3)
        angle = identity
        opened = np.zeros(len(FINGERS))
        closed = np.array(CLOSED)
        # Keep the unused middle digit above the low cradle lip during the two-digit pinch.
        closed[3:5] = 0
        pregrasp = source + [-.095, 0, .006]
        if tabletop:
            closed[5:] *= 1.35
            self.stand.lower[self.fingers] = -.65
            self.stand.upper[self.fingers] = .65
        elevated = source + [0, 0, .074]
        above = destination + [-.012, 0, .135]
        release = destination + [0, 0, .100]
        rollout = Rotation.from_rotvec([-1.57, 0, 0]).as_matrix()
        clear_tray = release + [0, 0, .035]
        retreat = clear_tray + [-.10, 0, 0]
        stages = (('pregrasp', pregrasp, angle, opened, 2.3),
                  ('approach', source, angle, opened, 1.35),
                  ('close', source, angle, closed, 1.45),
                  ('lift', elevated, angle, closed, 1.20),
                  ('transfer', above, angle, closed, 1.8),
                  ('orient_release', above, rollout, closed, 1.5),
                  ('lower', release, rollout, closed, 1.4),
                  ('release', release, rollout, opened, 1.2),
                  ('clear_tray', clear_tray, rollout, opened, 1.05),
                  ('retreat', retreat, rollout, opened, 1.4),
                  ('settle', retreat, rollout, opened, 1.25))
        if tabletop:
            stages = tuple((name, xyz, rotation, closed if grip else opened, duration)
                           for name, xyz, rotation, grip, duration in
                           tabletop_stages(source, destination, self.root_reference, transfer_yaw=transfer_yaw))
        dt = self.model.opt.timestep
        period = round(.04 / dt)
        baseline_z = float(self.data.geom_xpos[object_geom, 2])
        lifted = 0.0
        airborne_grip_steps = 0
        settled_steps = 0
        worst_penetration = 0.0
        penetration_contact = {}
        object_body = self.model.geom_bodyid[object_geom]
        object_dof = self.model.jnt_dofadr[self.model.body_jntadr[object_body]]
        for stage, point, rotation, fingers, duration in stages:
            previous_fingers = self.stand.target[self.fingers].copy()
            first_point = self.arm.point(self.data).copy()
            current = Rotation.from_matrix(self.data.xmat[self.arm.body].reshape(3, 3).copy())
            rotation_delta = (current.inv() * Rotation.from_matrix(rotation)).as_rotvec()
            self.goal = self.data.qpos[self.arm.qpos].copy()
            joint_interpolation = stage == ('raise_arm' if tabletop else 'pregrasp')
            if joint_interpolation:
                result = self.solve(point, rotation)
                if not result.converged:
                    record({'stage': stage, 'ik_error_m': result.error_m})
                    return 'ik_failed', {}
            start_joints = self.data.qpos[self.arm.qpos].copy()
            ik_failed_steps = 0
            max_ik_error = 0.0
            opposed = 0
            consecutive_grip = 0
            grip_gap = 0
            for i in range(round(duration / dt)):
                t = min(1.0, (i+1) * dt / (duration * .78))
                blend = t*t*(3-2*t)
                if i % period == 0 and (not joint_interpolation or t == 1):
                    goal_point = first_point + blend * (point - first_point)
                    goal_rotation = (current * Rotation.from_rotvec(blend * rotation_delta)).as_matrix()
                    result = self.solve(goal_point, goal_rotation)
                    max_ik_error = max(max_ik_error, result.error_m)
                    ik_failed_steps = 0 if result.converged else ik_failed_steps + 1
                self.stand.target[self.arm_motors] = start_joints + blend * (self.goal - start_joints) if joint_interpolation else self.goal
                self.stand.target[self.fingers] = previous_fingers + blend * (fingers-previous_fingers)
                tick(stage)
                if self.data.xmat[self.stand.pelvis, 8] < .85 or self.data.xpos[self.stand.pelvis, 2] < .61:
                    record({'stage': stage, 'time_s': self.data.time, 'reason': 'G1 pelvis lost support'})
                    return 'g1_lost_balance', {}
                if ik_failed_steps >= 9:
                    record({'stage': stage, 'ik_error_m': max_ik_error, 'target_xyz_m': point.tolist()})
                    return 'ik_failed', {'lift_height_m': lifted, 'airborne_grip_seconds': airborne_grip_steps*dt}
                contacts, supports = contact_summary(self.model, self.data, object_geom, prefix='g1/')
                for contact in self.data.contact:
                    if object_geom in contact.geom:
                        depth = -float(contact.dist)
                        if depth > worst_penetration:
                            worst_penetration = depth
                            penetration_contact = {'stage': stage, 'time_s': float(self.data.time),
                                                   'geoms': [self.model.geom(j).name or self.model.body(self.model.geom_bodyid[j]).name for j in contact.geom]}
                has_grip = 'thumb' in contacts and ('index' in contacts or 'middle' in contacts)
                opposed += has_grip
                consecutive_grip = consecutive_grip + 1 if has_grip else 0
                z = float(self.data.geom_xpos[object_geom, 2])
                lifted = max(lifted, z-baseline_z)
                airborne_grip_steps += has_grip and not supports and z-baseline_z > .055
                if tabletop and stage in ('transfer', 'lower'):
                    grip_gap = 0 if has_grip and not supports else grip_gap + 1
                    if grip_gap * dt > .08:
                        record({'stage': stage, 'time_s': self.data.time, 'reason': 'lost airborne grip',
                                'finger_force_n': contacts, 'support_contacts': supports})
                        return 'dropped_in_transfer', {'lift_height_m': lifted, 'failed_stage': stage}
                if stage in ('release', 'clear_tray', 'retreat', 'settle'):
                    tray_name = self.model.geom(tray).name
                    object_pos = self.data.geom_xpos[object_geom]
                    tray_axes = self.data.geom_xmat[tray].reshape(3, 3)
                    tray_local = tray_axes.T @ (object_pos - self.data.geom_xpos[tray])
                    clearance = tray_clearance(self.model, self.data, object_geom, tray_axes) if tabletop else RADIUS
                    if tabletop:
                        clearance += .006  # Tray rim thickness.
                    inside = np.all(np.abs(tray_local[:2]) + clearance < self.model.geom_size[tray, :2])
                    is_resting = (tray_name in supports and inside and not contacts and
                                  np.linalg.norm(self.data.qvel[object_dof:object_dof+3]) < .05)
                    settled_steps = settled_steps + 1 if is_resting else 0
                if stage == 'close' and consecutive_grip * dt >= .025:
                    break
            record({'stage': stage, 'time_s': float(self.data.time), 'opposed_seconds': float(opposed * dt),
                    'tool_error_m': float(np.linalg.norm(self.arm.point(self.data)-point)),
                    'tool_xyz_m': self.arm.point(self.data).tolist(), 'arm_joint_error_rad': (self.goal-self.data.qpos[self.arm.qpos]).tolist(),
                    'object_xyz_m': self.data.geom_xpos[object_geom].tolist(), 'end_finger_force_n': contacts,
                    'end_supports': supports})
            if stage == 'close' and consecutive_grip * dt < .025:
                return 'no_opposed_grasp', {'lift_height_m': lifted}
            if stage == 'lift' and airborne_grip_steps * dt < .06:
                return 'lift_failed', {'lift_height_m': lifted, 'failed_stage': stage}
            if stage in ('transfer', 'orient_release', 'lower') and not has_grip:
                return 'dropped_in_transfer', {'lift_height_m': lifted, 'failed_stage': stage}
        evidence = {'lift_height_m': lifted, 'airborne_grip_seconds': float(airborne_grip_steps * dt),
                    'settled_tray_seconds': float(settled_steps * dt), 'peak_object_penetration_m': worst_penetration,
                    'penetration_contact': penetration_contact}
        evidence['object_penetration_limit_m'] = MAX_OBJECT_PENETRATION_M
        if worst_penetration > MAX_OBJECT_PENETRATION_M:
            if tabletop:
                evidence['failed_stage'] = penetration_contact['stage']
            return 'excess_penetration', evidence
        return ('placed' if settled_steps * dt >= .4 else 'placement_failed'), evidence
