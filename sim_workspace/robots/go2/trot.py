"""Feedback velocity controller using analytic swing/stance IK and foot support torques.

Gait baseline adapted from syswonder/robot-unitree-go2-mujoco, commit d6e5464.
See LICENSE-trot for the upstream MIT notice.
"""
import mujoco as mj
import numpy as np


class Trot:
    def __init__(self, model):
        legs = ('FL', 'FR', 'RL', 'RR')
        names = [f'go2/{leg}_{joint}' for leg in legs for joint in ('hip', 'thigh', 'calf')]
        self.actuators = [model.actuator(name).id for name in names]
        joints = model.actuator_trnid[self.actuators, 0]
        self.qpos = model.jnt_qposadr[joints]
        self.dofs = model.jnt_dofadr[joints]
        self.root = model.body('go2/base_link').id
        self.feet = [model.body(f'go2/{leg}_foot').id for leg in legs]
        self.foot_geoms = [model.geom_bodyid == body for body in self.feet]
        self.limits = model.actuator_ctrlrange[self.actuators]
        self.phase, self.blend = 0.0, 0.0
        self.velocity = np.zeros(6)
        self.integral = np.zeros(3)
        self.jac = np.zeros((3, model.nv))

    def apply(self, model, data, command):
        dt = model.opt.timestep
        desired = np.asarray(command, dtype=float) * np.clip((data.xmat[self.root, 8] - 0.72) / .18, 0, 1)
        active = np.linalg.norm(desired) > 0.005
        self.blend += (float(active) - self.blend) * 6 * dt
        self.phase = (self.phase + (2 * dt if self.blend > 0.01 else 0)) % 1
        mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, self.root, self.velocity, 1)
        cap = np.array([0.4, 0.15, 0.5])
        desired = np.clip(desired, -cap, cap)
        error = desired - self.velocity[[3, 4, 2]]
        raw = desired / cap + self.integral
        self.integral += np.array([4, 4, 2]) * dt * error * ((abs(raw) < 1) | (raw * error < 0))
        if not active:
            self.integral[:] = 0
        command = np.clip(desired / cap + self.integral, -1, 1) * self.blend
        targets = []
        for front, side, phase in ((1, 1, 0), (1, -1, .5), (-1, 1, .5), (-1, -1, 0)):
            cycle = (self.phase + phase) % 1
            stance = cycle < 0.56
            progress = cycle / .56 if stance else (cycle - .56) / .44
            travel = .5 - progress if stance else -.5 + progress
            x = .13 * np.clip(command[0] - side * command[2] * .65, -1, 1) * travel
            y = (.075 * command[1] + .115 * front * command[2]) * travel
            z = -2 * .213 * np.cos(.9) + (.045 * np.sin(np.pi * progress) * self.blend if not stance else 0)
            sagittal_z = -np.hypot(y, z)
            knee = -np.arccos(np.clip((x*x + sagittal_z*sagittal_z) / (2*.213**2) - 1, -.999, .999))
            targets.extend([np.arctan2(y, -z), np.arctan2(-x, -sagittal_z) - knee * .5, knee])
        contact_geoms = {int(i) for c in data.contact for i in c.geom if abs(c.frame[2]) > .5 and c.dist < .003}
        supporting = [foot for foot, mask in zip(self.feet, self.foot_geoms)
                      if any(mask[geom] for geom in contact_geoms)]
        feedforward = np.zeros(12)
        if supporting:
            force = -model.opt.gravity * model.body_subtreemass[self.root] / len(supporting)
            for foot in supporting:
                mj.mj_jacBody(model, data, self.jac, None, foot)
                feedforward -= self.jac[:, self.dofs].T @ force
        torque = (np.tile([45, 55, 60], 4) * (targets - data.qpos[self.qpos])
                  - np.tile([1.5, 2, 2], 4) * data.qvel[self.dofs] + feedforward)
        data.ctrl[self.actuators] = np.clip(torque, self.limits[:, 0], self.limits[:, 1])
