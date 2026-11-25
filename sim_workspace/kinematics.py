"""Joint-limit-aware MuJoCo inverse kinematics for the shared robot models."""
from dataclasses import dataclass

import mujoco as mj
import numpy as np


@dataclass(frozen=True)
class IKResult:
    converged: bool
    error_m: float
    iterations: int


class PositionIK:
    """Solve a body-fixed point in world coordinates using the model Jacobian."""

    def __init__(self, model, body, joints, offset=(0, 0, 0)):
        self.model = model
        self.body = model.body(body).id
        self.offset = np.asarray(offset)
        ids = np.array([model.joint(name).id for name in joints])
        if not np.all(model.jnt_type[ids] == mj.mjtJoint.mjJNT_HINGE):
            raise ValueError('IK expects scalar hinge joints')
        self.qpos = model.jnt_qposadr[ids]
        self.dofs = model.jnt_dofadr[ids]
        lower, upper = model.jnt_range[ids].T
        limited = model.jnt_limited[ids].astype(bool)
        self.lower = np.where(limited, lower, -np.inf)
        self.upper = np.where(limited, upper, np.inf)
        self.jac = np.zeros((3, model.nv))
        self.rot_jac = np.zeros_like(self.jac)

    def point(self, data):
        return data.xpos[self.body] + data.xmat[self.body].reshape(3, 3) @ self.offset

    def solve(self, data, target, tolerance=0.002, max_iterations=90, accept=None, orientation=None, trace=None):
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError('IK target must contain three finite world coordinates')
        desired = np.empty(4)
        weight = 0.15
        if orientation is not None:
            mj.mju_mat2Quat(desired, np.asarray(orientation).reshape(9))

        def error_at(point):
            linear = target - point
            if orientation is None:
                return linear, 0.0
            matrix = data.xmat[self.body]
            current, angular = np.empty(4), np.empty(3)
            mj.mju_mat2Quat(current, matrix)
            mj.mju_subQuat(angular, desired, current)
            return np.r_[linear, weight * (matrix.reshape(3, 3) @ angular)], np.linalg.norm(angular)

        trajectory = []
        initial = data.qpos[self.qpos].copy()
        data.qpos[self.qpos] = np.clip(initial, self.lower, self.upper)
        mj.mj_forward(self.model, data)
        for iteration in range(max_iterations + 1):
            point = self.point(data)
            residual, angular_error = error_at(point)
            error = float(np.linalg.norm(residual[:3]))
            cost = np.linalg.norm(residual)
            if error <= tolerance and angular_error <= 0.025 and (accept is None or accept(data)):
                if trace is not None:
                    trace.extend(trajectory)
                return IKResult(True, error, iteration)
            if iteration == max_iterations:
                break
            mj.mj_jac(self.model, data, self.jac, self.rot_jac, point, self.body)
            jac = self.jac[:, self.dofs]
            if orientation is not None:
                jac = np.vstack([jac, weight * self.rot_jac[:, self.dofs]])
            step = jac.T @ np.linalg.solve(jac @ jac.T + 0.015**2 * np.eye(len(residual)), residual)
            step *= min(1.0, 0.22 / max(1e-12, np.max(np.abs(step))))
            current = data.qpos[self.qpos].copy()
            for scale in (1.0, 0.5, 0.25, 0.125):
                data.qpos[self.qpos] = np.clip(current + scale * step, self.lower, self.upper)
                mj.mj_forward(self.model, data)
                if np.linalg.norm(error_at(self.point(data))[0]) < cost - 1e-9 and (accept is None or accept(data)):
                    if trace is not None:
                        trajectory.append(data.qpos.copy())
                    break
            else:
                break
        data.qpos[self.qpos] = initial
        mj.mj_forward(self.model, data)
        return IKResult(False, error, iteration)


def walk(model, data, base, feet, destination, steps, phases, lift, keep_sole_attitude=False):
    """Preview a prescribed base route with contact-relative IK foot targets."""
    start = data.qpos[base:base + 3].copy()
    delta = np.asarray(destination) - start
    neutral = [foot.point(data) - start for foot in feet]
    rotations = [data.xmat[foot.body].copy() if keep_sole_attitude else None for foot in feet]
    cycles = max(1, round(np.linalg.norm(delta[:2]) / 0.22))
    stride = delta[:2] / (2 * cycles)
    poses, errors = [], []
    for fraction in np.linspace(0, 1, steps + 1)[1:]:
        pelvis = start + fraction * delta
        previous = data.qpos.copy()
        data.qpos[base:base + 3] = pelvis
        mj.mj_forward(model, data)
        for foot, nominal, offset, rotation in zip(feet, neutral, phases, rotations):
            phase = (fraction * cycles + offset) % 1.0
            amplitude = min(1.0, fraction * steps / 3, (1 - fraction) * steps / 3)
            target = pelvis + nominal
            target[2] = start[2] + nominal[2]
            if phase < 0.5:
                target[:2] += amplitude * stride * (0.5 - 2 * phase)
            else:
                target[:2] += amplitude * stride * (2 * phase - 1.5)
                target[2] += amplitude * lift * np.sin(2 * np.pi * (phase - 0.5))
            result = foot.solve(data, target, orientation=rotation)
            if not result.converged:
                data.qpos[:] = previous
                mj.mj_forward(model, data)
                return poses, errors, result
            errors.append(result.error_m)
        poses.append(data.qpos.copy())
    return poses, errors, None
