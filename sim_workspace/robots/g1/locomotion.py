"""Unitree's G1 29-DoF velocity policy on the free-base Dex3 model."""
from collections import deque
from hashlib import sha256

import mujoco as mj
import numpy as np

REVISION = '4960b84732b0c2ec593dccbfe963fda1bcd7b1e3'
POLICY_SHA256 = '610c27e463a8f666aa50a06346678c00b4df3859f10b54bcc1f817c28251406f'

# Hardware order and policy order from the upstream velocity/v0 deploy.yaml.
JOINTS = [
    *(f'{side}_{joint}' for side in ('left', 'right') for joint in
      ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')),
    'waist_yaw', 'waist_roll', 'waist_pitch',
    *(f'{side}_{joint}' for side in ('left', 'right') for joint in
      ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
       'wrist_roll', 'wrist_pitch', 'wrist_yaw')),
]
POLICY_ORDER = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23,
                5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]
DEFAULT = np.array([-.1, -.1, 0, 0, 0, 0, 0, 0, 0, .3, .3, .3, .3, -.2, -.2,
                    .25, -.25, 0, 0, 0, 0, .97, .97, .15, -.15, 0, 0, 0, 0])


class Locomotion:
    def __init__(self, stand, model, data, weights):
        import onnxruntime as ort

        if sha256(weights.read_bytes()).hexdigest() != POLICY_SHA256:
            raise ValueError(f'Expected Unitree velocity/v0 weights at {weights}')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(weights), options, providers=['CPUExecutionProvider'])
        self.stand, self.model, self.data = stand, model, data
        self.ids = np.array([model.actuator('g1/' + JOINTS[j] + '_joint').id for j in POLICY_ORDER])
        self.qpos = stand.qpos[self.ids]
        self.dofs = stand.dofs[self.ids]
        self.rotation = data.xmat[stand.pelvis].reshape(3, 3)
        self.action = np.zeros(29)
        self.velocity = np.zeros(6)
        self.history = None
        self.next_time = 0.0
        g1_joints = np.array([model.actuator('g1/' + name + '_joint').id for name in JOINTS])
        stand.kp[g1_joints] = [100, 100, 100, 150, 40, 40] * 2 + [200] * 3 + [40] * 14
        stand.kd[g1_joints] = [2, 2, 2, 4, 2, 2] * 2 + [5] * 3 + [10] * 14
        stand.ankle_attitude_gain = 0

    def apply(self, command):
        if self.data.time + 1e-9 < self.next_time:
            return
        self.next_time = self.data.time + .02
        mj.mj_objectVelocity(self.model, self.data, mj.mjtObj.mjOBJ_BODY,
                             self.stand.pelvis, self.velocity, 1)
        terms = (self.velocity[:3] * .2, -self.rotation[2], command.copy(),
                 self.data.qpos[self.qpos] - DEFAULT, self.data.qvel[self.dofs] * .05,
                 self.action)
        if self.history is None:
            self.history = [deque([term] * 5, maxlen=5) for term in terms]
        else:
            for history, term in zip(self.history, terms):
                history.append(term.copy())
        obs = np.concatenate([np.concatenate(history) for history in self.history])
        self.action = self.session.run(None, {'obs': obs.astype(np.float32)[None, :]})[0][0]
        self.stand.target[self.ids] = DEFAULT + .25 * self.action
