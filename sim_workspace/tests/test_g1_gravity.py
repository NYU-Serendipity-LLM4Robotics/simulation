import unittest

import mujoco as mj
import numpy as np

from robots.g1 import pick_place


class FixedG1GravityTests(unittest.TestCase):
    def test_ball_free_fall_against_zero_gravity(self):
        for gravity in (-9.81, 0.0):
            with self.subTest(gravity=gravity):
                model, data = pick_place.make_scene(pick_place.SOURCE, pick_place.DESTINATION)
                np.testing.assert_allclose(model.opt.gravity, [0.0, 0.0, -9.81])
                self.assertFalse(np.any(model.body_gravcomp))
                self.assertEqual(model.neq, 0)
                model.opt.gravity[2] = gravity
                joint = model.joint('workpiece_free')
                qpos = int(joint.qposadr[0])
                dof = int(joint.dofadr[0])
                ball = model.geom('workpiece').id
                start_z = 1.2
                data.qpos[qpos:qpos + 3] = [0.9, 0.7, start_z]
                mj.mj_forward(model, data)
                duration = 0.2
                for _ in range(round(duration / model.opt.timestep)):
                    mj.mj_step(model, data)
                    for contact in data.contact:
                        self.assertNotIn(ball, (contact.geom1, contact.geom2))
                self.assertAlmostEqual(data.qvel[dof + 2], gravity * duration, places=6)
                self.assertAlmostEqual(data.qpos[qpos + 2] - start_z,
                                       0.5 * gravity * duration ** 2, delta=0.001)

    def test_unpowered_wrist_falls_under_gravity(self):
        def wrist_displacement(gravity):
            model, data = pick_place.make_scene(pick_place.SOURCE, pick_place.DESTINATION)
            model.opt.gravity[2] = gravity
            model.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_ACTUATION)
            # Keep the test ball clear of the arm's initial configuration.
            ball_ads = int(model.joint('workpiece_free').qposadr[0])
            data.qpos[ball_ads:ball_ads + 3] = [0.9, 0.7, 1.2]
            mj.mj_forward(model, data)
            wrist = model.body(pick_place.WRIST).id
            initial_z = data.xpos[wrist, 2]
            mj.mj_step(model, data, nstep=round(0.1 / model.opt.timestep))
            mj.mj_forward(model, data)
            self.assertFalse(np.any(data.actuator_force))
            return float(data.xpos[wrist, 2] - initial_z)

        self.assertLess(wrist_displacement(-9.81), -0.001)
        self.assertAlmostEqual(wrist_displacement(0.0), 0.0, places=6)
