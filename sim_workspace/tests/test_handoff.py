import copy
import json
import unittest

import handoff


class HandoffContractTest(unittest.TestCase):
    def sample(self, located=True, foot_failure=False):
        report = ({'xyz_m': [1.1, -0.2, 0.5], 'capture_index': 7, 'method': 'oracle_rgbd'}
                  if located else None)
        return handoff.from_scout(0, 'red apple', 'green tray', (-0.9, 1.1, 0.48), report, foot_failure)

    def test_localized_message_round_trips(self):
        message = self.sample()
        self.assertEqual(handoff.validate(json.loads(json.dumps(message))), message)
        self.assertEqual(message['status'], 'localized')
        self.assertEqual(message['observation']['position_kind'], 'surface_point')

    def test_failure_carries_no_object_pose(self):
        for message, status in [(self.sample(False), 'target_not_visible'),
                                (self.sample(foot_failure=True), 'go2_foot_ik_failed')]:
            with self.subTest(status=status):
                self.assertEqual(message['status'], status)
                self.assertIsNone(message['object_xyz_m'])
                self.assertIsNone(message['observation'])

    def test_rejects_frame_units_schema_and_invalid_positions(self):
        for key, value in [('schema_version', 9), ('frame_id', 'camera'), ('units', 'cm'),
                           ('status', 'unknown'), ('object_xyz_m', [1.0, float('nan'), 1.0])]:
            with self.subTest(field=key):
                message = copy.deepcopy(self.sample())
                message[key] = value
                with self.assertRaises(ValueError):
                    handoff.validate(message)

    def test_failed_message_still_requires_nullable_fields(self):
        message = self.sample(False)
        del message['object_xyz_m']
        with self.assertRaises(ValueError):
            handoff.validate(message)
