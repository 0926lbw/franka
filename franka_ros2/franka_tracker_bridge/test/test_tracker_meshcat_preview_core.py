# Copyright 2026 lbw
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# flake8: noqa: E402
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.modules.pop('franka_tracker_bridge', None)

from franka_tracker_bridge.pose_math import PoseValue, QuaternionValue, Vector3
from franka_tracker_bridge.tracker_meshcat_preview_core import (
    clamp_configuration_to_limits,
    extract_complete_joint_positions,
    joint_state_to_configuration,
    limit_configuration_step,
    make_triad_line_data,
    pose_value_to_transform_matrix,
    validate_joint_position_limits,
)


class TrackerMeshcatPreviewCoreTest(unittest.TestCase):
    def test_pose_value_to_transform_matrix_uses_translation_and_quaternion(self):
        pose = PoseValue(
            position=Vector3(1.0, 2.0, 3.0),
            orientation=QuaternionValue(
                x=0.0,
                y=0.0,
                z=math.sin(math.pi / 4.0),
                w=math.cos(math.pi / 4.0),
            ),
        )

        transform = pose_value_to_transform_matrix(pose)

        self.assertEqual(len(transform), 4)
        self.assertEqual(len(transform[0]), 4)
        self.assertAlmostEqual(transform[0][0], 0.0, places=6)
        self.assertAlmostEqual(transform[0][1], -1.0, places=6)
        self.assertAlmostEqual(transform[1][0], 1.0, places=6)
        self.assertAlmostEqual(transform[1][1], 0.0, places=6)
        self.assertAlmostEqual(transform[2][2], 1.0, places=6)
        self.assertEqual([transform[0][3], transform[1][3], transform[2][3]], [1.0, 2.0, 3.0])
        self.assertEqual(transform[3], [0.0, 0.0, 0.0, 1.0])

    def test_make_triad_line_data_returns_three_colored_axes(self):
        points, colors = make_triad_line_data(length=0.2)

        self.assertEqual(points, [
            (0.0, 0.0, 0.0), (0.2, 0.0, 0.0),
            (0.0, 0.0, 0.0), (0.0, 0.2, 0.0),
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.2),
        ])
        self.assertEqual(colors, [
            (1.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0), (0.0, 1.0, 0.0),
            (0.0, 0.2, 1.0), (0.0, 0.2, 1.0),
        ])

    def test_extract_complete_joint_positions_maps_by_name_and_ignores_extras(self):
        positions = extract_complete_joint_positions(
            joint_names=['finger', 'fr3_joint2', 'fr3_joint1'],
            positions=[0.04, -0.2, 0.1],
            required_joint_names=['fr3_joint1', 'fr3_joint2'],
        )

        self.assertEqual(positions, [0.1, -0.2])

    def test_extract_complete_joint_positions_rejects_invalid_samples(self):
        invalid_samples = [
            (['fr3_joint1'], [], ['fr3_joint1']),
            (['fr3_joint1'], [0.1], ['fr3_joint1', 'fr3_joint2']),
            (['fr3_joint1', 'fr3_joint1'], [0.1, 0.2], ['fr3_joint1']),
            (['fr3_joint1'], [math.nan], ['fr3_joint1']),
            (['fr3_joint1'], [math.inf], ['fr3_joint1']),
        ]

        for names, positions, required in invalid_samples:
            with self.subTest(names=names, positions=positions, required=required):
                self.assertIsNone(
                    extract_complete_joint_positions(names, positions, required)
                )

    def test_validate_joint_position_limits_rejects_bad_values_and_clamps_epsilon(self):
        self.assertIsNone(
            validate_joint_position_limits(
                positions=[0.0, 1.2],
                lower_limits=[-1.0, -1.0],
                upper_limits=[1.0, 1.0],
            )
        )
        self.assertEqual(
            validate_joint_position_limits(
                positions=[-1.0000005, 1.0000005],
                lower_limits=[-1.0, -1.0],
                upper_limits=[1.0, 1.0],
            ),
            [-1.0, 1.0],
        )

    def test_joint_state_to_configuration_maps_known_joint_names(self):
        q = joint_state_to_configuration(
            joint_names=['fr3_joint1', 'fr3_joint2', 'fr3_joint3'],
            positions=[0.1, 0.2, 0.3],
            model_joint_names=['universe', 'fr3_joint1', 'fr3_joint2', 'fr3_joint4'],
            neutral_q=[0.0, 0.0, 0.0],
        )

        self.assertEqual(q, [0.1, 0.2, 0.0])

    def test_clamp_configuration_to_limits_clamps_each_joint(self):
        q = clamp_configuration_to_limits(
            q=[-2.0, 0.5, 4.0],
            lower=[-1.0, 0.0, -3.0],
            upper=[1.0, 1.0, 3.0],
        )

        self.assertEqual(q, [-1.0, 0.5, 3.0])

    def test_limit_configuration_step_caps_per_joint_delta(self):
        q = limit_configuration_step(
            current=[0.0, 1.0, -1.0],
            target=[0.5, 0.2, -1.02],
            max_step=0.1,
        )

        self.assertEqual(q, [0.1, 0.9, -1.02])


if __name__ == '__main__':
    unittest.main()
