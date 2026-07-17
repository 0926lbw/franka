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

from franka_tracker_bridge.pose_math import (
    PoseValue,
    QuaternionValue,
    TimedPose,
    Vector3,
    quaternion_from_axis_angle,
    quaternion_to_axis_angle,
)
from franka_tracker_bridge.tracker_pose_target_core import (
    PoseTargetConfig,
    TrackerPoseTargetMapper,
)


def pose(x, y, z, quat=None):
    return PoseValue(
        position=Vector3(x=x, y=y, z=z),
        orientation=quat or QuaternionValue(x=0.0, y=0.0, z=0.0, w=1.0),
    )


class TrackerPoseTargetCoreTest(unittest.TestCase):
    def test_default_motion_limits_match_preview_launch_defaults(self):
        config = PoseTargetConfig()

        self.assertEqual(config.translation_limit, (-1.0, -1.0, -1.0))
        self.assertAlmostEqual(config.rotation_limit, math.pi, places=12)

    def test_first_tracker_pose_calibrates_and_returns_robot_start_when_enabled(self):
        robot_start = pose(0.4, 0.0, 0.5)
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=robot_start,
            config=PoseTargetConfig(translation_scale=(1.0, 1.0, 1.0)),
        )

        mapper.set_deadman(True)
        target = mapper.update(TimedPose(1.0, pose(1.0, 2.0, 3.0)))

        self.assertEqual(target, robot_start)

    def test_first_tracker_pose_calibrates_even_when_deadman_disabled(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(translation_limit=(1.0, 1.0, 1.0)),
        )

        target = mapper.update(TimedPose(1.0, pose(1.0, 2.0, 3.0)))
        self.assertIsNone(target)

        mapper.set_deadman(True)
        target = mapper.update(TimedPose(1.1, pose(1.1, 2.0, 3.0)))

        self.assertAlmostEqual(target.position.x, 0.1, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

    def test_deadman_toggle_keeps_dexcap_relative_to_start_motion(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(translation_limit=(1.0, 1.0, 1.0)),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(0.1, 0.0, 0.0)))
        self.assertAlmostEqual(target.position.x, 0.1, places=6)

        mapper.set_deadman(False)
        self.assertIsNone(mapper.update(TimedPose(1.2, pose(0.15, 0.0, 0.0))))

        mapper.set_deadman(True)
        target = mapper.update(TimedPose(1.3, pose(0.2, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.2, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

    def test_tracker_position_low_pass_filters_relative_translation(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                tracker_low_pass_alpha=0.5,
                translation_limit=(2.0, 2.0, 2.0),
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(1.0, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.5, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

        target = mapper.update(TimedPose(1.2, pose(1.0, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.75, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

    def test_tracker_orientation_low_pass_filters_relative_rotation(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                tracker_low_pass_alpha=0.5,
                rotation_limit=math.pi,
            ),
        )
        tracker_quat = quaternion_from_axis_angle((0.0, 0.0, 1.0), math.pi)

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(0.0, 0.0, 0.0, tracker_quat)))

        self.assertAlmostEqual(target.orientation.x, 0.0, places=6)
        self.assertAlmostEqual(target.orientation.y, 0.0, places=6)
        self.assertAlmostEqual(abs(target.orientation.z), math.sin(math.pi / 4.0), places=6)
        self.assertAlmostEqual(abs(target.orientation.w), math.cos(math.pi / 4.0), places=6)

    def test_tracker_deadband_suppresses_small_relative_motion(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                translation_deadband=0.02,
                rotation_deadband=0.10,
                translation_limit=(1.0, 1.0, 1.0),
            ),
        )
        tracker_quat = quaternion_from_axis_angle((0.0, 0.0, 1.0), 0.05)

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(0.01, 0.0, 0.0, tracker_quat)))

        self.assertAlmostEqual(target.position.x, 0.0, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)
        self.assertAlmostEqual(target.orientation.x, 0.0, places=6)
        self.assertAlmostEqual(target.orientation.y, 0.0, places=6)
        self.assertAlmostEqual(target.orientation.z, 0.0, places=6)
        self.assertAlmostEqual(target.orientation.w, 1.0, places=6)

    def test_calibration_sample_count_delays_tracker_start_until_filtered_pose_stabilizes(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                calibration_sample_count=3,
                translation_limit=(1.0, 1.0, 1.0),
            ),
        )

        mapper.set_deadman(True)
        self.assertIsNone(mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0))))
        self.assertIsNone(mapper.update(TimedPose(1.1, pose(0.1, 0.0, 0.0))))
        target = mapper.update(TimedPose(1.2, pose(0.2, 0.0, 0.0)))
        self.assertEqual(target, pose(0.0, 0.0, 0.0))

        target = mapper.update(TimedPose(1.3, pose(0.3, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.1, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

    def test_calibration_duration_averages_tracker_start_before_publishing(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                calibration_duration_sec=3.0,
                tracker_low_pass_alpha=1.0,
                translation_limit=(1.0, 1.0, 1.0),
            ),
        )

        mapper.set_deadman(True)
        self.assertIsNone(mapper.update(TimedPose(10.0, pose(0.0, 0.0, 0.0))))
        self.assertIsNone(mapper.update(TimedPose(11.0, pose(0.3, 0.0, 0.0))))
        self.assertIsNone(mapper.update(TimedPose(12.0, pose(0.6, 0.0, 0.0))))

        target = mapper.update(TimedPose(13.0, pose(0.9, 0.0, 0.0)))

        self.assertEqual(target, pose(0.0, 0.0, 0.0))

        target = mapper.update(TimedPose(13.1, pose(1.0, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.55, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

    def test_tracker_relative_translation_maps_to_robot_target_pose(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.4, 0.0, 0.5),
            config=PoseTargetConfig(
                translation_scale=(0.5, 2.0, 1.0),
                translation_limit=(1.0, 1.0, 1.0),
                map_matrix=((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(1.0, 1.0, 1.0)))
        target = mapper.update(TimedPose(1.1, pose(1.2, 1.1, 0.7)))

        self.assertAlmostEqual(target.position.x, 0.35, places=6)
        self.assertAlmostEqual(target.position.y, 0.4, places=6)
        self.assertAlmostEqual(target.position.z, 0.2, places=6)

    def test_fairino_style_translation_mapping_swaps_flips_and_rotates_xy(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.4, 0.0, 0.5),
            config=PoseTargetConfig(
                coord_swap=(1, 0, 2),
                coord_flip=(1.0, -1.0, 1.0),
                coord_scale=(1.0, 1.0, 1.0),
                base_xy_rotation_deg=45.0,
                translation_limit=(-1.0, -1.0, -1.0),
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(0.2, 0.1, 0.3)))

        root_half = math.sqrt(0.5)
        self.assertAlmostEqual(target.position.x, 0.4 + 0.3 * root_half, places=6)
        self.assertAlmostEqual(target.position.y, -0.1 * root_half, places=6)
        self.assertAlmostEqual(target.position.z, 0.8, places=6)

    def test_fairino_style_jump_suppression_limits_single_frame_delta_step(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                translation_limit=(-1.0, -1.0, -1.0),
                tracker_pos_soft_limit_mm=15.0,
                tracker_pos_hard_limit_mm=40.0,
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(0.10, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.04, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

        target = mapper.update(TimedPose(1.2, pose(0.12, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.08, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

    def test_tracker_translation_delta_stays_in_tracker_world_frame(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(translation_limit=(1.0, 1.0, 1.0)),
        )
        tracker_start_orientation = quaternion_from_axis_angle(
            (0.0, 0.0, 1.0),
            math.pi / 2.0,
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0, tracker_start_orientation)))
        target = mapper.update(TimedPose(1.1, pose(0.0, 0.1, 0.0, tracker_start_orientation)))

        self.assertAlmostEqual(target.position.x, 0.0, places=6)
        self.assertAlmostEqual(target.position.y, 0.1, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

    def test_robot_start_orientation_does_not_rotate_world_frame_translation_delta(self):
        robot_start_orientation = quaternion_from_axis_angle(
            (0.0, 0.0, 1.0),
            math.pi / 2.0,
        )
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.5, 0.0, 0.2, robot_start_orientation),
            config=PoseTargetConfig(translation_limit=(1.0, 1.0, 1.0)),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(0.1, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.6, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.2, places=6)

    def test_translation_limit_clamps_target_delta(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                translation_scale=(1.0, 1.0, 1.0),
                translation_limit=(0.2, 0.3, 0.4),
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(1.0, -1.0, 1.0)))

        self.assertAlmostEqual(target.position.x, 0.2, places=6)
        self.assertAlmostEqual(target.position.y, -0.3, places=6)
        self.assertAlmostEqual(target.position.z, 0.4, places=6)

    def test_negative_translation_limit_disables_target_delta_clamp(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                translation_scale=(1.0, 1.0, 1.0),
                translation_limit=(-1.0, -1.0, -1.0),
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(1.0, -1.0, 1.0)))

        self.assertAlmostEqual(target.position.x, 1.0, places=6)
        self.assertAlmostEqual(target.position.y, -1.0, places=6)
        self.assertAlmostEqual(target.position.z, 1.0, places=6)

    def test_translation_limit_clamps_total_relative_motion_from_start(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                translation_scale=(1.0, 1.0, 1.0),
                translation_limit=(0.2, 1.0, 1.0),
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        target = mapper.update(TimedPose(1.1, pose(0.15, 0.0, 0.0)))
        self.assertAlmostEqual(target.position.x, 0.15, places=6)

        target = mapper.update(TimedPose(1.2, pose(0.30, 0.0, 0.0)))

        self.assertAlmostEqual(target.position.x, 0.2, places=6)
        self.assertAlmostEqual(target.position.y, 0.0, places=6)
        self.assertAlmostEqual(target.position.z, 0.0, places=6)

    def test_tracker_relative_rotation_maps_to_robot_target_orientation(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                rotation_scale=(1.0, 1.0, 0.5),
                rotation_limit=math.pi,
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        tracker_quat = quaternion_from_axis_angle((0.0, 0.0, 1.0), math.pi)
        target = mapper.update(TimedPose(1.1, pose(0.0, 0.0, 0.0, tracker_quat)))

        self.assertAlmostEqual(target.orientation.x, 0.0, places=6)
        self.assertAlmostEqual(target.orientation.y, 0.0, places=6)
        self.assertAlmostEqual(abs(target.orientation.z), math.sin(math.pi / 4.0), places=6)
        self.assertAlmostEqual(abs(target.orientation.w), math.cos(math.pi / 4.0), places=6)

    def test_reflected_map_matrix_maps_rotation_as_se3_conjugation(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                rotation_limit=math.pi,
                map_matrix=((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        tracker_quat = quaternion_from_axis_angle((0.0, 1.0, 0.0), math.pi / 2.0)
        target = mapper.update(TimedPose(1.1, pose(0.0, 0.0, 0.0, tracker_quat)))
        axis_angle = quaternion_to_axis_angle(target.orientation)

        self.assertAlmostEqual(axis_angle.x, 0.0, places=6)
        self.assertAlmostEqual(axis_angle.y, -math.pi / 2.0, places=6)
        self.assertAlmostEqual(axis_angle.z, 0.0, places=6)

    def test_tracker_relative_rotation_uses_left_world_delta(self):
        robot_start_orientation = quaternion_from_axis_angle(
            (1.0, 0.0, 0.0),
            math.pi / 2.0,
        )
        tracker_start_orientation = quaternion_from_axis_angle(
            (1.0, 0.0, 0.0),
            math.pi / 2.0,
        )
        tracker_current_orientation = quaternion_from_axis_angle(
            (0.0, 1.0, 0.0),
            math.pi / 2.0,
        )
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0, robot_start_orientation),
            config=PoseTargetConfig(rotation_limit=math.pi),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0, tracker_start_orientation)))
        target = mapper.update(TimedPose(1.1, pose(0.0, 0.0, 0.0, tracker_current_orientation)))
        axis_angle = quaternion_to_axis_angle(target.orientation)

        self.assertAlmostEqual(axis_angle.x, 0.0, places=6)
        self.assertAlmostEqual(axis_angle.y, math.pi / 2.0, places=6)
        self.assertAlmostEqual(axis_angle.z, 0.0, places=6)

    def test_fairino_style_orientation_alignment_and_axis_scale_maps_tracker_rotation(self):
        mapper = TrackerPoseTargetMapper(
            robot_start_pose=pose(0.0, 0.0, 0.0),
            config=PoseTargetConfig(
                orientation_alignment_rpy_deg=(0.0, 0.0, 180.0),
                tracker_rotation_scale=-1.0,
                tracker_rotation_axis_scale=(1.0, 1.0, -1.0),
                tracker_rotation_axis_order=(0, 1, 2),
                rotation_limit=math.pi,
            ),
        )

        mapper.set_deadman(True)
        mapper.update(TimedPose(1.0, pose(0.0, 0.0, 0.0)))
        tracker_quat = quaternion_from_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)
        target = mapper.update(TimedPose(1.1, pose(0.0, 0.0, 0.0, tracker_quat)))
        axis_angle = quaternion_to_axis_angle(target.orientation)

        self.assertAlmostEqual(axis_angle.x, math.pi / 2.0, places=6)
        self.assertAlmostEqual(axis_angle.y, 0.0, places=6)
        self.assertAlmostEqual(axis_angle.z, 0.0, places=6)

    def test_deadman_disabled_blocks_target_updates(self):
        mapper = TrackerPoseTargetMapper(robot_start_pose=pose(0.0, 0.0, 0.0))

        self.assertIsNone(mapper.update(TimedPose(1.0, pose(1.0, 0.0, 0.0))))


if __name__ == '__main__':
    unittest.main()
