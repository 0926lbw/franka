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
sys.modules.pop("franka_tracker_bridge", None)

from franka_tracker_bridge.debug_curve_core import DebugCurveBuffer
from franka_tracker_bridge.pose_math import PoseValue, QuaternionValue, TimedPose, Vector3


def pose(x, y, z, yaw=0.0):
    return PoseValue(
        position=Vector3(x, y, z),
        orientation=QuaternionValue(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        ),
    )


class DebugCurveCoreTest(unittest.TestCase):
    def test_records_tracker_pose_xyz_rpy_and_deadman(self):
        buffer = DebugCurveBuffer(max_samples=10)

        buffer.add_tracker_pose(TimedPose(stamp_sec=1.0, pose=pose(0.1, -0.2, 0.3, yaw=0.5)))
        buffer.add_deadman(stamp_sec=1.2, enabled=True)

        snapshot = buffer.snapshot()
        self.assertEqual(snapshot.tracker_time, [1.0])
        self.assertEqual(snapshot.tracker_xyz[0], [0.1])
        self.assertEqual(snapshot.tracker_xyz[1], [-0.2])
        self.assertEqual(snapshot.tracker_xyz[2], [0.3])
        self.assertAlmostEqual(snapshot.tracker_rpy[2][0], 0.5, places=6)
        self.assertEqual(snapshot.deadman_time, [1.2])
        self.assertEqual(snapshot.deadman, [1.0])

    def test_records_target_pose_and_delta_from_first_target(self):
        buffer = DebugCurveBuffer(max_samples=10)

        buffer.add_target_pose(TimedPose(stamp_sec=2.0, pose=pose(0.4, 0.0, 0.6, yaw=0.1)))
        buffer.add_target_pose(TimedPose(stamp_sec=2.5, pose=pose(0.45, -0.02, 0.7, yaw=0.4)))

        snapshot = buffer.snapshot()
        self.assertEqual(snapshot.target_time, [2.0, 2.5])
        self.assertEqual(snapshot.target_xyz[0], [0.4, 0.45])
        self.assertEqual(snapshot.target_xyz[1], [0.0, -0.02])
        self.assertEqual(snapshot.target_xyz[2], [0.6, 0.7])
        self.assertEqual(snapshot.target_delta_xyz[0], [0.0, 0.05])
        self.assertEqual(snapshot.target_delta_xyz[1], [0.0, -0.02])
        self.assertEqual(snapshot.target_delta_xyz[2], [0.0, 0.1])
        self.assertAlmostEqual(snapshot.target_delta_rpy[2][0], 0.0, places=6)
        self.assertAlmostEqual(snapshot.target_delta_rpy[2][1], 0.3, places=6)

    def test_records_seven_kdl_desired_joints(self):
        buffer = DebugCurveBuffer(max_samples=10)

        buffer.add_kdl_desired_joint_state(
            stamp_sec=3.0,
            names=[f"fr3_joint{i}" for i in range(1, 8)],
            positions=[0.1 * i for i in range(7)],
        )

        snapshot = buffer.snapshot()
        self.assertEqual(snapshot.kdl_time, [3.0])
        self.assertEqual(snapshot.kdl_joint_names, [f"fr3_joint{i}" for i in range(1, 8)])
        self.assertEqual(snapshot.kdl_positions[0], [0.0])
        self.assertEqual(snapshot.kdl_positions[6], [0.6000000000000001])

    def test_limits_all_series_to_max_samples(self):
        buffer = DebugCurveBuffer(max_samples=2)

        for index in range(4):
            buffer.add_tracker_pose(TimedPose(stamp_sec=float(index), pose=pose(index, 0.0, 0.0)))
            buffer.add_deadman(stamp_sec=float(index), enabled=index % 2 == 0)

        snapshot = buffer.snapshot()
        self.assertEqual(snapshot.tracker_time, [2.0, 3.0])
        self.assertEqual(snapshot.tracker_xyz[0], [2.0, 3.0])
        self.assertEqual(snapshot.deadman_time, [2.0, 3.0])
        self.assertEqual(snapshot.deadman, [1.0, 0.0])


if __name__ == "__main__":
    unittest.main()
