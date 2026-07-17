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


BRIDGE_ROOT = Path(__file__).resolve().parents[1]
ROS_ROOT = BRIDGE_ROOT.parent
CONTROLLER_ROOT = ROS_ROOT / 'franka_arm_controllers'
sys.path.insert(0, str(BRIDGE_ROOT))
sys.modules.pop('franka_tracker_bridge', None)

from franka_tracker_bridge.pose_math import (
    PoseValue,
    QuaternionValue,
    TimedPose,
    Vector3,
    quaternion_from_axis_angle,
)
from franka_tracker_bridge.tracker_pose_target_core import (
    PoseTargetConfig,
    TrackerPoseTargetMapper,
    compose_relative_pose,
)


def pose(x, y, z, orientation=None):
    return PoseValue(
        position=Vector3(x, y, z),
        orientation=orientation or QuaternionValue(0.0, 0.0, 0.0, 1.0),
    )


def test_relative_mapper_publishes_zero_then_tracker_delta():
    start_orientation = quaternion_from_axis_angle((0.0, 0.0, 1.0), math.pi / 2.0)
    mapper = TrackerPoseTargetMapper(
        robot_start_pose=pose(0.4, -0.2, 0.5, start_orientation),
        config=PoseTargetConfig(
            target_pose_is_relative=True,
            tracker_low_pass_alpha=1.0,
            translation_limit=(1.0, 1.0, 1.0),
        ),
    )
    mapper.set_deadman(True)

    baseline = mapper.update(TimedPose(1.0, pose(1.0, 2.0, 3.0)))
    assert baseline == pose(0.0, 0.0, 0.0)

    relative = mapper.update(TimedPose(1.1, pose(1.1, 1.8, 3.3)))
    assert math.isclose(relative.position.x, 0.1, abs_tol=1e-9)
    assert math.isclose(relative.position.y, -0.2, abs_tol=1e-9)
    assert math.isclose(relative.position.z, 0.3, abs_tol=1e-9)
    assert relative.orientation == QuaternionValue(0.0, 0.0, 0.0, 1.0)


def test_controller_style_composition_recovers_absolute_target():
    reference = pose(
        0.4,
        -0.2,
        0.5,
        quaternion_from_axis_angle((0.0, 0.0, 1.0), math.pi / 2.0),
    )
    relative = pose(
        0.1,
        -0.2,
        0.3,
        quaternion_from_axis_angle((1.0, 0.0, 0.0), 0.2),
    )

    absolute = compose_relative_pose(reference, relative)

    assert math.isclose(absolute.position.x, 0.5, abs_tol=1e-9)
    assert math.isclose(absolute.position.y, -0.4, abs_tol=1e-9)
    assert math.isclose(absolute.position.z, 0.8, abs_tol=1e-9)


def test_fake_and_real_configs_use_absolute_pose_contract_by_default():
    fake_bridge = (BRIDGE_ROOT / 'config' / 'tracker_bridge_preview.yaml').read_text()
    real_bridge = (BRIDGE_ROOT / 'config' / 'tracker_bridge_fr3v2_1_real.yaml').read_text()
    fake_arm = (CONTROLLER_ROOT / 'config' / 'tracker_fake_fr3_config.yaml').read_text()
    real_arm = (CONTROLLER_ROOT / 'config' / 'real_fr3v2_1_config.yaml').read_text()
    override = (CONTROLLER_ROOT / 'config' / 'controllers_relative_target.yaml').read_text()
    launch = (
        CONTROLLER_ROOT / 'launch' / 'joint_impedance_pose_controller.launch.py'
    ).read_text()

    assert fake_bridge.count('target_pose_is_relative: false') == 2
    assert real_bridge.count('target_pose_is_relative: false') == 2
    assert 'target_pose_is_relative: "false"' in fake_arm
    assert 'target_pose_is_relative: "false"' in real_arm
    # Relative mode remains available only as an explicit opt-in profile.
    assert 'target_pose_is_relative: true' in override
    assert 'controllers_relative_target.yaml' in launch


def test_controller_composes_relative_pose_before_ik():
    source = (
        CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp'
    ).read_text()

    assert 'auto_declare<bool>("target_pose_is_relative", false)' in source
    assert 'startup_position_ + relative_target_position_' in source
    assert 'relative_target_orientation_ * startup_orientation_' in source
