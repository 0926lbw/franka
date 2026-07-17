#!/usr/bin/env python3
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
"""Static checks for the default tracker preview configuration."""

from pathlib import Path


CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'tracker_bridge_preview.yaml'
REAL_FR3V2_1_CONFIG = (
    Path(__file__).resolve().parents[1] / 'config' / 'tracker_bridge_fr3v2_1_real.yaml'
)


def test_tracker_pose_target_waits_for_current_joint_state_start():
    config = CONFIG.read_text(encoding='utf-8')
    node = (
        Path(__file__).resolve().parents[1]
        / 'franka_tracker_bridge'
        / 'tracker_pose_target_node.py'
    ).read_text(encoding='utf-8')
    launch = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'tracker_preview.launch.py'
    ).read_text(encoding='utf-8')

    assert 'joint_states_topic: /joint_states' in config
    assert 'robot_start_source' not in config
    assert 'robot_start_position' not in config
    assert 'robot_start_orientation' not in config
    assert 'robot_start_source' not in node
    assert 'robot_start_position' not in node
    assert 'robot_start_orientation' not in node
    assert "declare_parameter('joint_states_topic', '/joint_states')" in node
    assert 'self.mapper: TrackerPoseTargetMapper | None = None' in node
    assert 'extract_complete_joint_positions(' in node
    assert 'validate_joint_position_limits(' in node
    assert 'qos_profile_sensor_data' in node
    assert "DeclareLaunchArgument('joint_states_topic', default_value='/joint_states')" in launch
    assert "'joint_states_topic': joint_states_topic" in launch


def test_tracker_pose_target_publishes_targets_in_controller_base_frame():
    config = CONFIG.read_text(encoding='utf-8')
    node = (
        Path(__file__).resolve().parents[1]
        / 'franka_tracker_bridge'
        / 'tracker_pose_target_node.py'
    ).read_text(encoding='utf-8')
    launch = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'tracker_preview.launch.py'
    ).read_text(encoding='utf-8')

    assert 'target_frame: base' in config
    assert "declare_parameter('target_frame', 'base')" in node
    assert "DeclareLaunchArgument('target_frame', default_value='base')" in launch


def test_meshcat_preview_has_no_configurable_fixed_joint_start():
    config = CONFIG.read_text(encoding='utf-8')
    node = (
        Path(__file__).resolve().parents[1]
        / 'franka_tracker_bridge'
        / 'tracker_meshcat_preview_node.py'
    ).read_text(encoding='utf-8')

    assert 'initial_joint_positions' not in config
    assert 'initial_joint_positions' not in node
    assert 'pin.neutral(self.model)' in node


def test_tracker_pose_target_uses_three_second_startup_calibration():
    config = CONFIG.read_text(encoding='utf-8')

    assert 'calibration_duration_sec: 3.0' in config


def test_tracker_pose_target_unlimits_translation_and_rotation_by_default():
    config = CONFIG.read_text(encoding='utf-8')
    pose_target_config = config.split('tracker_pose_target_node:')[1].split(
        'tracker_meshcat_preview_node:'
    )[0]
    node = (
        Path(__file__).resolve().parents[1]
        / 'franka_tracker_bridge'
        / 'tracker_pose_target_node.py'
    ).read_text(encoding='utf-8')

    assert 'translation_limit: [-1.0, -1.0, -1.0]' in pose_target_config
    assert 'rotation_limit: 3.141592653589793' in pose_target_config
    assert "declare_parameter('translation_limit', [-1.0, -1.0, -1.0])" in node
    assert "declare_parameter('rotation_limit', 3.141592653589793)" in node


def test_tracker_pose_target_uses_identity_axis_map_by_default():
    config = CONFIG.read_text(encoding='utf-8')
    pose_target_config = config.split('tracker_pose_target_node:')[1].split(
        'tracker_meshcat_preview_node:'
    )[0]

    assert 'map_matrix: [1.0,  0.0, 0.0,' in pose_target_config
    assert '0.0,  1.0, 0.0' in pose_target_config
    assert '0.0,  0.0, 1.0]' in pose_target_config


def test_tracker_pose_target_uses_explicit_tracker_mapping_defaults():
    config = CONFIG.read_text(encoding='utf-8')
    pose_target_config = config.split('tracker_pose_target_node:')[1].split(
        'tracker_meshcat_preview_node:'
    )[0]
    node = (
        Path(__file__).resolve().parents[1]
        / 'franka_tracker_bridge'
        / 'tracker_pose_target_node.py'
    ).read_text(encoding='utf-8')

    assert 'coord_swap: [1, 0, 2]' in pose_target_config
    assert 'coord_flip: [1.0, -1.0, 1.0]' in pose_target_config
    assert 'base_xy_rotation_deg: 90.0' in pose_target_config
    assert 'orientation_alignment_rpy_deg: [0.0, 0.0, 180.0]' in pose_target_config
    assert 'tracker_rotation_scale: -1.0' in pose_target_config
    assert 'tracker_rotation_axis_scale: [1.0, 1.0, -1.0]' in pose_target_config
    assert 'tracker_pos_soft_limit_mm: 15.0' in pose_target_config
    assert 'tracker_pos_hard_limit_mm: 40.0' in pose_target_config
    assert 'tracker_rot_soft_limit_deg: 6.0' in pose_target_config
    assert 'tracker_rot_hard_limit_deg: 15.0' in pose_target_config
    assert "declare_parameter('coord_swap', [1, 0, 2])" in node
    assert "declare_parameter('tracker_pos_soft_limit_mm', 15.0)" in node


def test_fr3v2_1_real_preview_uses_matching_model_frames_and_one_to_one_scaling():
    config = REAL_FR3V2_1_CONFIG.read_text(encoding='utf-8')
    launch = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'tracker_preview.launch.py'
    ).read_text(encoding='utf-8')

    assert config.count('robot_type: fr3v2_1') == 2
    assert config.count('ik_frame_name: fr3v2_1_link8') == 2
    assert 'translation_scale: [1.0, 1.0, 1.0]' in config
    assert 'rotation_scale: [1.0, 1.0, 1.0]' in config
    assert 'deadman_initially_enabled: false' in config
    assert 'publish_deadman: false' in config
    assert (
        "DeclareLaunchArgument('publish_deadman', default_value='false')" in launch
    )
    assert (
        "DeclareLaunchArgument('ik_frame_name', default_value=[robot_type, '_link8'])"
        in launch
    )
