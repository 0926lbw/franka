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
"""Static regression checks for the tracker KDL-to-Meshcat preview wiring."""

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[3]
CONTROLLER_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_ROOT = REPO_ROOT / 'franka_ros2' / 'franka_tracker_bridge'


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def test_workspace_is_trimmed_to_tracker_pose_controller():
    plugin_xml = read(CONTROLLER_ROOT / 'franka_arm_controllers.xml')
    cmake = read(CONTROLLER_ROOT / 'CMakeLists.txt')
    config = read(CONTROLLER_ROOT / 'config' / 'controllers.yaml')

    assert (CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp').is_file()
    assert (
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_pose_controller.hpp'
    ).is_file()
    assert not (CONTROLLER_ROOT / 'src' / 'joint_impedance_ik_controller.cpp').exists()
    assert not (
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_ik_controller.hpp'
    ).exists()
    assert not (REPO_ROOT / 'franka_spacemouse' / 'src' / 'spacemouse_publisher').exists()
    assert not (REPO_ROOT / 'franka_spacemouse' / 'src' / 'gripper_manager').exists()

    assert 'franka_arm_controllers/JointImpedancePoseController' in plugin_xml
    assert 'JointImpedanceIKController' not in plugin_xml
    assert 'src/joint_impedance_pose_controller.cpp' in cmake
    assert 'src/joint_impedance_ik_controller.cpp' not in cmake
    assert 'joint_impedance_pose_controller:' in config
    assert 'joint_impedance_ik_controller:' not in config
    assert 'target_cartesian_velocity_percent' not in config

def test_pose_controller_publishes_kdl_desired_joint_states():
    header = read(
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_pose_controller.hpp'
    )
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')
    config = read(CONTROLLER_ROOT / 'config' / 'controllers.yaml')

    assert '#include <sensor_msgs/msg/joint_state.hpp>' in header
    assert 'kdl_desired_joint_states_topic_' in header
    assert 'publish_desired_joint_state_' in header
    assert 'rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr' in header

    assert (
        'auto_declare<std::string>("kdl_desired_joint_states_topic", '
        '"/franka_controller/kdl_desired_joint_states")'
    ) in source
    assert 'create_publisher<sensor_msgs::msg::JointState>' in source
    assert 'publish_desired_joint_state_(time)' in source
    assert 'desired_joint_state_msg_.name = kdl_joint_names_' in source
    assert 'desired_joint_state_msg_.position = joint_positions_desired_' in source
    assert 'desired_joint_state_pub_->publish(desired_joint_state_msg_)' in source

    assert (
        'kdl_desired_joint_states_topic: /franka_controller/kdl_desired_joint_states'
        in config
    )


def test_fake_kdl_preview_accumulates_from_previous_desired_state():
    header = read(
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_pose_controller.hpp'
    )
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')
    config = read(CONTROLLER_ROOT / 'config' / 'controllers.yaml')

    assert 'fake_preview_follow_desired_state_' in header
    assert 'fake_preview_q_' in header
    assert 'fake_preview_q_initialized_' in header

    assert 'auto_declare<bool>("fake_preview_follow_desired_state", true)' in source
    assert 'fake_preview_follow_desired_state_' in source
    assert 'fake_preview_q_(i) = q_result_(i)' in source
    assert 'q_init_(i) = fake_preview_q_(i)' in source

    assert 'fake_preview_follow_desired_state: true' in config


def test_fake_kdl_preview_starts_from_current_mock_joint_state():
    header = read(
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_pose_controller.hpp'
    )
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')
    config = read(CONTROLLER_ROOT / 'config' / 'controllers.yaml')

    assert 'fake_preview_initial_q' not in header
    assert 'fake_preview_initial_q' not in source
    assert 'fake_preview_initial_q' not in config
    assert 'fake_preview_q_(i) = joint_positions_current_[i];' in source


def test_franka_state_broadcaster_receives_robot_type_and_arm_prefix():
    launch = read(CONTROLLER_ROOT / 'launch' / 'franka.launch.py')

    assert '{"arm_id": arm_id}' in launch
    assert '{"robot_type": arm_id}' in launch
    assert '{"arm_prefix": arm_prefix}' in launch
    assert 'parameters=[{"arm_id":' not in launch


def test_fake_kdl_preview_does_not_reset_to_mock_joint_state_when_target_is_stale():
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')

    stale_branch = source.split('if (!target_is_fresh_(time)) {', 1)[1].split('} else {', 1)[0]

    assert 'joint_positions_desired_ = fake_preview_q_to_vector_();' in stale_branch
    assert 'fake_preview_q_(i) = joint_positions_current_[i];' not in stale_branch


def test_pose_controller_holds_position_instead_of_throwing_on_ik_failure():
    header = read(
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_pose_controller.hpp'
    )
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')

    assert 'bool solve_ik_' in header
    assert 'bool should_publish_desired_joint_state' in source
    assert 'if (!solve_ik_(new_position, new_orientation))' in source
    assert 'should_publish_desired_joint_state = false' in source
    assert 'RCLCPP_WARN_THROTTLE' in source
    assert 'return false;' in source
    assert 'throw std::runtime_error("IK Failed")' not in source


def test_pose_controller_logs_kdl_nonconvergence_diagnostics():
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')

    assert 'KDL IK did not converge' in source
    assert 'KDL IK failed' not in source
    assert 'target position' in source
    assert 'target orientation xyzw' in source
    assert 'startup delta' in source
    assert 'current seed q' in source
    assert 'format_joint_array_' in source


def test_pose_controller_retries_kdl_ik_with_multiple_seed_candidates():
    header = read(
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_pose_controller.hpp'
    )
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')

    assert 'build_ik_seed_candidates_' in header
    assert 'make_biased_seed_' in header
    assert 'last_successful_q_' in header
    assert 'startup_q_' in header

    assert 'const auto seed_candidates = build_ik_seed_candidates_();' in source
    assert 'for (const auto& seed : seed_candidates)' in source
    assert 'make_biased_seed_(q_init_, 5, 0.35)' in source
    assert 'make_biased_seed_(q_init_, 5, -0.35)' in source
    assert 'make_biased_seed_(q_init_, 6, 0.50)' in source
    assert 'make_biased_seed_(q_init_, 6, -0.50)' in source
    assert 'last_successful_q_ = q_result_;' in source


def test_pose_controller_exposes_kdl_ik_solver_parameters():
    header = read(
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_pose_controller.hpp'
    )
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')
    config = read(CONTROLLER_ROOT / 'config' / 'controllers.yaml')

    assert 'ik_max_iterations_' in header
    assert 'ik_eps_' in header
    assert 'auto_declare<int>("ik_max_iterations", 100)' in source
    assert 'auto_declare<double>("ik_eps", 1e-6)' in source
    assert 'ik_max_iterations: 1000' in config
    assert 'ik_eps: 1.0e-3' in config
    assert 'ik_max_iterations_, ik_eps_' in source


def test_pose_controller_can_use_moveit_compute_ik_service_backend():
    header = read(
        CONTROLLER_ROOT
        / 'include'
        / 'franka_arm_controllers'
        / 'joint_impedance_pose_controller.hpp'
    )
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')
    config = read(CONTROLLER_ROOT / 'config' / 'controllers.yaml')
    package = read(CONTROLLER_ROOT / 'package.xml')
    cmake = read(CONTROLLER_ROOT / 'CMakeLists.txt')

    assert '#include <moveit_msgs/srv/get_position_ik.hpp>' in header
    assert 'rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr' in header
    assert 'moveit_ik_future_' in header
    assert 'create_moveit_ik_request_' in header
    assert 'request_moveit_ik_' in header
    assert 'consume_ready_moveit_ik_response_' in header

    assert 'auto_declare<std::string>("ik_backend", "kdl")' in source
    assert 'auto_declare<std::string>("moveit_compute_ik_service", "/compute_ik")' in source
    assert 'auto_declare<std::string>("moveit_group_name", "")' in source
    assert 'auto_declare<std::string>("moveit_base_frame", "base")' in source
    assert 'auto_declare<std::string>("moveit_ik_link_name", "")' in source
    assert 'auto_declare<std::string>("tcp_link_name", "")' in source
    assert 'moveit_group_name_ = arm_id_ + "_arm";' in source
    assert 'moveit_ik_link_name_ = arm_id_ + "_link8";' in source
    assert 'tcp_link_name_ = arm_id_ + "_link8";' in source
    assert 'arm_id parameter must not be empty' in source
    assert 'create_client<moveit_msgs::srv::GetPositionIK>' in source
    assert 'ik_backend_ == "moveit_service"' in source
    assert 'compute_ik_client_->async_send_request' in source
    assert 'moveit_ik_future_.wait_for(0s)' in source
    assert 'response->error_code.val == response->error_code.SUCCESS' in source
    assert 'extract_moveit_joint_positions_' in source

    assert 'ik_backend: kdl' in config
    assert 'moveit_compute_ik_service: /compute_ik' in config
    assert 'moveit_group_name: ""' in config
    assert 'moveit_base_frame: base' in config
    assert 'moveit_ik_link_name: ""' in config
    assert 'tcp_link_name: ""' in config
    assert 'request frame=%s' in source
    assert 'target position xyz=' in source
    assert 'target orientation xyzw=' in source
    assert 'startup delta xyz=' in source

    assert '<depend>moveit_msgs</depend>' in package
    assert 'find_package(moveit_msgs REQUIRED)' in cmake
    assert 'moveit_msgs' in cmake


def test_meshcat_preview_reads_kdl_joint_states_instead_of_running_own_ik():
    config = read(BRIDGE_ROOT / 'config' / 'tracker_bridge_preview.yaml')

    assert 'joint_states_topic: /franka_controller/kdl_desired_joint_states' in config
    assert 'enable_robot_ik: false' in config


def test_fr3v2_1_real_config_matches_the_target_hardware():
    config = read(CONTROLLER_ROOT / 'config' / 'real_fr3v2_1_config.yaml')
    source = read(CONTROLLER_ROOT / 'src' / 'joint_impedance_pose_controller.cpp')

    assert 'arm_id: "fr3v2_1"' in config
    assert 'robot_ip: "172.16.0.2"' in config
    assert 'use_fake_hardware: "false"' in config
    assert 'load_gripper: "false"' in config
    assert 'moveit_group_name_ = arm_id_ + "_arm";' in source
    assert 'moveit_ik_link_name_ = arm_id_ + "_link8";' in source
    assert 'tcp_link_name_ = arm_id_ + "_link8";' in source


class KdlJointStateWiringTestCase(unittest.TestCase):
    def test_all_static_wiring_checks(self):
        checks = [
            test_workspace_is_trimmed_to_tracker_pose_controller,
            test_pose_controller_publishes_kdl_desired_joint_states,
            test_fake_kdl_preview_accumulates_from_previous_desired_state,
            test_fake_kdl_preview_starts_from_current_mock_joint_state,
            test_fake_kdl_preview_does_not_reset_to_mock_joint_state_when_target_is_stale,
            test_pose_controller_holds_position_instead_of_throwing_on_ik_failure,
            test_pose_controller_logs_kdl_nonconvergence_diagnostics,
            test_pose_controller_retries_kdl_ik_with_multiple_seed_candidates,
            test_pose_controller_exposes_kdl_ik_solver_parameters,
            test_pose_controller_can_use_moveit_compute_ik_service_backend,
            test_meshcat_preview_reads_kdl_joint_states_instead_of_running_own_ik,
            test_fr3v2_1_real_config_matches_the_target_hardware,
        ]
        for check in checks:
            with self.subTest(check=check.__name__):
                check()


if __name__ == '__main__':
    unittest.main()
