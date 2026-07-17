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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    robot_type = LaunchConfiguration('robot_type')
    load_gripper = LaunchConfiguration('load_gripper')
    meshcat_zmq_url = LaunchConfiguration('meshcat_zmq_url')
    open_browser = LaunchConfiguration('open_browser')
    target_frame = LaunchConfiguration('target_frame')
    ik_frame_name = LaunchConfiguration('ik_frame_name')
    use_libsurvive = LaunchConfiguration('use_libsurvive')
    target_serial = LaunchConfiguration('target_serial')
    libsurvive_args = LaunchConfiguration('libsurvive_args')
    publish_deadman = LaunchConfiguration('publish_deadman')
    tracker_low_pass_alpha = LaunchConfiguration('tracker_low_pass_alpha')
    translation_deadband = LaunchConfiguration('translation_deadband')
    rotation_deadband = LaunchConfiguration('rotation_deadband')
    calibration_duration_sec = LaunchConfiguration('calibration_duration_sec')
    calibration_sample_count = LaunchConfiguration('calibration_sample_count')
    debug_curves = LaunchConfiguration('debug_curves')

    default_config = PathJoinSubstitution([
        FindPackageShare('franka_tracker_bridge'),
        'config',
        'tracker_bridge_preview.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('meshcat_zmq_url', default_value=''),
        DeclareLaunchArgument('open_browser', default_value='true'),
        DeclareLaunchArgument('debug_curves', default_value='false'),
        DeclareLaunchArgument('target_frame', default_value='base'),
        DeclareLaunchArgument('ik_frame_name', default_value=[robot_type, '_link8']),
        DeclareLaunchArgument('use_libsurvive', default_value='true'),
        DeclareLaunchArgument('target_serial', default_value=''),
        DeclareLaunchArgument('libsurvive_args', default_value=''),
        DeclareLaunchArgument('publish_deadman', default_value='false'),
        DeclareLaunchArgument('tracker_low_pass_alpha', default_value='0.25'),
        DeclareLaunchArgument('translation_deadband', default_value='0.003'),
        DeclareLaunchArgument('rotation_deadband', default_value='0.02'),
        DeclareLaunchArgument('calibration_duration_sec', default_value='3.0'),
        DeclareLaunchArgument('calibration_sample_count', default_value='30'),
        Node(
            package='franka_tracker_bridge',
            executable='libsurvive_pose_node',
            name='libsurvive_pose_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'target_serial': target_serial,
                    'libsurvive_args': libsurvive_args,
                    'publish_deadman': ParameterValue(publish_deadman, value_type=bool),
                },
            ],
            condition=IfCondition(use_libsurvive),
        ),
        Node(
            package='franka_tracker_bridge',
            executable='tracker_pose_target_node',
            name='tracker_pose_target_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'joint_states_topic': joint_states_topic,
                    'robot_type': robot_type,
                    'load_gripper': ParameterValue(load_gripper, value_type=bool),
                    'target_frame': target_frame,
                    'ik_frame_name': ik_frame_name,
                    'tracker_low_pass_alpha': ParameterValue(
                        tracker_low_pass_alpha,
                        value_type=float,
                    ),
                    'translation_deadband': ParameterValue(
                        translation_deadband,
                        value_type=float,
                    ),
                    'rotation_deadband': ParameterValue(
                        rotation_deadband,
                        value_type=float,
                    ),
                    'calibration_duration_sec': ParameterValue(
                        calibration_duration_sec,
                        value_type=float,
                    ),
                    'calibration_sample_count': ParameterValue(
                        calibration_sample_count,
                        value_type=int,
                    ),
                },
            ],
        ),
        Node(
            package='franka_tracker_bridge',
            executable='tracker_meshcat_preview_node',
            name='tracker_meshcat_preview_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'robot_type': robot_type,
                    'load_gripper': ParameterValue(load_gripper, value_type=bool),
                    'ik_frame_name': ik_frame_name,
                    'meshcat_zmq_url': meshcat_zmq_url,
                    'open_browser': ParameterValue(open_browser, value_type=bool),
                },
            ],
        ),
        Node(
            package='franka_tracker_bridge',
            executable='tracker_debug_curve_node',
            name='tracker_debug_curve_node',
            output='screen',
            condition=IfCondition(debug_curves),
        ),
    ])
