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
"""Single-arm real-robot tracker entry point for the FR3 v2.1 setup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    robot_type = LaunchConfiguration('robot_type')
    ik_frame_name = LaunchConfiguration('ik_frame_name')
    publish_deadman = LaunchConfiguration('publish_deadman')
    package_share = FindPackageShare('franka_tracker_bridge')
    preview_launch = PathJoinSubstitution([
        package_share,
        'launch',
        'tracker_preview.launch.py',
    ])
    real_config = PathJoinSubstitution([
        package_share,
        'config',
        'tracker_bridge_fr3v2_1_real.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=real_config),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('robot_type', default_value='fr3v2_1'),
        DeclareLaunchArgument('ik_frame_name', default_value='fr3v2_1_link8'),
        DeclareLaunchArgument('publish_deadman', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(preview_launch),
            launch_arguments={
                'config_file': config_file,
                'joint_states_topic': joint_states_topic,
                'robot_type': robot_type,
                'ik_frame_name': ik_frame_name,
                'publish_deadman': publish_deadman,
            }.items(),
        ),
    ])
