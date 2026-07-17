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
"""Start the joint impedance pose controller for the single FR3 v2.1 arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_config_file = LaunchConfiguration('robot_config_file')
    controller_name = LaunchConfiguration('controller_name')
    controller_launch = PathJoinSubstitution([
        FindPackageShare('franka_arm_controllers'),
        'launch',
        'joint_impedance_pose_controller.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value='real_fr3v2_1_config.yaml',
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_impedance_pose_controller',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            launch_arguments={
                'robot_config_file': robot_config_file,
                'controller_name': controller_name,
            }.items(),
        ),
    ])
