#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def generate_robot_nodes(context):
    config_file_name = LaunchConfiguration("robot_config_file").perform(context)
    controller_name = LaunchConfiguration("controller_name").perform(context)
    package_config_dir = FindPackageShare("franka_arm_controllers").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = load_yaml(config_file)
    nodes = []
    relative_controller_params = os.path.join(
        package_config_dir,
        "config",
        "controllers_relative_target.yaml",
    )
    for item_name, config in configs.items():
        namespace = config["namespace"]
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("franka_arm_controllers"), "launch", "franka.launch.py"]
                    )
                ),
                launch_arguments={
                    "arm_id": str(config["arm_id"]),
                    "arm_prefix": str(config["arm_prefix"]),
                    "namespace": str(namespace),
                    "urdf_package": str(config.get("urdf_package", "franka_description")),
                    "urdf_file": str(config["urdf_file"]),
                    "robot_ip": str(config["robot_ip"]),
                    "load_gripper": str(config["load_gripper"]),
                    "use_fake_hardware": str(config["use_fake_hardware"]),
                    "fake_sensor_commands": str(config["fake_sensor_commands"]),
                    "joint_sources": ",".join(config["joint_sources"]),
                    "joint_state_rate": str(config["joint_state_rate"]),
                    "arm_mounting_orientation": str(config["arm_mounting_orientation"]),
                }.items(),
            )
        )
        spawner_arguments = [
            controller_name,
            "--controller-manager-timeout",
            "30",
        ]
        target_pose_is_relative = str(
            config.get("target_pose_is_relative", "false")
        ).lower() in ("true", "1", "yes")
        if target_pose_is_relative:
            spawner_arguments.extend(["--param-file", relative_controller_params])
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=spawner_arguments,
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_arm_controllers"),
                            "config",
                            "controllers.yaml",
                        ]
                    )
                ],
                output="screen",
            )
        )
    if any(str(config.get("use_rviz", "false")).lower() == "true" for config in configs.values()):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "--display-config",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("franka_description"),
                            "rviz",
                            "visualize_franka_duo.rviz",
                        ]
                    ),
                ],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config_file",
                default_value="tracker_fake_fr3_config.yaml",
                description="Name of the tracker robot configuration file to load (relative to config/ in franka_arm_controllers)",
            ),
            DeclareLaunchArgument(
                "controller_name",
                default_value="joint_impedance_pose_controller",
                description="Controller to spawn from franka_arm_controllers/config/controllers.yaml",
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
