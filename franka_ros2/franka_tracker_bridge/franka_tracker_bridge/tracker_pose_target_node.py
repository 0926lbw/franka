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
"""ROS 2 node that maps tracker relative pose to robot target pose."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool

from franka_tracker_bridge.ros_conversions import (
    pose_stamped_msg_to_timed_pose,
    pose_value_to_msg,
)
from franka_tracker_bridge.pose_math import (
    PoseValue,
    QuaternionValue,
    Vector3,
    parse_matrix3,
)
from franka_tracker_bridge.tracker_meshcat_preview_core import (
    extract_complete_joint_positions,
    validate_joint_position_limits,
)
from franka_tracker_bridge.tracker_pose_target_core import (
    PoseTargetConfig,
    TrackerPoseTargetMapper,
)


def _tuple3(values, name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f'{name} must contain exactly three values')
    return (float(values[0]), float(values[1]), float(values[2]))


def _int_tuple3(values, name: str) -> tuple[int, int, int]:
    if len(values) != 3:
        raise ValueError(f'{name} must contain exactly three values')
    return (int(values[0]), int(values[1]), int(values[2]))


class TrackerPoseTargetNode(Node):
    def __init__(self):
        super().__init__('tracker_pose_target_node')

        self.declare_parameter('pose_topic', '/tracker/pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('deadman_topic', '/tracker/deadman')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_joy_button', -1)
        self.declare_parameter(
            'target_pose_topic',
            '/franka_controller/target_cartesian_pose',
        )
        self.declare_parameter('target_frame', 'base')
        self.declare_parameter('target_pose_is_relative', False)
        self.declare_parameter('deadman_initially_enabled', False)
        self.declare_parameter('robot_type', 'fr3')
        self.declare_parameter('load_gripper', False)
        self.declare_parameter('ik_frame_name', 'fr3_link8')
        self.declare_parameter(
            'urdf_output_path',
            '/tmp/franka_tracker_bridge/fr3_pose_target.urdf',
        )
        self.declare_parameter('translation_scale', [1.0, 1.0, 1.0])
        self.declare_parameter('rotation_scale', [1.0, 1.0, 1.0])
        self.declare_parameter('translation_limit', [-1.0, -1.0, -1.0])
        self.declare_parameter('rotation_limit', 3.141592653589793)
        self.declare_parameter('tracker_low_pass_alpha', 0.25)
        self.declare_parameter('translation_deadband', 0.003)
        self.declare_parameter('rotation_deadband', 0.02)
        self.declare_parameter('calibration_duration_sec', 3.0)
        self.declare_parameter('calibration_sample_count', 30)
        self.declare_parameter('coord_swap', [1, 0, 2])
        self.declare_parameter('coord_flip', [1.0, -1.0, 1.0])
        self.declare_parameter('coord_scale', [1.0, 1.0, 1.0])
        self.declare_parameter('base_xy_rotation_deg', 0.0)
        self.declare_parameter('orientation_alignment_rpy_deg', [0.0, 0.0, 180.0])
        self.declare_parameter('tracker_rotation_scale', -1.0)
        self.declare_parameter('tracker_rotation_axis_scale', [1.0, 1.0, -1.0])
        self.declare_parameter('tracker_rotation_axis_order', [0, 1, 2])
        self.declare_parameter('tracker_pos_soft_limit_mm', 15.0)
        self.declare_parameter('tracker_pos_hard_limit_mm', 40.0)
        self.declare_parameter('tracker_rot_soft_limit_deg', 6.0)
        self.declare_parameter('tracker_rot_hard_limit_deg', 15.0)
        self.declare_parameter('map_matrix', [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ])

        self._np = None
        self._pin = None
        self._franka_model = None
        self._franka_data = None
        self._franka_neutral_q = None
        self._arm_joint_names: list[str] = []
        self._arm_joint_q_indices: list[int] = []
        self._arm_joint_lower_limits: list[float] = []
        self._arm_joint_upper_limits: list[float] = []
        self._ik_frame_id = None
        self.mapper: TrackerPoseTargetMapper | None = None
        self._invalid_joint_state_warning_emitted = False
        self.target_frame = str(self.get_parameter('target_frame').value)
        self._load_franka_model()
        self._pose_target_config = PoseTargetConfig(
            target_pose_is_relative=bool(
                self.get_parameter('target_pose_is_relative').value,
            ),
            translation_scale=_tuple3(
                self.get_parameter('translation_scale').value,
                'translation_scale',
            ),
            rotation_scale=_tuple3(
                self.get_parameter('rotation_scale').value,
                'rotation_scale',
            ),
            translation_limit=_tuple3(
                self.get_parameter('translation_limit').value,
                'translation_limit',
            ),
            rotation_limit=float(self.get_parameter('rotation_limit').value),
            tracker_low_pass_alpha=float(
                self.get_parameter('tracker_low_pass_alpha').value,
            ),
            translation_deadband=float(self.get_parameter('translation_deadband').value),
            rotation_deadband=float(self.get_parameter('rotation_deadband').value),
            calibration_duration_sec=float(
                self.get_parameter('calibration_duration_sec').value,
            ),
            calibration_sample_count=int(
                self.get_parameter('calibration_sample_count').value,
            ),
            map_matrix=parse_matrix3(self.get_parameter('map_matrix').value),
            coord_swap=_int_tuple3(
                self.get_parameter('coord_swap').value,
                'coord_swap',
            ),
            coord_flip=_tuple3(
                self.get_parameter('coord_flip').value,
                'coord_flip',
            ),
            coord_scale=_tuple3(
                self.get_parameter('coord_scale').value,
                'coord_scale',
            ),
            base_xy_rotation_deg=float(
                self.get_parameter('base_xy_rotation_deg').value,
            ),
            orientation_alignment_rpy_deg=_tuple3(
                self.get_parameter('orientation_alignment_rpy_deg').value,
                'orientation_alignment_rpy_deg',
            ),
            tracker_rotation_scale=float(
                self.get_parameter('tracker_rotation_scale').value,
            ),
            tracker_rotation_axis_scale=_tuple3(
                self.get_parameter('tracker_rotation_axis_scale').value,
                'tracker_rotation_axis_scale',
            ),
            tracker_rotation_axis_order=_int_tuple3(
                self.get_parameter('tracker_rotation_axis_order').value,
                'tracker_rotation_axis_order',
            ),
            tracker_pos_soft_limit_mm=float(
                self.get_parameter('tracker_pos_soft_limit_mm').value,
            ),
            tracker_pos_hard_limit_mm=float(
                self.get_parameter('tracker_pos_hard_limit_mm').value,
            ),
            tracker_rot_soft_limit_deg=float(
                self.get_parameter('tracker_rot_soft_limit_deg').value,
            ),
            tracker_rot_hard_limit_deg=float(
                self.get_parameter('tracker_rot_hard_limit_deg').value,
            ),
        )
        self._deadman_enabled = bool(
            self.get_parameter('deadman_initially_enabled').value,
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            self.get_parameter('target_pose_topic').value,
            10,
        )
        self.deadman_joy_button = int(self.get_parameter('deadman_joy_button').value)
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('deadman_topic').value,
            self._on_deadman,
            10,
        )
        if self.deadman_joy_button >= 0:
            self.create_subscription(
                Joy,
                self.get_parameter('joy_topic').value,
                self._on_joy,
                10,
            )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            self._on_pose,
            10,
        )

        self.get_logger().info(
            'Started tracker_pose_target_node: '
            f'target_pose_topic={self.get_parameter("target_pose_topic").value} '
            f'target_frame={self.target_frame} '
            f'joint_states_topic={self.get_parameter("joint_states_topic").value} '
            f'target_pose_is_relative='
            f'{self._pose_target_config.target_pose_is_relative} '
            f'calibration_duration_sec={self.get_parameter("calibration_duration_sec").value}'
        )

    def _load_franka_model(self) -> Path:
        if self._franka_model is not None:
            return Path(str(self.get_parameter('urdf_output_path').value))

        try:
            import numpy as np
            import pinocchio as pin
            import xacro
        except ImportError as exc:
            self.get_logger().error(
                'Franka model loading requires numpy, pinocchio, '
                f'and xacro in the active ROS environment: {exc}'
            )
            raise

        robot_type = str(self.get_parameter('robot_type').value)
        load_gripper = bool(self.get_parameter('load_gripper').value)
        ik_frame_name = str(self.get_parameter('ik_frame_name').value)
        franka_share = Path(get_package_share_directory('franka_description'))
        xacro_file = franka_share / 'robots' / robot_type / f'{robot_type}.urdf.xacro'
        urdf_output_path = Path(str(self.get_parameter('urdf_output_path').value))
        urdf_output_path.parent.mkdir(parents=True, exist_ok=True)

        doc = xacro.process_file(
            str(xacro_file),
            mappings={
                'robot_type': robot_type,
                'hand': 'true' if load_gripper else 'false',
                'no_prefix': 'false',
            },
        )
        urdf_output_path.write_text(doc.toprettyxml(indent='  '), encoding='utf-8')
        model, _, _ = pin.buildModelsFromUrdf(
            str(urdf_output_path),
            package_dirs=[str(franka_share.parent)],
        )
        frame_id = int(model.getFrameId(ik_frame_name))
        if frame_id >= len(model.frames):
            raise ValueError(f'IK frame {ik_frame_name!r} was not found in {xacro_file}')

        arm_joint_names = [f'{robot_type}_joint{index}' for index in range(1, 8)]
        arm_joint_q_indices = []
        for joint_name in arm_joint_names:
            joint_id = int(model.getJointId(joint_name))
            if joint_id >= model.njoints:
                raise ValueError(f'Arm joint {joint_name!r} was not found in {xacro_file}')
            if int(model.nqs[joint_id]) != 1:
                raise ValueError(f'Arm joint {joint_name!r} must have exactly one position')
            arm_joint_q_indices.append(int(model.idx_qs[joint_id]))
        arm_joint_lower_limits = [
            float(model.lowerPositionLimit[index]) for index in arm_joint_q_indices
        ]
        arm_joint_upper_limits = [
            float(model.upperPositionLimit[index]) for index in arm_joint_q_indices
        ]

        self._np = np
        self._pin = pin
        self._franka_model = model
        self._franka_data = model.createData()
        self._franka_neutral_q = np.array(pin.neutral(model), dtype=float)
        self._arm_joint_names = arm_joint_names
        self._arm_joint_q_indices = arm_joint_q_indices
        self._arm_joint_lower_limits = arm_joint_lower_limits
        self._arm_joint_upper_limits = arm_joint_upper_limits
        self._ik_frame_id = frame_id
        return urdf_output_path

    def _pose_from_configuration(self, configuration) -> PoseValue:
        self._pin.framesForwardKinematics(
            self._franka_model,
            self._franka_data,
            configuration,
        )
        placement = self._franka_data.oMf[self._ik_frame_id]
        quat = self._pin.Quaternion(placement.rotation)
        quat.normalize()
        return PoseValue(
            position=Vector3(
                float(placement.translation[0]),
                float(placement.translation[1]),
                float(placement.translation[2]),
            ),
            orientation=QuaternionValue(
                x=float(quat.x),
                y=float(quat.y),
                z=float(quat.z),
                w=float(quat.w),
            ),
        )

    def _configuration_from_joint_state(self, msg: JointState):
        positions = extract_complete_joint_positions(
            list(msg.name),
            list(msg.position),
            self._arm_joint_names,
        )
        if positions is None:
            return None
        positions = validate_joint_position_limits(
            positions,
            self._arm_joint_lower_limits,
            self._arm_joint_upper_limits,
        )
        if positions is None:
            return None

        configuration = self._franka_neutral_q.copy()
        for q_index, position in zip(self._arm_joint_q_indices, positions):
            configuration[q_index] = position
        return configuration, positions

    def _on_joint_state(self, msg: JointState) -> None:
        if self.mapper is not None:
            return

        startup_state = self._configuration_from_joint_state(msg)
        if startup_state is None:
            if not self._invalid_joint_state_warning_emitted:
                self.get_logger().warning(
                    'Waiting for a complete, finite, in-limit startup joint state containing: '
                    + ', '.join(self._arm_joint_names)
                )
                self._invalid_joint_state_warning_emitted = True
            return

        configuration, positions = startup_state
        robot_start_pose = self._pose_from_configuration(configuration)
        self.mapper = TrackerPoseTargetMapper(
            robot_start_pose,
            self._pose_target_config,
        )
        self.mapper.set_deadman(self._deadman_enabled)
        self.get_logger().info(
            'Captured tracker teleoperation start from current joints: '
            + ', '.join(
                f'{name}={position:.6f}'
                for name, position in zip(self._arm_joint_names, positions)
            )
        )

    def _on_deadman(self, msg: Bool) -> None:
        self._deadman_enabled = bool(msg.data)
        if self.mapper is not None:
            self.mapper.set_deadman(self._deadman_enabled)

    def _on_joy(self, msg: Joy) -> None:
        if self.deadman_joy_button >= len(msg.buttons):
            self._deadman_enabled = False
            if self.mapper is not None:
                self.mapper.set_deadman(False)
            return
        self._deadman_enabled = bool(msg.buttons[self.deadman_joy_button])
        if self.mapper is not None:
            self.mapper.set_deadman(self._deadman_enabled)

    def _on_pose(self, msg: PoseStamped) -> None:
        if self.mapper is None:
            return
        target = self.mapper.update(pose_stamped_msg_to_timed_pose(msg))
        if target is None:
            return

        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = self.target_frame
        target_msg.pose = pose_value_to_msg(target, type(target_msg.pose))
        self.publisher.publish(target_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackerPoseTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
