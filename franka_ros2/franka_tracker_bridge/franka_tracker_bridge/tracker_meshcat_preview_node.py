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
"""ROS 2 node that previews tracker pose targets in Meshcat."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from franka_tracker_bridge.ros_conversions import pose_msg_to_value
from franka_tracker_bridge.pose_math import PoseValue, QuaternionValue, Vector3
from franka_tracker_bridge.tracker_pose_target_core import compose_relative_pose
from franka_tracker_bridge.tracker_meshcat_preview_core import (
    clamp_configuration_to_limits,
    joint_state_to_configuration,
    limit_configuration_step,
    make_triad_line_data,
    pose_value_to_transform_matrix,
)


def _tuple3(values, name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f'{name} must contain exactly three values')
    return (float(values[0]), float(values[1]), float(values[2]))


class TrackerMeshcatPreviewNode(Node):
    def __init__(self):
        super().__init__('tracker_meshcat_preview_node')

        self.declare_parameter(
            'target_pose_topic',
            '/franka_controller/target_cartesian_pose',
        )
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('target_pose_is_relative', False)
        self.declare_parameter('robot_type', 'fr3')
        self.declare_parameter('load_gripper', False)
        self.declare_parameter('initial_position', [0.4, 0.0, 0.4])
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('triad_length', 0.12)
        self.declare_parameter('meshcat_zmq_url', '')
        self.declare_parameter('open_browser', False)
        self.declare_parameter(
            'urdf_output_path',
            '/tmp/franka_tracker_bridge/fr3_meshcat.urdf',
        )
        self.declare_parameter('enable_robot_ik', True)
        self.declare_parameter('ik_frame_name', 'fr3_link8')
        self.declare_parameter('sync_target_to_robot_on_start', True)
        self.declare_parameter('ik_max_iterations', 8)
        self.declare_parameter('ik_error_tolerance', 1e-3)
        self.declare_parameter('ik_damping', 1e-4)
        self.declare_parameter('ik_dt', 0.35)
        self.declare_parameter('max_joint_step', 0.04)

        initial_position = _tuple3(
            self.get_parameter('initial_position').value,
            'initial_position',
        )
        self.target_pose = PoseValue(
            position=Vector3(*initial_position),
            orientation=QuaternionValue(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        self.enable_robot_ik = bool(self.get_parameter('enable_robot_ik').value)
        self.target_pose_is_relative = bool(self.get_parameter('target_pose_is_relative').value)
        self.ik_frame_name = str(self.get_parameter('ik_frame_name').value)
        self.ik_max_iterations = max(
            1,
            int(self.get_parameter('ik_max_iterations').value),
        )
        self.ik_error_tolerance = max(
            0.0,
            float(self.get_parameter('ik_error_tolerance').value),
        )
        self.ik_damping = max(1e-12, float(self.get_parameter('ik_damping').value))
        self.ik_dt = max(0.0, float(self.get_parameter('ik_dt').value))
        self.max_joint_step = max(0.0, float(self.get_parameter('max_joint_step').value))
        self._last_robot_display_time = 0.0
        self._ik_frame_id = None
        self._joint_lower_limits = []
        self._joint_upper_limits = []
        self._relative_reference_pose: PoseValue | None = None
        self._pending_relative_target: PoseValue | None = None

        self._init_meshcat()
        self.create_subscription(
            PoseStamped,
            self.get_parameter('target_pose_topic').value,
            self._on_target_pose,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._on_joint_state,
            10,
        )

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / publish_rate, self._on_timer)

    def _init_meshcat(self) -> None:
        try:
            import meshcat
            import meshcat.geometry as g
            import numpy as np
            import pinocchio as pin
            from pinocchio.visualize import MeshcatVisualizer
            import xacro
        except ImportError as exc:
            self.get_logger().error(
                'Meshcat preview requires meshcat, pinocchio, numpy, '
                f'and xacro in the active ROS environment: {exc}'
            )
            raise

        self._meshcat_geometry = g
        self._np = np
        self._pin = pin

        robot_type = str(self.get_parameter('robot_type').value)
        load_gripper = bool(self.get_parameter('load_gripper').value)
        franka_share = Path(get_package_share_directory('franka_description'))
        xacro_file = franka_share / 'robots' / robot_type / f'{robot_type}.urdf.xacro'
        urdf_output_path = Path(str(self.get_parameter('urdf_output_path').value))
        urdf_output_path.parent.mkdir(parents=True, exist_ok=True)

        try:
            doc = xacro.process_file(
                str(xacro_file),
                mappings={
                    'robot_type': robot_type,
                    'hand': 'true' if load_gripper else 'false',
                    'no_prefix': 'false',
                },
            )
            urdf_output_path.write_text(doc.toprettyxml(indent='  '), encoding='utf-8')
            self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
                str(urdf_output_path),
                package_dirs=[str(franka_share.parent)],
            )
        except Exception as exc:  # noqa: BLE001 - xacro and Pinocchio raise mixed exception types.
            self.get_logger().error(f'Failed to load Franka model from {xacro_file}: {exc}')
            raise

        self.data = self.model.createData()
        neutral_q = [float(value) for value in pin.neutral(self.model)]
        self._joint_lower_limits = [
            float(value) for value in self.model.lowerPositionLimit
        ]
        self._joint_upper_limits = [
            float(value) for value in self.model.upperPositionLimit
        ]
        self.q = clamp_configuration_to_limits(
            neutral_q,
            self._joint_lower_limits,
            self._joint_upper_limits,
        )
        self._configure_robot_ik()

        zmq_url = str(self.get_parameter('meshcat_zmq_url').value)
        viewer = meshcat.Visualizer(zmq_url=zmq_url or None)
        self.visualizer = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
        self.visualizer.initViewer(
            viewer=viewer,
            open=bool(self.get_parameter('open_browser').value),
            loadModel=False,
        )
        self.visualizer.loadViewerModel(rootNodeName='franka')
        self.visualizer.display(self._np.array(self.q))

        points, colors = make_triad_line_data(float(self.get_parameter('triad_length').value))
        line_geometry = g.PointsGeometry(self._np.array(points).T, self._np.array(colors).T)
        line_material = g.LineBasicMaterial(vertexColors=True, linewidth=4.0)
        self.visualizer.viewer['tracker_preview/ee_axes'].set_object(
            g.LineSegments(line_geometry, line_material),
        )
        self._display_preview_pose()

        web_url = getattr(self.visualizer.viewer.window, 'web_url', '')
        if web_url:
            self.get_logger().info(f'Meshcat preview URL: {web_url}')
        self.get_logger().info(
            f'Loaded Franka {robot_type} model for Meshcat preview '
            f'from {urdf_output_path}'
        )

    def _configure_robot_ik(self) -> None:
        if not self.enable_robot_ik and not self.target_pose_is_relative:
            return

        frame_id = int(self.model.getFrameId(self.ik_frame_name))
        if frame_id >= len(self.model.frames):
            self.get_logger().error(
                f'Meshcat IK frame {self.ik_frame_name!r} was not found; '
                'Franka model will stay static'
            )
            self.enable_robot_ik = False
            return

        self._ik_frame_id = frame_id
        if not self.enable_robot_ik:
            return
        if self.target_pose_is_relative and self.enable_robot_ik:
            self._relative_reference_pose = self._pose_value_from_ik_frame()
            self.target_pose = self._relative_reference_pose
        if self.enable_robot_ik and bool(self.get_parameter('sync_target_to_robot_on_start').value):
            self.target_pose = self._pose_value_from_ik_frame()

        self.get_logger().info(
            'Meshcat IK enabled: '
            f'frame={self.ik_frame_name}, max_joint_step={self.max_joint_step:.3f} rad/update'
        )

    def _pose_value_from_ik_frame(self) -> PoseValue:
        self._pin.framesForwardKinematics(
            self.model,
            self.data,
            self._np.array(self.q, dtype=float),
        )
        placement = self.data.oMf[self._ik_frame_id]
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

    def _target_pose_to_se3(self):
        transform = self._np.array(
            pose_value_to_transform_matrix(self.target_pose),
            dtype=float,
        )
        return self._pin.SE3(transform[:3, :3], transform[:3, 3])

    def _solve_robot_ik(self) -> None:
        if not self.enable_robot_ik or self._ik_frame_id is None:
            return

        q = self._np.array(self.q, dtype=float)
        target = self._target_pose_to_se3()
        for _ in range(self.ik_max_iterations):
            self._pin.computeJointJacobians(self.model, self.data, q)
            self._pin.updateFramePlacements(self.model, self.data)
            current = self.data.oMf[self._ik_frame_id]
            frame_error = current.actInv(target)
            error = self._pin.log(frame_error).vector
            if float(self._np.linalg.norm(error)) <= self.ik_error_tolerance:
                break

            jacobian = self._pin.getFrameJacobian(
                self.model,
                self.data,
                self._ik_frame_id,
                self._pin.ReferenceFrame.LOCAL,
            )
            jacobian = -self._pin.Jlog6(frame_error.inverse()) @ jacobian
            lhs = jacobian @ jacobian.T + self.ik_damping * self._np.eye(6)
            try:
                delta_q = -jacobian.T @ self._np.linalg.solve(lhs, error)
            except self._np.linalg.LinAlgError:
                self.get_logger().warning(
                    'Meshcat IK solve reached a singular matrix; keeping previous robot pose',
                    throttle_duration_sec=1.0,
                )
                return

            q = self._pin.integrate(self.model, q, delta_q * self.ik_dt)
            q = self._np.array(
                clamp_configuration_to_limits(
                    q.tolist(),
                    self._joint_lower_limits,
                    self._joint_upper_limits,
                ),
                dtype=float,
            )

        limited_q = limit_configuration_step(self.q, q.tolist(), self.max_joint_step)
        self.q = clamp_configuration_to_limits(
            limited_q,
            self._joint_lower_limits,
            self._joint_upper_limits,
        )
        self._display_robot()

    def _on_target_pose(self, msg: PoseStamped) -> None:
        target = pose_msg_to_value(msg.pose)
        if self.target_pose_is_relative:
            self._pending_relative_target = target
            if self._relative_reference_pose is None:
                return
            target = compose_relative_pose(self._relative_reference_pose, target)
        self.target_pose = target
        self._display_preview_pose()
        self._solve_robot_ik()

    def _on_joint_state(self, msg: JointState) -> None:
        if self.enable_robot_ik:
            return

        self.q = joint_state_to_configuration(
            joint_names=list(msg.name),
            positions=list(msg.position),
            model_joint_names=list(self.model.names),
            neutral_q=self.q,
        )
        if self.target_pose_is_relative and self._relative_reference_pose is None:
            self._relative_reference_pose = self._pose_value_from_ik_frame()
            self.target_pose = self._relative_reference_pose
            if self._pending_relative_target is not None:
                self.target_pose = compose_relative_pose(
                    self._relative_reference_pose,
                    self._pending_relative_target,
                )
        self._display_robot()

    def _on_timer(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._display_preview_pose()
        if self.enable_robot_ik:
            self._solve_robot_ik()
        elif now_sec - self._last_robot_display_time > 1.0:
            self._display_robot()

    def _display_robot(self) -> None:
        self.visualizer.display(self._np.array(self.q))
        self._last_robot_display_time = self.get_clock().now().nanoseconds * 1e-9

    def _display_preview_pose(self) -> None:
        transform = self._np.array(pose_value_to_transform_matrix(self.target_pose))
        self.visualizer.viewer['tracker_preview/ee_axes'].set_transform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackerMeshcatPreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
