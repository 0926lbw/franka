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
"""Tracker relative pose to robot end-effector target pose mapping."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from franka_tracker_bridge.pose_math import (
    Matrix3,
    PoseValue,
    QuaternionValue,
    TimedPose,
    Vector3,
    invert_quaternion,
    matrix_vector_mul,
    multiply_quaternion,
    normalize_quaternion,
    parse_matrix3,
    quaternion_from_axis_angle,
    quaternion_to_axis_angle,
)


def _validate_vector3(values: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f'{name} must have exactly three values')
    return tuple(float(value) for value in values)


def _validate_index_order3(values: Sequence[int], name: str) -> tuple[int, int, int]:
    if len(values) != 3:
        raise ValueError(f'{name} must have exactly three values')
    result = tuple(int(value) for value in values)
    if sorted(result) != [0, 1, 2]:
        raise ValueError(f'{name} must be a permutation of [0, 1, 2]')
    return result


def _validate_translation_limit(
    values: Sequence[float],
    name: str,
) -> tuple[float, float, float]:
    return _validate_vector3(values, name)


def _clamp(value: float, limit: float) -> float:
    if limit < 0.0:
        return value
    return max(-limit, min(limit, value))


def _clamp_vector(values: Sequence[float], limits: Sequence[float]) -> tuple[float, float, float]:
    return tuple(_clamp(float(value), float(limit)) for value, limit in zip(values, limits))


def _limit_rotation_vector(
    values: Sequence[float],
    limit: float,
) -> tuple[float, float, float]:
    vector = tuple(float(value) for value in values)
    norm = math.sqrt(sum(value * value for value in vector))
    if norm == 0.0 or norm <= limit:
        return vector
    scale = limit / norm
    return tuple(value * scale for value in vector)


def _deadband_vector(
    values: Sequence[float],
    deadband: float,
) -> tuple[float, float, float]:
    return tuple(0.0 if abs(float(value)) < deadband else float(value) for value in values)


def _rotate_vector(
    orientation: QuaternionValue,
    vector: Sequence[float],
) -> tuple[float, float, float]:
    q = normalize_quaternion(orientation)
    vx, vy, vz = (float(vector[0]), float(vector[1]), float(vector[2]))
    ux, uy, uz = q.x, q.y, q.z
    dot = ux * vx + uy * vy + uz * vz
    uu = ux * ux + uy * uy + uz * uz
    cross = (
        uy * vz - uz * vy,
        uz * vx - ux * vz,
        ux * vy - uy * vx,
    )
    vector_scale = q.w * q.w - uu
    return (
        2.0 * dot * ux + vector_scale * vx + 2.0 * q.w * cross[0],
        2.0 * dot * uy + vector_scale * vy + 2.0 * q.w * cross[1],
        2.0 * dot * uz + vector_scale * vz + 2.0 * q.w * cross[2],
    )


def _matrix_transpose(matrix: Matrix3) -> Matrix3:
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def _matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            left[row][0] * right[0][col]
            + left[row][1] * right[1][col]
            + left[row][2] * right[2][col]
            for col in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def _rotation_matrix_x(angle: float) -> Matrix3:
    c = math.cos(angle)
    s = math.sin(angle)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rotation_matrix_y(angle: float) -> Matrix3:
    c = math.cos(angle)
    s = math.sin(angle)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rotation_matrix_z(angle: float) -> Matrix3:
    c = math.cos(angle)
    s = math.sin(angle)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _rpy_deg_to_matrix(rpy_deg: Sequence[float]) -> Matrix3:
    roll, pitch, yaw = (math.radians(float(value)) for value in rpy_deg)
    return _matrix_multiply(
        _matrix_multiply(_rotation_matrix_z(yaw), _rotation_matrix_y(pitch)),
        _rotation_matrix_x(roll),
    )


def _quaternion_to_matrix(orientation: QuaternionValue) -> Matrix3:
    q = normalize_quaternion(orientation)
    xx = q.x * q.x
    yy = q.y * q.y
    zz = q.z * q.z
    xy = q.x * q.y
    xz = q.x * q.z
    yz = q.y * q.z
    wx = q.w * q.x
    wy = q.w * q.y
    wz = q.w * q.z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _matrix_to_quaternion(matrix: Matrix3) -> QuaternionValue:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return normalize_quaternion(
            QuaternionValue(
                x=(matrix[2][1] - matrix[1][2]) / scale,
                y=(matrix[0][2] - matrix[2][0]) / scale,
                z=(matrix[1][0] - matrix[0][1]) / scale,
                w=0.25 * scale,
            )
        )

    if matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        return normalize_quaternion(
            QuaternionValue(
                x=0.25 * scale,
                y=(matrix[0][1] + matrix[1][0]) / scale,
                z=(matrix[0][2] + matrix[2][0]) / scale,
                w=(matrix[2][1] - matrix[1][2]) / scale,
            )
        )

    if matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        return normalize_quaternion(
            QuaternionValue(
                x=(matrix[0][1] + matrix[1][0]) / scale,
                y=0.25 * scale,
                z=(matrix[1][2] + matrix[2][1]) / scale,
                w=(matrix[0][2] - matrix[2][0]) / scale,
            )
        )

    scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
    return normalize_quaternion(
        QuaternionValue(
            x=(matrix[0][2] + matrix[2][0]) / scale,
            y=(matrix[1][2] + matrix[2][1]) / scale,
            z=0.25 * scale,
            w=(matrix[1][0] - matrix[0][1]) / scale,
        )
    )


def _map_delta_orientation(
    orientation: QuaternionValue,
    map_matrix: Matrix3,
) -> QuaternionValue:
    mapped_matrix = _matrix_multiply(
        _matrix_multiply(map_matrix, _quaternion_to_matrix(orientation)),
        _matrix_transpose(map_matrix),
    )
    return _matrix_to_quaternion(mapped_matrix)


def _slerp_quaternion(
    start: QuaternionValue,
    end: QuaternionValue,
    alpha: float,
) -> QuaternionValue:
    q0 = normalize_quaternion(start)
    q1 = normalize_quaternion(end)
    dot = q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w
    if dot < 0.0:
        q1 = QuaternionValue(-q1.x, -q1.y, -q1.z, -q1.w)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))

    if dot > 0.9995:
        beta = 1.0 - alpha
        return normalize_quaternion(
            QuaternionValue(
                beta * q0.x + alpha * q1.x,
                beta * q0.y + alpha * q1.y,
                beta * q0.z + alpha * q1.z,
                beta * q0.w + alpha * q1.w,
            )
        )

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)
    scale_0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_1 = sin_theta / sin_theta_0
    return normalize_quaternion(
        QuaternionValue(
            scale_0 * q0.x + scale_1 * q1.x,
            scale_0 * q0.y + scale_1 * q1.y,
            scale_0 * q0.z + scale_1 * q1.z,
            scale_0 * q0.w + scale_1 * q1.w,
        )
    )


def compose_relative_pose(reference: PoseValue, relative: PoseValue) -> PoseValue:
    """Compose a base-frame pose delta with a captured robot reference pose."""
    return PoseValue(
        position=Vector3(
            reference.position.x + relative.position.x,
            reference.position.y + relative.position.y,
            reference.position.z + relative.position.z,
        ),
        orientation=multiply_quaternion(
            relative.orientation,
            reference.orientation,
        ),
    )


def relative_pose_from_start(target: PoseValue, reference: PoseValue) -> PoseValue:
    """Express an absolute target as a base-frame delta from a captured reference pose."""
    return PoseValue(
        position=Vector3(
            target.position.x - reference.position.x,
            target.position.y - reference.position.y,
            target.position.z - reference.position.z,
        ),
        orientation=multiply_quaternion(
            target.orientation,
            invert_quaternion(reference.orientation),
        ),
    )


@dataclass
class PoseTargetConfig:
    translation_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    rotation_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    translation_limit: tuple[float, float, float] = (-1.0, -1.0, -1.0)
    rotation_limit: float = math.pi
    tracker_low_pass_alpha: float = 1.0
    translation_deadband: float = 0.0
    rotation_deadband: float = 0.0
    calibration_duration_sec: float = 0.0
    calibration_sample_count: int = 1
    map_matrix: Matrix3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    coord_swap: tuple[int, int, int] = (0, 1, 2)
    coord_flip: tuple[float, float, float] = (1.0, 1.0, 1.0)
    coord_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    base_xy_rotation_deg: float = 0.0
    orientation_alignment_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tracker_rotation_scale: float = 1.0
    tracker_rotation_axis_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    tracker_rotation_axis_order: tuple[int, int, int] = (0, 1, 2)
    tracker_pos_soft_limit_mm: float = -1.0
    tracker_pos_hard_limit_mm: float = -1.0
    tracker_rot_soft_limit_deg: float = -1.0
    tracker_rot_hard_limit_deg: float = -1.0
    target_pose_is_relative: bool = False

    def __post_init__(self) -> None:
        self.translation_scale = _validate_vector3(
            self.translation_scale,
            'translation_scale',
        )
        self.rotation_scale = _validate_vector3(
            self.rotation_scale,
            'rotation_scale',
        )
        self.translation_limit = _validate_translation_limit(
            self.translation_limit,
            'translation_limit',
        )
        self.rotation_limit = float(self.rotation_limit)
        if self.rotation_limit < 0.0:
            raise ValueError('rotation_limit must be non-negative')
        self.tracker_low_pass_alpha = float(self.tracker_low_pass_alpha)
        if not 0.0 <= self.tracker_low_pass_alpha <= 1.0:
            raise ValueError('tracker_low_pass_alpha must be in [0, 1]')
        self.translation_deadband = float(self.translation_deadband)
        if self.translation_deadband < 0.0:
            raise ValueError('translation_deadband must be non-negative')
        self.rotation_deadband = float(self.rotation_deadband)
        if self.rotation_deadband < 0.0:
            raise ValueError('rotation_deadband must be non-negative')
        self.calibration_duration_sec = float(self.calibration_duration_sec)
        if self.calibration_duration_sec < 0.0:
            raise ValueError('calibration_duration_sec must be non-negative')
        self.calibration_sample_count = int(self.calibration_sample_count)
        if self.calibration_sample_count < 1:
            raise ValueError('calibration_sample_count must be at least 1')
        self.map_matrix = parse_matrix3(self.map_matrix)
        self.coord_swap = _validate_index_order3(self.coord_swap, 'coord_swap')
        self.coord_flip = _validate_vector3(self.coord_flip, 'coord_flip')
        self.coord_scale = _validate_vector3(self.coord_scale, 'coord_scale')
        self.base_xy_rotation_deg = float(self.base_xy_rotation_deg)
        self.orientation_alignment_rpy_deg = _validate_vector3(
            self.orientation_alignment_rpy_deg,
            'orientation_alignment_rpy_deg',
        )
        self.tracker_rotation_scale = float(self.tracker_rotation_scale)
        self.tracker_rotation_axis_scale = _validate_vector3(
            self.tracker_rotation_axis_scale,
            'tracker_rotation_axis_scale',
        )
        self.tracker_rotation_axis_order = _validate_index_order3(
            self.tracker_rotation_axis_order,
            'tracker_rotation_axis_order',
        )
        self.tracker_pos_soft_limit_mm = float(self.tracker_pos_soft_limit_mm)
        self.tracker_pos_hard_limit_mm = float(self.tracker_pos_hard_limit_mm)
        self.tracker_rot_soft_limit_deg = float(self.tracker_rot_soft_limit_deg)
        self.tracker_rot_hard_limit_deg = float(self.tracker_rot_hard_limit_deg)
        self.target_pose_is_relative = bool(self.target_pose_is_relative)


class TrackerPoseTargetMapper:
    """Map tracker pose relative to a baseline into a robot target pose."""

    def __init__(
        self,
        robot_start_pose: PoseValue,
        config: PoseTargetConfig | None = None,
    ):
        self.config = config or PoseTargetConfig()
        self.robot_start_pose = robot_start_pose
        self._tracker_start_pose: PoseValue | None = None
        self._filtered_tracker_pose: PoseValue | None = None
        self._calibration_start_sec: float | None = None
        self._calibration_sample_count = 0
        self._calibration_position_sum = [0.0, 0.0, 0.0]
        self._calibration_orientation_average: QuaternionValue | None = None
        self._deadman_enabled = False
        self._last_accepted_delta_pos = (0.0, 0.0, 0.0)
        self._last_accepted_delta_rotvec = (0.0, 0.0, 0.0)

    def set_deadman(self, enabled: bool) -> None:
        self._deadman_enabled = bool(enabled)

    def reset_robot_start_pose(self, robot_start_pose: PoseValue) -> None:
        self.robot_start_pose = robot_start_pose
        self._tracker_start_pose = None
        self._filtered_tracker_pose = None
        self._calibration_start_sec = None
        self._calibration_sample_count = 0
        self._calibration_position_sum = [0.0, 0.0, 0.0]
        self._calibration_orientation_average = None
        self._last_accepted_delta_pos = (0.0, 0.0, 0.0)
        self._last_accepted_delta_rotvec = (0.0, 0.0, 0.0)

    def update(self, current: TimedPose) -> PoseValue | None:
        tracker_pose = self._filter_tracker_pose(current.pose)

        if self._tracker_start_pose is None:
            if not self._update_calibration(tracker_pose, current.stamp_sec):
                return None
            if not self._deadman_enabled:
                return None
            if self.config.target_pose_is_relative:
                return relative_pose_from_start(self.robot_start_pose, self.robot_start_pose)
            return self.robot_start_pose

        if not self._deadman_enabled:
            return None

        target = self._target_from_relative_pose(
            self._tracker_start_pose,
            tracker_pose,
            self.robot_start_pose,
        )
        if self.config.target_pose_is_relative:
            return relative_pose_from_start(target, self.robot_start_pose)
        return target

    def _update_calibration(self, tracker_pose: PoseValue, stamp_sec: float) -> bool:
        self._calibration_sample_count += 1

        if self.config.calibration_duration_sec <= 0.0:
            if self._calibration_sample_count < self.config.calibration_sample_count:
                return False
            self._tracker_start_pose = tracker_pose
            return True

        if self._calibration_start_sec is None:
            self._calibration_start_sec = stamp_sec

        self._calibration_position_sum[0] += tracker_pose.position.x
        self._calibration_position_sum[1] += tracker_pose.position.y
        self._calibration_position_sum[2] += tracker_pose.position.z

        if self._calibration_orientation_average is None:
            self._calibration_orientation_average = tracker_pose.orientation
        else:
            alpha = 1.0 / float(self._calibration_sample_count)
            self._calibration_orientation_average = _slerp_quaternion(
                self._calibration_orientation_average,
                tracker_pose.orientation,
                alpha,
            )

        elapsed_sec = stamp_sec - self._calibration_start_sec
        if elapsed_sec < self.config.calibration_duration_sec:
            return False

        sample_count = float(self._calibration_sample_count)
        self._tracker_start_pose = PoseValue(
            position=Vector3(
                self._calibration_position_sum[0] / sample_count,
                self._calibration_position_sum[1] / sample_count,
                self._calibration_position_sum[2] / sample_count,
            ),
            orientation=self._calibration_orientation_average or tracker_pose.orientation,
        )
        self._last_accepted_delta_pos = (0.0, 0.0, 0.0)
        self._last_accepted_delta_rotvec = (0.0, 0.0, 0.0)
        return True

    def _filter_tracker_pose(self, pose: PoseValue) -> PoseValue:
        raw = PoseValue(
            position=Vector3(
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ),
            orientation=normalize_quaternion(pose.orientation),
        )
        previous = self._filtered_tracker_pose
        if previous is None:
            self._filtered_tracker_pose = raw
            return raw

        alpha = self.config.tracker_low_pass_alpha
        beta = 1.0 - alpha
        filtered = PoseValue(
            position=Vector3(
                beta * previous.position.x + alpha * raw.position.x,
                beta * previous.position.y + alpha * raw.position.y,
                beta * previous.position.z + alpha * raw.position.z,
            ),
            orientation=_slerp_quaternion(previous.orientation, raw.orientation, alpha),
        )
        self._filtered_tracker_pose = filtered
        return filtered

    def _map_tracker_translation(
        self,
        tracker_delta: Sequence[float],
    ) -> tuple[float, float, float]:
        matrix_mapped = matrix_vector_mul(self.config.map_matrix, tracker_delta)
        transformed = [
            matrix_mapped[self.config.coord_swap[index]]
            * self.config.coord_flip[index]
            * self.config.coord_scale[index]
            * self.config.translation_scale[index]
            for index in range(3)
        ]

        angle = math.radians(self.config.base_xy_rotation_deg)
        c = math.cos(angle)
        s = math.sin(angle)
        x = transformed[0]
        y = transformed[1]
        transformed[0] = x * c - y * s
        transformed[1] = x * s + y * c
        return (transformed[0], transformed[1], transformed[2])

    def _apply_jump_limit(
        self,
        current: Sequence[float],
        previous: Sequence[float],
        soft_limit: float,
        hard_limit: float,
        unit_scale: float,
    ) -> tuple[float, float, float]:
        current_tuple = tuple(float(value) for value in current)
        if soft_limit <= 0.0:
            return current_tuple

        step = tuple(current_tuple[index] - float(previous[index]) for index in range(3))
        step_norm = math.sqrt(sum(value * value for value in step))
        step_size = step_norm * unit_scale
        if step_norm < 1e-12 or step_size <= soft_limit:
            return current_tuple

        max_step = min(step_size, hard_limit) if hard_limit > 0.0 else soft_limit
        scale = (max_step / unit_scale) / step_norm
        return tuple(float(previous[index]) + step[index] * scale for index in range(3))

    def _apply_tracker_jump_suppression(
        self,
        delta_pos: Sequence[float],
        delta_rotvec: Sequence[float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        filtered_pos = self._apply_jump_limit(
            delta_pos,
            self._last_accepted_delta_pos,
            self.config.tracker_pos_soft_limit_mm,
            self.config.tracker_pos_hard_limit_mm,
            1000.0,
        )
        filtered_rotvec = self._apply_jump_limit(
            delta_rotvec,
            self._last_accepted_delta_rotvec,
            self.config.tracker_rot_soft_limit_deg,
            self.config.tracker_rot_hard_limit_deg,
            180.0 / math.pi,
        )
        self._last_accepted_delta_pos = filtered_pos
        self._last_accepted_delta_rotvec = filtered_rotvec
        return filtered_pos, filtered_rotvec

    def _map_tracker_rotation_vector(
        self,
        tracker_delta_orientation: QuaternionValue,
    ) -> tuple[float, float, float]:
        alignment = _rpy_deg_to_matrix(self.config.orientation_alignment_rpy_deg)
        aligned_matrix = _matrix_multiply(
            _matrix_multiply(alignment, _quaternion_to_matrix(tracker_delta_orientation)),
            _matrix_transpose(alignment),
        )
        aligned_axis_angle = quaternion_to_axis_angle(
            _matrix_to_quaternion(aligned_matrix),
        ).as_tuple()
        ordered = tuple(
            aligned_axis_angle[self.config.tracker_rotation_axis_order[index]]
            for index in range(3)
        )
        return tuple(
            ordered[index]
            * self.config.tracker_rotation_scale
            * self.config.tracker_rotation_axis_scale[index]
            * self.config.rotation_scale[index]
            for index in range(3)
        )

    def _target_from_relative_pose(
        self,
        tracker_start: PoseValue,
        tracker_current: PoseValue,
        robot_start: PoseValue,
    ) -> PoseValue:
        tracker_world_delta = (
            tracker_current.position.x - tracker_start.position.x,
            tracker_current.position.y - tracker_start.position.y,
            tracker_current.position.z - tracker_start.position.z,
        )
        tracker_delta = tracker_world_delta
        tracker_delta = _deadband_vector(tracker_delta, self.config.translation_deadband)
        mapped_delta = self._map_tracker_translation(tracker_delta)
        limited_delta = _clamp_vector(mapped_delta, self.config.translation_limit)

        dq = multiply_quaternion(
            tracker_current.orientation,
            invert_quaternion(tracker_start.orientation),
        )
        axis_angle = self._map_tracker_rotation_vector(dq)
        axis_angle = _deadband_vector(axis_angle, self.config.rotation_deadband)
        limited_axis_angle = _limit_rotation_vector(
            axis_angle,
            self.config.rotation_limit,
        )
        limited_delta, limited_axis_angle = self._apply_tracker_jump_suppression(
            limited_delta,
            limited_axis_angle,
        )
        angle = math.sqrt(sum(value * value for value in limited_axis_angle))
        delta_orientation = QuaternionValue(0.0, 0.0, 0.0, 1.0)
        if angle > 0.0:
            axis = tuple(value / angle for value in limited_axis_angle)
            tracker_delta_orientation = quaternion_from_axis_angle(axis, angle)
            delta_orientation = _map_delta_orientation(
                tracker_delta_orientation,
                self.config.map_matrix,
            )

        return PoseValue(
            position=Vector3(
                robot_start.position.x + limited_delta[0],
                robot_start.position.y + limited_delta[1],
                robot_start.position.z + limited_delta[2],
            ),
            orientation=multiply_quaternion(
                delta_orientation,
                robot_start.orientation,
            ),
        )
