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
"""Pure Python helpers for Meshcat tracker preview visualization."""

from __future__ import annotations

import math

from franka_tracker_bridge.pose_math import PoseValue


def pose_value_to_transform_matrix(pose: PoseValue) -> list[list[float]]:
    """Convert a PoseValue into a Meshcat-compatible 4x4 transform matrix."""
    x = pose.orientation.x
    y = pose.orientation.y
    z = pose.orientation.z
    w = pose.orientation.w
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm == 0.0:
        x = y = z = 0.0
        w = 1.0
    else:
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), pose.position.x],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), pose.position.y],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), pose.position.z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def make_triad_line_data(
    length: float,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
    """Return local line endpoints and per-vertex colors for XYZ axes."""
    axis_length = float(length)
    points = [
        (0.0, 0.0, 0.0), (axis_length, 0.0, 0.0),
        (0.0, 0.0, 0.0), (0.0, axis_length, 0.0),
        (0.0, 0.0, 0.0), (0.0, 0.0, axis_length),
    ]
    colors = [
        (1.0, 0.0, 0.0), (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0), (0.0, 1.0, 0.0),
        (0.0, 0.2, 1.0), (0.0, 0.2, 1.0),
    ]
    return points, colors


def clamp_configuration_to_limits(
    q: list[float],
    lower: list[float],
    upper: list[float],
) -> list[float]:
    """Clamp a robot configuration to per-joint position limits."""
    return [
        max(float(lo), min(float(hi), float(value)))
        for value, lo, hi in zip(q, lower, upper)
    ]


def limit_configuration_step(
    current: list[float],
    target: list[float],
    max_step: float,
) -> list[float]:
    """Limit per-joint motion between two configurations."""
    step_limit = abs(float(max_step))
    limited = []
    for current_value, target_value in zip(current, target):
        current_float = float(current_value)
        target_float = float(target_value)
        delta = target_float - current_float
        if delta > step_limit:
            limited.append(current_float + step_limit)
        elif delta < -step_limit:
            limited.append(current_float - step_limit)
        else:
            limited.append(target_float)
    return limited


def extract_complete_joint_positions(
    joint_names: list[str],
    positions: list[float],
    required_joint_names: list[str],
) -> list[float] | None:
    """Return required joint positions in model order when the sample is valid."""
    if len(joint_names) != len(positions):
        return None

    by_name: dict[str, float] = {}
    for name, position in zip(joint_names, positions):
        if name in by_name:
            return None
        try:
            value = float(position)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        by_name[name] = value

    if any(name not in by_name for name in required_joint_names):
        return None
    return [by_name[name] for name in required_joint_names]


def validate_joint_position_limits(
    positions: list[float],
    lower_limits: list[float],
    upper_limits: list[float],
    tolerance: float = 1e-6,
) -> list[float] | None:
    """Reject out-of-range positions and clamp values within numeric tolerance."""
    if not (len(positions) == len(lower_limits) == len(upper_limits)):
        return None

    validated = []
    margin = abs(float(tolerance))
    for position, lower, upper in zip(positions, lower_limits, upper_limits):
        value = float(position)
        lo = float(lower)
        hi = float(upper)
        if not all(math.isfinite(item) for item in (value, lo, hi)) or lo > hi:
            return None
        if value < lo - margin or value > hi + margin:
            return None
        validated.append(max(lo, min(hi, value)))
    return validated


def joint_state_to_configuration(
    joint_names: list[str],
    positions: list[float],
    model_joint_names: list[str],
    neutral_q: list[float],
) -> list[float]:
    """Map ROS joint states into a Pinocchio configuration vector."""
    by_name = {name: float(position) for name, position in zip(joint_names, positions)}
    q = [float(value) for value in neutral_q]
    for model_index, name in enumerate(model_joint_names):
        q_index = model_index - 1
        if q_index < 0 or q_index >= len(q):
            continue
        if name in by_name:
            q[q_index] = by_name[name]
    return q
