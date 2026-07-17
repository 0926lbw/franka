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
"""Pose and quaternion helpers for tracker relative-pose teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class QuaternionValue:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class PoseValue:
    position: Vector3
    orientation: QuaternionValue


@dataclass(frozen=True)
class TimedPose:
    stamp_sec: float
    pose: PoseValue


Matrix3 = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


def parse_matrix3(values: Iterable[Iterable[float]] | Sequence[float]) -> Matrix3:
    flat: list[float] = []
    for item in values:
        if isinstance(item, (list, tuple)):
            flat.extend(float(v) for v in item)
        else:
            flat.append(float(item))
    if len(flat) != 9:
        raise ValueError('map_matrix must contain exactly 9 values')
    return (
        (flat[0], flat[1], flat[2]),
        (flat[3], flat[4], flat[5]),
        (flat[6], flat[7], flat[8]),
    )


def quaternion_from_axis_angle(axis: Sequence[float], angle: float) -> QuaternionValue:
    norm = math.sqrt(sum(float(v) * float(v) for v in axis))
    if norm == 0.0 or angle == 0.0:
        return QuaternionValue(x=0.0, y=0.0, z=0.0, w=1.0)
    half = angle * 0.5
    scale = math.sin(half) / norm
    return normalize_quaternion(
        QuaternionValue(
            x=float(axis[0]) * scale,
            y=float(axis[1]) * scale,
            z=float(axis[2]) * scale,
            w=math.cos(half),
        )
    )


def normalize_quaternion(q: QuaternionValue) -> QuaternionValue:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm == 0.0:
        return QuaternionValue(x=0.0, y=0.0, z=0.0, w=1.0)
    return QuaternionValue(x=q.x / norm, y=q.y / norm, z=q.z / norm, w=q.w / norm)


def invert_quaternion(q: QuaternionValue) -> QuaternionValue:
    q = normalize_quaternion(q)
    return QuaternionValue(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def multiply_quaternion(a: QuaternionValue, b: QuaternionValue) -> QuaternionValue:
    return normalize_quaternion(
        QuaternionValue(
            x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        )
    )


def quaternion_to_axis_angle(q: QuaternionValue) -> Vector3:
    q = normalize_quaternion(q)
    if q.w < 0.0:
        q = QuaternionValue(x=-q.x, y=-q.y, z=-q.z, w=-q.w)
    vector_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z)
    if vector_norm < 1e-12:
        return Vector3(0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, q.w)
    return Vector3(q.x / vector_norm * angle, q.y / vector_norm * angle, q.z / vector_norm * angle)


def matrix_vector_mul(matrix: Matrix3, vector: Sequence[float]) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )
