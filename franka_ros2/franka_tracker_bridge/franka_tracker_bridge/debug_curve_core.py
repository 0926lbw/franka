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
"""Core buffering and pose conversion for tracker debug curves."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Sequence

from franka_tracker_bridge.pose_math import (
    PoseValue,
    QuaternionValue,
    TimedPose,
    invert_quaternion,
    multiply_quaternion,
    normalize_quaternion,
)

Series3 = tuple[list[float], list[float], list[float]]
Series7 = tuple[
    list[float],
    list[float],
    list[float],
    list[float],
    list[float],
    list[float],
    list[float],
]
DequeSeries3 = tuple[Deque[float], Deque[float], Deque[float]]
DequeSeries7 = tuple[
    Deque[float],
    Deque[float],
    Deque[float],
    Deque[float],
    Deque[float],
    Deque[float],
    Deque[float],
]


@dataclass(frozen=True)
class DebugCurveSnapshot:
    tracker_time: list[float]
    tracker_xyz: Series3
    tracker_rpy: Series3
    deadman_time: list[float]
    deadman: list[float]
    target_time: list[float]
    target_xyz: Series3
    target_rpy: Series3
    target_delta_xyz: Series3
    target_delta_rpy: Series3
    kdl_time: list[float]
    kdl_joint_names: list[str]
    kdl_positions: Series7


def _new_series3() -> DequeSeries3:
    return (deque(), deque(), deque())


def _new_series7() -> DequeSeries7:
    return (deque(), deque(), deque(), deque(), deque(), deque(), deque())


def _series3_to_lists(series: DequeSeries3) -> Series3:
    return (list(series[0]), list(series[1]), list(series[2]))


def _series7_to_lists(series: DequeSeries7) -> Series7:
    return (
        list(series[0]),
        list(series[1]),
        list(series[2]),
        list(series[3]),
        list(series[4]),
        list(series[5]),
        list(series[6]),
    )


def _append_limited(values: Deque[float], value: float, max_samples: int) -> None:
    values.append(float(value))
    while len(values) > max_samples:
        values.popleft()


def _append_series3(
    series: DequeSeries3,
    values: Sequence[float],
    max_samples: int,
) -> None:
    for axis, value in zip(series, values):
        _append_limited(axis, value, max_samples)


def _append_series7(
    series: DequeSeries7,
    values: Sequence[float],
    max_samples: int,
) -> None:
    for axis, value in zip(series, values[:7]):
        _append_limited(axis, value, max_samples)


def _clean_delta(value: float) -> float:
    return round(float(value), 12)


def quaternion_to_rpy(q: QuaternionValue) -> tuple[float, float, float]:
    q = normalize_quaternion(q)

    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


class DebugCurveBuffer:
    def __init__(self, max_samples: int = 600):
        self.max_samples = max(1, int(max_samples))
        self._tracker_time: Deque[float] = deque()
        self._tracker_xyz = _new_series3()
        self._tracker_rpy = _new_series3()
        self._deadman_time: Deque[float] = deque()
        self._deadman: Deque[float] = deque()
        self._target_time: Deque[float] = deque()
        self._target_xyz = _new_series3()
        self._target_rpy = _new_series3()
        self._target_delta_xyz = _new_series3()
        self._target_delta_rpy = _new_series3()
        self._kdl_time: Deque[float] = deque()
        self._kdl_joint_names: list[str] = []
        self._kdl_positions = _new_series7()
        self._target_start_pose: PoseValue | None = None

    def add_tracker_pose(self, timed_pose: TimedPose) -> None:
        self._add_pose_sample(
            timed_pose,
            self._tracker_time,
            self._tracker_xyz,
            self._tracker_rpy,
        )

    def add_deadman(self, stamp_sec: float, enabled: bool) -> None:
        _append_limited(self._deadman_time, stamp_sec, self.max_samples)
        _append_limited(self._deadman, 1.0 if enabled else 0.0, self.max_samples)

    def add_target_pose(self, timed_pose: TimedPose) -> None:
        if self._target_start_pose is None:
            self._target_start_pose = timed_pose.pose
        self._add_pose_sample(
            timed_pose,
            self._target_time,
            self._target_xyz,
            self._target_rpy,
        )
        start = self._target_start_pose
        delta_xyz = (
            _clean_delta(timed_pose.pose.position.x - start.position.x),
            _clean_delta(timed_pose.pose.position.y - start.position.y),
            _clean_delta(timed_pose.pose.position.z - start.position.z),
        )
        relative_orientation = multiply_quaternion(
            invert_quaternion(start.orientation),
            timed_pose.pose.orientation,
        )
        delta_rpy = tuple(_clean_delta(value) for value in quaternion_to_rpy(relative_orientation))
        _append_series3(self._target_delta_xyz, delta_xyz, self.max_samples)
        _append_series3(self._target_delta_rpy, delta_rpy, self.max_samples)

    def add_kdl_desired_joint_state(
        self,
        stamp_sec: float,
        names: Sequence[str],
        positions: Sequence[float],
    ) -> None:
        if len(positions) < 7:
            return
        if not self._kdl_joint_names:
            self._kdl_joint_names = [str(name) for name in names[:7]]
        _append_limited(self._kdl_time, stamp_sec, self.max_samples)
        _append_series7(self._kdl_positions, positions, self.max_samples)

    def snapshot(self) -> DebugCurveSnapshot:
        return DebugCurveSnapshot(
            tracker_time=list(self._tracker_time),
            tracker_xyz=_series3_to_lists(self._tracker_xyz),
            tracker_rpy=_series3_to_lists(self._tracker_rpy),
            deadman_time=list(self._deadman_time),
            deadman=list(self._deadman),
            target_time=list(self._target_time),
            target_xyz=_series3_to_lists(self._target_xyz),
            target_rpy=_series3_to_lists(self._target_rpy),
            target_delta_xyz=_series3_to_lists(self._target_delta_xyz),
            target_delta_rpy=_series3_to_lists(self._target_delta_rpy),
            kdl_time=list(self._kdl_time),
            kdl_joint_names=list(self._kdl_joint_names),
            kdl_positions=_series7_to_lists(self._kdl_positions),
        )

    def _add_pose_sample(
        self,
        timed_pose: TimedPose,
        time_series: Deque[float],
        xyz_series: DequeSeries3,
        rpy_series: DequeSeries3,
    ) -> None:
        pose = timed_pose.pose
        _append_limited(time_series, timed_pose.stamp_sec, self.max_samples)
        _append_series3(
            xyz_series,
            (pose.position.x, pose.position.y, pose.position.z),
            self.max_samples,
        )
        _append_series3(rpy_series, quaternion_to_rpy(pose.orientation), self.max_samples)
