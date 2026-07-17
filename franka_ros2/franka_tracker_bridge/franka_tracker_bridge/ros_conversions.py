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
"""Conversions between ROS messages and pure Python pose values."""

from __future__ import annotations

from franka_tracker_bridge.pose_math import PoseValue, QuaternionValue, TimedPose, Vector3


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_msg_to_value(pose_msg) -> PoseValue:
    return PoseValue(
        position=Vector3(
            x=float(pose_msg.position.x),
            y=float(pose_msg.position.y),
            z=float(pose_msg.position.z),
        ),
        orientation=QuaternionValue(
            x=float(pose_msg.orientation.x),
            y=float(pose_msg.orientation.y),
            z=float(pose_msg.orientation.z),
            w=float(pose_msg.orientation.w),
        ),
    )


def pose_stamped_msg_to_timed_pose(msg) -> TimedPose:
    return TimedPose(stamp_sec=stamp_to_sec(msg.header.stamp), pose=pose_msg_to_value(msg.pose))


def transform_msg_to_timed_pose(msg) -> TimedPose:
    transform = msg.transform
    return TimedPose(
        stamp_sec=stamp_to_sec(msg.header.stamp),
        pose=PoseValue(
            position=Vector3(
                x=float(transform.translation.x),
                y=float(transform.translation.y),
                z=float(transform.translation.z),
            ),
            orientation=QuaternionValue(
                x=float(transform.rotation.x),
                y=float(transform.rotation.y),
                z=float(transform.rotation.z),
                w=float(transform.rotation.w),
            ),
        ),
    )


def pose_value_to_msg(pose: PoseValue, msg_type):
    msg = msg_type()
    msg.position.x = pose.position.x
    msg.position.y = pose.position.y
    msg.position.z = pose.position.z
    msg.orientation.x = pose.orientation.x
    msg.orientation.y = pose.orientation.y
    msg.orientation.z = pose.orientation.z
    msg.orientation.w = pose.orientation.w
    return msg
