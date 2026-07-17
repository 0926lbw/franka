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
"""Move the arm once to the requested joint configuration."""

import rclpy
from franka_msgs.action import PTPMotion
from franka_msgs.msg import TargetStatus
from rclpy.action import ActionClient
from rclpy.node import Node


TARGET = [0.0, -0.524, 0.0, -2.094, 0.0, 1.68, 0.0]
MAXIMUM_JOINT_VELOCITIES = [0.30] * 7
GOAL_TOLERANCE = 0.01


class MoveToJointGoal(Node):
    def __init__(self):
        super().__init__('move_to_joint_goal')
        self.ptp_client = ActionClient(
            self, PTPMotion, '/action_server/ptp_motion'
        )

    def move(self):
        if not self.ptp_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('PTP action server is unavailable')
        goal = PTPMotion.Goal()
        goal.goal_joint_configuration = TARGET
        goal.maximum_joint_velocities = MAXIMUM_JOINT_VELOCITIES
        goal.goal_tolerance = GOAL_TOLERANCE
        future = self.ptp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('PTP goal was rejected')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.target_status.status != TargetStatus.TARGET_REACHED:
            raise RuntimeError(result.error_message or 'PTP motion failed')


def main():
    rclpy.init()
    node = MoveToJointGoal()
    try:
        node.move()
        node.get_logger().info('Reached target joint configuration')
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
