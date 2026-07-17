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
"""ROS 2 node that plots tracker teleoperation debug curves."""

from __future__ import annotations

import threading

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from franka_tracker_bridge.debug_curve_core import DebugCurveBuffer, DebugCurveSnapshot
from franka_tracker_bridge.ros_conversions import pose_stamped_msg_to_timed_pose, stamp_to_sec


_AXIS_LABELS = ("x", "y", "z")
_RPY_LABELS = ("roll", "pitch", "yaw")


def _stamp_or_now(node: Node, stamp) -> float:
    value = stamp_to_sec(stamp)
    if value > 0.0:
        return value
    return float(node.get_clock().now().nanoseconds) * 1e-9


class TrackerDebugCurveNode(Node):
    def __init__(self):
        super().__init__("tracker_debug_curve_node")

        self.declare_parameter("tracker_pose_topic", "/tracker/pose")
        self.declare_parameter("deadman_topic", "/tracker/deadman")
        self.declare_parameter("target_pose_topic", "/franka_controller/target_cartesian_pose")
        self.declare_parameter(
            "kdl_desired_joint_states_topic",
            "/franka_controller/kdl_desired_joint_states",
        )
        self.declare_parameter("max_samples", 600)
        self.declare_parameter("plot_update_rate_hz", 10.0)
        self.declare_parameter("window_title", "Franka tracker debug curves")

        self._buffer = DebugCurveBuffer(max_samples=int(self.get_parameter("max_samples").value))
        self._lock = threading.Lock()
        self._init_plot()

        self.create_subscription(
            PoseStamped,
            self.get_parameter("tracker_pose_topic").value,
            self._on_tracker_pose,
            50,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("deadman_topic").value,
            self._on_deadman,
            50,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("target_pose_topic").value,
            self._on_target_pose,
            50,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("kdl_desired_joint_states_topic").value,
            self._on_kdl_desired_joint_state,
            50,
        )

        update_rate = max(0.5, float(self.get_parameter("plot_update_rate_hz").value))
        self.create_timer(1.0 / update_rate, self._on_timer)
        self.get_logger().info(
            "Started tracker_debug_curve_node with tracker pose, deadman, "
            "target pose, and KDL desired joint-state curves"
        )

    def _init_plot(self) -> None:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            self.get_logger().error(
                "tracker_debug_curve_node requires matplotlib in the active environment: "
                f"{exc}"
            )
            raise

        self._plt = plt
        plt.ion()
        self._figure, self._axes = plt.subplots(4, 2, figsize=(14, 10))
        self._figure.canvas.manager.set_window_title(
            str(self.get_parameter("window_title").value)
        )
        self._figure.tight_layout(pad=2.0)
        self._figure.show()

    def _on_tracker_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._buffer.add_tracker_pose(pose_stamped_msg_to_timed_pose(msg))

    def _on_deadman(self, msg: Bool) -> None:
        with self._lock:
            self._buffer.add_deadman(
                stamp_sec=float(self.get_clock().now().nanoseconds) * 1e-9,
                enabled=bool(msg.data),
            )

    def _on_target_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._buffer.add_target_pose(pose_stamped_msg_to_timed_pose(msg))

    def _on_kdl_desired_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._buffer.add_kdl_desired_joint_state(
                stamp_sec=_stamp_or_now(self, msg.header.stamp),
                names=msg.name,
                positions=msg.position,
            )

    def _on_timer(self) -> None:
        with self._lock:
            snapshot = self._buffer.snapshot()
        self._draw_snapshot(snapshot)

    def _draw_snapshot(self, snapshot: DebugCurveSnapshot) -> None:
        axes = self._axes
        self._plot_three_axis(
            axes[0][0], snapshot.tracker_time, snapshot.tracker_xyz, _AXIS_LABELS,
            "tracker xyz", "m"
        )
        self._plot_three_axis(
            axes[0][1], snapshot.tracker_time, snapshot.tracker_rpy, _RPY_LABELS,
            "tracker rpy", "rad"
        )
        self._plot_three_axis(
            axes[1][0], snapshot.target_time, snapshot.target_xyz, _AXIS_LABELS,
            "target xyz", "m"
        )
        self._plot_three_axis(
            axes[1][1], snapshot.target_time, snapshot.target_rpy, _RPY_LABELS,
            "target rpy", "rad"
        )
        self._plot_three_axis(
            axes[2][0], snapshot.target_time, snapshot.target_delta_xyz,
            _AXIS_LABELS, "target delta xyz", "m"
        )
        self._plot_three_axis(
            axes[2][1], snapshot.target_time, snapshot.target_delta_rpy,
            _RPY_LABELS, "target delta rpy", "rad"
        )
        self._plot_kdl_joints(axes[3][0], snapshot)
        self._plot_deadman(axes[3][1], snapshot)
        self._figure.canvas.draw_idle()
        self._figure.canvas.flush_events()

    def _plot_three_axis(
        self, axis, time, series, labels, title: str, ylabel: str
    ) -> None:
        axis.clear()
        for values, label in zip(series, labels):
            if time and values:
                axis.plot(time, values, label=label)
        axis.set_title(title)
        axis.set_xlabel("time [s]")
        axis.set_ylabel(ylabel)
        axis.grid(True)
        axis.legend(loc="upper right")

    def _plot_kdl_joints(self, axis, snapshot: DebugCurveSnapshot) -> None:
        axis.clear()
        names = snapshot.kdl_joint_names or [f"q{i + 1}" for i in range(7)]
        for values, name in zip(snapshot.kdl_positions, names):
            if snapshot.kdl_time and values:
                axis.plot(snapshot.kdl_time, values, label=name)
        axis.set_title("KDL q_des")
        axis.set_xlabel("time [s]")
        axis.set_ylabel("rad")
        axis.grid(True)
        axis.legend(loc="upper right", ncol=2)

    def _plot_deadman(self, axis, snapshot: DebugCurveSnapshot) -> None:
        axis.clear()
        if snapshot.deadman_time and snapshot.deadman:
            axis.step(snapshot.deadman_time, snapshot.deadman, where="post", label="deadman")
        axis.set_title("deadman")
        axis.set_xlabel("time [s]")
        axis.set_ylabel("enabled")
        axis.set_ylim(-0.1, 1.1)
        axis.grid(True)
        axis.legend(loc="upper right")


def main(args=None):
    rclpy.init(args=args)
    node = TrackerDebugCurveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
