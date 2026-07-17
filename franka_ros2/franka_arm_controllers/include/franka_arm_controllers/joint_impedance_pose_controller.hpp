// Copyright (c) 2025 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Dense>
#include <future>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <franka_arm_controllers/robot_utils.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_arm_controllers {

/**
 * Joint impedance controller that tracks an externally supplied Cartesian target pose.
 *
 * The controller accepts PoseStamped targets and converts them to joint targets with KDL IK.
 * The final command is a joint impedance torque command for ros2_control.
 */
class JointImpedancePoseController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void update_joint_states_();
  Vector7d compute_torque_command_(const Vector7d& joint_positions_desired,
                                   const Vector7d& joint_positions_current,
                                   const Vector7d& joint_velocities_current);
  bool assign_parameters_();
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool target_is_fresh_(const rclcpp::Time& time) const;
  std::vector<double> fake_preview_q_to_vector_() const;
  std::string format_joint_array_(const KDL::JntArray& joints) const;
  std::vector<KDL::JntArray> build_ik_seed_candidates_() const;
  KDL::JntArray make_biased_seed_(const KDL::JntArray& seed,
                                  unsigned int joint_index,
                                  double offset) const;
  void log_ik_nonconvergence_(int last_status,
                              const Eigen::Vector3d& new_position,
                              const Eigen::Quaterniond& new_orientation,
                              const KDL::JntArray& first_seed,
                              size_t attempts) const;
  bool use_moveit_service_ik_() const;
  std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_moveit_ik_request_(
      const Eigen::Vector3d& new_position,
      const Eigen::Quaterniond& new_orientation) const;
  bool request_moveit_ik_(const Eigen::Vector3d& new_position,
                          const Eigen::Quaterniond& new_orientation);
  void consume_ready_moveit_ik_response_();
  bool extract_moveit_joint_positions_(const sensor_msgs::msg::JointState& joint_state,
                                       std::vector<double>& joint_positions) const;
  void accept_ik_solution_(const std::vector<double>& joint_positions);
  void update_current_pose_();
  void publish_desired_joint_state_(const rclcpp::Time& time);
  Eigen::Vector3d limited_position_target_(const Eigen::Vector3d& current_position) const;
  Eigen::Quaterniond limited_orientation_target_(
      const Eigen::Quaterniond& current_orientation) const;
  bool solve_ik_(const Eigen::Vector3d& new_position, const Eigen::Quaterniond& new_orientation);

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond startup_orientation_;
  Eigen::Vector3d startup_position_;
  Eigen::Quaterniond relative_target_orientation_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d relative_target_position_{0.0, 0.0, 0.0};
  bool startup_pose_initialized_{false};

  const bool k_elbow_activated_{false};

  std::string arm_id_;
  std::string namespace_prefix_;
  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  unsigned int nj_;
  KDL::JntArray q_min_, q_max_, q_init_, q_result_;
  KDL::JntArray fake_preview_q_;
  KDL::JntArray startup_q_;
  std::optional<KDL::JntArray> last_successful_q_;
  std::vector<std::string> kdl_joint_names_;
  bool is_gripper_loaded_ = true;
  bool use_fake_hardware_{false};
  bool fake_preview_follow_desired_state_{true};
  bool fake_preview_q_initialized_{false};
  std::vector<double> arm_mounting_orientation_;

  std::string robot_description_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  const int num_joints_{7};

  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_efforts_current_{0, 0, 0, 0, 0, 0, 0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture moveit_ik_future_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_state_pub_;
  sensor_msgs::msg::JointState desired_joint_state_msg_;
  std::string target_pose_topic_;
  bool target_pose_is_relative_{false};
  std::string kdl_desired_joint_states_topic_;
  std::string ik_backend_;
  std::string moveit_compute_ik_service_;
  std::string moveit_group_name_;
  std::string moveit_base_frame_;
  std::string moveit_ik_link_name_;
  Eigen::Vector3d moveit_ik_requested_position_{0.0, 0.0, 0.0};
  Eigen::Quaterniond moveit_ik_requested_orientation_{1.0, 0.0, 0.0, 0.0};
  KDL::JntArray moveit_ik_requested_seed_;
  std::string base_link_name_;
  std::string tcp_link_name_;
  double max_target_linear_step_{0.007};
  double max_target_angular_step_{0.03};
  double target_timeout_sec_{0.25};
  double kdl_desired_joint_states_publish_rate_hz_{60.0};
  int ik_max_iterations_{100};
  double ik_eps_{1e-6};
  bool has_published_desired_joint_state_{false};
  rclcpp::Time last_desired_joint_state_publish_time_;
  bool moveit_ik_request_in_flight_{false};
  bool target_pose_received_{false};
  rclcpp::Time last_target_time_;
  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  Eigen::Quaterniond target_orientation_{1.0, 0.0, 0.0, 0.0};
};
}  // namespace franka_arm_controllers
