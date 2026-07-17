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

#include <franka_arm_controllers/default_robot_behavior_utils.hpp>
#include <franka_arm_controllers/joint_impedance_pose_controller.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>

using namespace std::chrono_literals;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace franka_arm_controllers {

controller_interface::InterfaceConfiguration
JointImpedancePoseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedancePoseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!use_fake_hardware_) {
    config.names = franka_cartesian_pose_->get_state_interface_names();
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) +
                           "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) +
                           "/velocity");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  if (!use_fake_hardware_) {
    for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(franka_robot_model_name);
    }
  }

  return config;
}

void JointImpedancePoseController::update_joint_states_() {
  const auto joint_state_start_index = use_fake_hardware_ ? 0 : 16;
  for (auto i = 0; i < num_joints_; ++i) {
    const auto& position_interface = state_interfaces_.at(joint_state_start_index + i);
    const auto& velocity_interface = state_interfaces_.at(joint_state_start_index + 7 + i);
    const auto& effort_interface = state_interfaces_.at(joint_state_start_index + 14 + i);
    joint_positions_current_[i] = position_interface.get_value();
    q_init_(i) = joint_positions_current_[i];
    if (use_fake_hardware_ && fake_preview_follow_desired_state_ &&
        !fake_preview_q_initialized_) {
      fake_preview_q_(i) = joint_positions_current_[i];
    }
    joint_velocities_current_[i] = velocity_interface.get_value();
    joint_efforts_current_[i] = effort_interface.get_value();
  }
  if (use_fake_hardware_ && fake_preview_follow_desired_state_) {
    fake_preview_q_initialized_ = true;
    for (auto i = 0; i < num_joints_; ++i) {
      q_init_(i) = fake_preview_q_(i);
    }
  }
}

Vector7d JointImpedancePoseController::compute_torque_command_(
    const Vector7d& joint_positions_desired,
    const Vector7d& joint_positions_current,
    const Vector7d& joint_velocities_current) {
  Vector7d coriolis = Vector7d::Zero();
  if (!use_fake_hardware_) {
    std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
    coriolis = Vector7d(coriolis_array.data());
  }
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
  Vector7d q_error = joint_positions_desired - joint_positions_current;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_) + coriolis;

  return tau_d_calculated;
}

controller_interface::return_type JointImpedancePoseController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  update_joint_states_();
  bool should_publish_desired_joint_state = true;
  if (use_moveit_service_ik_()) {
    consume_ready_moveit_ik_response_();
  }

  if (!target_is_fresh_(time)) {
    if (use_fake_hardware_ && fake_preview_follow_desired_state_ && fake_preview_q_initialized_) {
      joint_positions_desired_ = fake_preview_q_to_vector_();
    } else {
      joint_positions_desired_ = joint_positions_current_;
    }
  } else {
    update_current_pose_();

    if (target_pose_is_relative_) {
      target_position_ = startup_position_ + relative_target_position_;
      target_orientation_ = relative_target_orientation_ * startup_orientation_;
      target_orientation_.normalize();
    }

    auto new_position = limited_position_target_(position_);
    auto new_orientation = limited_orientation_target_(orientation_);

    if (use_moveit_service_ik_()) {
      if (!request_moveit_ik_(new_position, new_orientation)) {
        should_publish_desired_joint_state = false;
        if (joint_positions_desired_.empty()) {
          joint_positions_desired_ = joint_positions_current_;
        }
      }
    } else if (!solve_ik_(new_position, new_orientation)) {
      joint_positions_desired_ = joint_positions_current_;
      should_publish_desired_joint_state = false;
    }
  }

  if (joint_positions_desired_.empty()) {
    return controller_interface::return_type::OK;
  }
  if (should_publish_desired_joint_state) {
    publish_desired_joint_state_(time);
  }

  Vector7d joint_positions_desired_eigen(joint_positions_desired_.data());
  Vector7d joint_positions_current_eigen(joint_positions_current_.data());
  Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());

  auto tau_d_calculated = compute_torque_command_(
      joint_positions_desired_eigen, joint_positions_current_eigen, joint_velocities_current_eigen);

  for (int i = 0; i < num_joints_; i++) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedancePoseController::on_init() {
  auto_declare<std::string>("target_pose_topic", "/franka_controller/target_cartesian_pose");
  auto_declare<bool>("target_pose_is_relative", false);
  auto_declare<std::string>("kdl_desired_joint_states_topic", "/franka_controller/kdl_desired_joint_states");
  auto_declare<std::string>("ik_backend", "kdl");
  auto_declare<std::string>("moveit_compute_ik_service", "/compute_ik");
  auto_declare<std::string>("moveit_group_name", "");
  auto_declare<std::string>("moveit_base_frame", "base");
  auto_declare<std::string>("moveit_ik_link_name", "");
  auto_declare<std::string>("base_link_name", "base");
  auto_declare<std::string>("tcp_link_name", "");
  auto_declare<double>("max_target_linear_step", 0.007);
  auto_declare<double>("max_target_angular_step", 0.03);
  auto_declare<double>("target_timeout_sec", 0.25);
  auto_declare<double>("kdl_desired_joint_states_publish_rate_hz", 60.0);
  auto_declare<int>("ik_max_iterations", 100);
  auto_declare<double>("ik_eps", 1e-6);
  auto_declare<std::string>("use_fake_hardware", "false");
  auto_declare<bool>("fake_preview_follow_desired_state", true);

  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

bool JointImpedancePoseController::assign_parameters_() {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  if (arm_id_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "arm_id parameter must not be empty");
    return false;
  }
  is_gripper_loaded_ = get_node()->get_parameter("load_gripper").as_string() == "true";
  auto use_fake_hardware_parameter = get_node()->get_parameter("use_fake_hardware");
  if (use_fake_hardware_parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    use_fake_hardware_ = use_fake_hardware_parameter.as_bool();
  } else {
    use_fake_hardware_ = use_fake_hardware_parameter.as_string() == "true";
  }
  fake_preview_follow_desired_state_ =
      get_node()->get_parameter("fake_preview_follow_desired_state").as_bool();
  arm_mounting_orientation_ =
      get_node()->get_parameter("arm_mounting_orientation").as_double_array();

  target_pose_topic_ = get_node()->get_parameter("target_pose_topic").as_string();
  target_pose_is_relative_ =
      get_node()->get_parameter("target_pose_is_relative").as_bool();
  kdl_desired_joint_states_topic_ =
      get_node()->get_parameter("kdl_desired_joint_states_topic").as_string();
  ik_backend_ = get_node()->get_parameter("ik_backend").as_string();
  moveit_compute_ik_service_ =
      get_node()->get_parameter("moveit_compute_ik_service").as_string();
  moveit_group_name_ = get_node()->get_parameter("moveit_group_name").as_string();
  moveit_base_frame_ = get_node()->get_parameter("moveit_base_frame").as_string();
  moveit_ik_link_name_ = get_node()->get_parameter("moveit_ik_link_name").as_string();
  base_link_name_ = get_node()->get_parameter("base_link_name").as_string();
  tcp_link_name_ = get_node()->get_parameter("tcp_link_name").as_string();
  if (moveit_group_name_.empty()) {
    moveit_group_name_ = arm_id_ + "_arm";
  }
  if (moveit_ik_link_name_.empty()) {
    moveit_ik_link_name_ = arm_id_ + "_link8";
  }
  if (tcp_link_name_.empty()) {
    tcp_link_name_ = arm_id_ + "_link8";
  }
  max_target_linear_step_ = get_node()->get_parameter("max_target_linear_step").as_double();
  max_target_angular_step_ = get_node()->get_parameter("max_target_angular_step").as_double();
  target_timeout_sec_ = get_node()->get_parameter("target_timeout_sec").as_double();
  kdl_desired_joint_states_publish_rate_hz_ =
      get_node()->get_parameter("kdl_desired_joint_states_publish_rate_hz").as_double();
  ik_max_iterations_ = get_node()->get_parameter("ik_max_iterations").as_int();
  ik_eps_ = get_node()->get_parameter("ik_eps").as_double();

  if (target_pose_topic_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "target_pose_topic parameter must not be empty");
    return false;
  }
  if (kdl_desired_joint_states_topic_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "kdl_desired_joint_states_topic parameter must not be empty");
    return false;
  }
  if (ik_backend_ != "kdl" && ik_backend_ != "moveit_service") {
    RCLCPP_FATAL(get_node()->get_logger(), "ik_backend must be 'kdl' or 'moveit_service'");
    return false;
  }
  if (use_moveit_service_ik_() && moveit_compute_ik_service_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "moveit_compute_ik_service parameter must not be empty");
    return false;
  }
  if (use_moveit_service_ik_() && moveit_group_name_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "moveit_group_name parameter must not be empty");
    return false;
  }
  if (use_moveit_service_ik_() && moveit_base_frame_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "moveit_base_frame parameter must not be empty");
    return false;
  }
  if (use_moveit_service_ik_() && moveit_ik_link_name_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "moveit_ik_link_name parameter must not be empty");
    return false;
  }
  if (base_link_name_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "base_link_name parameter must not be empty");
    return false;
  }
  if (tcp_link_name_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "tcp_link_name parameter must not be empty");
    return false;
  }
  if (max_target_linear_step_ <= 0.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "max_target_linear_step must be positive");
    return false;
  }
  if (max_target_angular_step_ <= 0.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "max_target_angular_step must be positive");
    return false;
  }
  if (target_timeout_sec_ < 0.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "target_timeout_sec must be >= 0");
    return false;
  }
  if (kdl_desired_joint_states_publish_rate_hz_ <= 0.0) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "kdl_desired_joint_states_publish_rate_hz must be positive");
    return false;
  }
  if (ik_max_iterations_ <= 0) {
    RCLCPP_FATAL(get_node()->get_logger(), "ik_max_iterations must be positive");
    return false;
  }
  if (ik_eps_ <= 0.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "ik_eps must be positive");
    return false;
  }
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return false;
  }
  if (k_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints_, k_gains.size());
    return false;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return false;
  }
  if (d_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints_, d_gains.size());
    return false;
  }
  for (int i = 0; i < num_joints_; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  return true;
}

CallbackReturn JointImpedancePoseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!assign_parameters_()) {
    return CallbackReturn::FAILURE;
  }

  namespace_prefix_ = get_node()->get_namespace();

  if (namespace_prefix_ == "/" || namespace_prefix_.empty()) {
    namespace_prefix_.clear();
  } else {
    namespace_prefix_ = namespace_prefix_.substr(1) + "_";
  }

  if (!namespace_prefix_.empty() && tcp_link_name_.rfind(namespace_prefix_, 0) != 0) {
    tcp_link_name_ = namespace_prefix_ + tcp_link_name_;
  }

  if (!use_fake_hardware_) {
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                     arm_id_ + "/" + k_robot_state_interface_name));

    auto collision_client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");

    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
    auto future_result = collision_client->async_send_request(request);

    auto success = future_result.get();

    if (!success->success) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
    }
  } else {
    RCLCPP_INFO(get_node()->get_logger(),
                "Using fake hardware: skipping Franka collision service and robot model.");
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  target_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->target_pose_callback(msg);
      });

  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to %s.", target_pose_topic_.c_str());
  if (use_moveit_service_ik_()) {
    compute_ik_client_ =
        get_node()->create_client<moveit_msgs::srv::GetPositionIK>(moveit_compute_ik_service_);
    RCLCPP_INFO(get_node()->get_logger(), "Using MoveIt compute_ik service %s.",
                moveit_compute_ik_service_.c_str());
  }
  desired_joint_state_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      kdl_desired_joint_states_topic_, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(get_node()->get_logger(), "Publishing KDL desired joint states to %s.",
              kdl_desired_joint_states_topic_.c_str());

  if (!model_.initString(robot_description_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to parse processed URDF");
    return CallbackReturn::FAILURE;
  }

  if (!kdl_parser::treeFromUrdfModel(model_, tree_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to convert URDF to KDL tree.");
    return CallbackReturn::FAILURE;
  }

  if (!tree_.getChain(base_link_name_, tcp_link_name_, chain_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to extract KDL chain from %s to %s.",
                 base_link_name_.c_str(), tcp_link_name_.c_str());
    return CallbackReturn::FAILURE;
  }

  nj_ = chain_.getNrOfJoints();
  q_min_ = KDL::JntArray(nj_);
  q_max_ = KDL::JntArray(nj_);
  q_init_ = KDL::JntArray(nj_);
  q_result_ = KDL::JntArray(nj_);
  fake_preview_q_ = KDL::JntArray(nj_);
  startup_q_ = KDL::JntArray(nj_);
  moveit_ik_requested_seed_ = KDL::JntArray(nj_);
  last_successful_q_.reset();
  fake_preview_q_initialized_ = false;
  kdl_joint_names_.clear();
  kdl_joint_names_.reserve(nj_);

  unsigned int j = 0;
  for (const auto& segment : chain_.segments) {
    const KDL::Joint& kdl_joint = segment.getJoint();
    if (kdl_joint.getType() == KDL::Joint::None) {
      continue;
    }

    const std::string& joint_name = kdl_joint.getName();
    auto joint = model_.getJoint(joint_name);
    if (!joint || !joint->limits) {
      RCLCPP_FATAL(get_node()->get_logger(), "No limits found for joint: %s", joint_name.c_str());
      return CallbackReturn::FAILURE;
    }

    q_min_(j) = joint->limits->lower;
    q_max_(j) = joint->limits->upper;
    q_init_(j) = (q_max_(j) + q_min_(j)) / 2;
    startup_q_(j) = q_init_(j);
    kdl_joint_names_.push_back(joint_name);
    ++j;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedancePoseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  dq_filtered_.setZero();
  joint_positions_desired_.reserve(num_joints_);
  joint_positions_current_.reserve(num_joints_);
  joint_velocities_current_.reserve(num_joints_);
  joint_efforts_current_.reserve(num_joints_);
  target_pose_received_ = false;
  has_published_desired_joint_state_ = false;
  moveit_ik_request_in_flight_ = false;
  last_successful_q_.reset();
  fake_preview_q_initialized_ = false;
  startup_pose_initialized_ = false;
  relative_target_position_.setZero();
  relative_target_orientation_.setIdentity();
  desired_joint_state_msg_.name = kdl_joint_names_;
  desired_joint_state_msg_.position.clear();
  desired_joint_state_msg_.velocity.clear();
  desired_joint_state_msg_.effort.clear();

  if (!use_fake_hardware_) {
    franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  }

  update_joint_states_();
  startup_q_ = q_init_;
  update_current_pose_();
  startup_position_ = position_;
  startup_orientation_ = orientation_;
  startup_pose_initialized_ = true;

  RCLCPP_INFO(get_node()->get_logger(), "Target pose input mode: %s.",
              target_pose_is_relative_ ? "relative to controller start pose" : "absolute in base");

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedancePoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!use_fake_hardware_) {
    franka_cartesian_pose_->release_interfaces();
  }
  startup_pose_initialized_ = false;
  return CallbackReturn::SUCCESS;
}

void JointImpedancePoseController::target_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const auto& pose = msg->pose;
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w)) {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring non-finite target Cartesian pose.");
    return;
  }

  Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z);
  const double norm = orientation.norm();
  if (norm < 1e-9) {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring target Cartesian pose with zero quaternion.");
    return;
  }
  orientation.normalize();

  const Eigen::Vector3d message_position(pose.position.x, pose.position.y, pose.position.z);
  if (target_pose_is_relative_) {
    relative_target_position_ = message_position;
    relative_target_orientation_ = orientation;
  } else {
    target_position_ = message_position;
    target_orientation_ = orientation;
  }
  target_pose_received_ = true;
  last_target_time_ = get_node()->now();
}

bool JointImpedancePoseController::target_is_fresh_(const rclcpp::Time& time) const {
  if (!target_pose_received_) {
    return false;
  }
  if (target_timeout_sec_ == 0.0) {
    return true;
  }
  return (time - last_target_time_).seconds() <= target_timeout_sec_;
}

std::vector<double> JointImpedancePoseController::fake_preview_q_to_vector_() const {
  std::vector<double> joint_vector;
  joint_vector.reserve(static_cast<size_t>(num_joints_));
  for (auto i = 0; i < num_joints_; ++i) {
    joint_vector.push_back(fake_preview_q_(i));
  }
  return joint_vector;
}

std::string JointImpedancePoseController::format_joint_array_(const KDL::JntArray& joints) const {
  std::ostringstream stream;
  stream << "[";
  for (unsigned int i = 0; i < joints.rows(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << joints(i);
  }
  stream << "]";
  return stream.str();
}

KDL::JntArray JointImpedancePoseController::make_biased_seed_(const KDL::JntArray& seed,
                                                              unsigned int joint_index,
                                                              double offset) const {
  KDL::JntArray biased(seed);
  if (joint_index >= biased.rows()) {
    return biased;
  }
  biased(joint_index) = std::clamp(biased(joint_index) + offset, q_min_(joint_index),
                                   q_max_(joint_index));
  return biased;
}

std::vector<KDL::JntArray> JointImpedancePoseController::build_ik_seed_candidates_() const {
  std::vector<KDL::JntArray> seeds;
  seeds.reserve(7);
  seeds.push_back(q_init_);
  if (last_successful_q_.has_value()) {
    seeds.push_back(*last_successful_q_);
  }
  seeds.push_back(startup_q_);
  seeds.push_back(make_biased_seed_(q_init_, 5, 0.35));
  seeds.push_back(make_biased_seed_(q_init_, 5, -0.35));
  seeds.push_back(make_biased_seed_(q_init_, 6, 0.50));
  seeds.push_back(make_biased_seed_(q_init_, 6, -0.50));
  return seeds;
}

void JointImpedancePoseController::log_ik_nonconvergence_(
    int last_status,
    const Eigen::Vector3d& new_position,
    const Eigen::Quaterniond& new_orientation,
    const KDL::JntArray& first_seed,
    size_t attempts) const {
  const Eigen::Vector3d startup_delta = new_position - startup_position_;
  RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "KDL IK did not converge after %zu seed attempts; last status %d; holding current joint "
      "target. target position xyz=[%.6f, %.6f, %.6f], target orientation xyzw=[%.6f, %.6f, "
      "%.6f, %.6f], startup delta xyz=[%.6f, %.6f, %.6f], current seed q=%s",
      attempts, last_status, new_position.x(), new_position.y(), new_position.z(),
      new_orientation.x(), new_orientation.y(), new_orientation.z(), new_orientation.w(),
      startup_delta.x(), startup_delta.y(), startup_delta.z(),
      format_joint_array_(first_seed).c_str());
}

bool JointImpedancePoseController::use_moveit_service_ik_() const {
  return ik_backend_ == "moveit_service";
}

std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request>
JointImpedancePoseController::create_moveit_ik_request_(
    const Eigen::Vector3d& new_position,
    const Eigen::Quaterniond& new_orientation) const {
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

  request->ik_request.group_name = moveit_group_name_;
  request->ik_request.ik_link_name = moveit_ik_link_name_;
  request->ik_request.pose_stamped.header.frame_id = moveit_base_frame_;
  request->ik_request.pose_stamped.pose.position.x = new_position.x();
  request->ik_request.pose_stamped.pose.position.y = new_position.y();
  request->ik_request.pose_stamped.pose.position.z = new_position.z();
  request->ik_request.pose_stamped.pose.orientation.x = new_orientation.x();
  request->ik_request.pose_stamped.pose.orientation.y = new_orientation.y();
  request->ik_request.pose_stamped.pose.orientation.z = new_orientation.z();
  request->ik_request.pose_stamped.pose.orientation.w = new_orientation.w();

  request->ik_request.robot_state.joint_state.name = kdl_joint_names_;
  request->ik_request.robot_state.joint_state.position.reserve(static_cast<size_t>(num_joints_));
  for (auto i = 0; i < num_joints_; ++i) {
    request->ik_request.robot_state.joint_state.position.push_back(q_init_(i));
  }
  request->ik_request.robot_state.joint_state.velocity = joint_velocities_current_;
  request->ik_request.robot_state.joint_state.effort = joint_efforts_current_;
  return request;
}

bool JointImpedancePoseController::request_moveit_ik_(
    const Eigen::Vector3d& new_position,
    const Eigen::Quaterniond& new_orientation) {
  if (!compute_ik_client_) {
    return false;
  }
  if (moveit_ik_request_in_flight_) {
    return true;
  }
  if (!compute_ik_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "MoveIt compute_ik service %s is not ready; holding current joint target.",
                         moveit_compute_ik_service_.c_str());
    return false;
  }

  auto request = create_moveit_ik_request_(new_position, new_orientation);
  moveit_ik_requested_position_ = new_position;
  moveit_ik_requested_orientation_ = new_orientation;
  moveit_ik_requested_seed_ = q_init_;
  auto future_and_request_id = compute_ik_client_->async_send_request(request);
  moveit_ik_future_ = future_and_request_id.future.share();
  moveit_ik_request_in_flight_ = true;
  return true;
}

void JointImpedancePoseController::consume_ready_moveit_ik_response_() {
  if (!moveit_ik_request_in_flight_ || !moveit_ik_future_.valid()) {
    return;
  }
  if (moveit_ik_future_.wait_for(0s) != std::future_status::ready) {
    return;
  }

  const auto response = moveit_ik_future_.get();
  moveit_ik_request_in_flight_ = false;
  if (response->error_code.val == response->error_code.SUCCESS) {
    std::vector<double> joint_positions;
    if (!extract_moveit_joint_positions_(response->solution.joint_state, joint_positions)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "MoveIt compute_ik response did not contain all controller joints.");
      return;
    }

    accept_ik_solution_(joint_positions);
  } else {
    const Eigen::Vector3d startup_delta = moveit_ik_requested_position_ - startup_position_;
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "MoveIt compute_ik did not return a solution; error code %d; request frame=%s, "
                         "link=%s, target position xyz=[%.6f, %.6f, %.6f], target "
                         "orientation xyzw=[%.6f, %.6f, %.6f, %.6f], startup delta xyz=[%.6f, "
                         "%.6f, %.6f], seed q=%s.",
                         response->error_code.val, moveit_base_frame_.c_str(),
                         moveit_ik_link_name_.c_str(), moveit_ik_requested_position_.x(),
                         moveit_ik_requested_position_.y(), moveit_ik_requested_position_.z(),
                         moveit_ik_requested_orientation_.x(), moveit_ik_requested_orientation_.y(),
                         moveit_ik_requested_orientation_.z(), moveit_ik_requested_orientation_.w(),
                         startup_delta.x(), startup_delta.y(), startup_delta.z(),
                         format_joint_array_(moveit_ik_requested_seed_).c_str());
  }
}

bool JointImpedancePoseController::extract_moveit_joint_positions_(
    const sensor_msgs::msg::JointState& joint_state,
    std::vector<double>& joint_positions) const {
  if (joint_state.name.size() != joint_state.position.size()) {
    return false;
  }

  joint_positions.clear();
  joint_positions.reserve(static_cast<size_t>(num_joints_));
  for (const auto& joint_name : kdl_joint_names_) {
    const auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
    if (it == joint_state.name.end()) {
      return false;
    }
    const auto index = static_cast<size_t>(std::distance(joint_state.name.begin(), it));
    joint_positions.push_back(joint_state.position.at(index));
  }
  return joint_positions.size() == static_cast<size_t>(num_joints_);
}

void JointImpedancePoseController::accept_ik_solution_(
    const std::vector<double>& joint_positions) {
  if (joint_positions.size() != static_cast<size_t>(num_joints_)) {
    return;
  }

  joint_positions_desired_ = joint_positions;
  q_result_ = KDL::JntArray(nj_);
  for (auto i = 0; i < num_joints_; ++i) {
    q_result_(i) = joint_positions[i];
  }
  last_successful_q_ = q_result_;
  if (use_fake_hardware_ && fake_preview_follow_desired_state_) {
    for (auto i = 0; i < num_joints_; ++i) {
      fake_preview_q_(i) = q_result_(i);
    }
    fake_preview_q_initialized_ = true;
  }
}

void JointImpedancePoseController::publish_desired_joint_state_(const rclcpp::Time& time) {
  if (!desired_joint_state_pub_) {
    return;
  }
  if (joint_positions_desired_.size() != kdl_joint_names_.size()) {
    return;
  }

  if (has_published_desired_joint_state_) {
    const double publish_period = 1.0 / kdl_desired_joint_states_publish_rate_hz_;
    if ((time - last_desired_joint_state_publish_time_).seconds() < publish_period) {
      return;
    }
  }

  desired_joint_state_msg_.header.stamp = time;
  desired_joint_state_msg_.name = kdl_joint_names_;
  desired_joint_state_msg_.position = joint_positions_desired_;
  desired_joint_state_pub_->publish(desired_joint_state_msg_);
  last_desired_joint_state_publish_time_ = time;
  has_published_desired_joint_state_ = true;
}

void JointImpedancePoseController::update_current_pose_() {
  if (!use_fake_hardware_) {
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    return;
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain_);
  KDL::Frame current_pose;
  const int status = fk_solver.JntToCart(q_init_, current_pose);
  if (status < 0) {
    RCLCPP_FATAL(get_node()->get_logger(), "FK Failed with error code: %d", status);
    throw std::runtime_error("FK Failed");
  }

  double x, y, z, w;
  current_pose.M.GetQuaternion(x, y, z, w);
  orientation_ = Eigen::Quaterniond(w, x, y, z);
  orientation_.normalize();
  position_ =
      Eigen::Vector3d(current_pose.p.x(), current_pose.p.y(), current_pose.p.z());
}

Eigen::Vector3d JointImpedancePoseController::limited_position_target_(
    const Eigen::Vector3d& current_position) const {
  Eigen::Vector3d delta = target_position_ - current_position;
  const double distance = delta.norm();
  if (distance <= max_target_linear_step_ || distance < 1e-12) {
    return target_position_;
  }
  return current_position + delta / distance * max_target_linear_step_;
}

Eigen::Quaterniond JointImpedancePoseController::limited_orientation_target_(
    const Eigen::Quaterniond& current_orientation) const {
  Eigen::Quaterniond delta = current_orientation.inverse() * target_orientation_;
  if (delta.w() < 0.0) {
    delta.coeffs() *= -1.0;
  }
  delta.normalize();

  Eigen::AngleAxisd angle_axis(delta);
  const double angle = angle_axis.angle();
  if (angle <= max_target_angular_step_ || angle < 1e-12) {
    return target_orientation_;
  }

  Eigen::Quaterniond result =
      current_orientation * Eigen::Quaterniond(Eigen::AngleAxisd(max_target_angular_step_,
                                                                 angle_axis.axis()));
  result.normalize();
  return result;
}

bool JointImpedancePoseController::solve_ik_(const Eigen::Vector3d& new_position,
                                             const Eigen::Quaterniond& new_orientation) {
  KDL::ChainFkSolverPos_recursive fk_solver(chain_);
  KDL::ChainIkSolverVel_pinv vel_solver(chain_);
  KDL::ChainIkSolverPos_NR_JL ik_solver(chain_, q_min_, q_max_, fk_solver, vel_solver,
                                        ik_max_iterations_, ik_eps_);

  KDL::Rotation kdl_rot = KDL::Rotation::Quaternion(new_orientation.x(), new_orientation.y(),
                                                    new_orientation.z(), new_orientation.w());
  KDL::Vector kdl_pos(new_position.x(), new_position.y(), new_position.z());
  KDL::Frame desired_pose(kdl_rot, kdl_pos);

  const auto seed_candidates = build_ik_seed_candidates_();
  int status = KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED;
  for (const auto& seed : seed_candidates) {
    status = ik_solver.CartToJnt(seed, desired_pose, q_result_);
    if (status >= 0) {
      break;
    }
  }

  if (status < 0) {
    log_ik_nonconvergence_(status, new_position, new_orientation, seed_candidates.front(),
                           seed_candidates.size());
    return false;
  }

  std::vector<double> joint_vector(q_result_.data.data(), q_result_.data.data() + q_result_.rows());

  joint_positions_desired_ = joint_vector;
  last_successful_q_ = q_result_;
  if (use_fake_hardware_ && fake_preview_follow_desired_state_) {
    for (auto i = 0; i < num_joints_; ++i) {
      fake_preview_q_(i) = q_result_(i);
    }
    fake_preview_q_initialized_ = true;
  }
  return true;
}

}  // namespace franka_arm_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_arm_controllers::JointImpedancePoseController,
                       controller_interface::ControllerInterface)
