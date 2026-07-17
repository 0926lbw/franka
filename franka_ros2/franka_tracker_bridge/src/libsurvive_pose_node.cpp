// Copyright 2026 lbw
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
#include <libsurvive/survive_api.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "franka_tracker_bridge/libsurvive_pose_conversions.hpp"

#ifndef FRANKA_TRACKER_BRIDGE_DEFAULT_LIBSURVIVE_ROOT
#define FRANKA_TRACKER_BRIDGE_DEFAULT_LIBSURVIVE_ROOT ""
#endif

namespace franka_tracker_bridge
{
namespace
{

std::vector<std::string> split_args(const std::string & text)
{
  std::istringstream stream(text);
  std::vector<std::string> args;
  std::string item;
  while (stream >> item) {
    args.push_back(item);
  }
  return args;
}

std::string safe_string(const char * value)
{
  return value == nullptr ? std::string() : std::string(value);
}

}  // namespace

class LibsurvivePoseNode : public rclcpp::Node
{
public:
  LibsurvivePoseNode()
  : Node("libsurvive_pose_node")
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/tracker/pose");
    deadman_topic_ = declare_parameter<std::string>("deadman_topic", "/tracker/deadman");
    world_frame_ = declare_parameter<std::string>("world_frame", "libsurvive_world");
    target_serial_ = declare_parameter<std::string>("target_serial", "");
    lock_first_object_ = declare_parameter<bool>("lock_first_object", true);
    ignore_lighthouses_ = declare_parameter<bool>("ignore_lighthouses", true);
    publish_deadman_ = declare_parameter<bool>("publish_deadman", true);
    deadman_value_ = declare_parameter<bool>("deadman_value", true);
    libsurvive_args_ = declare_parameter<std::string>("libsurvive_args", "");
    working_directory_ = declare_parameter<std::string>(
      "working_directory",
      FRANKA_TRACKER_BRIDGE_DEFAULT_LIBSURVIVE_ROOT);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    deadman_pub_ = create_publisher<std_msgs::msg::Bool>(deadman_topic_, 10);

    init_libsurvive();
    running_.store(true);
    worker_ = std::thread([this]() {event_loop();});
  }

  ~LibsurvivePoseNode() override
  {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    if (ctx_ != nullptr) {
      survive_simple_close(ctx_);
      ctx_ = nullptr;
    }
  }

private:
  void init_libsurvive()
  {
    if (!working_directory_.empty()) {
      if (chdir(working_directory_.c_str()) != 0) {
        throw std::runtime_error(
                "failed to chdir to libsurvive working_directory: " +
                working_directory_);
      }
    }

    std::vector<std::string> arg_storage;
    arg_storage.emplace_back("libsurvive_pose_node");
    for (const auto & arg : split_args(libsurvive_args_)) {
      arg_storage.push_back(arg);
    }

    std::vector<char *> argv;
    argv.reserve(arg_storage.size());
    for (auto & arg : arg_storage) {
      argv.push_back(arg.data());
    }

    ctx_ = survive_simple_init(static_cast<int>(argv.size()), argv.data());
    if (ctx_ == nullptr) {
      throw std::runtime_error("survive_simple_init returned null");
    }

    survive_simple_start_thread(ctx_);
    RCLCPP_INFO(
      get_logger(),
      "Started libsurvive_pose_node: pose_topic=%s target_serial='%s' args='%s'",
      pose_topic_.c_str(),
      target_serial_.c_str(),
      libsurvive_args_.c_str());
  }

  void event_loop()
  {
    using namespace std::chrono_literals;
    while (running_.load() && rclcpp::ok()) {
      SurviveSimpleEvent event = {};
      const auto event_type = survive_simple_next_event(ctx_, &event);
      if (event_type == SurviveSimpleEventType_None) {
        std::this_thread::sleep_for(2ms);
        continue;
      }
      if (event_type == SurviveSimpleEventType_Shutdown) {
        RCLCPP_WARN(get_logger(), "libsurvive reported shutdown");
        break;
      }
      if (event_type == SurviveSimpleEventType_DeviceAdded) {
        const auto * object_event = survive_simple_get_object_event(&event);
        if (object_event != nullptr) {
          RCLCPP_INFO(
            get_logger(),
            "libsurvive device added: name=%s serial=%s",
            safe_string(survive_simple_object_name(object_event->object)).c_str(),
            safe_string(survive_simple_serial_number(object_event->object)).c_str());
        }
        continue;
      }
      if (event_type != SurviveSimpleEventType_PoseUpdateEvent) {
        continue;
      }

      const auto * pose_event = survive_simple_get_pose_updated_event(&event);
      if (pose_event == nullptr || pose_event->object == nullptr) {
        continue;
      }
      if (ignore_lighthouses_ &&
        survive_simple_object_get_type(pose_event->object) == SurviveSimpleObject_LIGHTHOUSE)
      {
        continue;
      }

      const std::string serial = safe_string(survive_simple_serial_number(pose_event->object));
      if (!should_publish_serial(serial)) {
        continue;
      }

      publish_pose(serial, pose_event->pose);
    }
  }

  bool should_publish_serial(const std::string & serial)
  {
    if (!target_serial_.empty()) {
      return serial == target_serial_;
    }
    if (!lock_first_object_) {
      return true;
    }
    if (active_serial_.empty()) {
      active_serial_ = serial;
      RCLCPP_INFO(
        get_logger(), "Locked libsurvive tracker source to serial '%s'",
        active_serial_.c_str());
    }
    return serial == active_serial_;
  }

  void publish_pose(const std::string & serial, const SurvivePose & pose)
  {
    if (announced_serials_.insert(serial).second) {
      RCLCPP_INFO(
        get_logger(),
        "Publishing libsurvive pose for serial '%s'",
        serial.c_str());
    }

    const LibsurvivePoseSample sample{
      world_frame_,
      serial,
      {static_cast<double>(pose.Pos[0]), static_cast<double>(pose.Pos[1]),
        static_cast<double>(pose.Pos[2])},
      {static_cast<double>(pose.Rot[0]), static_cast<double>(pose.Rot[1]),
        static_cast<double>(pose.Rot[2]),
        static_cast<double>(pose.Rot[3])},
      get_clock()->now(),
    };
    pose_pub_->publish(to_pose_stamped(sample));

    if (publish_deadman_) {
      std_msgs::msg::Bool deadman;
      deadman.data = deadman_value_;
      deadman_pub_->publish(deadman);
    }
  }

  std::string pose_topic_;
  std::string deadman_topic_;
  std::string world_frame_;
  std::string target_serial_;
  std::string active_serial_;
  bool lock_first_object_{true};
  bool ignore_lighthouses_{true};
  bool publish_deadman_{true};
  bool deadman_value_{true};
  std::string libsurvive_args_;
  std::string working_directory_;

  SurviveSimpleContext * ctx_{nullptr};
  std::atomic_bool running_{false};
  std::thread worker_;
  std::set<std::string> announced_serials_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
};

}  // namespace franka_tracker_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<franka_tracker_bridge::LibsurvivePoseNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("libsurvive_pose_node"), "%s", exc.what());
  }
  rclcpp::shutdown();
  return 0;
}
