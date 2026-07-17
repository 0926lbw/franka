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
#pragma once

#include <array>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_tracker_bridge
{

struct LibsurvivePoseSample
{
  std::string frame_id;
  std::string serial;
  std::array<double, 3> position;
  // libsurvive stores quaternion as w, x, y, z.
  std::array<double, 4> rotation_wxyz;
  rclcpp::Time stamp;
};

inline geometry_msgs::msg::PoseStamped to_pose_stamped(const LibsurvivePoseSample & sample)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = sample.stamp;
  msg.header.frame_id = sample.frame_id;
  msg.pose.position.x = sample.position[0];
  msg.pose.position.y = sample.position[1];
  msg.pose.position.z = sample.position[2];
  msg.pose.orientation.w = sample.rotation_wxyz[0];
  msg.pose.orientation.x = sample.rotation_wxyz[1];
  msg.pose.orientation.y = sample.rotation_wxyz[2];
  msg.pose.orientation.z = sample.rotation_wxyz[3];
  return msg;
}

}  // namespace franka_tracker_bridge
