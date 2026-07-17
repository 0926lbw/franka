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
#include <gtest/gtest.h>

#include <array>
#include <string>

#include "franka_tracker_bridge/libsurvive_pose_conversions.hpp"

namespace
{

TEST(LibsurvivePoseConversions, ConvertsPositionQuaternionFrameAndStamp) {
  const auto stamp = rclcpp::Time(12, 345000000, RCL_SYSTEM_TIME);
  const franka_tracker_bridge::LibsurvivePoseSample sample{
    "libsurvive_world",
    "LHR_TEST",
    {1.0, 2.0, 3.0},
    {0.7, 0.1, 0.2, 0.3},
    stamp,
  };

  const auto msg = franka_tracker_bridge::to_pose_stamped(sample);

  EXPECT_EQ(msg.header.frame_id, "libsurvive_world");
  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.header.stamp.nanosec, 345000000u);
  EXPECT_DOUBLE_EQ(msg.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.w, 0.7);
}

}  // namespace
