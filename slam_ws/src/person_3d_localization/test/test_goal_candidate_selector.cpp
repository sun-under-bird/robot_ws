// Copyright 2026 bird
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

#include <cmath>
#include <cstddef>

#include "person_3d_localization/goal_candidate_selector.hpp"

namespace person_3d_localization
{
namespace
{

// 构造已知空闲的全局代价地图，供障碍和边界测试修改。
nav2_msgs::msg::Costmap MakeFreeCostmap()
{
  nav2_msgs::msg::Costmap costmap;
  costmap.header.frame_id = "map";
  costmap.metadata.resolution = 0.1F;
  costmap.metadata.size_x = 200;
  costmap.metadata.size_y = 200;
  costmap.metadata.origin.position.x = -10.0;
  costmap.metadata.origin.position.y = -10.0;
  costmap.metadata.origin.orientation.w = 1.0;
  costmap.data.assign(200U * 200U, 0U);
  return costmap;
}

// 验证原直线目标优先，同时在人体周围生成其他角度和更远半径。
TEST(GoalCandidateSelectorTest, GeneratesRingAroundPerson)
{
  GoalCandidateSelector selector(GoalSearchConfig{});
  geometry_msgs::msg::PointStamped person;
  person.header.frame_id = "map";
  person.point.x = 4.0;
  person.point.y = 2.0;
  const auto candidates = selector.Generate(person, 0.0, 2.0, 1.5);
  ASSERT_EQ(candidates.size(), 48U);
  EXPECT_NEAR(candidates[0].pose.position.x, 2.5, 1e-6);
  EXPECT_NEAR(candidates[0].pose.position.y, 2.0, 1e-6);
  EXPECT_NEAR(candidates[1].pose.position.x, 2.2, 1e-6);
  EXPECT_NEAR(candidates[2].pose.position.x, 1.9, 1e-6);
  for (const auto & candidate : candidates) {
    EXPECT_GE(
      std::hypot(
        candidate.pose.position.x - 4.0,
        candidate.pose.position.y - 2.0), 1.5 - 1e-6);
  }
}

// 验证候选点周围出现障碍、未知或高代价值时被拒绝。
TEST(GoalCandidateSelectorTest, RejectsBlockedOrUnknownGoal)
{
  GoalCandidateSelector selector(GoalSearchConfig{});
  auto costmap = MakeFreeCostmap();
  geometry_msgs::msg::PoseStamped candidate;
  candidate.header.frame_id = "map";
  candidate.pose.position.x = 0.05;
  candidate.pose.position.y = 0.05;
  EXPECT_TRUE(selector.IsSafe(costmap, candidate));

  const std::size_t center = 100U * 200U + 100U;
  costmap.data[center] = 254U;
  EXPECT_FALSE(selector.IsSafe(costmap, candidate));
  costmap.data[center] = 255U;
  EXPECT_FALSE(selector.IsSafe(costmap, candidate));
  costmap.data[center] = 200U;
  EXPECT_FALSE(selector.IsSafe(costmap, candidate));
  costmap.data[center] = 0U;
  EXPECT_TRUE(selector.IsSafe(costmap, candidate));
}

// 验证地图外、边界附近和坐标系不一致的目标不会被误判为安全。
TEST(GoalCandidateSelectorTest, RejectsOutOfBoundsAndWrongFrame)
{
  GoalCandidateSelector selector(GoalSearchConfig{});
  const auto costmap = MakeFreeCostmap();
  geometry_msgs::msg::PoseStamped candidate;
  candidate.header.frame_id = "map";
  candidate.pose.position.x = 10.0;
  EXPECT_FALSE(selector.IsSafe(costmap, candidate));
  candidate.pose.position.x = 9.8;
  EXPECT_FALSE(selector.IsSafe(costmap, candidate));
  candidate.pose.position.x = 0.0;
  candidate.header.frame_id = "odom";
  EXPECT_FALSE(selector.IsSafe(costmap, candidate));
}

// 验证正前方多个半径被障碍占据时，侧向候选点仍可供后续路径验证。
TEST(GoalCandidateSelectorTest, HasSideAlternativeWhenFrontIsBlocked)
{
  GoalCandidateSelector selector(GoalSearchConfig{});
  auto costmap = MakeFreeCostmap();
  geometry_msgs::msg::PointStamped person;
  person.header.frame_id = "map";
  const auto candidates = selector.Generate(person, -4.0, 0.0, 1.5);
  for (std::size_t index = 0; index < 3U; ++index) {
    const auto x = static_cast<std::size_t>(
      std::floor((candidates[index].pose.position.x + 10.0) / 0.1));
    costmap.data[100U * 200U + x] = 254U;
    EXPECT_FALSE(selector.IsSafe(costmap, candidates[index]));
  }
  EXPECT_TRUE(selector.IsSafe(costmap, candidates[3]));
}

}  // namespace
}  // namespace person_3d_localization
