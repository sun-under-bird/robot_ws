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

#ifndef PERSON_3D_LOCALIZATION__GOAL_CANDIDATE_SELECTOR_HPP_
#define PERSON_3D_LOCALIZATION__GOAL_CANDIDATE_SELECTOR_HPP_

#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>

namespace person_3d_localization
{

struct GoalSearchConfig
{
  int angular_samples{16};
  int radial_levels{3};
  double radial_step{0.3};
  double clearance_radius{0.45};
  int max_goal_cost{199};
};

class GoalCandidateSelector
{
public:
  // 保存采样与代价地图过滤参数。
  explicit GoalCandidateSelector(const GoalSearchConfig & config);

  // 按优先级生成绕人的候选位姿，优先选择机器人朝向人体的一侧。
  std::vector<geometry_msgs::msg::PoseStamped> Generate(
    const geometry_msgs::msg::PointStamped & person,
    double robot_x, double robot_y, double stand_off_distance) const;

  // 用完整代价地图检查候选点及机器人周围的栅格是否已知且安全。
  bool IsSafe(
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PoseStamped & candidate) const;

private:
  GoalSearchConfig config_;
};

}  // namespace person_3d_localization

#endif  // PERSON_3D_LOCALIZATION__GOAL_CANDIDATE_SELECTOR_HPP_
