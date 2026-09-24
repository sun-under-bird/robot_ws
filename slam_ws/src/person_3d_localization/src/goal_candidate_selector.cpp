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

#include "person_3d_localization/goal_candidate_selector.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace person_3d_localization
{

// 校验搜索参数，防止无效配置产生不安全的候选点。
GoalCandidateSelector::GoalCandidateSelector(const GoalSearchConfig & config)
: config_(config)
{
  if (config_.angular_samples < 4 || config_.radial_levels < 1 ||
    !std::isfinite(config_.radial_step) || config_.radial_step < 0.0 ||
    !std::isfinite(config_.clearance_radius) || config_.clearance_radius <= 0.0 ||
    config_.max_goal_cost < 0 || config_.max_goal_cost >= 253)
  {
    throw std::invalid_argument("导航候选点搜索参数不合法");
  }
}

// 在人体周围按角度和半径采样，所有候选点均不小于请求的安全距离。
std::vector<geometry_msgs::msg::PoseStamped> GoalCandidateSelector::Generate(
  const geometry_msgs::msg::PointStamped & person,
  double robot_x, double robot_y, double stand_off_distance) const
{
  std::vector<geometry_msgs::msg::PoseStamped> candidates;
  candidates.reserve(
    static_cast<std::size_t>(config_.angular_samples) * config_.radial_levels);
  const double base_angle = std::atan2(
    robot_y - person.point.y, robot_x - person.point.x);
  constexpr double two_pi = 6.28318530717958647692;
  for (int sample = 0; sample < config_.angular_samples; ++sample) {
    const int offset = sample == 0 ? 0 :
      (sample % 2 == 1 ? (sample + 1) / 2 : -sample / 2);
    const double angle = base_angle +
      two_pi * static_cast<double>(offset) / config_.angular_samples;
    for (int level = 0; level < config_.radial_levels; ++level) {
      const double radius = stand_off_distance + level * config_.radial_step;
      geometry_msgs::msg::PoseStamped candidate;
      candidate.header = person.header;
      candidate.pose.position.x = person.point.x + radius * std::cos(angle);
      candidate.pose.position.y = person.point.y + radius * std::sin(angle);
      candidate.pose.position.z = 0.0;
      // 机器人抵达候选点后仍面向人体，而不是沿圆周切线方向。
      const double yaw = std::atan2(
        person.point.y - candidate.pose.position.y,
        person.point.x - candidate.pose.position.x);
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw);
      candidate.pose.orientation = tf2::toMsg(orientation);
      candidates.push_back(candidate);
    }
  }
  return candidates;
}

// 将候选点反变换到栅格坐标，并拒绝障碍、未知和地图边界外的区域。
bool GoalCandidateSelector::IsSafe(
  const nav2_msgs::msg::Costmap & costmap,
  const geometry_msgs::msg::PoseStamped & candidate) const
{
  const auto & metadata = costmap.metadata;
  if (costmap.header.frame_id != candidate.header.frame_id ||
    metadata.size_x == 0U || metadata.size_y == 0U ||
    !std::isfinite(metadata.resolution) || metadata.resolution <= 0.0F ||
    costmap.data.size() !=
    static_cast<std::size_t>(metadata.size_x) * metadata.size_y)
  {
    return false;
  }

  const auto & rotation = metadata.origin.orientation;
  const double quaternion_norm = std::hypot(
    std::hypot(rotation.x, rotation.y), std::hypot(rotation.z, rotation.w));
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-6) {
    return false;
  }
  tf2::Quaternion origin_orientation;
  tf2::fromMsg(rotation, origin_orientation);
  origin_orientation.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(origin_orientation).getRPY(roll, pitch, yaw);
  const double dx = candidate.pose.position.x - metadata.origin.position.x;
  const double dy = candidate.pose.position.y - metadata.origin.position.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double resolution = metadata.resolution;
  if (!std::isfinite(local_x) || !std::isfinite(local_y) ||
    local_x < 0.0 || local_y < 0.0 ||
    local_x >= static_cast<double>(metadata.size_x) * resolution ||
    local_y >= static_cast<double>(metadata.size_y) * resolution)
  {
    return false;
  }

  const int center_x = static_cast<int>(std::floor(local_x / resolution));
  const int center_y = static_cast<int>(std::floor(local_y / resolution));
  const int search_cells = static_cast<int>(
    std::ceil(config_.clearance_radius / resolution)) + 1;
  // 把与机器人外接圆相交的栅格也算进去，避免障碍只擦到栅格边缘。
  const double checked_radius = config_.clearance_radius +
    resolution * 0.7071067811865476;
  for (int y = center_y - search_cells; y <= center_y + search_cells; ++y) {
    for (int x = center_x - search_cells; x <= center_x + search_cells; ++x) {
      const double cell_dx = (static_cast<double>(x) + 0.5) * resolution - local_x;
      const double cell_dy = (static_cast<double>(y) + 0.5) * resolution - local_y;
      if (std::hypot(cell_dx, cell_dy) > checked_radius) {
        continue;
      }
      if (x < 0 || y < 0 || x >= static_cast<int>(metadata.size_x) ||
        y >= static_cast<int>(metadata.size_y))
      {
        return false;
      }
      const auto cost = costmap.data[
        static_cast<std::size_t>(y) * metadata.size_x + static_cast<std::size_t>(x)];
      if (cost > config_.max_goal_cost) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace person_3d_localization
