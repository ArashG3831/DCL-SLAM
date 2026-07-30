#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include "frontier_exploration_ros2/frontier_search.hpp"
namespace my_epuck_frontier_candidates {
uint64_t fnv1a64(const void *data,size_t size,uint64_t seed=14695981039346656037ULL);
uint64_t map_checksum(const nav_msgs::msg::OccupancyGrid &map);
uint64_t stable_frontier_id(const frontier_exploration_ros2::FrontierCandidate &f,const frontier_exploration_ros2::OccupancyGrid2d &map,double quantum);
std::optional<double> path_length(const nav_msgs::msg::Path &path,double robot_x,double robot_y,double goal_x,double goal_y,double tolerance);
bool clearance_ok(const frontier_exploration_ros2::OccupancyGrid2d &map,double wx,double wy,double clearance,int blocked_threshold);
bool inside_with_margin(const frontier_exploration_ros2::OccupancyGrid2d &map,double wx,double wy,double margin);
std::optional<frontier_exploration_ros2::Cell> find_safe_approach(
  const frontier_exploration_ros2::OccupancyGrid2d &map,double target_x,double target_y,
  double frontier_x,double frontier_y,double search_radius,double inward_margin,
  double clearance,int blocked_threshold);
double normalized_value(double value,double minimum,double maximum);
bool async_request_is_current(uint64_t request_generation,uint64_t active_request,
  uint64_t request_revision,uint64_t cycle_revision,bool path_checking);
}
