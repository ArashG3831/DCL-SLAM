// Adapted from frontier-exploration-ros2 v1.6.0; project modifications described in THIRD_PARTY.md.
// Copyright 2026 Mert Güler. Licensed under Apache-2.0.
#pragma once
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
namespace frontier_exploration_ros2 {
struct Cell {int x; int y;};
struct FrontierCandidate {
  std::vector<Cell> cells;
  std::vector<Cell> approach_cells;
  double centroid_x{0}, centroid_y{0};
  double min_x{0}, min_y{0}, max_x{0}, max_y{0};
};
class OccupancyGrid2d {
public:
 explicit OccupancyGrid2d(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map): map_(std::move(map)) {}
 int width() const; int height() const; int cost(int x,int y) const; bool inside(int x,int y) const;
 std::pair<double,double> mapToWorld(int x,int y) const;
 bool worldToMap(double wx,double wy,int &x,int &y) const;
 double resolution() const; const nav_msgs::msg::OccupancyGrid & message() const {return *map_;}
private: nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
};
class FrontierCache {public: void reset(int width,int height); std::vector<uint8_t> visited, frontier;};
std::vector<FrontierCandidate> get_frontier(const OccupancyGrid2d &map, double robot_x,double robot_y,
 int free_threshold,int minimum_cells,bool diagonal_connectivity,FrontierCache &cache);
std::optional<Cell> choose_accessible_frontier_goal(const FrontierCandidate &f,const OccupancyGrid2d &map,
 double robot_x,double robot_y,double minimum_robot_distance);
}
