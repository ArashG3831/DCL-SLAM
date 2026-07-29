#pragma once

#include <cstdint>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "my_epuck_cooperative_exploration/protocol.hpp"

namespace my_epuck_cooperative_exploration
{

struct CoordinatorDiagnostics
{
  uint64_t raw_candidate_callbacks{0};
  uint64_t accepted_candidate_batches{0};
  bool action_server_ready{false};
};

rclcpp::Node::SharedPtr make_coordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
CoordinatorDiagnostics coordinator_diagnostics(const rclcpp::Node::SharedPtr & node);

}  // namespace my_epuck_cooperative_exploration
