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
  uint64_t navigation_goals_sent{0};
  uint64_t navigation_goals_accepted{0};
  uint64_t exploration_cycles{0};
  uint64_t stale_action_callbacks_ignored{0};
  bool action_server_ready{false};
  bool continuous_mode{false};
  bool mission_complete{false};
  InternalState state{InternalState::WAITING_FOR_INPUTS};
};

rclcpp::Node::SharedPtr make_coordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
CoordinatorDiagnostics coordinator_diagnostics(const rclcpp::Node::SharedPtr & node);

}  // namespace my_epuck_cooperative_exploration
