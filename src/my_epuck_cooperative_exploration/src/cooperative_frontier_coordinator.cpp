#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <my_epuck_interfaces/msg/exploration_event.hpp>
#include <my_epuck_interfaces/msg/exploration_status.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "my_epuck_cooperative_exploration/coordinator.hpp"
#include "my_epuck_cooperative_exploration/protocol.hpp"

using namespace std::chrono_literals;

namespace my_epuck_cooperative_exploration
{

class Coordinator : public rclcpp::Node
{
  using CandidateArray = my_epuck_interfaces::msg::FrontierCandidateArray;
  using Candidate = my_epuck_interfaces::msg::FrontierCandidate;
  using Event = my_epuck_interfaces::msg::ExplorationEvent;
  using Nav = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav>;

  struct Metrics
  {
    uint64_t candidate_batches_received{};
    uint64_t stale_candidate_batches{};
    uint64_t candidates_rejected{};
    uint64_t skipped_reserved{};
    uint64_t skipped_suppressed{};
    uint64_t claims_created{};
    uint64_t claims_published{};
    uint64_t peer_claims_received{};
    uint64_t peer_claims_accepted{};
    uint64_t peer_claim_expirations{};
    uint64_t conflicts_detected{};
    uint64_t arbitration_wins{};
    uint64_t arbitration_losses{};
    uint64_t navigation_goals_sent{};
    uint64_t navigation_goals_accepted{};
    uint64_t navigation_goals_rejected{};
    uint64_t navigation_successes{};
    uint64_t navigation_failures{};
    uint64_t navigation_cancellations{};
    uint64_t navigation_timeouts{};
    uint64_t cancellation_requests{};
    uint64_t cancellation_ack_timeouts{};
    uint64_t stale_action_callbacks_ignored{};
    uint64_t status_messages_published{};
    uint64_t peer_status_received{};
    uint64_t peer_status_accepted{};
    uint64_t state_transitions{};
  };

  struct Suppression
  {
    Claim region;
    double first_failure_at{};
    double last_failure_at{};
    uint32_t failure_count{};
    std::string category;
    double expires_at{};
    uint64_t originating_claim_id{};
    uint64_t originating_map_revision{};
    bool peer_observed{};
    bool success{};
    bool expiry_reported{};
  };

public:
  explicit Coordinator(const rclcpp::NodeOptions & options)
  : Node("cooperative_frontier_coordinator", options)
  {
#define P(T, N, D) N ## _ = declare_parameter<T>(#N, D)
    P(std::string, robot_id, "");
    P(std::string, peer_robot_id, "");
    P(std::string, operating_mode, "single_goal");
    P(std::string, own_candidate_topic, "frontier_candidates");
    P(std::string, own_shared_map_topic, "shared_map");
    P(std::string, peer_claim_topic, "peer/exploration_claim");
    P(std::string, own_claim_topic, "exploration_claim");
    P(std::string, peer_status_topic, "peer/exploration_status");
    P(std::string, own_status_topic, "exploration_status");
    P(std::string, own_event_topic, "exploration_event");
    P(std::string, navigate_to_pose_action, "navigate_to_pose");
    P(std::string, global_frame, "map");
    P(double, arbitration_window_s, 1.0);
    P(double, claim_heartbeat_rate_hz, 2.0);
    P(double, claim_ttl_s, 3.0);
    P(double, status_heartbeat_rate_hz, 2.0);
    P(double, status_ttl_s, 3.0);
    P(double, minimum_peer_ttl_s, 0.5);
    P(double, maximum_peer_ttl_s, 10.0);
    P(double, terminal_broadcast_duration_s, 1.0);
    P(double, terminal_broadcast_rate_hz, 4.0);
    P(double, cycle_cooldown_s, 2.5);
    P(double, candidate_refresh_timeout_s, 5.0);
    P(double, equivalent_frontier_centroid_tolerance_m, 0.15);
    P(double, equivalent_frontier_bbox_margin_m, 0.05);
    P(double, path_cost_tie_tolerance_m, 0.02);
    P(double, success_region_cooldown_s, 25.0);
    P(double, first_failure_suppression_s, 20.0);
    P(double, second_failure_suppression_s, 60.0);
    P(double, maximum_failure_suppression_s, 180.0);
    P(double, peer_failure_caution_s, 8.0);
    P(double, arbitration_loss_cooldown_s, 5.0);
    P(double, require_fresh_candidate_age_s, 5.0);
    P(double, require_fresh_shared_map_age_s, 5.0);
    P(double, navigation_goal_timeout_s, 90.0);
    P(double, cancellation_ack_timeout_s, 5.0);
    P(double, no_candidate_grace_s, 18.0);
    P(double, map_stability_window_s, 15.0);
    P(int, minimum_known_cell_gain_for_activity, 5);
    P(double, completion_consensus_grace_s, 8.0);
    P(int, maximum_proposals_per_round, 3);
    P(int, maximum_round_exclusion_records, 16);
    P(int, maximum_retired_session_records, 32);
    P(int, maximum_suppression_records, 128);
    P(bool, autostart, true);
    P(bool, one_goal_only, true);
    P(int, test_candidate_rank, -1);
    P(std::string, test_frontier_id, "");
#undef P

    if (robot_id_.empty() || peer_robot_id_.empty() || robot_id_ == peer_robot_id_) {
      throw std::runtime_error("exactly one distinct robot_id and peer_robot_id are required");
    }
    if (operating_mode_ != "single_goal" && operating_mode_ != "continuous") {
      throw std::runtime_error("operating_mode must be single_goal or continuous");
    }
    continuous_ = operating_mode_ == "continuous";
    if (!continuous_ && !one_goal_only_) {
      throw std::runtime_error("single_goal mode requires one_goal_only=true");
    }
    if (continuous_ && one_goal_only_) {
      throw std::runtime_error("continuous mode requires one_goal_only=false");
    }
    status_ttl_s_ =
      std::clamp(status_ttl_s_, minimum_peer_ttl_s_, maximum_peer_ttl_s_);

    session_ = new_uuid();
    cfg_ = {
      robot_id_, peer_robot_id_, global_frame_, minimum_peer_ttl_s_, maximum_peer_ttl_s_,
      equivalent_frontier_centroid_tolerance_m_, equivalent_frontier_bbox_margin_m_,
      path_cost_tie_tolerance_m_, size_t(maximum_retired_session_records_)};
    peer_claim_ = std::make_unique<PeerSessionTracker>(cfg_);
    peer_status_ = std::make_unique<PeerStatusTracker>(cfg_);

    const auto claim_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto status_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    candidate_sub_ = create_subscription<CandidateArray>(
      own_candidate_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      [this](CandidateArray::ConstSharedPtr batch) {candidate_cb(batch);});
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      own_shared_map_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {map_cb(map);});
    peer_claim_sub_ = create_subscription<Claim>(
      peer_claim_topic_, claim_qos, [this](Claim::ConstSharedPtr claim) {peer_claim_cb(*claim);});
    peer_status_sub_ = create_subscription<Status>(
      peer_status_topic_, status_qos,
      [this](Status::ConstSharedPtr status) {peer_status_cb(*status);});
    claim_pub_ = create_publisher<Claim>(own_claim_topic_, claim_qos);
    status_pub_ = create_publisher<Status>(own_status_topic_, status_qos);
    event_pub_ = create_publisher<Event>(
      own_event_topic_, rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile());
    nav_ = rclcpp_action::create_client<Nav>(this, navigate_to_pose_action_);

    started_ = steady();
    last_heartbeat_ = last_terminal_ = last_status_ = last_diag_ = started_;
    tick_ = create_wall_timer(50ms, [this]() {step();});
    publish_status(Status::STARTING, "waiting_for_inputs");
    RCLCPP_INFO(
      get_logger(),
      "coordinator robot=%s peer=%s mode=%s candidate=%s map=%s peer_claim=%s "
      "own_claim=%s peer_status=%s own_status=%s action=%s "
      "claim/status_qos=reliable,volatile,keep_last,depth=10; "
      "no peer action, peer candidate, ComputePathToPose, or cmd_vel interfaces; session=%s",
      robot_id_.c_str(), peer_robot_id_.c_str(), operating_mode_.c_str(),
      candidate_sub_->get_topic_name(), map_sub_->get_topic_name(),
      peer_claim_sub_->get_topic_name(), claim_pub_->get_topic_name(),
      peer_status_sub_->get_topic_name(), status_pub_->get_topic_name(),
      navigate_to_pose_action_.c_str(), uuid_key(session_).c_str());
  }

  CoordinatorDiagnostics diagnostics() const
  {
    return {
      raw_candidate_callbacks_.load(), accepted_candidate_batches_.load(),
      m_.navigation_goals_sent, m_.navigation_goals_accepted, cycle_number_,
      m_.stale_action_callbacks_ignored, nav_->action_server_is_ready(), continuous_,
      core_.state == InternalState::MISSION_COMPLETE, core_.state};
  }

private:
  double steady() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static Uuid new_uuid()
  {
    Uuid uuid{};
    std::random_device random;
    for (auto & byte : uuid) {
      byte = uint8_t(random());
    }
    uuid[6] = (uuid[6] & 0x0f) | 0x40;
    uuid[8] = (uuid[8] & 0x3f) | 0x80;
    return uuid;
  }

  void transition(InternalState state)
  {
    if (core_.state == state) {
      return;
    }
    core_.state = state;
    ++m_.state_transitions;
  }

  bool candidate_valid(const Candidate & candidate) const
  {
    const auto & position = candidate.approach_pose.pose.position;
    const auto & quaternion = candidate.approach_pose.pose.orientation;
    const double norm =
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w;
    return candidate.reachability_state == Candidate::REACHABLE &&
           std::isfinite(candidate.path_length_m) && candidate.path_length_m >= 0.0 &&
           std::isfinite(candidate.information_gain) && std::isfinite(candidate.score) &&
           std::isfinite(candidate.centroid.x) && std::isfinite(candidate.centroid.y) &&
           std::isfinite(candidate.bounding_box_min.x) &&
           std::isfinite(candidate.bounding_box_min.y) &&
           std::isfinite(candidate.bounding_box_max.x) &&
           std::isfinite(candidate.bounding_box_max.y) &&
           candidate.bounding_box_min.x <= candidate.bounding_box_max.x &&
           candidate.bounding_box_min.y <= candidate.bounding_box_max.y &&
           std::isfinite(position.x) && std::isfinite(position.y) &&
           std::isfinite(norm) && norm > 0.5 && norm < 1.5 &&
           candidate.approach_pose.header.frame_id == global_frame_;
  }

  void candidate_cb(CandidateArray::ConstSharedPtr batch)
  {
    ++raw_candidate_callbacks_;
    ++m_.candidate_batches_received;
    if (batch->source_robot_id != robot_id_ || batch->header.frame_id != global_frame_ ||
      batch->map_revision == 0)
    {
      m_.candidates_rejected += batch->candidates.size();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "candidate batch rejected: source=%s frame=%s revision=%lu",
        batch->source_robot_id.c_str(), batch->header.frame_id.c_str(), batch->map_revision);
      return;
    }
    latest_ = batch;
    candidate_received_ = steady();
    ++latest_batch_sequence_;
    ++accepted_candidate_batches_;

    if ((core_.state == InternalState::PROPOSING ||
      core_.state == InternalState::ARBITRATING) && current_ && !selected_present(*batch))
    {
      release_proposal("candidate_disappeared");
    }
    if ((core_.state == InternalState::LOCALLY_EXHAUSTED || consensus_started_at_ > 0.0) &&
      any_eligible(false))
    {
      consensus_started_at_ = 0.0;
      exhaustion_started_at_ = 0.0;
      emit_event("EXPLORATION_RESUMED", "fresh eligible candidate");
      transition(InternalState::SELECTING_NEXT);
    }
  }

  void map_cb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
  {
    if (map->header.frame_id != global_frame_ || map->data.empty()) {
      return;
    }
    map_received_ = steady();
    shared_map_checksum_ = 1469598103934665603ULL;
    uint64_t known = 0;
    for (const auto cell : map->data) {
      shared_map_checksum_ ^= static_cast<uint8_t>(cell);
      shared_map_checksum_ *= 1099511628211ULL;
      if (cell >= 0) {
        ++known;
      }
    }
    known_cells_ = known;
    known_history_.emplace_back(map_received_, known);
    while (!known_history_.empty() &&
      map_received_ - known_history_.front().first > map_stability_window_s_ * 2.0)
    {
      known_history_.pop_front();
    }
  }

  bool selected_present(const CandidateArray & batch) const
  {
    for (const auto & candidate : batch.candidates) {
      if (candidate.frontier_id == current_->frontier_id ||
        equivalent_frontiers(
          *current_, claim_from(candidate, current_->claim_id, Claim::PROPOSING, ""),
          equivalent_frontier_centroid_tolerance_m_, equivalent_frontier_bbox_margin_m_))
      {
        return true;
      }
    }
    return false;
  }

  void peer_claim_cb(const Claim & claim)
  {
    ++m_.peer_claims_received;
    const double time = steady();
    const auto decision = peer_claim_->accept(claim, time, !peer_claim_->expired(time));
    if (decision.code != ValidationCode::OK) {
      return;
    }
    ++m_.peer_claims_accepted;
    peer_claim_was_fresh_ = true;
    const auto peer = peer_claim_->claim(time);
    if (!peer) {
      return;
    }
    observe_peer_terminal(*peer);
    if (!current_ || !reserving(peer->state)) {
      return;
    }
    const auto result = arbitrate(
      *current_, *peer, equivalent_frontier_centroid_tolerance_m_,
      equivalent_frontier_bbox_margin_m_, path_cost_tie_tolerance_m_, true);
    if (result != ArbitrationResult::NO_CONFLICT) {
      ++m_.conflicts_detected;
    }
    if (core_.state == InternalState::NAVIGATING &&
      result == ArbitrationResult::PEER_WINS)
    {
      request_cancel("ARBITRATION_CANCELLATION", true);
    }
  }

  void peer_status_cb(const Status & status)
  {
    ++m_.peer_status_received;
    const double time = steady();
    const auto old = peer_status_->status(time);
    const auto decision = peer_status_->accept(status, time, !peer_status_->expired(time));
    if (decision.code != ValidationCode::OK) {
      return;
    }
    ++m_.peer_status_accepted;
    peer_status_was_fresh_ = true;
    if (!old || old->state != status.state || decision.replacement) {
      emit_event(
        "PEER_STATUS_CHANGED", decision.replacement ? "peer_session_restarted" : status.reason);
    }
  }

  Claim claim_from(
    const Candidate & candidate, uint64_t claim_id, uint8_t state,
    const std::string & reason) const
  {
    Claim claim;
    claim.header.frame_id = global_frame_;
    claim.source_robot_id = robot_id_;
    std::copy(session_.begin(), session_.end(), claim.source_session_id.uuid.begin());
    claim.claim_id = claim_id;
    claim.state = state;
    claim.frontier_id = candidate.frontier_id;
    claim.map_revision = latest_ ? latest_->map_revision : 0;
    claim.frontier_centroid = candidate.centroid;
    claim.bounding_box_min = candidate.bounding_box_min;
    claim.bounding_box_max = candidate.bounding_box_max;
    claim.approach_pose = candidate.approach_pose;
    claim.path_length_m = candidate.path_length_m;
    claim.information_gain = candidate.information_gain;
    claim.utility_score = candidate.score;
    claim.claim_ttl.sec = int32_t(claim_ttl_s_);
    claim.claim_ttl.nanosec =
      uint32_t((claim_ttl_s_ - std::floor(claim_ttl_s_)) * 1e9);
    claim.state_reason = reason;
    return claim;
  }

  void publish_claim(uint8_t state, const std::string & reason)
  {
    if (!current_) {
      return;
    }
    current_->state = state;
    current_->state_reason = reason;
    current_->header.stamp = now();
    current_->message_revision = ++message_revision_;
    claim_pub_->publish(*current_);
    ++m_.claims_published;
  }

  uint8_t external_status() const
  {
    if (core_.state == InternalState::MISSION_COMPLETE) {
      return Status::COMPLETE;
    }
    if (core_.state == InternalState::LOCALLY_EXHAUSTED) {
      return Status::NO_ELIGIBLE_CANDIDATES;
    }
    if (core_.state == InternalState::NAVIGATING) {
      return Status::NAVIGATING;
    }
    if (core_.state == InternalState::IDLE_STOPPED) {
      return Status::STOPPED;
    }
    if (core_.state == InternalState::WAITING_FOR_INPUTS) {
      return Status::STARTING;
    }
    return Status::ACTIVE;
  }

  void publish_status(uint8_t state, const std::string & reason)
  {
    Status status;
    status.header.frame_id = global_frame_;
    status.header.stamp = now();
    status.source_robot_id = robot_id_;
    std::copy(session_.begin(), session_.end(), status.source_session_id.uuid.begin());
    status.message_revision = ++status_revision_;
    status.state = state;
    status.candidate_count = latest_ ? latest_->candidates.size() : 0;
    status.eligible_candidate_count = eligible_count(false);
    status.active_claim_id = current_ && reserving(current_->state) ? current_->claim_id : 0;
    status.active_frontier_id = current_ && reserving(current_->state) ? current_->frontier_id : 0;
    status.local_map_revision = latest_ ? latest_->map_revision : 0;
    status.shared_map_checksum = shared_map_checksum_;
    status.status_ttl.sec = int32_t(status_ttl_s_);
    status.status_ttl.nanosec =
      uint32_t((status_ttl_s_ - std::floor(status_ttl_s_)) * 1e9);
    status.reason = reason;
    status_pub_->publish(status);
    ++m_.status_messages_published;
  }

  void emit_event(const std::string & type, const std::string & reason)
  {
    Event event;
    event.header.frame_id = global_frame_;
    event.header.stamp = now();
    event.source_robot_id = robot_id_;
    std::copy(session_.begin(), session_.end(), event.source_session_id.uuid.begin());
    event.event_revision = ++event_revision_;
    event.event_type = type;
    event.cycle_number = cycle_number_;
    if (current_) {
      event.claim_id = current_->claim_id;
      event.frontier_id = current_->frontier_id;
      event.candidate_map_revision = current_->map_revision;
      event.selected_rank = selected_rank_;
      event.path_length_m = current_->path_length_m;
      event.information_gain = current_->information_gain;
      event.goal_pose = current_->approach_pose;
    } else {
      event.selected_rank = -1;
    }
    event.terminal_result = terminal_reason_;
    event.duration_s =
      cycle_started_at_ > 0.0 ? static_cast<float>(steady() - cycle_started_at_) : 0.0F;
    event.travelled_distance_m = 0.0F;
    event.reason = reason;
    event_pub_->publish(event);
  }

  bool equivalent(const Claim & region, const Suppression & record) const
  {
    return equivalent_frontiers(
      region, record.region, equivalent_frontier_centroid_tolerance_m_,
      equivalent_frontier_bbox_margin_m_);
  }

  bool suppression_active(
    const Claim & region, uint64_t map_revision, bool emit_skip) const
  {
    const double time = steady();
    for (const auto & record : suppressions_) {
      if (!equivalent(region, record)) {
        continue;
      }
      const bool active_time = time < record.expires_at;
      const bool awaiting_new_evidence =
        record.success && map_revision <= record.originating_map_revision;
      if (active_time || awaiting_new_evidence) {
        if (emit_skip) {
          const_cast<Coordinator *>(this)->emit_event(
            "CANDIDATE_SKIPPED_SUPPRESSED", record.category);
          ++const_cast<Coordinator *>(this)->m_.skipped_suppressed;
        }
        return true;
      }
    }
    return false;
  }

  void expire_suppressions()
  {
    const double time = steady();
    for (auto & record : suppressions_) {
      if (!record.expiry_reported && time >= record.expires_at &&
        (!record.success || (latest_ && latest_->map_revision > record.originating_map_revision)))
      {
        record.expiry_reported = true;
        emit_event("SUPPRESSION_EXPIRED", record.category);
      }
    }
    suppressions_.erase(
      std::remove_if(
        suppressions_.begin(), suppressions_.end(),
        [this, time](const Suppression & record) {
          return record.expiry_reported && time - record.expires_at >
                 std::max(maximum_failure_suppression_s_, success_region_cooldown_s_);
        }),
      suppressions_.end());
  }

  void add_suppression(
    const Claim & region, const std::string & category, bool peer, bool success)
  {
    const double time = steady();
    auto existing = std::find_if(
      suppressions_.begin(), suppressions_.end(),
      [this, &region, peer, success](const Suppression & record) {
        return record.peer_observed == peer && record.success == success &&
               equivalent_frontiers(
          region, record.region, equivalent_frontier_centroid_tolerance_m_,
          equivalent_frontier_bbox_margin_m_);
      });

    double duration = success ? success_region_cooldown_s_ :
      (peer ? peer_failure_caution_s_ : first_failure_suppression_s_);
    bool extended = false;
    if (existing != suppressions_.end()) {
      extended = true;
      existing->last_failure_at = time;
      existing->category = category;
      existing->originating_claim_id = region.claim_id;
      existing->originating_map_revision = region.map_revision;
      existing->expiry_reported = false;
      if (!success && !peer) {
        ++existing->failure_count;
        duration = existing->failure_count == 1 ? first_failure_suppression_s_ :
          (existing->failure_count == 2 ? second_failure_suppression_s_ :
          maximum_failure_suppression_s_);
      }
      existing->expires_at = time + std::min(duration, maximum_failure_suppression_s_);
      existing->region = region;
    } else {
      Suppression record;
      record.region = region;
      record.first_failure_at = time;
      record.last_failure_at = time;
      record.failure_count = (!success && !peer) ? 1U : 0U;
      record.category = category;
      record.expires_at = time + duration;
      record.originating_claim_id = region.claim_id;
      record.originating_map_revision = region.map_revision;
      record.peer_observed = peer;
      record.success = success;
      suppressions_.push_back(record);
      while (suppressions_.size() > size_t(maximum_suppression_records_)) {
        suppressions_.erase(suppressions_.begin());
      }
    }
    emit_event(
      extended ? "SUPPRESSION_EXTENDED" :
      (success ? "SUCCESS_COOLDOWN_CREATED" : "FAILURE_SUPPRESSION_CREATED"),
      category);
  }

  void add_arbitration_cooldown()
  {
    if (!current_) {
      return;
    }
    Suppression record;
    record.region = *current_;
    record.first_failure_at = record.last_failure_at = steady();
    record.category = "ARBITRATION_CANCELLATION";
    record.expires_at = steady() + arbitration_loss_cooldown_s_;
    record.originating_claim_id = current_->claim_id;
    record.originating_map_revision = current_->map_revision;
    suppressions_.push_back(record);
    while (suppressions_.size() > size_t(maximum_suppression_records_)) {
      suppressions_.erase(suppressions_.begin());
    }
  }

  void observe_peer_terminal(const Claim & claim)
  {
    if (claim.state != Claim::FAILED) {
      return;
    }
    const std::string key =
      uuid_key(uuid_from_msg(claim.source_session_id)) + ":" + std::to_string(claim.claim_id);
    if (!observed_peer_failures_.insert(key).second) {
      return;
    }
    add_suppression(claim, "PEER_FAILURE_CAUTION", true, false);
  }

  std::optional<Candidate> select(bool emit_skips)
  {
    selected_rank_ = -1;
    if (!latest_ || latest_->map_revision == 0 ||
      steady() - candidate_received_ > require_fresh_candidate_age_s_)
    {
      return std::nullopt;
    }
    int valid_rank = 0;
    for (const auto & candidate : latest_->candidates) {
      if (!candidate_valid(candidate)) {
        ++m_.candidates_rejected;
        continue;
      }
      const auto tentative =
        claim_from(candidate, core_.claim_id + 1, Claim::PROPOSING, "");
      const auto peer = peer_claim_->claim(steady());
      if (peer && reserving(peer->state) &&
        equivalent_frontiers(
          tentative, *peer, equivalent_frontier_centroid_tolerance_m_,
          equivalent_frontier_bbox_margin_m_))
      {
        if (emit_skips) {
          ++m_.skipped_reserved;
          emit_event("CANDIDATE_SKIPPED_RESERVED", "fresh_peer_reservation");
        }
        ++valid_rank;
        continue;
      }
      if (suppression_active(tentative, latest_->map_revision, emit_skips)) {
        ++valid_rank;
        continue;
      }
      if (std::find(
          excluded_.begin(), excluded_.end(), candidate.frontier_id) != excluded_.end())
      {
        ++valid_rank;
        continue;
      }
      if (!test_frontier_id_.empty() &&
        std::to_string(candidate.frontier_id) != test_frontier_id_)
      {
        ++valid_rank;
        continue;
      }
      if (test_candidate_rank_ >= 0 && valid_rank != test_candidate_rank_) {
        ++valid_rank;
        continue;
      }
      selected_rank_ = valid_rank;
      return candidate;
    }
    return std::nullopt;
  }

  uint32_t eligible_count(bool emit_skips) const
  {
    if (!latest_ || latest_->map_revision == 0 ||
      steady() - candidate_received_ > require_fresh_candidate_age_s_)
    {
      return 0;
    }
    uint32_t count = 0;
    for (const auto & candidate : latest_->candidates) {
      if (!candidate_valid(candidate)) {
        continue;
      }
      const auto tentative =
        claim_from(candidate, core_.claim_id + 1, Claim::PROPOSING, "");
      const auto peer = peer_claim_->claim(steady());
      if (peer && reserving(peer->state) &&
        equivalent_frontiers(
          tentative, *peer, equivalent_frontier_centroid_tolerance_m_,
          equivalent_frontier_bbox_margin_m_))
      {
        continue;
      }
      if (suppression_active(tentative, latest_->map_revision, emit_skips)) {
        continue;
      }
      ++count;
    }
    return count;
  }

  bool any_eligible(bool emit_skips) const
  {
    return eligible_count(emit_skips) > 0;
  }

  void create_proposal()
  {
    if (continuous_ && cycle_number_ > 0 &&
      latest_batch_sequence_ <= completed_batch_sequence_)
    {
      return;
    }
    if (!core_.can_propose(maximum_proposals_per_round_, continuous_)) {
      begin_terminal(Claim::RELEASED, "proposal_budget_exhausted", "");
      return;
    }
    const auto candidate = select(true);
    if (!candidate) {
      consider_exhaustion();
      return;
    }
    exhaustion_started_at_ = 0.0;
    consensus_started_at_ = 0.0;
    core_.create_proposal();
    ++m_.claims_created;
    ++cycle_number_;
    ++goal_generation_;
    cycle_started_at_ = steady();
    current_ = claim_from(*candidate, core_.claim_id, Claim::PROPOSING, "proposal");
    emit_event("EXPLORATION_CYCLE_STARTED", "proposal_created");
    emit_event("NEXT_FRONTIER_SELECTED", "best_eligible_candidate");
    publish_claim(Claim::PROPOSING, "proposal");
    proposal_started_ = steady();
    last_heartbeat_ = proposal_started_;
    transition(InternalState::ARBITRATING);
  }

  void release_proposal(const std::string & reason)
  {
    if (!current_) {
      return;
    }
    publish_claim(Claim::RELEASED, reason);
    excluded_.push_back(current_->frontier_id);
    while (excluded_.size() > size_t(maximum_round_exclusion_records_)) {
      excluded_.erase(excluded_.begin());
    }
    emit_event("EXPLORATION_CYCLE_ENDED", reason);
    current_.reset();
    transition(InternalState::IDLE);
  }

  void dispatch()
  {
    if (!current_ || core_.goals_sent != 0) {
      return;
    }
    if (!nav_->action_server_is_ready()) {
      begin_terminal(Claim::RELEASED, "action_server_unavailable", "");
      return;
    }
    Nav::Goal goal;
    goal.pose = current_->approach_pose;
    const uint64_t generation = goal_generation_;
    const uint64_t claim_id = current_->claim_id;
    ++core_.goals_sent;
    ++m_.navigation_goals_sent;
    auto options = rclcpp_action::Client<Nav>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation, claim_id](GoalHandle::SharedPtr handle) {
        if (!active_callback(generation, claim_id, InternalState::ARBITRATING)) {
          ++m_.stale_action_callbacks_ignored;
          return;
        }
        if (!handle) {
          ++m_.navigation_goals_rejected;
          add_suppression(*current_, "GOAL_REJECTION", false, false);
          begin_terminal(Claim::FAILED, "GOAL_REJECTION", "GOAL_REJECTION");
          return;
        }
        goal_ = handle;
        if (!core_.accept_goal(continuous_)) {
          ++m_.stale_action_callbacks_ignored;
          return;
        }
        ++m_.navigation_goals_accepted;
        accepted_at_ = steady();
        cancel_pending_ = false;
        publish_claim(Claim::NAVIGATING, "goal_accepted");
        transition(InternalState::NAVIGATING);
      };
    options.feedback_callback =
      [this, generation, claim_id](
      GoalHandle::SharedPtr, const std::shared_ptr<const Nav::Feedback>) {
        if (!active_callback(generation, claim_id, InternalState::NAVIGATING)) {
          ++m_.stale_action_callbacks_ignored;
        }
      };
    options.result_callback =
      [this, generation, claim_id](const GoalHandle::WrappedResult & result) {
        if (!active_callback(generation, claim_id, InternalState::NAVIGATING)) {
          ++m_.stale_action_callbacks_ignored;
          return;
        }
        goal_.reset();
        if (cancel_pending_) {
          const bool arbitration = cancel_is_arbitration_;
          if (arbitration) {
            ++m_.navigation_cancellations;
            add_arbitration_cooldown();
            begin_terminal(
              Claim::CANCELED, "ARBITRATION_CANCELLATION", "ARBITRATION_CANCELLATION");
          } else {
            ++m_.navigation_failures;
            add_suppression(*current_, cancel_reason_, false, false);
            begin_terminal(Claim::CANCELED, cancel_reason_, cancel_reason_);
          }
          return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->error_code == 0)
        {
          ++m_.navigation_successes;
          add_suppression(*current_, "SUCCESS", false, true);
          begin_terminal(Claim::SUCCEEDED, "navigation_succeeded", "SUCCESS");
          return;
        }
        ++m_.navigation_failures;
        const std::string error_message =
          result.result ? result.result->error_msg : "missing_result";
        std::string category = classify_nav2_failure(error_message);
        if (category == "UNKNOWN_NAV2_FAILURE") {
          if (result.code == rclcpp_action::ResultCode::ABORTED) {
            category = "ACTION_ABORTED";
          } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
            category = "ACTION_CANCELED";
          } else if (result.code == rclcpp_action::ResultCode::UNKNOWN) {
            category = "ACTION_UNKNOWN";
          }
        }
        std::string reason = category + ":action_status=" +
          std::to_string(static_cast<int>(result.code)) + ":error_code=" +
          std::to_string(result.result ? result.result->error_code : 0) + ":error_msg=" +
          error_message;
        add_suppression(*current_, category, false, false);
        begin_terminal(Claim::FAILED, reason, category);
      };
    nav_->async_send_goal(goal, options);
  }

  bool active_callback(
    uint64_t generation, uint64_t claim_id, InternalState expected) const
  {
    return current_ && action_callback_matches(
      generation, claim_id, goal_generation_, current_->claim_id,
      core_.state, expected);
  }

  void request_cancel(const std::string & reason, bool arbitration)
  {
    if (cancel_pending_ || !goal_ || core_.state != InternalState::NAVIGATING) {
      return;
    }
    cancel_pending_ = true;
    cancel_is_arbitration_ = arbitration;
    cancel_reason_ = reason;
    cancel_requested_at_ = steady();
    ++m_.cancellation_requests;
    const uint64_t generation = goal_generation_;
    const uint64_t claim_id = current_->claim_id;
    nav_->async_cancel_goal(
      goal_,
      [this, generation, claim_id](auto response) {
        if (!active_callback(generation, claim_id, InternalState::NAVIGATING) ||
          !cancel_pending_)
        {
          ++m_.stale_action_callbacks_ignored;
          return;
        }
        goal_.reset();
        if (response && !response->goals_canceling.empty()) {
          if (cancel_is_arbitration_) {
            ++m_.navigation_cancellations;
            add_arbitration_cooldown();
            begin_terminal(
              Claim::CANCELED, "ARBITRATION_CANCELLATION", "ARBITRATION_CANCELLATION");
          } else {
            ++m_.navigation_failures;
            add_suppression(*current_, cancel_reason_, false, false);
            begin_terminal(Claim::CANCELED, cancel_reason_, cancel_reason_);
          }
        } else {
          ++m_.navigation_failures;
          add_suppression(*current_, "CANCELLATION_TIMEOUT", false, false);
          begin_terminal(
            Claim::FAILED, cancel_reason_ + ":cancel_rejected", "CANCELLATION_TIMEOUT");
        }
      });
  }

  void begin_terminal(
    uint8_t state, const std::string & reason, const std::string & terminal_result)
  {
    if (!current_ || core_.state == InternalState::TERMINAL_BROADCAST ||
      core_.state == InternalState::COOLDOWN ||
      core_.state == InternalState::IDLE_STOPPED)
    {
      return;
    }
    terminal_state_ = state;
    terminal_reason_ = terminal_result.empty() ? reason : terminal_result;
    terminal_claim_reason_ = reason;
    terminal_started_ = steady();
    last_terminal_ = 0.0;
    cancel_pending_ = false;
    transition(InternalState::TERMINAL_BROADCAST);
    publish_claim(state, reason);
    emit_event("EXPLORATION_CYCLE_ENDED", reason);
  }

  void finish_terminal()
  {
    if (!continuous_) {
      transition(InternalState::IDLE_STOPPED);
      publish_status(Status::STOPPED, "single_goal_complete");
      return;
    }
    completed_batch_sequence_ = latest_batch_sequence_;
    cooldown_started_ = steady();
    transition(InternalState::COOLDOWN);
  }

  bool inputs_fresh() const
  {
    const double time = steady();
    return latest_ && latest_->map_revision > 0 &&
           time - candidate_received_ <= require_fresh_candidate_age_s_ &&
           map_received_ > 0.0 &&
           time - map_received_ <= require_fresh_shared_map_age_s_;
  }

  bool map_stable() const
  {
    if (known_history_.empty()) {
      return false;
    }
    const double time = steady();
    auto oldest = known_history_.end();
    for (auto it = known_history_.begin(); it != known_history_.end(); ++it) {
      if (time - it->first <= map_stability_window_s_) {
        oldest = it;
        break;
      }
    }
    if (oldest == known_history_.end() ||
      time - oldest->first < map_stability_window_s_ * 0.9)
    {
      return false;
    }
    return known_cells_ <= oldest->second +
           static_cast<uint64_t>(std::max(0, minimum_known_cell_gain_for_activity_));
  }

  void consider_exhaustion()
  {
    const double time = steady();
    if (!continuous_ || !inputs_fresh() ||
      latest_batch_sequence_ <= completed_batch_sequence_)
    {
      return;
    }
    if (any_eligible(false)) {
      exhaustion_started_at_ = 0.0;
      return;
    }
    if (exhaustion_started_at_ == 0.0) {
      exhaustion_started_at_ = time;
      return;
    }
    if (time - exhaustion_started_at_ < no_candidate_grace_s_ || !map_stable()) {
      return;
    }
    transition(InternalState::LOCALLY_EXHAUSTED);
    emit_event("LOCALLY_EXHAUSTED", "fresh stable inputs contain no eligible frontier");
    publish_status(Status::NO_ELIGIBLE_CANDIDATES, "locally_exhausted");
  }

  void completion_step()
  {
    if (!inputs_fresh() || any_eligible(false)) {
      consensus_started_at_ = 0.0;
      if (any_eligible(false)) {
        exhaustion_started_at_ = 0.0;
        emit_event("EXPLORATION_RESUMED", "candidate_appeared_during_consensus");
        transition(InternalState::SELECTING_NEXT);
      }
      return;
    }
    const auto peer_status = peer_status_->status(steady());
    const auto peer_claim = peer_claim_->claim(steady());
    const bool peer_exhausted = peer_status &&
      (peer_status->state == Status::NO_ELIGIBLE_CANDIDATES ||
      peer_status->state == Status::COMPLETE);
    const bool peer_navigating = peer_status && peer_status->state == Status::NAVIGATING;
    const bool peer_reserving = peer_claim && reserving(peer_claim->state);
    if (!peer_exhausted || peer_navigating || peer_reserving) {
      consensus_started_at_ = 0.0;
      return;
    }
    if (consensus_started_at_ == 0.0) {
      consensus_started_at_ = steady();
      emit_event("COMPLETION_CONSENSUS_STARTED", "both_fresh_statuses_exhausted");
      return;
    }
    if (steady() - consensus_started_at_ >= completion_consensus_grace_s_) {
      transition(InternalState::MISSION_COMPLETE);
      mission_complete_started_ = steady();
      emit_event("MISSION_COMPLETE", "decentralized_completion_consensus");
      publish_status(Status::COMPLETE, "mission_complete");
    }
  }

  void step()
  {
    const double time = steady();
    expire_suppressions();
    if (peer_claim_was_fresh_ && peer_claim_->expired(time)) {
      ++m_.peer_claim_expirations;
      peer_claim_was_fresh_ = false;
    }
    if (peer_status_was_fresh_ && peer_status_->expired(time)) {
      peer_status_was_fresh_ = false;
      consensus_started_at_ = 0.0;
      emit_event("PEER_STATUS_CHANGED", "peer_status_expired");
    }
    if (time - last_status_ >= 1.0 / std::max(0.1, status_heartbeat_rate_hz_)) {
      publish_status(external_status(), "heartbeat");
      last_status_ = time;
    }

    if (core_.state == InternalState::WAITING_FOR_INPUTS) {
      if (!autostart_ || !latest_ ||
        time - candidate_received_ > require_fresh_candidate_age_s_ ||
        !nav_->action_server_is_ready() || (continuous_ && !inputs_fresh()))
      {
        return;
      }
      transition(InternalState::IDLE);
    }

    if (core_.state == InternalState::IDLE ||
      core_.state == InternalState::SELECTING_NEXT)
    {
      create_proposal();
    } else if (core_.state == InternalState::ARBITRATING) {
      if (time - last_heartbeat_ >= 1.0 / std::max(0.1, claim_heartbeat_rate_hz_)) {
        publish_claim(Claim::PROPOSING, "arbitrating");
        last_heartbeat_ = time;
      }
      if (time - proposal_started_ >= arbitration_window_s_) {
        const auto peer = peer_claim_->claim(time);
        const auto result = peer ? arbitrate(
          *current_, *peer, equivalent_frontier_centroid_tolerance_m_,
          equivalent_frontier_bbox_margin_m_, path_cost_tie_tolerance_m_, true) :
          ArbitrationResult::NO_CONFLICT;
        if (result == ArbitrationResult::PEER_WINS) {
          ++m_.arbitration_losses;
          add_arbitration_cooldown();
          release_proposal("arbitration_lost");
        } else {
          ++m_.arbitration_wins;
          dispatch();
        }
      }
    } else if (core_.state == InternalState::NAVIGATING) {
      if (time - last_heartbeat_ >= 1.0 / std::max(0.1, claim_heartbeat_rate_hz_)) {
        publish_claim(Claim::NAVIGATING, "navigating");
        last_heartbeat_ = time;
      }
      if (!cancel_pending_ && time - accepted_at_ >= navigation_goal_timeout_s_) {
        ++m_.navigation_timeouts;
        request_cancel("NAVIGATION_TIMEOUT", false);
      }
      if (cancel_pending_ &&
        time - cancel_requested_at_ >= cancellation_ack_timeout_s_)
      {
        ++m_.cancellation_ack_timeouts;
        goal_.reset();
        ++m_.navigation_failures;
        add_suppression(*current_, "CANCELLATION_TIMEOUT", false, false);
        begin_terminal(
          Claim::FAILED, cancel_reason_ + ":ack_timeout", "CANCELLATION_TIMEOUT");
      }
    } else if (core_.state == InternalState::TERMINAL_BROADCAST) {
      if (time - last_terminal_ >=
        1.0 / std::max(0.1, terminal_broadcast_rate_hz_))
      {
        publish_claim(terminal_state_, terminal_claim_reason_);
        last_terminal_ = time;
      }
      if (time - terminal_started_ >= terminal_broadcast_duration_s_) {
        finish_terminal();
      }
    } else if (core_.state == InternalState::COOLDOWN) {
      if (time - cooldown_started_ >= cycle_cooldown_s_) {
        current_.reset();
        excluded_.clear();
        core_.reset_cycle();
        transition(InternalState::SELECTING_NEXT);
      }
    } else if (core_.state == InternalState::LOCALLY_EXHAUSTED) {
      completion_step();
    } else if (core_.state == InternalState::MISSION_COMPLETE) {
      if (time - mission_complete_started_ <= terminal_broadcast_duration_s_ &&
        time - last_terminal_ >= 1.0 / std::max(0.1, terminal_broadcast_rate_hz_))
      {
        publish_status(Status::COMPLETE, "mission_complete");
        last_terminal_ = time;
      }
    }

    if (time - last_diag_ >= 5.0) {
      RCLCPP_INFO(
        get_logger(),
        "COORD_METRICS mode=%s state=%d cycle=%lu claim=%lu peer_claim_age=%.3f "
        "peer_status_age=%.3f goals_sent=%lu accepted=%lu success=%lu fail=%lu "
        "cancel=%lu suppression=%zu stale_callbacks=%lu",
        operating_mode_.c_str(), int(core_.state), cycle_number_,
        current_ ? current_->claim_id : 0, peer_claim_->age(time), peer_status_->age(time),
        m_.navigation_goals_sent, m_.navigation_goals_accepted,
        m_.navigation_successes, m_.navigation_failures, m_.navigation_cancellations,
        suppressions_.size(), m_.stale_action_callbacks_ignored);
      last_diag_ = time;
    }
  }

  std::atomic<uint64_t> raw_candidate_callbacks_{0};
  std::atomic<uint64_t> accepted_candidate_batches_{0};
  ProtocolConfig cfg_;
  std::unique_ptr<PeerSessionTracker> peer_claim_;
  std::unique_ptr<PeerStatusTracker> peer_status_;
  RoundCore core_;
  Metrics m_;
  Uuid session_{};
  CandidateArray::ConstSharedPtr latest_;
  std::deque<std::pair<double, uint64_t>> known_history_;
  std::vector<uint64_t> excluded_;
  std::vector<Suppression> suppressions_;
  std::unordered_set<std::string> observed_peer_failures_;
  std::optional<Claim> current_;
  GoalHandle::SharedPtr goal_;

  double candidate_received_{};
  double map_received_{};
  double started_{};
  double proposal_started_{};
  double accepted_at_{};
  double terminal_started_{};
  double cooldown_started_{};
  double cycle_started_at_{};
  double cancel_requested_at_{};
  double exhaustion_started_at_{};
  double consensus_started_at_{};
  double mission_complete_started_{};
  double last_heartbeat_{};
  double last_terminal_{};
  double last_status_{};
  double last_diag_{};
  uint64_t message_revision_{};
  uint64_t status_revision_{};
  uint64_t event_revision_{};
  uint64_t cycle_number_{};
  uint64_t goal_generation_{};
  uint64_t latest_batch_sequence_{};
  uint64_t completed_batch_sequence_{};
  uint64_t shared_map_checksum_{};
  uint64_t known_cells_{};
  int selected_rank_{-1};
  uint8_t terminal_state_{};
  bool continuous_{};
  bool peer_claim_was_fresh_{};
  bool peer_status_was_fresh_{};
  bool cancel_pending_{};
  bool cancel_is_arbitration_{};
  std::string terminal_reason_;
  std::string terminal_claim_reason_;
  std::string cancel_reason_;

  rclcpp_action::Client<Nav>::SharedPtr nav_;
  rclcpp::Subscription<CandidateArray>::SharedPtr candidate_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<Claim>::SharedPtr peer_claim_sub_;
  rclcpp::Subscription<Status>::SharedPtr peer_status_sub_;
  rclcpp::Publisher<Claim>::SharedPtr claim_pub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<Event>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr tick_;

  std::string robot_id_;
  std::string peer_robot_id_;
  std::string operating_mode_;
  std::string own_candidate_topic_;
  std::string own_shared_map_topic_;
  std::string peer_claim_topic_;
  std::string own_claim_topic_;
  std::string peer_status_topic_;
  std::string own_status_topic_;
  std::string own_event_topic_;
  std::string navigate_to_pose_action_;
  std::string global_frame_;
  std::string test_frontier_id_;
  double arbitration_window_s_{};
  double claim_heartbeat_rate_hz_{};
  double claim_ttl_s_{};
  double status_heartbeat_rate_hz_{};
  double status_ttl_s_{};
  double minimum_peer_ttl_s_{};
  double maximum_peer_ttl_s_{};
  double terminal_broadcast_duration_s_{};
  double terminal_broadcast_rate_hz_{};
  double cycle_cooldown_s_{};
  double candidate_refresh_timeout_s_{};
  double equivalent_frontier_centroid_tolerance_m_{};
  double equivalent_frontier_bbox_margin_m_{};
  double path_cost_tie_tolerance_m_{};
  double success_region_cooldown_s_{};
  double first_failure_suppression_s_{};
  double second_failure_suppression_s_{};
  double maximum_failure_suppression_s_{};
  double peer_failure_caution_s_{};
  double arbitration_loss_cooldown_s_{};
  double require_fresh_candidate_age_s_{};
  double require_fresh_shared_map_age_s_{};
  double navigation_goal_timeout_s_{};
  double cancellation_ack_timeout_s_{};
  double no_candidate_grace_s_{};
  double map_stability_window_s_{};
  double completion_consensus_grace_s_{};
  int minimum_known_cell_gain_for_activity_{};
  int maximum_proposals_per_round_{};
  int maximum_round_exclusion_records_{};
  int maximum_retired_session_records_{};
  int maximum_suppression_records_{};
  int test_candidate_rank_{};
  bool autostart_{};
  bool one_goal_only_{};
};

rclcpp::Node::SharedPtr make_coordinator(const rclcpp::NodeOptions & options)
{
  return std::make_shared<Coordinator>(options);
}

CoordinatorDiagnostics coordinator_diagnostics(const rclcpp::Node::SharedPtr & node)
{
  const auto coordinator = std::dynamic_pointer_cast<Coordinator>(node);
  if (!coordinator) {
    throw std::invalid_argument("node is not a production Coordinator");
  }
  return coordinator->diagnostics();
}

}  // namespace my_epuck_cooperative_exploration
