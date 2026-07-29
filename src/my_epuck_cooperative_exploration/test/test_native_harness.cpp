#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <my_epuck_interfaces/msg/exploration_claim.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "my_epuck_cooperative_exploration/coordinator.hpp"

using namespace std::chrono_literals;
using CandidateArray = my_epuck_interfaces::msg::FrontierCandidateArray;
using Candidate = my_epuck_interfaces::msg::FrontierCandidate;
using Claim = my_epuck_interfaces::msg::ExplorationClaim;
using Navigate = nav2_msgs::action::NavigateToPose;

namespace
{

enum class MockMode {SUCCEED, REJECT, ABORT, NONZERO_ERROR, HOLD, SLOW_CANCEL};

Candidate make_candidate(uint64_t id, double x, double y, float path, float score = 1.0F)
{
  Candidate candidate;
  candidate.frontier_id = id;
  candidate.centroid.x = x;
  candidate.centroid.y = y;
  candidate.bounding_box_min.x = x - 0.10;
  candidate.bounding_box_min.y = y - 0.10;
  candidate.bounding_box_max.x = x + 0.10;
  candidate.bounding_box_max.y = y + 0.10;
  candidate.approach_pose.header.frame_id = "shared_map";
  candidate.approach_pose.pose.position.x = x - 0.20;
  candidate.approach_pose.pose.position.y = y;
  candidate.approach_pose.pose.orientation.w = 1.0;
  candidate.cell_count = 20;
  candidate.frontier_length_m = 0.20F;
  candidate.information_gain = 0.20F;
  candidate.euclidean_distance_m = path;
  candidate.path_length_m = path;
  candidate.score = score;
  candidate.reachability_state = Candidate::REACHABLE;
  return candidate;
}

CandidateArray make_batch(
  const std::string & robot, uint64_t revision, std::vector<Candidate> candidates)
{
  CandidateArray batch;
  batch.header.frame_id = "shared_map";
  batch.source_robot_id = robot;
  batch.map_revision = revision;
  batch.planner_id = "GridBased";
  batch.candidates = std::move(candidates);
  return batch;
}

bool valid_fixture(const CandidateArray & batch, const std::string & robot)
{
  if (batch.source_robot_id != robot || batch.map_revision == 0 ||
    batch.header.frame_id != "shared_map") return false;
  for (const auto & c : batch.candidates) {
    const auto & p = c.approach_pose.pose.position;
    const auto & q = c.approach_pose.pose.orientation;
    const double qn = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (c.reachability_state != Candidate::REACHABLE ||
      !std::isfinite(c.centroid.x) || !std::isfinite(c.centroid.y) ||
      !std::isfinite(c.bounding_box_min.x) || !std::isfinite(c.bounding_box_min.y) ||
      !std::isfinite(c.bounding_box_max.x) || !std::isfinite(c.bounding_box_max.y) ||
      c.bounding_box_min.x > c.bounding_box_max.x ||
      c.bounding_box_min.y > c.bounding_box_max.y ||
      !std::isfinite(c.path_length_m) || !std::isfinite(c.information_gain) ||
      !std::isfinite(c.score) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      c.approach_pose.header.frame_id != "shared_map" || std::abs(qn - 1.0) > 1e-6) return false;
  }
  return true;
}

Claim make_peer_claim(
  uint8_t session, uint64_t revision, float path, uint8_t state = Claim::PROPOSING)
{
  Claim claim;
  claim.header.frame_id = "shared_map";
  claim.source_robot_id = "robot2";
  claim.source_session_id.uuid[0] = session;
  claim.message_revision = revision;
  claim.claim_id = 77;
  claim.state = state;
  claim.frontier_id = 9002;
  claim.map_revision = 8;
  claim.frontier_centroid.x = 1.01;
  claim.frontier_centroid.y = 0.0;
  claim.bounding_box_min.x = 0.91;
  claim.bounding_box_min.y = -0.10;
  claim.bounding_box_max.x = 1.11;
  claim.bounding_box_max.y = 0.10;
  claim.approach_pose.header.frame_id = "shared_map";
  claim.approach_pose.pose.position.x = 0.81;
  claim.approach_pose.pose.orientation.w = 1.0;
  claim.path_length_m = path;
  claim.information_gain = 0.20F;
  claim.utility_score = 1.0F;
  claim.claim_ttl.nanosec = 600000000;
  return claim;
}

class MockNavigateServer : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<Navigate>;

  MockNavigateServer(const std::string & robot, MockMode mode)
  : Node(robot + "_mock_navigate"), mode_(mode)
  {
    server_ = rclcpp_action::create_server<Navigate>(
      this, "/" + robot + "/navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal) {
        goals_received_++;
        {std::lock_guard<std::mutex> lock(mutex_); goal_poses_.push_back(goal->pose);}
        return mode_ == MockMode::REJECT ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle>) {
        cancel_requests_++;
        if (mode_ == MockMode::SLOW_CANCEL) std::this_thread::sleep_for(800ms);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        goals_accepted_++;
        std::lock_guard<std::mutex> lock(thread_mutex_);
        workers_.emplace_back([this, handle]() { execute(handle); });
      });
  }

  ~MockNavigateServer() override
  {
    stopping_ = true;
    std::lock_guard<std::mutex> lock(thread_mutex_);
    for (auto & worker : workers_) if (worker.joinable()) worker.join();
  }

  void set_mode(MockMode mode) {mode_ = mode;}
  int goals_received() const {return goals_received_.load();}
  int goals_accepted() const {return goals_accepted_.load();}
  int cancel_requests() const {return cancel_requests_.load();}
  int successes() const {return successes_.load();}
  int aborts() const {return aborts_.load();}

private:
  void execute(const std::shared_ptr<GoalHandle> & handle)
  {
    auto feedback = std::make_shared<Navigate::Feedback>();
    feedback->distance_remaining = 0.5F;
    handle->publish_feedback(feedback);
    if (mode_ == MockMode::HOLD || mode_ == MockMode::SLOW_CANCEL) {
      while (rclcpp::ok() && !stopping_ && !handle->is_canceling()) std::this_thread::sleep_for(5ms);
      if (handle->is_canceling()) {
        handle->canceled(std::make_shared<Navigate::Result>());
        return;
      }
      return;
    }
    std::this_thread::sleep_for(120ms);
    if (mode_ == MockMode::ABORT) {
      auto result = std::make_shared<Navigate::Result>();
      result->error_code = 1;
      result->error_msg = "mock_abort";
      handle->abort(result);
      aborts_++;
      return;
    }
    auto result = std::make_shared<Navigate::Result>();
    if (mode_ == MockMode::NONZERO_ERROR) {
      result->error_code = 2;
      result->error_msg = "mock_nonzero";
    }
    handle->succeed(result);
    successes_++;
  }

  std::atomic<MockMode> mode_;
  std::atomic<bool> stopping_{false};
  std::atomic<int> goals_received_{0}, goals_accepted_{0}, cancel_requests_{0};
  std::atomic<int> successes_{0}, aborts_{0};
  std::mutex mutex_, thread_mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> goal_poses_;
  std::vector<std::thread> workers_;
  rclcpp_action::Server<Navigate>::SharedPtr server_;
};

rclcpp::NodeOptions coordinator_options(const std::string & robot, const std::string & peer)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/" + robot});
  options.use_intra_process_comms(false);
  options.parameter_overrides({
    rclcpp::Parameter("robot_id", robot),
    rclcpp::Parameter("peer_robot_id", peer),
    rclcpp::Parameter("own_candidate_topic", "/" + robot + "/frontier_candidates"),
    rclcpp::Parameter("peer_claim_topic", "/cslam/" + peer + "/exploration_claim"),
    rclcpp::Parameter("own_claim_topic", "/cslam/" + robot + "/exploration_claim"),
    rclcpp::Parameter("navigate_to_pose_action", "/" + robot + "/navigate_to_pose"),
    rclcpp::Parameter("global_frame", "shared_map"),
    rclcpp::Parameter("arbitration_window_s", 0.80),
    rclcpp::Parameter("claim_heartbeat_rate_hz", 10.0),
    rclcpp::Parameter("claim_ttl_s", 0.60),
    rclcpp::Parameter("minimum_peer_ttl_s", 0.10),
    rclcpp::Parameter("maximum_peer_ttl_s", 2.0),
    rclcpp::Parameter("terminal_broadcast_duration_s", 0.20),
    rclcpp::Parameter("terminal_broadcast_rate_hz", 10.0),
    rclcpp::Parameter("require_fresh_candidate_age_s", 2.0),
    rclcpp::Parameter("navigation_goal_timeout_s", 0.80),
    rclcpp::Parameter("cancellation_ack_timeout_s", 0.30),
    rclcpp::Parameter("maximum_proposals_per_round", 3),
    rclcpp::Parameter("one_goal_only", true)});
  return options;
}

class NativeHarness : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    fixture_ = std::make_shared<rclcpp::Node>("native_coordinator_fixture");
    server1_ = std::make_shared<MockNavigateServer>("robot1", MockMode::SUCCEED);
    server2_ = std::make_shared<MockNavigateServer>("robot2", MockMode::SUCCEED);
    const auto candidate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto claim_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    candidate1_ = fixture_->create_publisher<CandidateArray>("/robot1/frontier_candidates", candidate_qos);
    candidate2_ = fixture_->create_publisher<CandidateArray>("/robot2/frontier_candidates", candidate_qos);
    peer_injector_ = fixture_->create_publisher<Claim>("/cslam/robot2/exploration_claim", claim_qos);
    robot1_claim_injector_ = fixture_->create_publisher<Claim>("/cslam/robot1/exploration_claim", claim_qos);
    action_probe1_ = rclcpp_action::create_client<Navigate>(fixture_, "/robot1/navigate_to_pose");
    action_probe2_ = rclcpp_action::create_client<Navigate>(fixture_, "/robot2/navigate_to_pose");
    claim1_ = fixture_->create_subscription<Claim>(
      "/cslam/robot1/exploration_claim", claim_qos,
      [this](Claim::ConstSharedPtr claim) {record(0, *claim);});
    claim2_ = fixture_->create_subscription<Claim>(
      "/cslam/robot2/exploration_claim", claim_qos,
      [this](Claim::ConstSharedPtr claim) {record(1, *claim);});
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 6);
    executor_->add_node(fixture_);
    executor_->add_node(server1_);
    executor_->add_node(server2_);
    spinner_ = std::thread([this]() {executor_->spin();});
    const auto ready_deadline = std::chrono::steady_clock::now() + 3000ms;
    while ((!action_probe1_->action_server_is_ready() || !action_probe2_->action_server_is_ready()) &&
      std::chrono::steady_clock::now() < ready_deadline) std::this_thread::sleep_for(10ms);
    if (!action_probe1_->action_server_is_ready() || !action_probe2_->action_server_is_ready()) {
      throw std::runtime_error("mock NavigateToPose servers were not ready before coordinator construction");
    }
    std::thread build_one([this]() {
      coordinator1_ = my_epuck_cooperative_exploration::make_coordinator(
        coordinator_options("robot1", "robot2"));
    });
    std::thread build_two([this]() {
      coordinator2_ = my_epuck_cooperative_exploration::make_coordinator(
        coordinator_options("robot2", "robot1"));
    });
    build_one.join();
    build_two.join();
    executor_->add_node(coordinator1_);
    executor_->add_node(coordinator2_);
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) spinner_.join();
    for (const auto & node : std::vector<rclcpp::Node::SharedPtr>{fixture_, server1_, server2_, coordinator1_, coordinator2_}) {
      executor_->remove_node(node);
    }
    coordinator1_.reset(); coordinator2_.reset(); fixture_.reset();
    server1_.reset(); server2_.reset(); executor_.reset();
  }

  void record(size_t robot, const Claim & claim)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    claims_[robot].push_back(claim);
  }

  template<class Predicate>
  bool wait_for(Predicate predicate, std::chrono::milliseconds timeout = 4000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) return true;
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  bool claim_seen(size_t robot, uint8_t state, uint64_t frontier = 0)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    for (const auto & claim : claims_[robot]) {
      if (claim.state == state && (frontier == 0 || claim.frontier_id == frontier)) return true;
    }
    return false;
  }

  std::vector<Claim> claim_log(size_t robot)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    return claims_[robot];
  }

  void publish_until(
    const CandidateArray & one, const CandidateArray & two,
    const std::function<bool()> & done, std::chrono::milliseconds timeout = 2500ms)
  {
    const auto ready_deadline = std::chrono::steady_clock::now() + 3000ms;
    while (std::chrono::steady_clock::now() < ready_deadline) {
      const auto one = my_epuck_cooperative_exploration::coordinator_diagnostics(coordinator1_);
      const auto two = my_epuck_cooperative_exploration::coordinator_diagnostics(coordinator2_);
      if (one.action_server_ready && two.action_server_ready) break;
      std::this_thread::sleep_for(10ms);
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      candidate1_->publish(one);
      candidate2_->publish(two);
      std::this_thread::sleep_for(20ms);
    }
  }

  bool endpoint_owned_by(const std::string & topic, const std::string & robot)
  {
    for (const auto & endpoint : fixture_->get_subscriptions_info_by_topic(topic)) {
      if (endpoint.node_name() == "cooperative_frontier_coordinator" &&
        endpoint.node_namespace() == "/" + robot) {
        const auto qos = endpoint.qos_profile();
        return qos.reliability() == rclcpp::ReliabilityPolicy::Reliable &&
               qos.durability() == rclcpp::DurabilityPolicy::Volatile;
      }
    }
    return false;
  }

  bool subscription_owned_by(const std::string & topic, const std::string & robot)
  {
    for (const auto & endpoint : fixture_->get_subscriptions_info_by_topic(topic)) {
      if (endpoint.node_name() == "cooperative_frontier_coordinator" &&
        endpoint.node_namespace() == "/" + robot) return true;
    }
    return false;
  }

  void publish_robot1_claim_until(
    Claim claim, const std::function<bool()> & done,
    std::chrono::milliseconds timeout = 2000ms)
  {
    ASSERT_TRUE(wait_for([this]() {
      return subscription_owned_by("/cslam/robot1/exploration_claim", "robot2");
    }));
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      robot1_claim_injector_->publish(claim);
      ++claim.message_revision;
      std::this_thread::sleep_for(40ms);
    }
  }

  void publish_claim_burst(
    const rclcpp::Publisher<Claim>::SharedPtr & publisher, Claim claim,
    const std::string & topic, const std::string & owner, size_t count = 5)
  {
    ASSERT_TRUE(wait_for([this, &topic, &owner]() {
      return subscription_owned_by(topic, owner);
    }));
    for (size_t i = 0; i < count; ++i) {
      publisher->publish(claim);
      ++claim.message_revision;
      std::this_thread::sleep_for(40ms);
    }
  }

  std::shared_ptr<rclcpp::Node> fixture_, coordinator1_, coordinator2_;
  std::shared_ptr<MockNavigateServer> server1_, server2_;
  rclcpp::Publisher<CandidateArray>::SharedPtr candidate1_, candidate2_;
  rclcpp::Publisher<Claim>::SharedPtr peer_injector_, robot1_claim_injector_;
  rclcpp_action::Client<Navigate>::SharedPtr action_probe1_, action_probe2_;
  rclcpp::Subscription<Claim>::SharedPtr claim1_, claim2_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
  std::mutex log_mutex_;
  std::vector<Claim> claims_[2];
};

TEST(FixtureValidation, ValidatesProductionRequirements)
{
  auto batch = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 0.8F)});
  EXPECT_TRUE(valid_fixture(batch, "robot1"));
  batch.source_robot_id = "robot2";
  EXPECT_FALSE(valid_fixture(batch, "robot1"));
  batch = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 0.8F)});
  batch.candidates[0].approach_pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(valid_fixture(batch, "robot1"));
}

TEST_F(NativeHarness, TransportGatePublishesProposingBeforeDispatch)
{
  ASSERT_TRUE(wait_for([this]() {
    return endpoint_owned_by("/robot1/frontier_candidates", "robot1") &&
           endpoint_owned_by("/robot2/frontier_candidates", "robot2");
  }));
  EXPECT_EQ(std::string(coordinator1_->get_fully_qualified_name()), "/robot1/cooperative_frontier_coordinator");
  EXPECT_EQ(std::string(coordinator2_->get_fully_qualified_name()), "/robot2/cooperative_frontier_coordinator");
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 0.70F)});
  auto two = make_batch("robot2", 7, {make_candidate(3003, -2.0, 1.0, 1.20F)});
  publish_until(one, two, [this]() {
    return claim_seen(0, Claim::PROPOSING) && claim_seen(1, Claim::PROPOSING);
  });
  const auto d1 = my_epuck_cooperative_exploration::coordinator_diagnostics(coordinator1_);
  const auto d2 = my_epuck_cooperative_exploration::coordinator_diagnostics(coordinator2_);
  EXPECT_GT(d1.raw_candidate_callbacks, 0U); EXPECT_GT(d1.accepted_candidate_batches, 0U);
  EXPECT_GT(d2.raw_candidate_callbacks, 0U); EXPECT_GT(d2.accepted_candidate_batches, 0U);
  ASSERT_TRUE(claim_seen(0, Claim::PROPOSING)); ASSERT_TRUE(claim_seen(1, Claim::PROPOSING));
  EXPECT_EQ(server1_->goals_received(), 0); EXPECT_EQ(server2_->goals_received(), 0);
  ASSERT_TRUE(wait_for([this]() {
    return server1_->goals_accepted() == 1 && server2_->goals_accepted() == 1;
  }));
  ASSERT_TRUE(wait_for([this]() {
    return claim_seen(0, Claim::SUCCEEDED) && claim_seen(1, Claim::SUCCEEDED);
  }));
  EXPECT_EQ(server1_->goals_accepted(), 1); EXPECT_EQ(server2_->goals_accepted(), 1);
}

TEST_F(NativeHarness, LowerCostPeerForcesReleaseThenAlternateProposal)
{
  auto empty = make_batch("robot1", 1, {});
  auto two = make_batch("robot2", 7, {
    make_candidate(2001, 1.02, 0.0, 1.20F),
    make_candidate(2002, 2.0, 0.0, 1.00F, 0.8F)});
  publish_until(empty, two, [this]() {return claim_seen(1, Claim::PROPOSING, 2001);});
  ASSERT_TRUE(claim_seen(1, Claim::PROPOSING, 2001));
  auto winner = make_peer_claim(8, 1, 0.70F);
  winner.source_robot_id = "robot1";
  publish_robot1_claim_until(
    winner, [this]() {return claim_seen(1, Claim::RELEASED, 2001);});
  ASSERT_TRUE(wait_for([this]() {return claim_seen(1, Claim::RELEASED, 2001);}));
  ASSERT_TRUE(wait_for([this]() {return claim_seen(1, Claim::PROPOSING, 2002);}));
  ASSERT_TRUE(wait_for([this]() {return server2_->goals_accepted() == 1;}));
  auto log = claim_log(1);
  uint64_t released_id = 0, alternate_id = 0;
  for (const auto & claim : log) {
    if (claim.state == Claim::RELEASED && claim.frontier_id == 2001) released_id = claim.claim_id;
    if (claim.state == Claim::PROPOSING && claim.frontier_id == 2002) alternate_id = claim.claim_id;
    EXPECT_NE(claim.state, Claim::UNKNOWN);
  }
  EXPECT_GT(alternate_id, released_id);
  EXPECT_EQ(server2_->goals_accepted(), 1);
}

TEST_F(NativeHarness, RobotIdTieBreakIsStable)
{
  auto empty = make_batch("robot1", 1, {});
  auto two = make_batch("robot2", 2, {make_candidate(2202, 1.01, 0.0, 1.01F)});
  publish_until(empty, two, [this]() {return claim_seen(1, Claim::PROPOSING);});
  ASSERT_TRUE(claim_seen(1, Claim::PROPOSING));
  auto tied = make_peer_claim(7, 1, 1.00F);
  tied.source_robot_id = "robot1";
  publish_robot1_claim_until(tied, [this]() {return claim_seen(1, Claim::RELEASED);});
  ASSERT_TRUE(wait_for([this]() {return claim_seen(1, Claim::RELEASED);}));
  EXPECT_EQ(server2_->goals_accepted(), 0);
}

TEST_F(NativeHarness, NonEquivalentFrontiersProceedIndependently)
{
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 0.9F)});
  auto two = make_batch("robot2", 1, {make_candidate(3003, -2.0, 1.0, 0.8F)});
  publish_until(one, two, [this]() {
    return claim_seen(0, Claim::PROPOSING) && claim_seen(1, Claim::PROPOSING);
  });
  ASSERT_TRUE(wait_for([this]() {
    return server1_->goals_accepted() == 1 && server2_->goals_accepted() == 1;
  }));
  ASSERT_TRUE(wait_for([this]() {
    return claim_seen(0, Claim::SUCCEEDED) && claim_seen(1, Claim::SUCCEEDED);
  }));
  EXPECT_EQ(server1_->goals_accepted(), 1);
  EXPECT_EQ(server2_->goals_accepted(), 1);
}

TEST_F(NativeHarness, DelayedWinningPeerClaimCancelsOnce)
{
  server1_->set_mode(MockMode::HOLD);
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 1.0F)});
  auto empty = make_batch("robot2", 1, {});
  publish_until(one, empty, [this]() {return server1_->goals_accepted() == 1;});
  ASSERT_EQ(server1_->goals_accepted(), 1);
  auto peer = make_peer_claim(9, 1, 0.4F);
  peer_injector_->publish(peer);
  ASSERT_TRUE(wait_for([this]() {return server1_->cancel_requests() == 1;}));
  ASSERT_TRUE(wait_for([this]() {return claim_seen(0, Claim::CANCELED);}));
  EXPECT_EQ(server1_->cancel_requests(), 1);
  EXPECT_EQ(server1_->goals_accepted(), 1);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(server1_->goals_accepted(), 1);
}

TEST_F(NativeHarness, InvalidDelayedClaimsDoNotCancel)
{
  server1_->set_mode(MockMode::HOLD);
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 1.0F)});
  auto empty = make_batch("robot2", 1, {});
  publish_until(one, empty, [this]() {return server1_->goals_accepted() == 1;});
  ASSERT_EQ(server1_->goals_accepted(), 1);
  auto invalid = make_peer_claim(9, 1, 0.4F);
  invalid.approach_pose.header.frame_id = "wrong";
  peer_injector_->publish(invalid);
  auto terminal = make_peer_claim(9, 2, 0.4F, Claim::RELEASED);
  peer_injector_->publish(terminal);
  auto wrong_source = make_peer_claim(9, 3, 0.4F);
  wrong_source.source_robot_id = "robot3";
  peer_injector_->publish(wrong_source);
  std::this_thread::sleep_for(350ms);
  EXPECT_EQ(server1_->cancel_requests(), 0);
  EXPECT_EQ(server1_->goals_accepted(), 1);
}

TEST_F(NativeHarness, NavigationTimeoutCancellationIsTerminal)
{
  server1_->set_mode(MockMode::HOLD);
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 1.0F)});
  auto empty = make_batch("robot2", 1, {});
  publish_until(one, empty, [this]() {return server1_->goals_accepted() == 1;});
  ASSERT_EQ(server1_->goals_accepted(), 1);
  ASSERT_TRUE(wait_for([this]() {return server1_->cancel_requests() == 1;}, 2000ms));
  ASSERT_TRUE(wait_for([this]() {return claim_seen(0, Claim::CANCELED);}, 1500ms));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(server1_->cancel_requests(), 1);
  EXPECT_EQ(server1_->goals_accepted(), 1);
}

TEST_F(NativeHarness, ActionRejectionIsTerminal)
{
  server1_->set_mode(MockMode::REJECT);
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 1.0F)});
  auto empty = make_batch("robot2", 1, {});
  publish_until(one, empty, [this]() {return claim_seen(0, Claim::FAILED);});
  ASSERT_TRUE(claim_seen(0, Claim::FAILED));
  EXPECT_EQ(server1_->goals_received(), 1);
  EXPECT_EQ(server1_->goals_accepted(), 0);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(server1_->goals_received(), 1);
}

TEST_F(NativeHarness, NonzeroResultIsFailedAndSingleGoal)
{
  server1_->set_mode(MockMode::NONZERO_ERROR);
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 1.0F)});
  auto empty = make_batch("robot2", 1, {});
  publish_until(one, empty, [this]() {return claim_seen(0, Claim::FAILED);});
  ASSERT_TRUE(claim_seen(0, Claim::FAILED));
  EXPECT_EQ(server1_->goals_accepted(), 1);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(server1_->goals_accepted(), 1);
}

TEST_F(NativeHarness, RobotIdTieBreakIsStableWithPeerDeliveredFirst)
{
  auto tied = make_peer_claim(17, 1, 1.00F);
  tied.source_robot_id = "robot1";
  publish_claim_burst(
    robot1_claim_injector_, tied, "/cslam/robot1/exploration_claim", "robot2");
  auto empty = make_batch("robot1", 1, {});
  auto two = make_batch("robot2", 33, {make_candidate(2202, 1.01, 0.0, 1.01F)});
  publish_until(empty, two, [this]() {
    const auto diagnostics = my_epuck_cooperative_exploration::coordinator_diagnostics(coordinator2_);
    return diagnostics.accepted_candidate_batches > 0;
  });
  std::this_thread::sleep_for(1000ms);
  EXPECT_EQ(server2_->goals_received(), 0);
}

TEST_F(NativeHarness, PeerClaimExpiresByLocalReceiveTime)
{
  auto peer = make_peer_claim(18, 1, 0.40F);
  publish_claim_burst(
    peer_injector_, peer, "/cslam/robot2/exploration_claim", "robot1", 3);
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 1.0F)});
  auto empty = make_batch("robot2", 1, {});
  publish_until(one, empty, [this]() {
    const auto diagnostics = my_epuck_cooperative_exploration::coordinator_diagnostics(coordinator1_);
    return diagnostics.accepted_candidate_batches > 0;
  });
  std::this_thread::sleep_for(250ms);
  EXPECT_FALSE(claim_seen(0, Claim::PROPOSING));
  ASSERT_TRUE(wait_for([this]() {return claim_seen(0, Claim::PROPOSING);}, 1500ms));
  ASSERT_TRUE(wait_for([this]() {return server1_->goals_accepted() == 1;}, 2000ms));
}

TEST_F(NativeHarness, CancellationAcknowledgementTimeoutIsFailedAndTerminal)
{
  server1_->set_mode(MockMode::SLOW_CANCEL);
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 1.0F)});
  auto empty = make_batch("robot2", 1, {});
  publish_until(one, empty, [this]() {return server1_->goals_accepted() == 1;});
  ASSERT_EQ(server1_->goals_accepted(), 1);
  ASSERT_TRUE(wait_for([this]() {return server1_->cancel_requests() == 1;}, 2000ms));
  ASSERT_TRUE(wait_for([this]() {return claim_seen(0, Claim::FAILED);}, 1500ms));
  std::this_thread::sleep_for(900ms);
  EXPECT_EQ(server1_->cancel_requests(), 1);
  EXPECT_EQ(server1_->goals_accepted(), 1);
}

TEST_F(NativeHarness, ActionAbortIsFailedAndSingleGoal)
{
  server1_->set_mode(MockMode::ABORT);
  auto one = make_batch("robot1", 1, {make_candidate(1001, 1.0, 0.0, 1.0F)});
  auto empty = make_batch("robot2", 1, {});
  publish_until(one, empty, [this]() {return claim_seen(0, Claim::FAILED);});
  ASSERT_TRUE(claim_seen(0, Claim::FAILED));
  EXPECT_EQ(server1_->goals_accepted(), 1);
  EXPECT_EQ(server1_->aborts(), 1);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(server1_->goals_accepted(), 1);
}

TEST_F(NativeHarness, GraphIsolationUsesOnlyConfiguredRobotEndpoints)
{
  ASSERT_TRUE(wait_for([this]() {
    return endpoint_owned_by("/robot1/frontier_candidates", "robot1") &&
           endpoint_owned_by("/robot2/frontier_candidates", "robot2") &&
           subscription_owned_by("/cslam/robot2/exploration_claim", "robot1") &&
           subscription_owned_by("/cslam/robot1/exploration_claim", "robot2");
  }));
  EXPECT_FALSE(subscription_owned_by("/robot2/frontier_candidates", "robot1"));
  EXPECT_FALSE(subscription_owned_by("/robot1/frontier_candidates", "robot2"));
  for (const auto & endpoint : fixture_->get_publishers_info_by_topic("/robot1/cmd_vel")) {
    EXPECT_NE(endpoint.node_name(), "cooperative_frontier_coordinator");
  }
  for (const auto & endpoint : fixture_->get_publishers_info_by_topic("/robot2/cmd_vel")) {
    EXPECT_NE(endpoint.node_name(), "cooperative_frontier_coordinator");
  }
}

}  // namespace
