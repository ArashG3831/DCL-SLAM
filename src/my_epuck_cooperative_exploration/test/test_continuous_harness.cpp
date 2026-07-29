#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <my_epuck_interfaces/msg/exploration_claim.hpp>
#include <my_epuck_interfaces/msg/exploration_event.hpp>
#include <my_epuck_interfaces/msg/exploration_status.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "my_epuck_cooperative_exploration/coordinator.hpp"

using namespace std::chrono_literals;
using Candidate = my_epuck_interfaces::msg::FrontierCandidate;
using CandidateArray = my_epuck_interfaces::msg::FrontierCandidateArray;
using Claim = my_epuck_interfaces::msg::ExplorationClaim;
using Event = my_epuck_interfaces::msg::ExplorationEvent;
using Status = my_epuck_interfaces::msg::ExplorationStatus;
using Navigate = nav2_msgs::action::NavigateToPose;

namespace
{

Candidate candidate(uint64_t id, double x, double y, float path)
{
  Candidate value;
  value.frontier_id = id;
  value.centroid.x = x;
  value.centroid.y = y;
  value.bounding_box_min.x = x - 0.1;
  value.bounding_box_min.y = y - 0.1;
  value.bounding_box_max.x = x + 0.1;
  value.bounding_box_max.y = y + 0.1;
  value.approach_pose.header.frame_id = "shared_map";
  value.approach_pose.pose.position.x = x - 0.2;
  value.approach_pose.pose.position.y = y;
  value.approach_pose.pose.orientation.w = 1.0;
  value.path_length_m = path;
  value.information_gain = 0.2F;
  value.score = 1.0F;
  value.reachability_state = Candidate::REACHABLE;
  return value;
}

CandidateArray batch(
  const std::string & robot, uint64_t revision, std::vector<Candidate> values)
{
  CandidateArray result;
  result.header.frame_id = "shared_map";
  result.source_robot_id = robot;
  result.map_revision = revision;
  result.candidates = std::move(values);
  return result;
}

nav_msgs::msg::OccupancyGrid stable_map()
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "shared_map";
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 0.1F;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(100, 0);
  return map;
}

class MockNavigateServer : public rclcpp::Node
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<Navigate>;

  explicit MockNavigateServer(const std::string & robot)
  : Node(robot + "_continuous_mock")
  {
    server_ = rclcpp_action::create_server<Navigate>(
      this, "/" + robot + "/navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal) {
        std::lock_guard<std::mutex> lock(mutex_);
        poses_.push_back(goal->pose);
        ++received_;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<Handle>) {
        ++cancels_;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<Handle> handle) {
        ++accepted_;
        std::lock_guard<std::mutex> lock(worker_mutex_);
        workers_.emplace_back([this, handle]() {
          std::this_thread::sleep_for(50ms);
          if (handle->is_canceling()) {
            handle->canceled(std::make_shared<Navigate::Result>());
            return;
          }
          auto result = std::make_shared<Navigate::Result>();
          if (abort_next_.exchange(false)) {
            handle->abort(result);  // Deliberately aborted with error_code zero.
            ++failed_;
          } else {
            handle->succeed(result);
            ++succeeded_;
          }
        });
      });
  }

  ~MockNavigateServer() override
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  void abort_next() {abort_next_ = true;}
  int accepted() const {return accepted_.load();}
  int succeeded() const {return succeeded_.load();}
  int failed() const {return failed_.load();}
  int cancels() const {return cancels_.load();}
  std::vector<geometry_msgs::msg::PoseStamped> poses() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return poses_;
  }

private:
  std::atomic<bool> abort_next_{false};
  std::atomic<int> received_{0}, accepted_{0}, succeeded_{0}, failed_{0}, cancels_{0};
  mutable std::mutex mutex_, worker_mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::vector<std::thread> workers_;
  rclcpp_action::Server<Navigate>::SharedPtr server_;
};

rclcpp::NodeOptions options(const std::string & robot, const std::string & peer)
{
  rclcpp::NodeOptions value;
  value.arguments({"--ros-args", "-r", "__ns:=/" + robot});
  value.parameter_overrides({
    rclcpp::Parameter("robot_id", robot),
    rclcpp::Parameter("peer_robot_id", peer),
    rclcpp::Parameter("operating_mode", "continuous"),
    rclcpp::Parameter("one_goal_only", false),
    rclcpp::Parameter("own_candidate_topic", "/" + robot + "/frontier_candidates"),
    rclcpp::Parameter("own_shared_map_topic", "/" + robot + "/shared_map"),
    rclcpp::Parameter("peer_claim_topic", "/cslam/" + peer + "/exploration_claim"),
    rclcpp::Parameter("own_claim_topic", "/cslam/" + robot + "/exploration_claim"),
    rclcpp::Parameter("peer_status_topic", "/cslam/" + peer + "/exploration_status"),
    rclcpp::Parameter("own_status_topic", "/cslam/" + robot + "/exploration_status"),
    rclcpp::Parameter("own_event_topic", "/cslam/" + robot + "/exploration_event"),
    rclcpp::Parameter("navigate_to_pose_action", "/" + robot + "/navigate_to_pose"),
    rclcpp::Parameter("global_frame", "shared_map"),
    rclcpp::Parameter("arbitration_window_s", 0.15),
    rclcpp::Parameter("claim_heartbeat_rate_hz", 20.0),
    rclcpp::Parameter("claim_ttl_s", 0.30),
    rclcpp::Parameter("status_heartbeat_rate_hz", 20.0),
    rclcpp::Parameter("status_ttl_s", 0.30),
    rclcpp::Parameter("minimum_peer_ttl_s", 0.05),
    rclcpp::Parameter("maximum_peer_ttl_s", 1.0),
    rclcpp::Parameter("terminal_broadcast_duration_s", 0.08),
    rclcpp::Parameter("terminal_broadcast_rate_hz", 20.0),
    rclcpp::Parameter("cycle_cooldown_s", 0.10),
    rclcpp::Parameter("success_region_cooldown_s", 0.40),
    rclcpp::Parameter("first_failure_suppression_s", 0.50),
    rclcpp::Parameter("second_failure_suppression_s", 0.80),
    rclcpp::Parameter("maximum_failure_suppression_s", 1.0),
    rclcpp::Parameter("peer_failure_caution_s", 0.15),
    rclcpp::Parameter("arbitration_loss_cooldown_s", 0.20),
    rclcpp::Parameter("require_fresh_candidate_age_s", 2.0),
    rclcpp::Parameter("require_fresh_shared_map_age_s", 2.0),
    rclcpp::Parameter("navigation_goal_timeout_s", 1.0),
    rclcpp::Parameter("cancellation_ack_timeout_s", 0.20),
    rclcpp::Parameter("no_candidate_grace_s", 0.25),
    rclcpp::Parameter("map_stability_window_s", 0.25),
    rclcpp::Parameter("minimum_known_cell_gain_for_activity", 1),
    rclcpp::Parameter("completion_consensus_grace_s", 0.35)});
  return value;
}

class ContinuousHarness : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    fixture_ = std::make_shared<rclcpp::Node>("continuous_fixture");
    server1_ = std::make_shared<MockNavigateServer>("robot1");
    server2_ = std::make_shared<MockNavigateServer>("robot2");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 8);
    executor_->add_node(fixture_);
    executor_->add_node(server1_);
    executor_->add_node(server2_);
    spinner_ = std::thread([this]() {executor_->spin();});
    candidate1_ = fixture_->create_publisher<CandidateArray>(
      "/robot1/frontier_candidates", rclcpp::QoS(1).reliable());
    candidate2_ = fixture_->create_publisher<CandidateArray>(
      "/robot2/frontier_candidates", rclcpp::QoS(1).reliable());
    map1_ = fixture_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/robot1/shared_map", rclcpp::QoS(1).reliable().transient_local());
    map2_ = fixture_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/robot2/shared_map", rclcpp::QoS(1).reliable().transient_local());
    const auto event_qos = rclcpp::QoS(50).reliable();
    event1_ = fixture_->create_subscription<Event>(
      "/cslam/robot1/exploration_event", event_qos,
      [this](Event::ConstSharedPtr value) {record(0, *value);});
    event2_ = fixture_->create_subscription<Event>(
      "/cslam/robot2/exploration_event", event_qos,
      [this](Event::ConstSharedPtr value) {record(1, *value);});
    start_coordinators();
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    for (const auto & node : std::vector<rclcpp::Node::SharedPtr>{
        coordinator1_, coordinator2_, fixture_, server1_, server2_})
    {
      if (node) {
        executor_->remove_node(node);
      }
    }
    coordinator1_.reset();
    coordinator2_.reset();
    fixture_.reset();
    server1_.reset();
    server2_.reset();
    executor_.reset();
  }

  void start_coordinators()
  {
    coordinator1_ =
      my_epuck_cooperative_exploration::make_coordinator(options("robot1", "robot2"));
    coordinator2_ =
      my_epuck_cooperative_exploration::make_coordinator(options("robot2", "robot1"));
    executor_->add_node(coordinator1_);
    executor_->add_node(coordinator2_);
  }

  void stop_robot2_coordinator()
  {
    executor_->remove_node(coordinator2_);
    coordinator2_.reset();
  }

  void restart_robot2_coordinator()
  {
    coordinator2_ =
      my_epuck_cooperative_exploration::make_coordinator(options("robot2", "robot1"));
    executor_->add_node(coordinator2_);
  }

  void record(size_t robot, const Event & event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    events_[robot].push_back(event);
  }

  bool seen(size_t robot, const std::string & type)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(
      events_[robot].begin(), events_[robot].end(),
      [&type](const Event & event) {return event.event_type == type;});
  }

  size_t count(size_t robot, const std::string & type)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::count_if(
      events_[robot].begin(), events_[robot].end(),
      [&type](const Event & event) {return event.event_type == type;});
  }

  template<typename Predicate>
  bool wait_for(Predicate predicate, std::chrono::milliseconds timeout = 5000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  void publish(
    const CandidateArray & one, const CandidateArray & two,
    const std::function<bool()> & done,
    std::chrono::milliseconds timeout = 2500ms)
  {
    auto map = stable_map();
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      map.header.stamp = fixture_->now();
      map1_->publish(map);
      map2_->publish(map);
      candidate1_->publish(one);
      candidate2_->publish(two);
      std::this_thread::sleep_for(20ms);
    }
  }

  std::shared_ptr<rclcpp::Node> fixture_, coordinator1_, coordinator2_;
  std::shared_ptr<MockNavigateServer> server1_, server2_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
  rclcpp::Publisher<CandidateArray>::SharedPtr candidate1_, candidate2_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map1_, map2_;
  rclcpp::Subscription<Event>::SharedPtr event1_, event2_;
  std::mutex mutex_;
  std::vector<Event> events_[2];
};

TEST_F(ContinuousHarness, A_SeparateFrontiersCompleteThreeCyclesEach)
{
  for (uint64_t cycle = 1; cycle <= 3; ++cycle) {
    publish(
      batch("robot1", cycle, {candidate(100 + cycle, cycle, 0.0, 0.4F)}),
      batch("robot2", cycle, {candidate(200 + cycle, -double(cycle), 0.0, 0.4F)}),
      [this, cycle]() {
        return server1_->succeeded() >= int(cycle) &&
               server2_->succeeded() >= int(cycle);
      });
    ASSERT_GE(server1_->succeeded(), int(cycle));
    ASSERT_GE(server2_->succeeded(), int(cycle));
  }
  EXPECT_EQ(server1_->accepted(), 3);
  EXPECT_EQ(server2_->accepted(), 3);
  EXPECT_EQ(count(0, "EXPLORATION_CYCLE_STARTED"), 3U);
  EXPECT_EQ(count(1, "EXPLORATION_CYCLE_STARTED"), 3U);
}

TEST_F(ContinuousHarness, B_EquivalentFirstGoalHasOneWinnerAndLoserContinues)
{
  publish(
    batch("robot1", 1, {candidate(101, 1.0, 0.0, 0.4F)}),
    batch("robot2", 1, {
      candidate(201, 1.02, 0.0, 0.8F), candidate(202, -1.0, 0.0, 0.6F)}),
    [this]() {return server1_->succeeded() >= 1 && server2_->succeeded() >= 1;});
  ASSERT_EQ(server1_->succeeded(), 1);
  ASSERT_EQ(server2_->succeeded(), 1);
  ASSERT_FALSE(server2_->poses().empty());
  EXPECT_LT(server2_->poses().front().pose.position.x, 0.0);

  for (uint64_t cycle = 2; cycle <= 3; ++cycle) {
    publish(
      batch("robot1", cycle, {candidate(110 + cycle, cycle, 1.0, 0.4F)}),
      batch("robot2", cycle, {candidate(210 + cycle, -double(cycle), 1.0, 0.4F)}),
      [this, cycle]() {
        return server1_->succeeded() >= int(cycle) &&
               server2_->succeeded() >= int(cycle);
      });
  }
  EXPECT_EQ(server1_->succeeded(), 3);
  EXPECT_EQ(server2_->succeeded(), 3);
}

TEST_F(ContinuousHarness, C_LocalFailureSuppressesRegionAndSelectsDifferentFrontier)
{
  server1_->abort_next();
  auto empty = batch("robot2", 1, {});
  publish(
    batch("robot1", 1, {candidate(301, 1.0, 0.0, 0.3F)}), empty,
    [this]() {return server1_->failed() == 1;});
  ASSERT_EQ(server1_->failed(), 1);
  publish(
    batch("robot1", 2, {
      candidate(301, 1.0, 0.0, 0.2F), candidate(302, 2.0, 0.0, 0.5F)}),
    batch("robot2", 2, {}),
    [this]() {return server1_->succeeded() == 1;});
  ASSERT_EQ(server1_->succeeded(), 1);
  const auto poses = server1_->poses();
  ASSERT_EQ(poses.size(), 2U);
  EXPECT_GT(poses[1].pose.position.x, 1.5);
  EXPECT_TRUE(seen(0, "FAILURE_SUPPRESSION_CREATED"));
  EXPECT_TRUE(seen(0, "CANDIDATE_SKIPPED_SUPPRESSED"));
}

TEST_F(ContinuousHarness, D_PeerExpiryAllowsProgressAndRestartRejoins)
{
  stop_robot2_coordinator();
  publish(
    batch("robot1", 1, {candidate(401, 1.0, 0.0, 0.4F)}),
    batch("robot2", 1, {}),
    [this]() {return server1_->succeeded() == 1;});
  ASSERT_EQ(server1_->succeeded(), 1);

  restart_robot2_coordinator();
  publish(
    batch("robot1", 2, {candidate(402, 2.0, 0.0, 0.4F)}),
    batch("robot2", 2, {candidate(403, -2.0, 0.0, 0.4F)}),
    [this]() {return server1_->succeeded() == 2 && server2_->succeeded() == 1;});
  EXPECT_EQ(server1_->succeeded(), 2);
  EXPECT_EQ(server2_->succeeded(), 1);
  EXPECT_TRUE(seen(0, "PEER_STATUS_CHANGED"));
}

TEST_F(ContinuousHarness, E_StableEmptyPeersReachMissionComplete)
{
  publish(
    batch("robot1", 1, {}), batch("robot2", 1, {}),
    [this]() {
      return my_epuck_cooperative_exploration::coordinator_diagnostics(
        coordinator1_).mission_complete &&
             my_epuck_cooperative_exploration::coordinator_diagnostics(
        coordinator2_).mission_complete;
    },
    4000ms);
  EXPECT_TRUE(
    my_epuck_cooperative_exploration::coordinator_diagnostics(
      coordinator1_).mission_complete);
  EXPECT_TRUE(
    my_epuck_cooperative_exploration::coordinator_diagnostics(
      coordinator2_).mission_complete);
  EXPECT_EQ(server1_->accepted(), 0);
  EXPECT_EQ(server2_->accepted(), 0);
}

TEST_F(ContinuousHarness, F_NewCandidateCancelsCompletionConsensusAndResumes)
{
  publish(
    batch("robot1", 1, {}), batch("robot2", 1, {}),
    [this]() {
      return seen(0, "COMPLETION_CONSENSUS_STARTED") &&
             seen(1, "COMPLETION_CONSENSUS_STARTED");
    },
    3000ms);
  ASSERT_TRUE(seen(0, "COMPLETION_CONSENSUS_STARTED"));
  ASSERT_TRUE(seen(1, "COMPLETION_CONSENSUS_STARTED"));
  publish(
    batch("robot1", 2, {candidate(501, 1.0, 1.0, 0.4F)}),
    batch("robot2", 2, {candidate(502, -1.0, -1.0, 0.4F)}),
    [this]() {return server1_->succeeded() == 1 && server2_->succeeded() == 1;});
  EXPECT_EQ(server1_->succeeded(), 1);
  EXPECT_EQ(server2_->succeeded(), 1);
  EXPECT_TRUE(seen(0, "EXPLORATION_RESUMED"));
  EXPECT_TRUE(seen(1, "EXPLORATION_RESUMED"));
}

}  // namespace
