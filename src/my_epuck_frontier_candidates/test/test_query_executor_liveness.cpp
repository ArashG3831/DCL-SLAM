#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace {

using Action = nav2_msgs::action::ComputePathToPose;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Action>;

class PendingPlannerNode : public rclcpp::Node
{
public:
  PendingPlannerNode(const std::string & action_name, bool create_server)
  : Node("candidate_query_executor_liveness")
  {
    planner_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    watchdog_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    planner_ = rclcpp_action::create_client<Action>(this, action_name, planner_group_);

    if (create_server) {
      server_ = rclcpp_action::create_server<Action>(
        this, action_name,
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<ServerGoalHandle> &) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<ServerGoalHandle> & handle) {
          server_goal_ = handle;
          // Deliberately do not publish a result. This models an accepted
          // ComputePathToPose request whose terminal result is lost.
        });
    }

    submit_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this] {submit_query();}, planner_group_);
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this] {watchdog_tick();}, watchdog_group_);
  }

  std::shared_future<void> completion() {return completion_.get_future().share();}
  bool submitted() const {return submitted_.load();}
  bool timed_out() const {return timed_out_.load();}

private:
  void submit_query()
  {
    if (submitted_.exchange(true)) {
      return;
    }
    if (submit_timer_) {
      submit_timer_->cancel();
    }
    Action::Goal goal;
    goal.goal.header.frame_id = "shared_map";
    planner_->async_send_goal(goal);
  }

  void watchdog_tick()
  {
    if (!submitted_.load()) {
      return;
    }
    timed_out_.store(true);
    try {
      completion_.set_value();
    } catch (const std::future_error &) {
    }
    if (watchdog_timer_) {
      watchdog_timer_->cancel();
    }
  }

  rclcpp::CallbackGroup::SharedPtr planner_group_, watchdog_group_;
  rclcpp_action::Client<Action>::SharedPtr planner_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::shared_ptr<ServerGoalHandle> server_goal_;
  rclcpp::TimerBase::SharedPtr submit_timer_, watchdog_timer_;
  std::promise<void> completion_;
  std::atomic<bool> submitted_{false}, timed_out_{false};
};

class TimeoutRetryOwnershipNode : public rclcpp::Node
{
public:
  TimeoutRetryOwnershipNode()
  : Node("candidate_query_timeout_retry_ownership")
  {
    watchdog_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    retry_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    start_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this] {start_first_query();}, retry_group_);
  }

  std::shared_future<void> completion() {return completion_.get_future().share();}

private:
  void start_first_query()
  {
    if (started_) {
      return;
    }
    started_ = true;
    start_timer_->cancel();
    arm_watchdog(1, std::chrono::milliseconds(40));
  }

  void arm_watchdog(uint64_t request, std::chrono::milliseconds delay)
  {
    auto timer = create_wall_timer(
      delay,
      [this, request] {
        std::lock_guard<std::mutex> lock(mu_);
        if (request != watchdog_request_) {
          return;
        }
        if (watchdog_timer_) {
          watchdog_timer_->cancel();
          watchdog_timer_.reset();
        }
        watchdog_request_ = 0;
        if (request == 1) {
          schedule_retry();
          return;
        }
        try {
          completion_.set_value();
        } catch (const std::future_error &) {
        }
      },
      watchdog_group_);
    std::lock_guard<std::mutex> lock(mu_);
    watchdog_timer_ = std::move(timer);
    watchdog_request_ = request;
  }

  void schedule_retry()
  {
    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      [this] {
        retry_timer_->cancel();
        retry_timer_.reset();
        arm_watchdog(2, std::chrono::milliseconds(80));
        // Model a late cleanup callback belonging to request 1 arriving after
        // request 2 has installed its watchdog.
        late_cleanup_timer_ = create_wall_timer(
          std::chrono::milliseconds(10), [this] {cancel_watchdog_owned_by(1);}, retry_group_);
      },
      retry_group_);
  }

  void cancel_watchdog_owned_by(uint64_t request)
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (request != watchdog_request_) {
      return;
    }
    if (watchdog_timer_) {
      watchdog_timer_->cancel();
      watchdog_timer_.reset();
    }
    watchdog_request_ = 0;
  }

  rclcpp::CallbackGroup::SharedPtr watchdog_group_, retry_group_;
  rclcpp::TimerBase::SharedPtr start_timer_, watchdog_timer_, retry_timer_, late_cleanup_timer_;
  std::mutex mu_;
  uint64_t watchdog_request_{0};
  bool started_{false};
  std::promise<void> completion_;
};

void run_executor_liveness_case(const std::string & action_name, bool server)
{
  auto node = std::make_shared<PendingPlannerNode>(action_name, server);
  auto completion = node->completion();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&executor] {executor.spin();});

  ASSERT_EQ(completion.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(node->submitted());
  EXPECT_TRUE(node->timed_out());

  executor.cancel();
  spin_thread.join();
}

}  // namespace

TEST(CandidateExecutorLiveness, NoGoalResponseWatchdogRunsOnExecutor)
{
  rclcpp::init(0, nullptr);
  run_executor_liveness_case("/fixb_executor_no_goal_response", false);
  rclcpp::shutdown();
}

TEST(CandidateExecutorLiveness, AcceptedGoalWithoutResultWatchdogRunsOnExecutor)
{
  rclcpp::init(0, nullptr);
  run_executor_liveness_case("/fixb_executor_no_result", true);
  rclcpp::shutdown();
}

TEST(CandidateExecutorLiveness, TimeoutRetryCannotCancelSuccessorWatchdog)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<TimeoutRetryOwnershipNode>();
  auto completion = node->completion();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&executor] {executor.spin();});

  EXPECT_EQ(completion.wait_for(std::chrono::seconds(3)), std::future_status::ready);

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
}
