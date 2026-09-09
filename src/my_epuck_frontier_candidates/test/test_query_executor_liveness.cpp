#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
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
