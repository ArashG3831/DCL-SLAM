#include <functional>
#include <memory>

#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "slam_toolbox/slam_toolbox_async.hpp"
#include "tf2_ros/message_filter.h"

namespace reliable_slam_toolbox_wrapper
{

class ReliableAsynchronousSlamToolbox final
  : public slam_toolbox::AsynchronousSlamToolbox
{
public:
  explicit ReliableAsynchronousSlamToolbox(const rclcpp::NodeOptions & options)
  : slam_toolbox::AsynchronousSlamToolbox(options) {}

protected:
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    const auto result = slam_toolbox::AsynchronousSlamToolbox::on_activate(state);
    if (result != CallbackReturn::SUCCESS) {
      return result;
    }

    // Slam Toolbox 2.8.3 constructs its sensor subscription with the fixed
    // best-effort SensorDataQoS profile.  The WSL CycloneDDS loopback path
    // drops fragmented best-effort 720-reading scans, while the project scan
    // fixer already publishes the same sample reliably.  Recreate only this
    // subscription with reliable QoS; all mapping, TF, and lifecycle logic
    // remains in the upstream library.
    scan_filter_.reset();
    scan_filter_sub_.reset();
    auto qos = rmw_qos_profile_default;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos.depth = 100;
    scan_filter_sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp_lifecycle::LifecycleNode>>(
      shared_from_this().get(), scan_topic_, qos);
    scan_filter_ =
      std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      *scan_filter_sub_, *tf_, odom_frame_, scan_queue_size_,
      get_node_logging_interface(), get_node_clock_interface(),
      tf2::durationFromSec(transform_timeout_.seconds()));
    scan_filter_->registerCallback(
      std::bind(&ReliableAsynchronousSlamToolbox::laserCallback, this,
      std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Using reliable corrected LaserScan subscription");
    return result;
  }
};

}  // namespace reliable_slam_toolbox_wrapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<
    reliable_slam_toolbox_wrapper::ReliableAsynchronousSlamToolbox>(
    rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
