#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <my_epuck_interfaces/msg/scan_chunk.hpp>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace my_epuck_frontier_candidates {

class ChunkedLidarPlugin final : public webots_ros2_driver::PluginInterface {
public:
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override
  {
    node_ = node;
    robot_id_ = parameter(parameters, "robot_id", "");
    topic_ = parameter(parameters, "topic", "scan_d500_chunks");
    frame_id_ = parameter(parameters, "frame_id", "");
    chunk_beams_ = std::max<unsigned>(1U, to_uint(parameter(
      parameters, "chunk_beams", "180"), 180U));
    update_rate_hz_ = to_double(parameter(parameters, "update_rate", "0"));
    lidar_name_ = parameter(parameters, "lidar_name", "d500_lidar");
    lidar_ = wb_robot_get_device(lidar_name_.c_str());
    if (!lidar_) {
      RCLCPP_ERROR(node_->get_logger(), "Chunked lidar device '%s' not found",
                   lidar_name_.c_str());
      return;
    }
    const int timestep = wb_robot_get_basic_time_step();
    wb_lidar_enable(lidar_, timestep > 0 ? timestep : 20);
    // Chunk samples are each below the observed DDS fragmentation boundary;
    // reliable delivery is safe here and prevents losing one of four chunks
    // while retaining a complete 720-beam scan atomically.
    publisher_ = node_->create_publisher<my_epuck_interfaces::msg::ScanChunk>(
      topic_, rclcpp::QoS(rclcpp::KeepLast(8)).reliable());
    RCLCPP_INFO(node_->get_logger(),
                "Chunked lidar transport active: %s -> %s (%u beams/chunk)",
                lidar_name_.c_str(), topic_.c_str(), chunk_beams_);
  }

  void step() override
  {
    if (!node_ || !lidar_ || !publisher_) {
      return;
    }
    const double simulation_time = wb_robot_get_time();
    if (update_rate_hz_ > 0.0 && last_publish_time_ >= 0.0 &&
        simulation_time - last_publish_time_ < 1.0 / update_rate_hz_) {
      return;
    }
    // Number of range-image samples does not require Webots point-cloud
    // mode; using get_number_of_points() would implicitly request that mode
    // and emit an error when the stock point-cloud publisher is disabled.
    const int count = wb_lidar_get_horizontal_resolution(lidar_) *
      wb_lidar_get_number_of_layers(lidar_);
    const float *image = wb_lidar_get_range_image(lidar_);
    if (count <= 0 || !image) {
      return;
    }
    const unsigned total_chunks = static_cast<unsigned>(
      (count + static_cast<int>(chunk_beams_) - 1) /
      static_cast<int>(chunk_beams_));
    const double fov = wb_lidar_get_fov(lidar_);
    const float angle_min = static_cast<float>(-0.5 * fov);
    const float angle_max = static_cast<float>(0.5 * fov);
    const float increment = count > 1
      ? static_cast<float>(fov / static_cast<double>(count - 1)) : 0.0F;
    const float scan_time = static_cast<float>(
      wb_robot_get_basic_time_step() / 1000.0);
    const auto stamp = node_->get_clock()->now();
    const std::string frame = frame_id_.empty() ?
      (robot_id_.empty() ? "d500_lidar" : robot_id_ + "/d500_lidar") : frame_id_;
    ++sequence_;
    last_publish_time_ = simulation_time;
    for (unsigned chunk = 0; chunk < total_chunks; ++chunk) {
      const unsigned begin = chunk * chunk_beams_;
      const unsigned end = std::min<unsigned>(
        static_cast<unsigned>(count), begin + chunk_beams_);
      auto message = my_epuck_interfaces::msg::ScanChunk();
      message.header.stamp = stamp;
      message.header.frame_id = frame;
      message.source_robot_id = robot_id_;
      message.scan_sequence = sequence_;
      message.total_chunks = static_cast<uint16_t>(total_chunks);
      message.chunk_index = static_cast<uint16_t>(chunk);
      message.total_beams = static_cast<uint32_t>(count);
      message.angle_min = angle_min;
      message.angle_max = angle_max;
      message.angle_increment = increment;
      message.time_increment = count > 0 ? scan_time / count : 0.0F;
      message.scan_time = scan_time;
      message.range_min = static_cast<float>(wb_lidar_get_min_range(lidar_));
      message.range_max = static_cast<float>(wb_lidar_get_max_range(lidar_));
      message.ranges.assign(image + begin, image + end);
      message.checksum = checksum(message.ranges, sequence_, chunk);
      publisher_->publish(message);
    }
  }

private:
  static std::string parameter(
    const std::unordered_map<std::string, std::string> &parameters,
    const std::string &key, const std::string &fallback)
  {
    const auto it = parameters.find(key);
    return it == parameters.end() ? fallback : it->second;
  }

  static unsigned to_uint(const std::string &value, unsigned fallback)
  {
    try {
      return static_cast<unsigned>(std::stoul(value));
    } catch (...) {
      return fallback;
    }
  }

  static double to_double(const std::string &value)
  {
    try {
      return std::stod(value);
    } catch (...) {
      return 0.0;
    }
  }

  static uint32_t checksum(const std::vector<float> &values,
                           uint32_t sequence, unsigned chunk)
  {
    uint32_t hash = 2166136261u ^ sequence ^ static_cast<uint32_t>(chunk);
    for (const float value : values) {
      uint32_t bits = 0;
      std::memcpy(&bits, &value, sizeof(bits));
      for (unsigned byte = 0; byte < 4; ++byte) {
        hash ^= (bits >> (byte * 8)) & 0xffU;
        hash *= 16777619u;
      }
    }
    return hash;
  }

  webots_ros2_driver::WebotsNode *node_{nullptr};
  WbDeviceTag lidar_{0};
  rclcpp::Publisher<my_epuck_interfaces::msg::ScanChunk>::SharedPtr publisher_;
  std::string robot_id_;
  std::string topic_;
  std::string frame_id_;
  std::string lidar_name_;
  unsigned chunk_beams_{180};
  double update_rate_hz_{0.0};
  double last_publish_time_{-1.0};
  uint32_t sequence_{0};
};

}  // namespace my_epuck_frontier_candidates

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  my_epuck_frontier_candidates::ChunkedLidarPlugin,
  webots_ros2_driver::PluginInterface)
