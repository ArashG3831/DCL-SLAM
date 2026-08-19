#include <algorithm>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate_array.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_exploration_ros2/frontier_search.hpp"
#include "my_epuck_frontier_candidates/candidate_utils.hpp"

using namespace std::chrono_literals;

namespace my_epuck_frontier_candidates {

class Generator : public rclcpp::Node {
  using Action = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  enum class State {WAITING_FOR_INPUTS, IDLE, EXTRACTING, PATH_CHECKING, PUBLISHING};

  struct Work {
    frontier_exploration_ros2::FrontierCandidate region;
    uint64_t id{0};
    double frontier_length{0.0};
    double gain{0.0};
    double euclid{0.0};
    double heading{0.0};
    std::array<double, 4> bounds{};
    geometry_msgs::msg::PoseStamped pose;
    double path{0.0};
    double score{0.0};
    std::vector<geometry_msgs::msg::Point> path_samples;
    uint64_t map_context{0};
    uint64_t cost_context{0};
    uint64_t path_map_context{0};
    uint64_t path_cost_context{0};
    uint64_t cycles_not_queried{0};
    int64_t last_query_ns{0};
    bool path_refresh{false};
    uint64_t query_event_id{0};
  };

  struct RegionDiagnostic {
    uint64_t id{0};
    frontier_exploration_ros2::FrontierCandidate region;
    std::string status{"DETECTED_NOT_QUERIED"};
    double approach_x{0.0};
    double approach_y{0.0};
    bool has_approach{false};
    double visible_reveal_gain{std::numeric_limits<double>::quiet_NaN()};
  };

  struct IdentityReference {
    uint64_t id{0};
    double centroid_x{0.0};
    double centroid_y{0.0};
    std::array<double, 4> bounds{};
  };

  struct EvaluationCache {
    uint64_t geometry_id{0};
    uint64_t map_context{0};
    uint64_t cost_context{0};
    uint64_t path_map_context{0};
    uint64_t path_cost_context{0};
    std::string classification;
    Work work;
    bool has_work{false};
    uint64_t query_count{0};
    uint64_t cycles_seen{0};
    uint64_t cycles_not_queried{0};
    int64_t last_query_ns{0};
    int64_t last_seen_ns{0};
  };

  struct Suppression {
    uint64_t revision{0};
    int64_t expires_ns{0};
  };

public:
  Generator()
  : Node("frontier_candidate_generator"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
#define P(T, N, D) N##_ = declare_parameter<T>(#N, D)
    P(std::string, robot_id, "");
    P(std::string, map_topic, "shared_map");
    P(std::string, global_costmap_topic, "global_costmap/costmap");
    P(std::string, global_frame, "shared_map");
    P(std::string, robot_base_frame, "base_footprint");
    P(std::string, compute_path_action, "compute_path_to_pose");
    P(std::string, candidate_topic, "frontier_candidates");
    P(std::string, marker_topic, "frontier_candidate_markers");
    P(std::string, path_query_lock_path, "");
    P(double, processing_rate_hz, .5);
    P(int, occupied_threshold, 50);
    P(int, costmap_blocked_threshold, 1);
    P(int, minimum_frontier_cells, 5);
    P(double, minimum_frontier_length_m, .05);
    P(double, stable_id_quantization_m, .05);
    P(double, approach_clearance_m, .06);
    P(double, planner_tolerance_m, .5);
    P(double, minimum_robot_distance_m, .08);
    P(int, maximum_candidates_before_path_check, 8);
    P(int, maximum_path_queries_per_cycle, 5);
    P(double, path_query_timeout_s, 1.0);
    P(double, maximum_feasible_path_m, 18.0);
    P(std::string, planner_id, "GridBased");
    P(double, gain_weight, 1.0);
    P(double, distance_weight, 1.0);
    P(double, path_weight, 1.0);
    P(double, heading_weight, .2);
    P(double, unreachable_suppression_s, 7.0);
    P(int, maximum_suppression_records, 128);
    P(double, goal_tolerance_m, .03);
    P(double, visible_gain_range_m, 11.98);
    P(double, visible_gain_fov_deg, 360.0);
    P(double, visible_gain_ray_step_deg, 2.0);
    // Classification validity is local to the frontier/approach neighborhood.
    // Exact bid-path validity remains stricter via path_context_radius_m.
    P(double, classification_context_radius_m, .35);
    P(double, path_context_radius_m, .75);
    P(bool, forensic_clearance_cells, false);
    P(bool, diagnostic_frontier_capture, false);
#undef P
    if (robot_id_.empty()) {
      throw std::runtime_error("robot_id must be configured");
    }
    if (path_query_lock_path_.empty()) {
      path_query_lock_path_ = "/tmp/my_epuck_" + robot_id_ + "_compute_path.lock";
    }
    auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, transient_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {map_cb(message);});
    cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      global_costmap_topic_, transient_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {cost_cb(message);});
    pub_ = create_publisher<my_epuck_interfaces::msg::FrontierCandidateArray>(
      candidate_topic_, rclcpp::QoS(1).reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).reliable());
    planner_ = rclcpp_action::create_client<Action>(this, compute_path_action_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(.01, processing_rate_hz_)),
      [this] {tick();});
    receipt_summary_timer_ = create_wall_timer(1s, [this] {emit_receipt_summary();});
    RCLCPP_INFO(
      get_logger(),
      "candidate generator: persistent fair frontier evaluation map=%s costmap=%s planner=%s budget=%d",
      map_topic_.c_str(), global_costmap_topic_.c_str(), compute_path_action_.c_str(),
      maximum_path_queries_per_cycle_);
  }

  ~Generator() override
  {
    request_generation_++;
    active_request_ = 0;
    if (timeout_timer_) {timeout_timer_->cancel();}
    if (retry_timer_) {retry_timer_->cancel();}
    release_path_lock();
    if (active_) {planner_->async_cancel_goal(active_);}
  }

private:
  void map_cb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    const auto checksum = map_checksum(*message);
    const auto receipt = std::chrono::steady_clock::now().time_since_epoch();
    std::lock_guard<std::mutex> lock(mu_);
    const bool changed = !latest_map_ || checksum != map_sum_;
    latest_map_ = message;
    ++map_receipts_;
    if (changed) {
      map_sum_ = checksum;
      ++revision_;
      ++map_changed_;
      pending_ = true;
    }
    last_map_receipt_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(receipt).count();
  }

  void cost_cb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    const auto checksum = map_checksum(*message);
    const auto receipt = std::chrono::steady_clock::now().time_since_epoch();
    std::lock_guard<std::mutex> lock(mu_);
    const bool changed = !latest_cost_ || checksum != cost_sum_;
    latest_cost_ = message;
    ++cost_receipts_;
    if (changed) {
      cost_sum_ = checksum;
      ++cost_revision_;
      ++cost_changed_;
    }
    last_cost_receipt_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(receipt).count();
  }

  void emit_receipt_summary()
  {
    uint64_t map_receipts, map_changed, cost_receipts, cost_changed;
    uint64_t map_revision, cost_revision, map_checksum_value, cost_checksum_value;
    int64_t map_receipt_ns, cost_receipt_ns;
    {
      std::lock_guard<std::mutex> lock(mu_);
      map_receipts = map_receipts_;
      map_changed = map_changed_;
      cost_receipts = cost_receipts_;
      cost_changed = cost_changed_;
      map_revision = revision_;
      cost_revision = cost_revision_;
      map_checksum_value = map_sum_;
      cost_checksum_value = cost_sum_;
      map_receipt_ns = last_map_receipt_ns_;
      cost_receipt_ns = last_cost_receipt_ns_;
    }
    if (map_receipts == 0 && cost_receipts == 0) {return;}
    RCLCPP_INFO(
      get_logger(),
      "GENERATOR_INPUT_SUMMARY map_receipts=%lu map_changed=%lu map_revision=%lu map_checksum=%lu map_receipt_steady_ns=%ld costmap_receipts=%lu costmap_changed=%lu costmap_revision=%lu costmap_checksum=%lu costmap_receipt_steady_ns=%ld",
      map_receipts, map_changed, map_revision, map_checksum_value, map_receipt_ns,
      cost_receipts, cost_changed, cost_revision, cost_checksum_value, cost_receipt_ns);
  }

  void tick()
  {
    if (state_ == State::PATH_CHECKING || state_ == State::EXTRACTING ||
      state_ == State::PUBLISHING) {return;}
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, costmap;
    uint64_t map_revision, costmap_revision;
    {
      std::lock_guard<std::mutex> lock(mu_);
      map = latest_map_;
      costmap = latest_cost_;
      map_revision = revision_;
      costmap_revision = cost_revision_;
      pending_ = false;
    }
    if (!map || !costmap) {
      state_ = State::WAITING_FOR_INPUTS;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "waiting for shared map and global costmap");
      return;
    }
    if (map->header.frame_id != global_frame_ || costmap->header.frame_id != global_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "map/costmap frame mismatch");
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero, 100ms);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "robot pose unavailable: %s", error.what());
      return;
    }
    const auto q = transform.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    start_cycle(
      map, costmap, map_revision, costmap_revision,
      transform.transform.translation.x, transform.transform.translation.y, yaw);
  }

  Work make_work(
    const frontier_exploration_ros2::FrontierCandidate & region,
    uint64_t id, const frontier_exploration_ros2::OccupancyGrid2d & map,
    const frontier_exploration_ros2::OccupancyGrid2d & costmap,
    double rx, double ry, double yaw, int64_t last_query_ns, uint64_t cycles_not_queried)
  {
    Work work;
    work.region = region;
    work.id = id;
    work.frontier_length = static_cast<double>(region.size) * map.map().info.resolution;
    work.bounds = frontier_world_bounds(region, map);
    auto safe = find_safe_approach(
      costmap, region.goal_point->first, region.goal_point->second,
      region.centroid.first, region.centroid.second, .75,
      planner_tolerance_m_ + .5 * costmap.map().info.resolution + 1e-6,
      approach_clearance_m_, costmap_blocked_threshold_);
    if (!safe) {return work;}
    const auto world = costmap.mapToWorld(safe->first, safe->second);
    work.pose.header.frame_id = global_frame_;
    work.pose.header.stamp = now();
    work.pose.pose.position.x = world.first;
    work.pose.pose.position.y = world.second;
    const double goal_yaw = std::atan2(
      region.centroid.second - world.second, region.centroid.first - world.first);
    work.pose.pose.orientation.z = std::sin(goal_yaw / 2.0);
    work.pose.pose.orientation.w = std::cos(goal_yaw / 2.0);
    work.euclid = std::hypot(world.first - rx, world.second - ry);
    const double heading_delta = std::atan2(
      std::sin(goal_yaw - yaw), std::cos(goal_yaw - yaw));
    work.heading = std::abs(heading_delta);
    const auto visible = frontier_exploration_ros2::compute_visible_reveal_gain(
      work.pose.pose, map, costmap, std::nullopt, visible_gain_range_m_,
      visible_gain_fov_deg_, visible_gain_ray_step_deg_, region.visible_reveal_bounds);
    work.gain = visible ? visible->visible_reveal_length_m : 0.0;
    work.map_context = local_context_checksum(
      map, region.centroid.first, region.centroid.second, classification_context_radius_m_);
    work.cost_context = local_context_checksum(
      costmap, world.first, world.second, classification_context_radius_m_);
    work.path_map_context = local_context_checksum(
      map, region.centroid.first, region.centroid.second, path_context_radius_m_);
    work.path_cost_context = local_context_checksum(
      costmap, world.first, world.second, path_context_radius_m_);
    work.last_query_ns = last_query_ns;
    work.cycles_not_queried = cycles_not_queried;
    return work;
  }

  bool cached_context_matches(const EvaluationCache & cache, const Work & work) const
  {
    return cache.geometry_id == work.id && cache.map_context == work.map_context &&
           cache.cost_context == work.cost_context && !cache.classification.empty() &&
           cache.classification != "PLANNER_FAILED";
  }

  bool cached_path_context_matches(const EvaluationCache & cache, const Work & work) const
  {
    return cache.path_map_context == work.path_map_context &&
           cache.path_cost_context == work.path_cost_context && cache.has_work;
  }

  void set_region_status(uint64_t id, const std::string & status)
  {
    for (auto & diagnostic : region_diagnostics_) {
      if (diagnostic.id == id) {diagnostic.status = status;}
    }
  }

  void count_classification(const std::string & status)
  {
    if (status == "DETECTED_NOT_QUERIED") {++detected_not_queried_count_;}
    else if (status == "SMALL_BELOW_THRESHOLD") {++small_frontier_count_;}
    else if (status == "OUT_OF_RANGE") {++out_of_range_frontier_count_;}
    else if (status == "UNREACHABLE" || status == "UNREACHABLE_SAFE_APPROACH") {
      ++unreachable_frontier_count_;
    } else if (status == "PLANNER_FAILED") {++planner_failure_count_;}
  }

  void recount_region_statuses()
  {
    small_frontier_count_ = 0;
    out_of_range_frontier_count_ = 0;
    unreachable_frontier_count_ = 0;
    planner_failure_count_ = 0;
    detected_not_queried_count_ = 0;
    for (const auto & diagnostic : region_diagnostics_) {
      count_classification(diagnostic.status);
    }
  }

  void start_cycle(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap,
    uint64_t map_revision, uint64_t costmap_revision,
    double rx, double ry, double yaw)
  {
    state_ = State::EXTRACTING;
    const auto started = now();
    cycle_map_ = map;
    cycle_cost_ = costmap;
    cycle_revision_ = map_revision;
    cycle_cost_revision_ = costmap_revision;
    rx_ = rx;
    ry_ = ry;
    works_.clear();
    reachable_.clear();
    region_diagnostics_.clear();
    query_index_ = 0;
    queries_ = 0;
    safe_approach_rejections_ = 0;
    detected_frontier_count_ = 0;
    small_frontier_count_ = 0;
    out_of_range_frontier_count_ = 0;
    unreachable_frontier_count_ = 0;
    planner_failure_count_ = 0;
    detected_not_queried_count_ = 0;
    frontier_exploration_ros2::OccupancyGrid2d grid(map), cost_grid(costmap);
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = rx;
    robot_pose.position.y = ry;
    robot_pose.orientation.z = std::sin(yaw / 2.0);
    robot_pose.orientation.w = std::cos(yaw / 2.0);
    frontier_exploration_ros2::FrontierSearchOptions search_options;
    search_options.occ_threshold = occupied_threshold_;
    search_options.min_frontier_size_cells = minimum_frontier_cells_;
    search_options.candidate_min_goal_distance_m = minimum_robot_distance_m_;
    const auto regions = frontier_exploration_ros2::get_frontier(
      robot_pose, grid, cost_grid, std::nullopt, minimum_robot_distance_m_, false,
      search_options).frontiers;
    detected_frontier_count_ = static_cast<uint32_t>(regions.size());
    region_diagnostics_.reserve(regions.size());
    std::vector<FrontierEvaluationRecord> schedule_records;
    std::vector<Work> pending_work;
    const auto now_ns = now().nanoseconds();

    for (const auto & region : regions) {
      const uint64_t id = stable_frontier_id(region, grid, stable_id_quantization_m_);
      if (evaluation_cache_.find(id) == evaluation_cache_.end()) {
        const auto current_bounds = frontier_world_bounds(region, grid);
        double best_distance = 0.12;
        const IdentityReference * best = nullptr;
        for (const auto & previous : previous_identity_references_) {
          if (previous.id == id) {continue;}
          const double distance = std::hypot(
            previous.centroid_x - region.centroid.first,
            previous.centroid_y - region.centroid.second);
          const bool overlap_x = previous.bounds[0] <= current_bounds[2] &&
            current_bounds[0] <= previous.bounds[2];
          const bool overlap_y = previous.bounds[1] <= current_bounds[3] &&
            current_bounds[1] <= previous.bounds[3];
          if (distance < best_distance && overlap_x && overlap_y) {
            best_distance = distance;
            best = &previous;
          }
        }
        if (best) {
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_ID_ASSOCIATION old_id=%lu new_id=%lu centroid_distance_m=%.3f",
            best->id, id, best_distance);
        }
      }
      region_diagnostics_.push_back({id, region, "DETECTED_NOT_QUERIED", 0.0, 0.0, false});
      auto & cache = evaluation_cache_[id];
      cache.geometry_id = id;
      ++cache.cycles_seen;
      cache.last_seen_ns = now_ns;
      const double length = static_cast<double>(region.size) * grid.map().info.resolution;
      if (length + 1e-9 < minimum_frontier_length_m_ || !region.goal_point) {
        set_region_status(id, "SMALL_BELOW_THRESHOLD");
        count_classification("SMALL_BELOW_THRESHOLD");
        continue;
      }
      if (suppressed(id, map_revision)) {
        set_region_status(id, "UNREACHABLE");
        cache.classification = "UNREACHABLE";
        count_classification("UNREACHABLE");
        continue;
      }
      Work work = make_work(
        region, id, grid, cost_grid, rx, ry, yaw,
        cache.last_query_ns, cache.cycles_not_queried);
      if (!work.pose.header.frame_id.empty()) {
        auto & diagnostic = region_diagnostics_.back();
        diagnostic.approach_x = work.pose.pose.position.x;
        diagnostic.approach_y = work.pose.pose.position.y;
        diagnostic.visible_reveal_gain = work.gain;
        diagnostic.has_approach = true;
      }
      if (work.pose.header.frame_id.empty()) {
        set_region_status(id, "UNREACHABLE_SAFE_APPROACH");
        cache.classification = "UNREACHABLE";
        cache.map_context = local_context_checksum(
          grid, region.centroid.first, region.centroid.second, classification_context_radius_m_);
        cache.cost_context = 0;
        cache.last_query_ns = now_ns;
        cache.query_count++;
        cache.cycles_not_queried = 0;
        ++safe_approach_rejections_;
        count_classification("UNREACHABLE_SAFE_APPROACH");
        suppress(id, map_revision, true);
        continue;
      }
      const bool never_queried = cache.query_count == 0;
      const bool transient_failure = cache.classification == "PLANNER_FAILED";
      const bool map_context_changed =
        cache.query_count > 0 && cache.map_context != work.map_context;
      const bool cost_context_changed =
        cache.query_count > 0 && cache.cost_context != work.cost_context;
      const bool context_invalidated = map_context_changed || cost_context_changed;
      const bool classification_cached =
        cached_context_matches(cache, work) && cache.classification != "DETECTED_NOT_QUERIED";
      if (classification_cached) {
        set_region_status(id, cache.classification);
        ++classification_cache_hits_;
        RCLCPP_INFO(
          get_logger(), "FRONTIER_CLASSIFICATION_CACHE_HIT id=%lu canonical_id=%016lx status=%s map_revision=%lu costmap_revision=%lu",
          id, id, cache.classification.c_str(), map_revision, costmap_revision);
        if (cache.classification == "REACHABLE" && !cached_path_context_matches(cache, work)) {
          // The high-level classification remains valid, but the exact path
          // is not reused after its stricter local corridor context changes.
          // A fresh query refreshes the bid path; final dispatch validation is
          // still independent and remains mandatory.
          ++path_cache_invalidations_;
          work.path_refresh = true;
          work.cycles_not_queried = 0;
          pending_work.push_back(std::move(work));
          schedule_records.push_back({id, false, true, false, 0, cache.last_query_ns});
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_PATH_CACHE_INVALIDATED id=%lu reason=%s map_revision=%lu costmap_revision=%lu",
            id, map_context_changed && cost_context_changed ? "MAP_AND_COSTMAP_CONTEXT" :
            (map_context_changed ? "MAP_CONTEXT" : "COSTMAP_CONTEXT"), map_revision,
            costmap_revision);
          continue;
        }
        if (cache.classification == "REACHABLE" && cache.has_work) {
          reachable_.push_back(cache.work);
        } else {
          count_classification(cache.classification);
        }
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_LIFECYCLE query_id=0 id=%lu state=CACHE_SATISFIED status=%s",
          id, cache.classification.c_str());
        cache.cycles_not_queried = 0;
        continue;
      }
      const std::string old_classification = cache.classification;
      ++classification_cache_misses_;
      if (never_queried) {
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_CLASSIFICATION_INVALIDATED id=%lu old_status=%s reason=NEVER_QUERIED map_revision=%lu costmap_revision=%lu",
          id, old_classification.empty() ? "NONE" : old_classification.c_str(), map_revision,
          costmap_revision);
      } else {
        const char * reason = transient_failure ? "PREVIOUS_TRANSIENT_FAILURE" :
          (map_context_changed && cost_context_changed ? "MAP_AND_COSTMAP_CONTEXT" :
          (map_context_changed ? "MAP_CONTEXT" :
          (cost_context_changed ? "COSTMAP_CONTEXT" : "OTHER")));
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_CLASSIFICATION_INVALIDATED id=%lu old_status=%s reason=%s map_changed=%s costmap_changed=%s map_revision=%lu costmap_revision=%lu",
          id, old_classification.empty() ? "NONE" : old_classification.c_str(), reason,
          map_context_changed ? "true" : "false", cost_context_changed ? "true" : "false",
          map_revision, costmap_revision);
      }
      cache.classification = "DETECTED_NOT_QUERIED";
      ++cache.cycles_not_queried;
      work.cycles_not_queried = cache.cycles_not_queried;
      work.last_query_ns = cache.last_query_ns;
      pending_work.push_back(std::move(work));
      schedule_records.push_back({id, never_queried, context_invalidated,
                                  transient_failure, cache.cycles_not_queried,
                                  cache.last_query_ns});
    }

    normalize_coarse(pending_work);
    const auto selected_indices = fair_frontier_query_order(
      schedule_records, static_cast<std::size_t>(maximum_candidates_before_path_check_),
      static_cast<std::size_t>(maximum_path_queries_per_cycle_));
    for (const auto index : selected_indices) {
      works_.push_back(std::move(pending_work[index]));
      works_.back().query_event_id = ++query_event_sequence_;
      set_region_status(
        works_.back().id, works_.back().path_refresh ?
        evaluation_cache_[works_.back().id].classification : "DETECTED_NOT_QUERIED");
      RCLCPP_INFO(
        get_logger(), "FRONTIER_QUERY_SELECTED query_id=%lu id=%lu canonical_id=%016lx cycles_not_queried=%lu query_index=%zu",
        works_.back().query_event_id, works_.back().id, works_.back().id,
        works_.back().cycles_not_queried, works_.size() - 1);
    }
    extract_ms_ = (now() - started).seconds() * 1000.0;
    state_ = State::PATH_CHECKING;
    send_next();
  }

  void normalize_coarse(std::vector<Work> & work_items)
  {
    if (work_items.empty()) {return;}
    auto range = [](const auto & values, auto getter) {
        const auto result = std::minmax_element(
          values.begin(), values.end(), [&getter](const auto & first, const auto & second) {
            return getter(first) < getter(second);
          });
        return std::pair{getter(*result.first), getter(*result.second)};
      };
    const auto gain = range(work_items, [](const Work & work) {return work.gain;});
    const auto distance = range(work_items, [](const Work & work) {return work.euclid;});
    const auto heading = range(work_items, [](const Work & work) {return work.heading;});
    for (auto & work : work_items) {
      work.score = gain_weight_ * normalized_value(work.gain, gain.first, gain.second) -
        distance_weight_ * normalized_value(work.euclid, distance.first, distance.second) -
        heading_weight_ * normalized_value(work.heading, heading.first, heading.second);
    }
  }

  void send_next()
  {
    if (query_index_ >= works_.size() || queries_ >= static_cast<std::size_t>(maximum_path_queries_per_cycle_)) {
      finish();
      return;
    }
    if (!planner_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "planner action unavailable");
      for (std::size_t i = query_index_; i < works_.size(); ++i) {
        set_region_status(works_[i].id, "PLANNER_FAILED");
        evaluation_cache_[works_[i].id].classification = "PLANNER_FAILED";
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=RESULT_RECEIVED status=PLANNER_FAILED reason=ACTION_SERVER_UNAVAILABLE",
          works_[i].query_event_id, works_[i].id);
      }
      finish();
      return;
    }
    if (!acquire_path_lock()) {
      if (!retry_timer_) {
        retry_timer_ = create_wall_timer(50ms, [this] {
          if (retry_timer_) {retry_timer_->cancel();}
          retry_timer_.reset();
          send_next();
        });
      }
      return;
    }
    const auto candidate = works_[query_index_++];
    const auto revision = cycle_revision_;
    const auto cost_revision = cycle_cost_revision_;
    const auto request = ++request_generation_;
    ++queries_;
    auto & cache = evaluation_cache_[candidate.id];
    cache.last_query_ns = now().nanoseconds();
    cache.map_context = candidate.map_context;
    cache.cost_context = candidate.cost_context;
    cache.path_map_context = candidate.path_map_context;
    cache.path_cost_context = candidate.path_cost_context;
    cache.cycles_not_queried = 0;
    RCLCPP_INFO(
      get_logger(), "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=REQUEST_SENT request_id=%lu map_revision=%lu costmap_revision=%lu",
      candidate.query_event_id, candidate.id, request, revision, cost_revision);
    RCLCPP_INFO(
      get_logger(), "FRONTIER_QUERY_RESULT_PENDING query_id=%lu id=%lu canonical_id=%016lx map_revision=%lu costmap_revision=%lu",
      candidate.query_event_id, candidate.id, candidate.id, revision, cost_revision);
    const auto request_started = std::chrono::steady_clock::now();
    Action::Goal goal;
    goal.goal = candidate.pose;
    goal.planner_id = planner_id_;
    goal.use_start = false;
    auto options = rclcpp_action::Client<Action>::SendGoalOptions();
    options.goal_response_callback = [this, candidate, revision, request, request_started](GoalHandle::SharedPtr handle) {
        if (!async_request_is_current(request, request_generation_, revision, cycle_revision_, state_ == State::PATH_CHECKING)) {
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=SUPERSEDED phase=GOAL_RESPONSE",
            candidate.query_event_id, candidate.id);
          release_path_lock();
          ++stale_results_;
          return;
        }
        uint64_t current;
        {std::lock_guard<std::mutex> lock(mu_); current = revision_;}
        if (current != revision) {
          ++stale_results_;
          release_path_lock();
          finish(false);
          return;
        }
        if (!handle) {
          set_region_status(candidate.id, "PLANNER_FAILED");
          auto & cache = evaluation_cache_[candidate.id];
          cache.classification = "PLANNER_FAILED";
          cache.has_work = false;
          ++cache.query_count;
          ++planner_failure_count_;
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=RESULT_RECEIVED status=PLANNER_FAILED reason=GOAL_REJECTED",
            candidate.query_event_id, candidate.id);
          RCLCPP_WARN(get_logger(), "FRONTIER_QUERY_RESULT id=%lu status=PLANNER_FAILED reason=GOAL_REJECTED", candidate.id);
          release_path_lock();
          send_next();
          return;
        }
        active_ = handle;
        active_request_ = request;
        timeout_timer_ = create_wall_timer(std::chrono::duration<double>(path_query_timeout_s_), [this, candidate, revision, request, request_started] {
            if (request != active_request_ || !active_) {return;}
            planner_->async_cancel_goal(active_);
            active_.reset();
            active_request_ = 0;
            set_region_status(candidate.id, "PLANNER_FAILED");
            auto & cache = evaluation_cache_[candidate.id];
            cache.classification = "PLANNER_FAILED";
            cache.has_work = false;
            ++cache.query_count;
            ++planner_failure_count_;
            RCLCPP_INFO(
              get_logger(),
              "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=RESULT_RECEIVED status=PLANNER_FAILED reason=TIMEOUT",
              candidate.query_event_id, candidate.id);
            RCLCPP_WARN(
              get_logger(), "FRONTIER_QUERY_RESULT id=%lu status=PLANNER_FAILED reason=TIMEOUT duration_s=%.3f",
              candidate.id, std::chrono::duration<double>(std::chrono::steady_clock::now() - request_started).count());
            timeout_timer_->cancel();
            release_path_lock();
            send_next();
          });
      };
    options.result_callback = [this, candidate, revision, request, request_started](const GoalHandle::WrappedResult & result) {
        if (!async_request_is_current(request, active_request_, revision, cycle_revision_, state_ == State::PATH_CHECKING)) {
          RCLCPP_INFO(
            get_logger(),
            "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=SUPERSEDED phase=RESULT",
            candidate.query_event_id, candidate.id);
          release_path_lock();
          ++stale_results_;
          return;
        }
        if (timeout_timer_) {timeout_timer_->cancel();}
        active_.reset();
        active_request_ = 0;
        uint64_t current;
        {std::lock_guard<std::mutex> lock(mu_); current = revision_;}
        if (current != revision) {
          ++stale_results_;
          release_path_lock();
          finish(false);
          return;
        }
        const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->error_code == Action::Result::NONE;
        auto & cache = evaluation_cache_[candidate.id];
        ++cache.query_count;
        const auto duration = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - request_started).count();
        auto length = ok ? path_length(
          result.result->path, rx_, ry_, candidate.pose.pose.position.x,
          candidate.pose.pose.position.y, goal_tolerance_m_) : std::nullopt;
        if (length) {
          auto reachable = candidate;
          reachable.path = *length;
          const auto & poses = result.result->path.poses;
          const std::size_t count = std::min<std::size_t>(32, poses.size());
          reachable.path_samples.reserve(count);
          for (std::size_t i = 0; i < count; ++i) {
            const auto index = count < 2 ? 0 : std::llround(
              static_cast<double>(i) * (poses.size() - 1) / (count - 1));
            geometry_msgs::msg::Point point;
            point.x = poses[index].pose.position.x;
            point.y = poses[index].pose.position.y;
            reachable.path_samples.push_back(point);
          }
          cache.work = reachable;
          cache.has_work = true;
          cache.map_context = candidate.map_context;
          cache.cost_context = candidate.cost_context;
          cache.path_map_context = candidate.path_map_context;
          cache.path_cost_context = candidate.path_cost_context;
          cache.last_query_ns = now().nanoseconds();
          cache.cycles_not_queried = 0;
          if (*length > maximum_feasible_path_m_) {
            cache.classification = "OUT_OF_RANGE";
            set_region_status(candidate.id, "OUT_OF_RANGE");
            count_classification("OUT_OF_RANGE");
          } else {
            cache.classification = "REACHABLE";
            set_region_status(candidate.id, "REACHABLE");
            reachable_.push_back(reachable);
          }
        } else {
          const bool hard = result.result && (
            result.result->error_code == Action::Result::GOAL_OCCUPIED ||
            result.result->error_code == Action::Result::GOAL_OUTSIDE_MAP ||
            result.result->error_code == Action::Result::NO_VALID_PATH);
          cache.has_work = false;
          cache.map_context = candidate.map_context;
          cache.cost_context = candidate.cost_context;
          cache.path_map_context = candidate.path_map_context;
          cache.path_cost_context = candidate.path_cost_context;
          cache.last_query_ns = now().nanoseconds();
          cache.cycles_not_queried = 0;
          if (hard) {
            cache.classification = "UNREACHABLE";
            set_region_status(candidate.id, "UNREACHABLE");
            count_classification("UNREACHABLE");
            suppress(candidate.id, revision, false);
          } else {
            cache.classification = "PLANNER_FAILED";
            set_region_status(candidate.id, "PLANNER_FAILED");
            ++planner_failure_count_;
          }
        }
        RCLCPP_INFO(
          get_logger(), "FRONTIER_QUERY_RESULT query_id=%lu id=%lu canonical_id=%016lx status=%s error_code=%d duration_s=%.3f",
          candidate.query_event_id, candidate.id, candidate.id, cache.classification.c_str(),
          result.result ? result.result->error_code : -1, duration);
        RCLCPP_INFO(
          get_logger(),
          "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=RESULT_RECEIVED status=%s error_code=%d duration_s=%.3f",
          candidate.query_event_id, candidate.id, cache.classification.c_str(),
          result.result ? result.result->error_code : -1, duration);
        release_path_lock();
        send_next();
      };
    planner_->async_send_goal(goal, options);
  }

  void finish(bool publish = true)
  {
    for (std::size_t i = query_index_; i < works_.size(); ++i) {
      RCLCPP_INFO(
        get_logger(),
        "FRONTIER_QUERY_LIFECYCLE query_id=%lu id=%lu state=CANCELLED_BEFORE_REQUEST",
        works_[i].query_event_id, works_[i].id);
    }
    request_generation_++;
    active_request_ = 0;
    if (timeout_timer_) {timeout_timer_->cancel();}
    if (retry_timer_) {retry_timer_->cancel();}
    release_path_lock();
    state_ = State::PUBLISHING;
    if (publish) {
      normalize_final();
      publish_batch();
    }
    state_ = State::IDLE;
    works_.clear();
    reachable_.clear();
    cycle_map_.reset();
    cycle_cost_.reset();
  }

  bool acquire_path_lock()
  {
    if (path_lock_fd_ >= 0) {return true;}
    path_lock_fd_ = ::open(path_query_lock_path_.c_str(), O_CREAT | O_RDWR, 0666);
    if (path_lock_fd_ < 0 || ::flock(path_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      if (path_lock_fd_ >= 0) {::close(path_lock_fd_);}
      path_lock_fd_ = -1;
      return false;
    }
    return true;
  }

  void release_path_lock()
  {
    if (path_lock_fd_ >= 0) {
      ::flock(path_lock_fd_, LOCK_UN);
      ::close(path_lock_fd_);
      path_lock_fd_ = -1;
    }
  }

  void normalize_final()
  {
    if (reachable_.empty()) {return;}
    double g0 = 1e99, g1 = -1e99, p0 = 1e99, p1 = -1e99;
    double h0 = 1e99, h1 = -1e99;
    for (const auto & work : reachable_) {
      g0 = std::min(g0, work.gain); g1 = std::max(g1, work.gain);
      p0 = std::min(p0, work.path); p1 = std::max(p1, work.path);
      h0 = std::min(h0, work.heading); h1 = std::max(h1, work.heading);
    }
    for (auto & work : reachable_) {
      work.score = gain_weight_ * normalized_value(work.gain, g0, g1) -
        path_weight_ * normalized_value(work.path, p0, p1) -
        heading_weight_ * normalized_value(work.heading, h0, h1);
    }
    std::sort(reachable_.begin(), reachable_.end(), [](const Work & first, const Work & second) {
      if (first.score != second.score) {return first.score > second.score;}
      if (first.gain != second.gain) {return first.gain > second.gain;}
      if (first.path != second.path) {return first.path < second.path;}
      return first.id < second.id;
    });
  }

  std::string diagnostic_regions_json() const
  {
    if (!diagnostic_frontier_capture_ || !cycle_map_) {return {};}
    frontier_exploration_ros2::OccupancyGrid2d grid(cycle_map_);
    const auto & q = cycle_map_->info.origin.orientation;
    const double origin_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    std::ostringstream output;
    output << std::setprecision(8)
           << "{\"schema\":2,\"robot_id\":\"" << robot_id_
           << "\",\"map_revision\":" << cycle_revision_
           << ",\"costmap_revision\":" << cycle_cost_revision_
           << ",\"frame\":\"" << global_frame_ << "\",\"resolution\":"
           << cycle_map_->info.resolution << ",\"width\":" << cycle_map_->info.width
           << ",\"height\":" << cycle_map_->info.height << ",\"origin\":["
           << cycle_map_->info.origin.position.x << ","
           << cycle_map_->info.origin.position.y << "," << origin_yaw << "],\"regions\":[";
    for (std::size_t i = 0; i < region_diagnostics_.size(); ++i) {
      const auto & diagnostic = region_diagnostics_[i];
      const auto bounds = frontier_world_bounds(diagnostic.region, grid);
      if (i) {output << ',';}
      output << "{\"id\":" << diagnostic.id
             << ",\"physical_id\":" << diagnostic.id
             << ",\"canonical_id\":\"" << std::hex << std::setw(16)
             << std::setfill('0') << diagnostic.id << std::dec << std::setfill(' ')
             << "\",\"status\":\"" << diagnostic.status
             << "\",\"visible_reveal_gain\":";
      if (std::isfinite(diagnostic.visible_reveal_gain)) {
        output << diagnostic.visible_reveal_gain;
      } else {
        output << "null";
      }
      output
             << ",\"cell_count\":" << diagnostic.region.size
             << ",\"size_m\":" <<
                (diagnostic.region.size * grid.map().info.resolution)
             << ",\"centroid\":[" << diagnostic.region.centroid.first << ","
             << diagnostic.region.centroid.second << "],\"bbox\":[" << bounds[0]
             << "," << bounds[1] << "," << bounds[2] << "," << bounds[3]
             << "],\"approach\":";
      if (diagnostic.has_approach) {
        output << '[' << diagnostic.approach_x << ',' << diagnostic.approach_y << ']';
      } else {
        output << "null";
      }
      output << ",\"cells\":[";
      for (std::size_t j = 0; j < diagnostic.region.cells.size(); ++j) {
        if (j) {output << ',';}
        output << '[' << diagnostic.region.cells[j].first << ','
               << diagnostic.region.cells[j].second << ']';
      }
      output << "]}";
    }
    output << "]}";
    return output.str();
  }

  std::string terminal_frontier_regions_json() const
  {
    if (!cycle_map_) {return {};}  // bounded summary for normal production
    std::ostringstream output;
    output << std::setprecision(8) << "{\"resolution\":"
           << cycle_map_->info.resolution << ",\"regions\":[";
    for (std::size_t i = 0; i < region_diagnostics_.size(); ++i) {
      const auto & diagnostic = region_diagnostics_[i];
      if (i) {output << ',';}
      output << "{\"physical_id\":" << diagnostic.id
             << ",\"status\":\"" << diagnostic.status
             << "\",\"visible_reveal_gain\":";
      if (std::isfinite(diagnostic.visible_reveal_gain)) {
        output << diagnostic.visible_reveal_gain;
      } else {
        output << "null";
      }
      output
             << ",\"cell_count\":" << diagnostic.region.size
             << ",\"size_m\":" <<
                (diagnostic.region.size * cycle_map_->info.resolution) << "}";
    }
    output << "]}";
    return output.str();
  }

  void publish_batch()
  {
    // Planner callbacks complete asynchronously. Recompute the aggregate
    // evidence from the final per-region states so a queried region cannot
    // remain counted as DETECTED_NOT_QUERIED after it became reachable.
    recount_region_statuses();
    previous_identity_references_.clear();
    if (cycle_map_) {
      frontier_exploration_ros2::OccupancyGrid2d grid(cycle_map_);
      for (const auto & diagnostic : region_diagnostics_) {
        previous_identity_references_.push_back({
          diagnostic.id, diagnostic.region.centroid.first, diagnostic.region.centroid.second,
          frontier_world_bounds(diagnostic.region, grid)});
      }
    }
    my_epuck_interfaces::msg::FrontierCandidateArray message;
    message.header.frame_id = global_frame_;
    message.header.stamp = now();
    message.source_robot_id = robot_id_;
    message.map_revision = cycle_revision_;
    message.map_stamp = cycle_map_->header.stamp;
    message.planner_id = planner_id_;
    message.detected_frontier_count = detected_frontier_count_;
    message.small_frontier_count = small_frontier_count_;
    message.out_of_range_frontier_count = out_of_range_frontier_count_;
    message.unreachable_frontier_count = unreachable_frontier_count_;
    message.planner_failure_count = planner_failure_count_;
    message.detected_not_queried_count = detected_not_queried_count_;
    message.unclassified_frontier_count = detected_not_queried_count_;
    message.diagnostic_regions_json = diagnostic_regions_json();
    message.terminal_frontier_regions_json = terminal_frontier_regions_json();
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;
    for (const auto & work : reachable_) {
      my_epuck_interfaces::msg::FrontierCandidate candidate;
      candidate.frontier_id = work.id;
      candidate.physical_frontier_id = work.id;
      candidate.centroid.x = work.region.centroid.first;
      candidate.centroid.y = work.region.centroid.second;
      candidate.bounding_box_min.x = work.bounds[0];
      candidate.bounding_box_min.y = work.bounds[1];
      candidate.bounding_box_max.x = work.bounds[2];
      candidate.bounding_box_max.y = work.bounds[3];
      candidate.approach_pose = work.pose;
      candidate.cell_count = work.region.size;
      candidate.frontier_length_m = work.frontier_length;
      candidate.information_gain = work.gain;
      candidate.euclidean_distance_m = work.euclid;
      candidate.path_length_m = work.path;
      candidate.heading_change_rad = work.heading;
      candidate.score = work.score;
      candidate.reachability_state = candidate.REACHABLE;
      candidate.local_path_length_m = work.path;
      candidate.local_path_samples = work.path_samples;
      markers.markers.emplace_back();
      auto & marker = markers.markers.back();
      marker.header = message.header;
      marker.ns = "reachable_frontiers";
      marker.id = marker_id++;
      marker.type = marker.SPHERE;
      marker.action = marker.ADD;
      marker.pose = work.pose.pose;
      marker.scale.x = marker.scale.y = .04;
      marker.scale.z = .02;
      marker.color.g = 1.0;
      marker.color.a = .9;
      message.candidates.push_back(std::move(candidate));
    }
    pub_->publish(message);
    marker_pub_->publish(markers);
    RCLCPP_INFO(
      get_logger(),
      "CANDIDATE_METRICS source=FRONTIER_REACHABILITY revision=%lu costmap_revision=%lu detected=%u reachable=%zu queries=%zu cache_hits=%lu cache_misses=%lu detected_not_queried=%u small=%u out_of_range=%u unreachable=%u planner_failures=%u",
      cycle_revision_, cycle_cost_revision_, detected_frontier_count_, reachable_.size(), queries_,
      classification_cache_hits_, classification_cache_misses_, detected_not_queried_count_,
      small_frontier_count_, out_of_range_frontier_count_, unreachable_frontier_count_,
      planner_failure_count_);
    RCLCPP_INFO(
      get_logger(), "FRONTIER_CACHE_SUMMARY classification_hits=%lu classification_misses=%lu path_invalidations=%lu",
      classification_cache_hits_, classification_cache_misses_, path_cache_invalidations_);
  }

  bool suppressed(uint64_t id, uint64_t revision)
  {
    const auto it = suppression_.find(id);
    if (it == suppression_.end()) {return false;}
    if (it->second.revision != revision && it->second.expires_ns == 0) {
      suppression_.erase(it);
      return false;
    }
    if (it->second.expires_ns && now().nanoseconds() > it->second.expires_ns) {
      suppression_.erase(it);
      return false;
    }
    return true;
  }

  void suppress(uint64_t id, uint64_t revision, bool until_revision)
  {
    if (suppression_.size() >= static_cast<std::size_t>(maximum_suppression_records_)) {
      suppression_.erase(suppression_.begin());
    }
    suppression_[id] = {revision, until_revision ? 0 : now().nanoseconds() +
      static_cast<int64_t>(unreachable_suppression_s_ * 1e9)};
  }

  std::mutex mu_;
  State state_{State::WAITING_FOR_INPUTS};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_, latest_cost_, cycle_map_, cycle_cost_;
  uint64_t map_sum_{0}, cost_sum_{0}, revision_{0}, cost_revision_{0};
  uint64_t cycle_revision_{0}, cycle_cost_revision_{0}, stale_results_{0};
  uint64_t request_generation_{0}, active_request_{0}, query_event_sequence_{0};
  uint64_t map_receipts_{0}, map_changed_{0};
  uint64_t cost_receipts_{0}, cost_changed_{0}, classification_cache_hits_{0};
  uint64_t classification_cache_misses_{0}, path_cache_invalidations_{0};
  int64_t last_map_receipt_ns_{0}, last_cost_receipt_ns_{0};
  bool pending_{false};
  double rx_{0.0}, ry_{0.0}, extract_ms_{0.0};
  std::vector<Work> works_, reachable_;
  std::vector<RegionDiagnostic> region_diagnostics_;
  std::vector<IdentityReference> previous_identity_references_;
  std::unordered_map<uint64_t, EvaluationCache> evaluation_cache_;
  std::size_t query_index_{0}, queries_{0}, safe_approach_rejections_{0};
  uint32_t detected_frontier_count_{0}, small_frontier_count_{0};
  uint32_t out_of_range_frontier_count_{0}, unreachable_frontier_count_{0};
  uint32_t planner_failure_count_{0}, detected_not_queried_count_{0};
  std::unordered_map<uint64_t, Suppression> suppression_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<Action>::SharedPtr planner_;
  GoalHandle::SharedPtr active_;
  rclcpp::TimerBase::SharedPtr timer_, timeout_timer_, retry_timer_, receipt_summary_timer_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, cost_sub_;
  rclcpp::Publisher<my_epuck_interfaces::msg::FrontierCandidateArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  int path_lock_fd_{-1};
  std::string robot_id_, map_topic_, global_costmap_topic_, global_frame_, robot_base_frame_;
  std::string compute_path_action_, candidate_topic_, marker_topic_, path_query_lock_path_;
  std::string planner_id_;
  double processing_rate_hz_, minimum_frontier_length_m_, stable_id_quantization_m_;
  double approach_clearance_m_, planner_tolerance_m_, minimum_robot_distance_m_;
  double path_query_timeout_s_, maximum_feasible_path_m_, gain_weight_, distance_weight_;
  double path_weight_, heading_weight_, unreachable_suppression_s_, goal_tolerance_m_;
  double visible_gain_range_m_, visible_gain_fov_deg_, visible_gain_ray_step_deg_;
  double classification_context_radius_m_, path_context_radius_m_;
  int occupied_threshold_, costmap_blocked_threshold_, minimum_frontier_cells_;
  int maximum_candidates_before_path_check_, maximum_path_queries_per_cycle_;
  int maximum_suppression_records_;
  bool forensic_clearance_cells_{false}, diagnostic_frontier_capture_{false};
};

}  // namespace my_epuck_frontier_candidates

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_epuck_frontier_candidates::Generator>());
  rclcpp::shutdown();
}
