#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate.hpp>
#include <my_epuck_interfaces/msg/frontier_candidate_array.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/utils.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "frontier_exploration_ros2/frontier_search.hpp"
#include "my_epuck_frontier_candidates/candidate_utils.hpp"
using namespace std::chrono_literals;
namespace my_epuck_frontier_candidates {
class Generator:public rclcpp::Node {
 using Action=nav2_msgs::action::ComputePathToPose; using GoalHandle=rclcpp_action::ClientGoalHandle<Action>;
 enum class State{WAITING_FOR_INPUTS,IDLE,EXTRACTING,PATH_CHECKING,PUBLISHING};
 struct Work{frontier_exploration_ros2::FrontierCandidate region;uint64_t id;double gain,euclid,heading;geometry_msgs::msg::PoseStamped pose;double path{0},score{0};};
 struct Suppression{uint64_t revision;int64_t expires_ns;};
public:
 Generator():Node("frontier_candidate_generator"),tf_buffer_(get_clock()),tf_listener_(tf_buffer_){
#define P(T,N,D) N##_=declare_parameter<T>(#N,D)
  P(std::string,robot_id,"");P(std::string,map_topic,"shared_map");P(std::string,global_costmap_topic,"global_costmap/costmap");P(std::string,global_frame,"shared_map");P(std::string,robot_base_frame,"base_footprint");P(std::string,compute_path_action,"compute_path_to_pose");P(std::string,candidate_topic,"frontier_candidates");P(std::string,marker_topic,"frontier_candidate_markers");P(double,processing_rate_hz,.5);P(int,free_threshold,0);P(int,occupied_threshold,50);P(int,costmap_blocked_threshold,1);P(int,minimum_frontier_cells,5);P(double,minimum_frontier_length_m,.05);P(double,stable_id_quantization_m,.05);P(double,approach_clearance_m,.06);P(double,minimum_robot_distance_m,.08);P(int,maximum_candidates_before_path_check,8);P(int,maximum_path_queries_per_cycle,5);P(double,path_query_timeout_s,1.0);P(std::string,planner_id,"GridBased");P(double,gain_weight,1.0);P(double,distance_weight,1.0);P(double,path_weight,1.0);P(double,heading_weight,.2);P(double,unreachable_suppression_s,7.0);P(int,maximum_suppression_records,128);P(double,goal_tolerance_m,.03);P(bool,diagonal_connectivity,true);
#undef P
  if(robot_id_.empty())throw std::runtime_error("robot_id must be configured");
  auto tq=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_,tq,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){map_cb(m);});
  cost_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(global_costmap_topic_,tq,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){std::lock_guard<std::mutex>l(mu_);latest_cost_=m;});
  pub_=create_publisher<my_epuck_interfaces::msg::FrontierCandidateArray>(candidate_topic_,rclcpp::QoS(1).reliable());
  marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_,rclcpp::QoS(1).reliable());
  planner_=rclcpp_action::create_client<Action>(this,compute_path_action_);
  timer_=create_wall_timer(std::chrono::duration<double>(1./std::max(.01,processing_rate_hz_)),[this]{tick();});
  RCLCPP_INFO(get_logger(),"candidate-only generator: map=%s costmap=%s planner=%s; no NavigateToPose or cmd_vel interfaces",map_topic_.c_str(),global_costmap_topic_.c_str(),compute_path_action_.c_str());
 }
 ~Generator(){
   request_generation_++;active_request_=0;
   if(timeout_timer_)timeout_timer_->cancel();
   if(active_)planner_->async_cancel_goal(active_);
 }
private:
 void map_cb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){auto sum=map_checksum(*m);std::lock_guard<std::mutex>l(mu_);if(!latest_map_||sum!=map_sum_){latest_map_=m;map_sum_=sum;++revision_;pending_=true;}}
 void tick(){if(state_==State::PATH_CHECKING||state_==State::EXTRACTING||state_==State::PUBLISHING)return;nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,cost;uint64_t rev;{std::lock_guard<std::mutex>l(mu_);map=latest_map_;cost=latest_cost_;rev=revision_;pending_=false;}if(!map||!cost){state_=State::WAITING_FOR_INPUTS;RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"waiting for shared map and global costmap");return;}if(map->header.frame_id!=global_frame_||cost->header.frame_id!=global_frame_){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"map/costmap frame mismatch");return;}geometry_msgs::msg::TransformStamped tf;try{tf=tf_buffer_.lookupTransform(global_frame_,robot_base_frame_,tf2::TimePointZero,100ms);}catch(const std::exception&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"robot pose unavailable: %s",e.what());return;}auto q=tf.transform.rotation; double pose_yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); start_cycle(map,cost,rev,tf.transform.translation.x,tf.transform.translation.y,pose_yaw);}
 void start_cycle(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, nav_msgs::msg::OccupancyGrid::ConstSharedPtr cost,uint64_t rev,double rx,double ry,double yaw){state_=State::EXTRACTING;auto started=now();cycle_map_=map;cycle_cost_=cost;cycle_revision_=rev;rx_=rx;ry_=ry;works_.clear();query_index_=0;queries_=0;
  frontier_exploration_ros2::OccupancyGrid2d gm(map),gc(cost);auto regions=frontier_exploration_ros2::get_frontier(gm,rx,ry,free_threshold_,minimum_frontier_cells_,diagonal_connectivity_,cache_);
  for(auto&r:regions){double len=r.cells.size()*gm.resolution();if(len+1e-9<minimum_frontier_length_m_)continue;uint64_t id=stable_frontier_id(r,gm,stable_id_quantization_m_);if(suppressed(id,rev))continue;auto candidates=r.approach_cells;std::sort(candidates.begin(),candidates.end(),[&](auto a,auto b){auto wa=gm.mapToWorld(a.x,a.y),wb=gm.mapToWorld(b.x,b.y);return std::hypot(wa.first-r.centroid_x,wa.second-r.centroid_y)<std::hypot(wb.first-r.centroid_x,wb.second-r.centroid_y);});std::optional<frontier_exploration_ros2::Cell> chosen;for(auto c:candidates){auto w=gm.mapToWorld(c.x,c.y);int cx,cy;if(std::hypot(w.first-rx,w.second-ry)<minimum_robot_distance_m_||!gc.worldToMap(w.first,w.second,cx,cy)||gc.cost(cx,cy)<0||gc.cost(cx,cy)>=costmap_blocked_threshold_||!clearance_ok(gc,w.first,w.second,approach_clearance_m_,costmap_blocked_threshold_))continue;chosen=c;break;}if(!chosen){suppress(id,rev,true);continue;}auto w=gm.mapToWorld(chosen->x,chosen->y);Work x{r,id,len,std::hypot(w.first-rx,w.second-ry),0};x.heading=std::abs(std::atan2(std::sin(std::atan2(r.centroid_y-w.second,r.centroid_x-w.first)-yaw),std::cos(std::atan2(r.centroid_y-w.second,r.centroid_x-w.first)-yaw)));x.pose.header.frame_id=global_frame_;x.pose.header.stamp=now();x.pose.pose.position.x=w.first;x.pose.pose.position.y=w.second;double gyaw=std::atan2(r.centroid_y-w.second,r.centroid_x-w.first);x.pose.pose.orientation.z=std::sin(gyaw/2);x.pose.pose.orientation.w=std::cos(gyaw/2);works_.push_back(std::move(x));}
  normalize_coarse();std::sort(works_.begin(),works_.end(),[](auto&a,auto&b){if(a.score!=b.score)return a.score>b.score;if(a.gain!=b.gain)return a.gain>b.gain;return a.id<b.id;});if(works_.size()>size_t(maximum_candidates_before_path_check_))works_.resize(maximum_candidates_before_path_check_);extract_ms_=(now()-started).seconds()*1000.;state_=State::PATH_CHECKING;send_next();}
 void normalize_coarse(){if(works_.empty())return;auto mm=[](auto&v,auto f){auto p=std::minmax_element(v.begin(),v.end(),[&](auto&a,auto&b){return f(a)<f(b);});return std::pair{f(*p.first),f(*p.second)};};auto g=mm(works_,[](auto&x){return x.gain;}),d=mm(works_,[](auto&x){return x.euclid;}),h=mm(works_,[](auto&x){return x.heading;});for(auto&x:works_)x.score=gain_weight_*normalized_value(x.gain,g.first,g.second)-distance_weight_*normalized_value(x.euclid,d.first,d.second)-heading_weight_*normalized_value(x.heading,h.first,h.second);}
  void send_next(){
    if(query_index_>=works_.size()||queries_>=maximum_path_queries_per_cycle_){finish();return;}
    if(!planner_->action_server_is_ready()){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"planner action unavailable");finish();return;}
    const auto candidate=works_[query_index_++];
    const auto revision=cycle_revision_;
    const auto request=++request_generation_;
    queries_++;
    Action::Goal g;g.goal=candidate.pose;g.planner_id=planner_id_;g.use_start=false;
    auto opts=rclcpp_action::Client<Action>::SendGoalOptions();
    opts.goal_response_callback=[this,candidate,revision,request](GoalHandle::SharedPtr h){
      if(!async_request_is_current(request,request_generation_,revision,cycle_revision_,state_==State::PATH_CHECKING)){++stale_results_;return;}
      uint64_t current;{std::lock_guard<std::mutex>l(mu_);current=revision_;}
      if(current!=revision){++stale_results_;finish(false);return;}
      if(!h){suppress(candidate.id,revision,false);send_next();return;}
      active_=h;active_request_=request;
      timeout_timer_=create_wall_timer(std::chrono::duration<double>(path_query_timeout_s_),[this,candidate,revision,request]{
        if(request!=active_request_||!active_)return;
        uint64_t current;{std::lock_guard<std::mutex>l(mu_);current=revision_;}
        if(revision!=cycle_revision_||current!=revision){planner_->async_cancel_goal(active_);active_.reset();active_request_=0;timeout_timer_->cancel();finish(false);return;}
        planner_->async_cancel_goal(active_);active_.reset();active_request_=0;
        suppress(candidate.id,revision,false);timeout_timer_->cancel();send_next();
      });
    };
    opts.result_callback=[this,candidate,revision,request](const GoalHandle::WrappedResult&r){
      if(!async_request_is_current(request,active_request_,revision,cycle_revision_,state_==State::PATH_CHECKING)){++stale_results_;return;}
      if(timeout_timer_)timeout_timer_->cancel();active_.reset();active_request_=0;
      uint64_t current;{std::lock_guard<std::mutex>l(mu_);current=revision_;}
      if(current!=revision){stale_results_++;finish(false);return;}
      bool ok=r.code==rclcpp_action::ResultCode::SUCCEEDED&&r.result&&r.result->error_code==Action::Result::NONE;
      auto len=ok?path_length(r.result->path,rx_,ry_,candidate.pose.pose.position.x,candidate.pose.pose.position.y,goal_tolerance_m_):std::nullopt;
      if(len){auto reachable=candidate;reachable.path=*len;reachable_.push_back(std::move(reachable));}
      else suppress(candidate.id,revision,r.result&&(r.result->error_code==Action::Result::GOAL_OCCUPIED||r.result->error_code==Action::Result::GOAL_OUTSIDE_MAP));
      send_next();
    };
    planner_->async_send_goal(g,opts);
  }
 void finish(bool publish=true){request_generation_++;active_request_=0;if(timeout_timer_)timeout_timer_->cancel();state_=State::PUBLISHING;if(publish){normalize_final();publish_batch();}state_=State::IDLE;works_.clear();reachable_.clear();cycle_map_.reset();cycle_cost_.reset();}
 void normalize_final(){if(reachable_.empty())return;double g0=1e99,g1=-1e99,p0=1e99,p1=-1e99,h0=1e99,h1=-1e99;for(auto&x:reachable_){g0=std::min(g0,x.gain);g1=std::max(g1,x.gain);p0=std::min(p0,x.path);p1=std::max(p1,x.path);h0=std::min(h0,x.heading);h1=std::max(h1,x.heading);}for(auto&x:reachable_)x.score=gain_weight_*normalized_value(x.gain,g0,g1)-path_weight_*normalized_value(x.path,p0,p1)-heading_weight_*normalized_value(x.heading,h0,h1);std::sort(reachable_.begin(),reachable_.end(),[](auto&a,auto&b){if(a.score!=b.score)return a.score>b.score;if(a.gain!=b.gain)return a.gain>b.gain;if(a.path!=b.path)return a.path<b.path;return a.id<b.id;});}
 void publish_batch(){my_epuck_interfaces::msg::FrontierCandidateArray a;a.header.frame_id=global_frame_;a.header.stamp=now();a.source_robot_id=robot_id_;a.map_revision=cycle_revision_;a.map_stamp=cycle_map_->header.stamp;a.planner_id=planner_id_;visualization_msgs::msg::MarkerArray markers;int mid=0;for(auto&x:reachable_){my_epuck_interfaces::msg::FrontierCandidate c;c.frontier_id=x.id;c.centroid.x=x.region.centroid_x;c.centroid.y=x.region.centroid_y;c.bounding_box_min.x=x.region.min_x;c.bounding_box_min.y=x.region.min_y;c.bounding_box_max.x=x.region.max_x;c.bounding_box_max.y=x.region.max_y;c.approach_pose=x.pose;c.cell_count=x.region.cells.size();c.frontier_length_m=x.gain;c.information_gain=x.gain;c.euclidean_distance_m=x.euclid;c.path_length_m=x.path;c.heading_change_rad=x.heading;c.score=x.score;c.reachability_state=c.REACHABLE;a.candidates.push_back(c);visualization_msgs::msg::Marker m;m.header=a.header;m.ns="reachable_frontiers";m.id=mid++;m.type=m.SPHERE;m.action=m.ADD;m.pose=x.pose.pose;m.scale.x=m.scale.y=.04;m.scale.z=.02;m.color.g=1;m.color.a=.9;markers.markers.push_back(m);}pub_->publish(a);marker_pub_->publish(markers);RCLCPP_INFO(get_logger(),"CANDIDATE_METRICS revision=%lu regions=%zu coarse=%zu queries=%zu reachable=%zu suppressed=%zu extraction_ms=%.2f stale_results=%lu",cycle_revision_,works_.size(),works_.size(),queries_,reachable_.size(),suppression_.size(),extract_ms_,stale_results_);}
 bool suppressed(uint64_t id,uint64_t rev){auto it=suppression_.find(id);if(it==suppression_.end())return false;if(it->second.revision!=rev&&it->second.expires_ns==0){suppression_.erase(it);return false;}if(it->second.expires_ns&&now().nanoseconds()>it->second.expires_ns){suppression_.erase(it);return false;}return true;}
 void suppress(uint64_t id,uint64_t rev,bool until_revision){if(suppression_.size()>=size_t(maximum_suppression_records_))suppression_.erase(suppression_.begin());suppression_[id]={rev,until_revision?0:now().nanoseconds()+int64_t(unreachable_suppression_s_*1e9)};}
 std::mutex mu_;State state_{State::WAITING_FOR_INPUTS};nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_,latest_cost_,cycle_map_,cycle_cost_;uint64_t map_sum_{0},revision_{0},cycle_revision_{0},stale_results_{0},request_generation_{0},active_request_{0};bool pending_{false};double rx_{0},ry_{0},extract_ms_{0};frontier_exploration_ros2::FrontierCache cache_;std::vector<Work>works_,reachable_;size_t query_index_{0},queries_{0};std::unordered_map<uint64_t,Suppression>suppression_;tf2_ros::Buffer tf_buffer_;tf2_ros::TransformListener tf_listener_;rclcpp_action::Client<Action>::SharedPtr planner_;GoalHandle::SharedPtr active_;rclcpp::TimerBase::SharedPtr timer_,timeout_timer_;rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_,cost_sub_;rclcpp::Publisher<my_epuck_interfaces::msg::FrontierCandidateArray>::SharedPtr pub_;rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
 std::string robot_id_,map_topic_,global_costmap_topic_,global_frame_,robot_base_frame_,compute_path_action_,candidate_topic_,marker_topic_,planner_id_;double processing_rate_hz_,minimum_frontier_length_m_,stable_id_quantization_m_,approach_clearance_m_,minimum_robot_distance_m_,path_query_timeout_s_,gain_weight_,distance_weight_,path_weight_,heading_weight_,unreachable_suppression_s_,goal_tolerance_m_;int free_threshold_,occupied_threshold_,costmap_blocked_threshold_,minimum_frontier_cells_,maximum_candidates_before_path_check_,maximum_path_queries_per_cycle_,maximum_suppression_records_;bool diagonal_connectivity_;
};}
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<my_epuck_frontier_candidates::Generator>());rclcpp::shutdown();}
