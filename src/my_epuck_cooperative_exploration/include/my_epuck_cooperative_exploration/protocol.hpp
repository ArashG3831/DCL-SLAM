#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <my_epuck_interfaces/msg/exploration_claim.hpp>
namespace my_epuck_cooperative_exploration {
using Claim=my_epuck_interfaces::msg::ExplorationClaim; using Uuid=std::array<uint8_t,16>;
enum class ValidationCode{OK,UNEXPECTED_SOURCE,OWN_SOURCE,ZERO_SESSION,RETIRED_SESSION,CONCURRENT_SESSION,DUPLICATE_REVISION,STALE_REVISION,UNSUPPORTED_STATE,INVALID_TTL,NONFINITE,INVALID_BOUNDS,WRONG_FRAME,INVALID_QUATERNION};
enum class ArbitrationResult{LOCAL_WINS,PEER_WINS,NO_CONFLICT};
enum class InternalState{WAITING_FOR_INPUTS,IDLE,PROPOSING,ARBITRATING,NAVIGATING,TERMINAL_BROADCAST,IDLE_STOPPED};
struct ProtocolConfig{std::string own_robot_id,peer_robot_id,global_frame;double min_ttl{.5},max_ttl{10},centroid_tolerance{.15},bbox_margin{.05},path_tie_tolerance{.02};size_t max_retired_sessions{32};};
struct SessionDecision{ValidationCode code{ValidationCode::OK};bool replacement{false};};
Uuid uuid_from_msg(const unique_identifier_msgs::msg::UUID&); bool uuid_zero(const Uuid&); std::string uuid_key(const Uuid&);
bool reserving(uint8_t state); bool terminal(uint8_t state); bool finite_claim_geometry(const Claim&); bool valid_quaternion(const geometry_msgs::msg::Quaternion&);
ValidationCode validate_claim_shape(const Claim&,const ProtocolConfig&);
bool equivalent_frontiers(const Claim&,const Claim&,double centroid_tolerance,double bbox_margin);
ArbitrationResult arbitrate(const Claim&local,const Claim&peer,double centroid_tolerance,double bbox_margin,double tie_tolerance,bool peer_fresh=true);
class PeerSessionTracker {
public: explicit PeerSessionTracker(ProtocolConfig config):config_(std::move(config)){}
 SessionDecision accept(const Claim&,double now_s,bool active_session_fresh);
 bool expired(double now_s)const; std::optional<Claim> claim(double now_s)const; double age(double now_s)const;
 size_t retired_count()const{return retired_.size();} uint64_t highest_revision()const{return highest_revision_;} std::string active_key()const{return active_key_;}
private:void retire_active();ProtocolConfig config_;std::string active_key_;uint64_t highest_revision_{0};double received_at_{0},ttl_{0};std::optional<Claim> claim_;std::deque<std::string> retired_;};
struct RoundCore{InternalState state{InternalState::WAITING_FOR_INPUTS};uint64_t claim_id{0};uint32_t proposals{0};uint32_t goals_sent{0};uint32_t goals_accepted{0};bool budget_consumed{false};bool cancellation_requested{false};
 bool can_propose(uint32_t max)const{return !budget_consumed&&proposals<max;} void create_proposal();bool dispatch_once();bool accept_goal();void terminal_stop();};
}
