#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "my_epuck_cooperative_exploration/protocol.hpp"

namespace m = my_epuck_cooperative_exploration;

static m::Claim claim(
  const std::string & robot = "robot2", uint8_t session_byte = 2,
  uint64_t revision = 1, uint8_t state = m::Claim::PROPOSING)
{
  m::Claim c;
  c.source_robot_id = robot;
  c.source_session_id.uuid[0] = session_byte;
  c.message_revision = revision;
  c.claim_id = 3;
  c.state = state;
  c.frontier_id = 42;
  c.map_revision = 99;
  c.frontier_centroid.x = 1.0;
  c.frontier_centroid.y = 2.0;
  c.bounding_box_min.x = 0.9;
  c.bounding_box_min.y = 1.9;
  c.bounding_box_max.x = 1.1;
  c.bounding_box_max.y = 2.1;
  c.approach_pose.header.frame_id = "shared_map";
  c.approach_pose.pose.position.x = 0.8;
  c.approach_pose.pose.position.y = 2.0;
  c.approach_pose.pose.orientation.w = 1.0;
  c.path_length_m = 1.0F;
  c.information_gain = 0.5F;
  c.utility_score = 0.25F;
  c.claim_ttl.sec = 3;
  return c;
}

static m::ProtocolConfig config()
{
  return {"robot1", "robot2", "shared_map", 0.5, 10.0, 0.15, 0.05, 0.02, 2};
}

static m::Status status(
  uint8_t session_byte = 2, uint64_t revision = 1,
  uint8_t state = m::Status::ACTIVE)
{
  m::Status s;
  s.header.frame_id = "shared_map";
  s.source_robot_id = "robot2";
  s.source_session_id.uuid[0] = session_byte;
  s.message_revision = revision;
  s.state = state;
  s.status_ttl.sec = 3;
  return s;
}

TEST(ClaimValidation, RejectsMalformedWithoutAcceptingIt)
{
  auto cfg = config();
  EXPECT_EQ(m::validate_claim_shape(claim(), cfg), m::ValidationCode::OK);
  EXPECT_EQ(
    m::validate_claim_shape(claim("other"), cfg),
    m::ValidationCode::UNEXPECTED_SOURCE);
  EXPECT_EQ(
    m::validate_claim_shape(claim("robot1"), cfg),
    m::ValidationCode::OWN_SOURCE);

  auto zero = claim();
  zero.source_session_id.uuid.fill(0);
  EXPECT_EQ(m::validate_claim_shape(zero, cfg), m::ValidationCode::ZERO_SESSION);

  auto unsupported = claim();
  unsupported.state = m::Claim::UNKNOWN;
  EXPECT_EQ(
    m::validate_claim_shape(unsupported, cfg),
    m::ValidationCode::UNSUPPORTED_STATE);

  auto ttl = claim();
  ttl.claim_ttl.sec = 11;
  EXPECT_EQ(m::validate_claim_shape(ttl, cfg), m::ValidationCode::INVALID_TTL);
  ttl.claim_ttl.sec = 0;
  EXPECT_EQ(m::validate_claim_shape(ttl, cfg), m::ValidationCode::INVALID_TTL);

  auto numeric = claim();
  numeric.path_length_m = std::numeric_limits<float>::infinity();
  EXPECT_EQ(m::validate_claim_shape(numeric, cfg), m::ValidationCode::NONFINITE);
  numeric = claim();
  numeric.information_gain = NAN;
  EXPECT_EQ(m::validate_claim_shape(numeric, cfg), m::ValidationCode::NONFINITE);
  numeric = claim();
  numeric.utility_score = NAN;
  EXPECT_EQ(m::validate_claim_shape(numeric, cfg), m::ValidationCode::NONFINITE);

  auto frame = claim();
  frame.approach_pose.header.frame_id = "map";
  EXPECT_EQ(m::validate_claim_shape(frame, cfg), m::ValidationCode::WRONG_FRAME);
  auto quaternion = claim();
  quaternion.approach_pose.pose.orientation.w = 0.0;
  EXPECT_EQ(
    m::validate_claim_shape(quaternion, cfg),
    m::ValidationCode::INVALID_QUATERNION);
  auto bounds = claim();
  bounds.bounding_box_min.x = bounds.bounding_box_max.x = 1.0;
  bounds.bounding_box_min.y = bounds.bounding_box_max.y = 2.0;
  EXPECT_EQ(
    m::validate_claim_shape(bounds, cfg),
    m::ValidationCode::INVALID_BOUNDS);
}

TEST(ClaimValidation, AcceptsExactTtlBoundsAndRejectsOutside)
{
  auto cfg = config();
  auto value = claim();
  value.claim_ttl.sec = 0;
  value.claim_ttl.nanosec = 500000000;
  EXPECT_EQ(m::validate_claim_shape(value, cfg), m::ValidationCode::OK);
  value.claim_ttl.sec = 10;
  value.claim_ttl.nanosec = 0;
  EXPECT_EQ(m::validate_claim_shape(value, cfg), m::ValidationCode::OK);
  value.claim_ttl.sec = 0;
  value.claim_ttl.nanosec = 499999999;
  EXPECT_EQ(m::validate_claim_shape(value, cfg), m::ValidationCode::INVALID_TTL);
  value.claim_ttl.sec = 10;
  value.claim_ttl.nanosec = 1;
  EXPECT_EQ(m::validate_claim_shape(value, cfg), m::ValidationCode::INVALID_TTL);
}

TEST(SessionTracker, RevisionsExpiryReplacementAndRetirement)
{
  m::PeerSessionTracker tracker(config());
  EXPECT_EQ(tracker.accept(claim("robot2", 2, 1), 10.0, false).code, m::ValidationCode::OK);
  EXPECT_EQ(
    tracker.accept(claim("robot2", 2, 1), 10.1, true).code,
    m::ValidationCode::DUPLICATE_REVISION);
  EXPECT_NEAR(tracker.age(10.1), 0.1, 1e-9);  // rejection did not refresh TTL
  EXPECT_EQ(
    tracker.accept(claim("robot2", 2, 0), 10.2, true).code,
    m::ValidationCode::STALE_REVISION);
  EXPECT_EQ(
    tracker.accept(claim("robot2", 3, 1), 10.2, true).code,
    m::ValidationCode::CONCURRENT_SESSION);
  EXPECT_EQ(tracker.accept(claim("robot2", 2, 9), 10.3, true).code, m::ValidationCode::OK);
  EXPECT_EQ(tracker.highest_revision(), 9U);
  EXPECT_FALSE(tracker.expired(12.9));
  EXPECT_TRUE(tracker.expired(13.4));

  auto replacement = tracker.accept(claim("robot2", 3, 1), 13.4, false);
  EXPECT_EQ(replacement.code, m::ValidationCode::OK);
  EXPECT_TRUE(replacement.replacement);
  EXPECT_EQ(tracker.highest_revision(), 1U);  // replacement session has an independent baseline
  EXPECT_EQ(tracker.retired_count(), 1U);
  EXPECT_EQ(
    tracker.accept(claim("robot2", 2, 9), 13.5, true).code,
    m::ValidationCode::RETIRED_SESSION);

  EXPECT_TRUE(tracker.accept(claim("robot2", 4, 1), 17.0, false).replacement);
  EXPECT_TRUE(tracker.accept(claim("robot2", 5, 1), 21.0, false).replacement);
  EXPECT_EQ(tracker.retired_count(), 2U);  // deterministic bounded eviction
  EXPECT_EQ(
    tracker.accept(claim("robot2", 3, 99), 21.1, true).code,
    m::ValidationCode::RETIRED_SESSION);
}

TEST(StatusTracker, ExpiryRestartRetirementAndRevisionReset)
{
  m::PeerStatusTracker tracker(config());
  EXPECT_EQ(tracker.accept(status(2, 4), 1.0, false).code, m::ValidationCode::OK);
  EXPECT_EQ(
    tracker.accept(status(2, 3), 1.1, true).code,
    m::ValidationCode::STALE_REVISION);
  EXPECT_FALSE(tracker.expired(3.9));
  EXPECT_TRUE(tracker.expired(4.1));
  auto replacement = tracker.accept(status(7, 1), 4.1, false);
  EXPECT_EQ(replacement.code, m::ValidationCode::OK);
  EXPECT_TRUE(replacement.replacement);
  EXPECT_EQ(
    tracker.accept(status(2, 99), 4.2, true).code,
    m::ValidationCode::RETIRED_SESSION);
  ASSERT_TRUE(tracker.status(4.2));
  EXPECT_EQ(tracker.status(4.2)->message_revision, 1U);
}

TEST(StatusValidation, RequiresPeerIdentityFrameStateAndBoundedTtl)
{
  EXPECT_EQ(m::validate_status_shape(status(), config()), m::ValidationCode::OK);
  auto own = status();
  own.source_robot_id = "robot1";
  EXPECT_EQ(
    m::validate_status_shape(own, config()), m::ValidationCode::OWN_SOURCE);
  auto frame = status();
  frame.header.frame_id = "map";
  EXPECT_EQ(
    m::validate_status_shape(frame, config()), m::ValidationCode::WRONG_FRAME);
  auto unsupported = status();
  unsupported.state = 99;
  EXPECT_EQ(
    m::validate_status_shape(unsupported, config()),
    m::ValidationCode::UNSUPPORTED_STATE);
}

TEST(FrontierEquivalence, IdGeometryToleranceAndInvalidBounds)
{
  auto a = claim("robot1", 1);
  auto b = claim();
  EXPECT_TRUE(m::equivalent_frontiers(a, b, 0.15, 0.05));
  b.frontier_id = 43;
  EXPECT_TRUE(m::equivalent_frontiers(a, b, 0.15, 0.05));
  b.frontier_centroid.x += 0.15;
  b.bounding_box_min.x += 0.15;
  b.bounding_box_max.x += 0.15;
  EXPECT_TRUE(m::equivalent_frontiers(a, b, 0.15, 0.05));  // exact boundary
  b.bounding_box_min.x = 2.0;
  b.bounding_box_max.x = 2.2;
  EXPECT_FALSE(m::equivalent_frontiers(a, b, 0.15, 0.05));
  b = claim();
  b.frontier_id = 43;
  b.frontier_centroid.x = -4.0;
  EXPECT_FALSE(m::equivalent_frontiers(a, b, 0.15, 0.05));
  b = claim();
  b.frontier_id = 43;
  b.bounding_box_min.x = b.bounding_box_max.x;
  b.bounding_box_min.y = b.bounding_box_max.y;
  EXPECT_FALSE(m::equivalent_frontiers(a, b, 0.15, 0.05));
  b = claim();
  b.frontier_centroid.x = NAN;
  EXPECT_FALSE(m::equivalent_frontiers(a, b, 0.15, 0.05));
}

TEST(Arbitration, CostTieRobotIdSymmetryAndIgnoredProvenance)
{
  auto a = claim("robot1", 1);
  auto b = claim("robot2", 2);
  b.path_length_m = 1.2F;
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02), m::ArbitrationResult::LOCAL_WINS);
  EXPECT_EQ(m::arbitrate(b, a, .15, .05, .02), m::ArbitrationResult::PEER_WINS);
  b.path_length_m = 1.02F;
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02), m::ArbitrationResult::LOCAL_WINS);
  a.source_robot_id = "robot9";
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02), m::ArbitrationResult::PEER_WINS);

  a = claim("robot1", 9, 900);
  b = claim("robot2", 3, 1);
  a.claim_id = 999;
  a.map_revision = 9999;
  a.header.stamp.sec = 100;
  b.claim_id = 1;
  b.map_revision = 1;
  b.header.stamp.sec = 0;
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02), m::ArbitrationResult::LOCAL_WINS);
  b.state = m::Claim::RELEASED;
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02), m::ArbitrationResult::NO_CONFLICT);
  b.state = m::Claim::PROPOSING;
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02, false), m::ArbitrationResult::NO_CONFLICT);
  b.frontier_id = 99;
  b.frontier_centroid.x = 8.0;
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02), m::ArbitrationResult::NO_CONFLICT);
}

TEST(Arbitration, DefensiveNonFinitePaths)
{
  auto a = claim("robot1", 1);
  auto b = claim("robot2", 2);
  a.path_length_m = NAN;
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02), m::ArbitrationResult::PEER_WINS);
  b.path_length_m = NAN;
  EXPECT_EQ(m::arbitrate(a, b, .15, .05, .02), m::ArbitrationResult::LOCAL_WINS);
}

TEST(RoundCore, ProposalBudgetAndExactlyOneAcceptedGoal)
{
  m::RoundCore round;
  EXPECT_TRUE(round.can_propose(3));
  round.create_proposal();
  EXPECT_EQ(round.state, m::InternalState::PROPOSING);
  EXPECT_EQ(round.claim_id, 1U);
  EXPECT_TRUE(round.dispatch_once());
  EXPECT_FALSE(round.dispatch_once());
  EXPECT_TRUE(round.accept_goal());
  EXPECT_FALSE(round.accept_goal());
  EXPECT_TRUE(round.budget_consumed);
  EXPECT_EQ(round.goals_accepted, 1U);
  EXPECT_FALSE(round.can_propose(3));
  round.terminal_stop();
  EXPECT_EQ(round.state, m::InternalState::IDLE_STOPPED);
}

TEST(RoundCore, ContinuousResetAllowsSequentialButNeverConcurrentGoals)
{
  m::RoundCore core;
  core.create_proposal();
  EXPECT_TRUE(core.accept_goal(true));
  EXPECT_FALSE(core.accept_goal(true));
  const auto first = core.claim_id;
  core.reset_cycle();
  core.create_proposal();
  EXPECT_GT(core.claim_id, first);
  EXPECT_TRUE(core.accept_goal(true));
  EXPECT_EQ(core.goals_accepted, 1U);
}

TEST(ActionCallbackIsolation, DelayedGoalResponseFromCycleOneIsRejected)
{
  EXPECT_FALSE(m::action_callback_matches(
    1, 10, 2, 11, m::InternalState::ARBITRATING,
    m::InternalState::ARBITRATING));
}

TEST(ActionCallbackIsolation, DelayedResultFromCycleOneIsRejected)
{
  EXPECT_FALSE(m::action_callback_matches(
    1, 10, 2, 11, m::InternalState::NAVIGATING,
    m::InternalState::NAVIGATING));
}

TEST(ActionCallbackIsolation, DuplicateTerminalCallbackIsRejected)
{
  EXPECT_FALSE(m::action_callback_matches(
    2, 11, 2, 11, m::InternalState::TERMINAL_BROADCAST,
    m::InternalState::NAVIGATING));
}

TEST(ActionCallbackIsolation, DelayedCancellationAcknowledgementIsRejected)
{
  EXPECT_FALSE(m::action_callback_matches(
    1, 10, 2, 11, m::InternalState::NAVIGATING,
    m::InternalState::NAVIGATING));
}

TEST(ActionCallbackIsolation, FeedbackForInactiveGoalIsRejected)
{
  EXPECT_FALSE(m::action_callback_matches(
    2, 11, 2, 11, m::InternalState::COOLDOWN,
    m::InternalState::NAVIGATING));
  EXPECT_TRUE(m::action_callback_matches(
    2, 11, 2, 11, m::InternalState::NAVIGATING,
    m::InternalState::NAVIGATING));
}

TEST(Nav2FailureClassification, UsesStructuredPlannerEvidence)
{
  EXPECT_EQ(m::classify_nav2_failure("compute_path planner failed"), "PLANNER_FAILURE");
}

TEST(Nav2FailureClassification, UsesStructuredControllerEvidence)
{
  EXPECT_EQ(m::classify_nav2_failure("follow_path controller failed"), "CONTROLLER_FAILURE");
}

TEST(Nav2FailureClassification, CodeZeroWithoutEvidenceRemainsUnknown)
{
  EXPECT_EQ(m::classify_nav2_failure(""), "UNKNOWN_NAV2_FAILURE");
}

TEST(Nav2FailureClassification, PreservesSpecificFailureSignals)
{
  EXPECT_EQ(m::classify_nav2_failure("failed progress checker"), "PROGRESS_CHECK_FAILURE");
  EXPECT_EQ(m::classify_nav2_failure("recovery behavior exhausted"), "RECOVERY_FAILURE");
  EXPECT_EQ(m::classify_nav2_failure("transform unavailable"), "TRANSFORM_FAILURE");
}
