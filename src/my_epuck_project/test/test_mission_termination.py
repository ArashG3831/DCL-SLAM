"""Pure tests for bounded planner evidence and replicated termination."""

from my_epuck_project.distributed_assignment.failures import (
    classify_compute_path_result,
)
from my_epuck_project.distributed_assignment.models import FailureClass
from my_epuck_project.mission_termination import (
    CandidateEvidence,
    FrontierRegionEvidence,
    TerminalReason,
    classify_empty_frontiers,
    matching_terminal_reason,
    recommended_exit_code,
    summarize_frontier_regions,
)


def test_empty_evidence_maps_to_no_frontiers_when_caller_has_seen_both_sources():
    assert classify_empty_frontiers(CandidateEvidence(), CandidateEvidence()) == (
        TerminalReason.NO_FRONTIERS
    )


def test_small_frontiers_are_successfully_distinguished():
    evidence = CandidateEvidence(detected=3, small=3)
    assert classify_empty_frontiers(evidence, CandidateEvidence()) == (
        TerminalReason.ONLY_SMALL_FRONTIERS
    )


def test_out_of_range_frontiers_are_distinguished():
    evidence = CandidateEvidence(detected=2, out_of_range=2)
    assert classify_empty_frontiers(evidence, CandidateEvidence()) == (
        TerminalReason.ONLY_OUT_OF_RANGE_FRONTIERS
    )


def test_unreachable_frontiers_are_distinguished():
    evidence = CandidateEvidence(detected=1, unreachable=1)
    assert classify_empty_frontiers(evidence, CandidateEvidence()) == (
        TerminalReason.NO_REACHABLE_FRONTIERS
    )


def test_planner_failure_does_not_masquerade_as_successful_completion():
    evidence = CandidateEvidence(detected=1, planner_failures=1)
    assert classify_empty_frontiers(evidence, CandidateEvidence()) is None


def test_unclassified_frontier_does_not_masquerade_as_completion():
    evidence = CandidateEvidence(detected=1, unclassified=1)
    assert classify_empty_frontiers(evidence, CandidateEvidence()) is None


def test_detected_not_queried_frontier_does_not_masquerade_as_completion():
    evidence = CandidateEvidence(detected=1, detected_not_queried=1)
    assert classify_empty_frontiers(evidence, CandidateEvidence()) is None


def test_replicas_must_carry_the_same_terminal_reason():
    reason = TerminalReason.NO_FRONTIERS.value
    assert matching_terminal_reason(True, reason, True, reason) == reason
    assert matching_terminal_reason(True, reason, True,
                                    TerminalReason.NO_REACHABLE_FRONTIERS.value) is None
    assert matching_terminal_reason(True, reason, False, reason) is None


def test_terminal_state_blocks_future_dispatch_and_exit_codes_are_explicit():
    assert recommended_exit_code(TerminalReason.NO_FRONTIERS.value) == 0
    assert recommended_exit_code(TerminalReason.PLANNER_INFRASTRUCTURE.value) == 1
    assert recommended_exit_code('MISSION_NOT_TERMINATED') == 2


def test_compute_path_result_classification_preserves_nav2_semantics():
    assert classify_compute_path_result(0) == FailureClass.UNKNOWN
    assert classify_compute_path_result(208) == FailureClass.HARD_UNREACHABLE
    assert classify_compute_path_result(202, tf_unavailable=True) == FailureClass.TF_OR_LIFECYCLE
    assert classify_compute_path_result(207, timed_out=True) == FailureClass.TIMEOUT
    assert classify_compute_path_result(201) == FailureClass.PLANNER_FAILURE


def test_all_small_regions_complete_without_planner_evidence():
    evidence = summarize_frontier_regions(
        [
            FrontierRegionEvidence('a', 0.06, 'DETECTED_NOT_QUERIED'),
            FrontierRegionEvidence('b', 0.08, 'REACHABLE'),
            FrontierRegionEvidence('c', 0.11, 'DETECTED_NOT_QUERIED'),
            FrontierRegionEvidence('d', 0.09, 'OUT_OF_RANGE'),
        ],
        0.15,
    )
    assert evidence.terminal_small == 4
    assert evidence.meaningful_detected_not_queried == 0
    assert classify_empty_frontiers(evidence, CandidateEvidence()) == (
        TerminalReason.ONLY_SMALL_FRONTIERS
    )


def test_large_unqueried_region_blocks_even_with_small_fragments():
    evidence = summarize_frontier_regions(
        [
            FrontierRegionEvidence('tiny', 0.06, 'DETECTED_NOT_QUERIED'),
            FrontierRegionEvidence('large', 0.80, 'DETECTED_NOT_QUERIED'),
        ],
        0.15,
    )
    assert evidence.meaningful_detected_not_queried == 1
    assert classify_empty_frontiers(evidence, CandidateEvidence()) is None


def test_large_unreachable_or_out_of_range_can_be_terminal_with_small_regions():
    unreachable = summarize_frontier_regions(
        [FrontierRegionEvidence('tiny', 0.06, 'DETECTED_NOT_QUERIED'),
         FrontierRegionEvidence('blocked', 0.90, 'UNREACHABLE_SAFE_APPROACH')], 0.15)
    out_of_range = summarize_frontier_regions(
        [FrontierRegionEvidence('tiny', 0.06, 'DETECTED_NOT_QUERIED'),
         FrontierRegionEvidence('far', 19.0, 'OUT_OF_RANGE')], 0.15)
    assert classify_empty_frontiers(unreachable, CandidateEvidence()) == (
        TerminalReason.NO_REACHABLE_FRONTIERS)
    assert classify_empty_frontiers(out_of_range, CandidateEvidence()) == (
        TerminalReason.ONLY_OUT_OF_RANGE_FRONTIERS)


def test_planner_failed_and_meaningful_reachable_region_block():
    failed = summarize_frontier_regions(
        [FrontierRegionEvidence('bad', 0.70, 'PLANNER_FAILED')], 0.15)
    reachable = summarize_frontier_regions(
        [FrontierRegionEvidence('good', 0.60, 'REACHABLE')], 0.15)
    assert classify_empty_frontiers(failed, CandidateEvidence()) is None
    assert classify_empty_frontiers(reachable, CandidateEvidence()) is None


def test_peer_replicas_of_one_physical_frontier_are_counted_once():
    one = summarize_frontier_regions(
        [FrontierRegionEvidence('same', 0.06, 'DETECTED_NOT_QUERIED'),
         FrontierRegionEvidence('same', 0.06, 'DETECTED_NOT_QUERIED')], 0.15)
    assert one.detected == 1
    assert one.terminal_small == 1


def test_reachable_below_gain_is_terminally_acceptable_but_not_actionable():
    evidence = summarize_frontier_regions(
        [FrontierRegionEvidence('large', 1.0, 'REACHABLE', 0.0)],
        0.20,
        0.05,
    )
    assert evidence.reachable == 1
    assert evidence.below_minimum_gain == 1
    assert evidence.actionable_reachable == 0
    assert classify_empty_frontiers(evidence, CandidateEvidence()) == (
        TerminalReason.NO_ACTIONABLE_FRONTIERS
    )


def test_reachable_at_or_above_gain_remains_actionable():
    for gain in (0.05, 0.20):
        evidence = summarize_frontier_regions(
            [FrontierRegionEvidence('large', 1.0, 'REACHABLE', gain)],
            0.20,
            0.05,
        )
        assert evidence.actionable_reachable == 1
        assert evidence.below_minimum_gain == 0
        assert classify_empty_frontiers(evidence, CandidateEvidence()) is None


def test_below_gain_is_invalidated_when_gain_evidence_changes():
    below = summarize_frontier_regions(
        [FrontierRegionEvidence('large', 1.0, 'REACHABLE', 0.0)],
        0.20,
        0.05,
    )
    above = summarize_frontier_regions(
        [FrontierRegionEvidence('large', 1.0, 'REACHABLE', 0.12)],
        0.20,
        0.05,
    )
    assert below.below_minimum_gain == 1
    assert above.below_minimum_gain == 0
    assert above.actionable_reachable == 1


def test_mixed_terminal_categories_allow_completion_when_no_actionable_work_remains():
    evidence = summarize_frontier_regions(
        [
            FrontierRegionEvidence(f'small-{i}', 0.10, 'DETECTED_NOT_QUERIED')
            for i in range(12)
        ]
        + [
            FrontierRegionEvidence(f'unreachable-{i}', 0.90, 'UNREACHABLE_SAFE_APPROACH')
            for i in range(4)
        ]
        + [
            FrontierRegionEvidence(f'far-{i}', 19.0, 'OUT_OF_RANGE')
            for i in range(2)
        ]
        + [
            FrontierRegionEvidence(f'below-{i}', 1.0, 'REACHABLE', 0.0)
            for i in range(2)
        ],
        0.20,
        0.05,
    )
    assert evidence.actionable_reachable == 0
    assert evidence.below_minimum_gain == 2
    assert classify_empty_frontiers(evidence, CandidateEvidence()) == (
        TerminalReason.NO_ACTIONABLE_FRONTIERS
    )


def test_reachable_frontier_without_gain_evidence_still_blocks():
    evidence = summarize_frontier_regions(
        [FrontierRegionEvidence('unknown-gain', 1.0, 'REACHABLE')],
        0.20,
        0.05,
    )
    assert evidence.actionable_reachable == 1
    assert evidence.below_minimum_gain == 0
    assert classify_empty_frontiers(evidence, CandidateEvidence()) is None
