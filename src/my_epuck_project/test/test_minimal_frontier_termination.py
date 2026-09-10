"""Focused contract tests for the minimal termination adapter."""

from unittest.mock import Mock

import pytest

from my_epuck_project import minimal_frontier_termination as termination
from my_epuck_project.mission_termination import (
    CandidateEvidence,
    TerminalReason,
)


STABLE_FOR = 5.0
STABILITY_GRACE = 5.0


def classify(first, second=CandidateEvidence(), *, stable_for=STABLE_FOR):
    """Apply the spec's stable/unstable fixture to the adapter."""
    return termination.classify(
        first,
        second,
        stable_for_s=stable_for,
        stability_grace_s=STABILITY_GRACE,
    )


def test_reuses_the_conservative_shared_classifier(monkeypatch):
    shared_classifier = Mock(return_value=TerminalReason.NO_ACTIONABLE_FRONTIERS)
    monkeypatch.setattr(
        termination, 'classify_empty_frontiers', shared_classifier,
    )

    first = CandidateEvidence(detected=1, below_minimum_gain=1)
    second = CandidateEvidence()

    assert classify(first, second) is TerminalReason.NO_ACTIONABLE_FRONTIERS
    shared_classifier.assert_called_once_with(first, second)


@pytest.mark.parametrize(
    ('label', 'evidence'),
    [
        (
            'actionable reachable',
            CandidateEvidence(detected=1, reachable=1, actionable_reachable=1),
        ),
        (
            'unresolved planner evidence',
            CandidateEvidence(detected=1, planner_failures=1),
        ),
        (
            'unclassified',
            CandidateEvidence(detected=1, unclassified=1),
        ),
        (
            'pending not-queried evidence',
            CandidateEvidence(detected=1, detected_not_queried=1),
        ),
        (
            'meaningful unqueried evidence',
            CandidateEvidence(detected=1, meaningful_detected_not_queried=1),
        ),
    ],
)
def test_unresolved_or_actionable_evidence_blocks_even_when_stable(label, evidence):
    assert classify(evidence) is None, label


def test_stable_conclusively_unreachable_evidence_gets_the_spec_exception():
    evidence = CandidateEvidence(detected=3, unreachable=3)

    assert classify(evidence) is TerminalReason.NO_REACHABLE_FRONTIERS


def test_incomplete_unreachable_counts_are_not_conclusive():
    evidence = CandidateEvidence(detected=3, unreachable=2)

    assert classify(evidence) is None


@pytest.mark.parametrize(
    ('evidence', 'expected'),
    [
        (CandidateEvidence(), TerminalReason.NO_FRONTIERS),
        (
            CandidateEvidence(
                detected=1,
                reachable=1,
                below_minimum_gain=1,
                actionable_reachable=0,
            ),
            TerminalReason.NO_ACTIONABLE_FRONTIERS,
        ),
    ],
)
def test_stable_empty_or_no_actionable_evidence_may_terminate(evidence, expected):
    assert classify(evidence) is expected


@pytest.mark.parametrize(
    'evidence',
    [
        CandidateEvidence(),
        CandidateEvidence(detected=1, unreachable=1),
        CandidateEvidence(
            detected=1,
            reachable=1,
            below_minimum_gain=1,
            actionable_reachable=0,
        ),
    ],
)
def test_unstable_evidence_never_terminates_early(evidence):
    assert classify(evidence, stable_for=STABILITY_GRACE - 0.001) is None


def test_matching_normalizes_enum_and_string_reasons_to_a_plain_string():
    reason = TerminalReason.NO_FRONTIERS

    result = termination.matching(
        True, reason, True, reason.value,
    )

    assert type(result) is str
    assert result == reason.value
    assert termination.matching(
        True, reason.value, True, TerminalReason.NO_FRONTIERS.value,
    ) == reason.value
    assert termination.matching(
        True, reason.value, True, TerminalReason.NO_ACTIONABLE_FRONTIERS.value,
    ) is None
    assert termination.matching(True, reason.value, False, reason.value) is None


def test_adapter_has_no_terminal_ack_or_proof_surface():
    public_names = {
        name for name in vars(termination) if not name.startswith('_')
    }

    assert not any(
        marker in name.lower()
        for name in public_names
        for marker in ('ack', 'proof')
    )
