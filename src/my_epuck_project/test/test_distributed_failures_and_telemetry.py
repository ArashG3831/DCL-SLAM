"""Failure conservatism, suppression, and travelled-distance tests."""

from my_epuck_project.distributed_assignment.failures import (
    FailureSuppressor,
    classify_failure,
)
from my_epuck_project.distributed_assignment.models import (
    Bounds,
    FailureClass,
    FailureEvidence,
    FailureRecord,
    PhysicalTask,
    TravelDistance,
)


def proposal(signature='x'):
    """Create a task fixture for suppression tests."""
    return PhysicalTask(
        'robot1', 'session', 1, 1, signature, 1, (0, 0),
        Bounds((-0.1, -0.1), (0.1, 0.1)), (0, -0.2),
    )


def record(failure_class, alternative=False):
    """Create a peer failure record fixture."""
    return FailureRecord(
        'robot2', 'peer', 'round', 'task', 'x', (0, -0.2),
        failure_class, 1, 10.0, alternative,
    )


def test_unknown_class_when_diagnostics_are_insufficient():
    """Do not invent a cause from absent evidence."""
    assert classify_failure(FailureEvidence()) == FailureClass.UNKNOWN


def test_generic_action_rejection_is_not_manufactured_planner_diagnosis():
    """Keep a generic goal-handle rejection classified as rejection."""
    result = classify_failure(FailureEvidence(action_rejected=True))
    assert result == FailureClass.ACTION_REJECTION


def test_tf_and_lifecycle_failures_are_transient_and_not_suppressed():
    """Infrastructure startup faults never create task suppression."""
    failure = record(FailureClass.TF_OR_LIFECYCLE)
    suppressor = FailureSuppressor(repeated_evidence_threshold=2)
    assert not suppressor.observe(failure, 1.0)
    assert not suppressor.observe(failure, 2.0)
    assert not suppressor.suppressed(proposal(), 3.0)


def test_repeated_hard_evidence_creates_bounded_suppression():
    """Escalate only repeated hard evidence and expire it."""
    failure = record(FailureClass.HARD_UNREACHABLE)
    suppressor = FailureSuppressor(
        first_duration_s=5.0, maximum_duration_s=20.0,
        repeated_evidence_threshold=2,
    )
    assert not suppressor.observe(failure, 1.0)
    assert suppressor.observe(failure, 2.0)
    assert suppressor.suppressed(proposal(), 3.0)
    assert not suppressor.suppressed(proposal(), 8.0)


def test_peer_hard_failure_blocks_same_signature_without_alternative():
    """Allow peer retry only with an explicit alternative approach."""
    suppressor = FailureSuppressor()
    assert suppressor.suppressed(
        proposal(), 0.0, [record(FailureClass.HARD_UNREACHABLE)],
    )
    assert not suppressor.suppressed(
        proposal(), 0.0,
        [record(FailureClass.HARD_UNREACHABLE, alternative=True)],
    )


def test_actual_travelled_distance_becomes_nonzero_on_motion():
    """Integrate actual odometry displacement instead of emitting zero."""
    distance = TravelDistance(maximum_step_m=1.0)
    distance.observe((0.0, 0.0))
    distance.observe((0.3, 0.4))
    distance.observe((0.6, 0.8))
    assert distance.distance_m == 1.0
