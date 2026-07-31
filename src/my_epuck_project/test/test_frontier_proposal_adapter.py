"""Tests for the stable current-candidate proposal adapter."""

from my_epuck_interfaces.msg import FrontierCandidate

from my_epuck_project.frontier_proposal_adapter import physical_signature


def candidate(local_id=1, approach_x=1.0):
    """Create a candidate wire fixture."""
    value = FrontierCandidate()
    value.frontier_id = local_id
    value.centroid.x = 1.2
    value.centroid.y = 2.3
    value.bounding_box_min.x = 1.0
    value.bounding_box_min.y = 2.0
    value.bounding_box_max.x = 1.4
    value.bounding_box_max.y = 2.6
    value.approach_pose.pose.position.x = approach_x
    value.approach_pose.pose.position.y = 2.0
    return value


def test_signature_ignores_transient_frontier_id():
    """Physical identity is independent from source-local frontier IDs."""
    assert physical_signature(candidate(1)) == physical_signature(candidate(999))


def test_signature_quantizes_centimetre_noise():
    """Small approach jitter remains the same physical signature."""
    assert physical_signature(candidate(1, 1.001)) == physical_signature(
        candidate(2, 1.002),
    )


def test_signature_changes_for_materially_different_goal():
    """A displaced approach changes physical identity."""
    assert physical_signature(candidate(1, 1.0)) != physical_signature(
        candidate(1, 1.3),
    )
