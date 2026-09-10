"""Mock-based tests for the minimal coordinator's local Nav2 boundary."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

from my_epuck_interfaces.msg import FrontierCandidate
import pytest

from my_epuck_project import minimal_frontier_navigation as navigation
from my_epuck_project.distributed_assignment.local_nav2 import (
    FailureClass,
    NavigationOutcome,
)


def _candidate():
    """Create one complete frontier candidate for conversion assertions."""
    def point(x, y):
        return SimpleNamespace(x=x, y=y, z=0.0)

    orientation = SimpleNamespace(
        x=0.0, y=0.0,
        z=math.sin(math.pi / 8.0),
        w=math.cos(math.pi / 8.0),
    )
    return SimpleNamespace(
        frontier_id=42,
        centroid=point(1.0, 2.0),
        bounding_box_min=point(0.5, 1.5),
        bounding_box_max=point(1.5, 2.5),
        approach_pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=point(1.25, 2.25), orientation=orientation,
            ),
        ),
        information_gain=3.5,
        score=4.5,
        mrtsp_route_rank=7,
        mrtsp_route_generation=8,
        mrtsp_solver='test-solver',
        REACHABLE=FrontierCandidate.REACHABLE,
        reachability_state=FrontierCandidate.REACHABLE,
        local_path_length_m=2.75,
        path_length_m=9.0,
        heading_change_rad=0.4,
        local_path_samples=[point(0.0, 0.0), point(1.0, 1.0)],
    )


class _NavMock:
    """Record only the existing LocalNav2 methods used by the adapter."""

    def __init__(self, *, send_result=True, cancel_result=True):
        self.precondition_calls = []
        self.send_calls = []
        self.cancel_calls = 0
        self.send_result = send_result
        self.cancel_result = cancel_result

    def check_dispatch_preconditions(
            self, task, final_path_valid, callback, path_samples, path_frame_id):
        self.precondition_calls.append(
            (task, final_path_valid, callback, path_samples, path_frame_id),
        )

    def send_navigation(self, task, callback, diagnostic_path):
        self.send_calls.append((task, callback, diagnostic_path))
        return self.send_result

    def cancel_navigation(self):
        self.cancel_calls += 1
        return self.cancel_result


def test_candidate_is_converted_to_the_existing_physical_task_shape():
    task = navigation.to_physical_task(_candidate(), 'robot2')

    assert task.source_robot_id == 'robot2'
    assert task.physical_signature == '42'
    assert task.local_frontier_id == 42
    assert task.centroid == (1.0, 2.0)
    assert task.bounds.minimum == (0.5, 1.5)
    assert task.bounds.maximum == (1.5, 2.5)
    assert task.approach == (1.25, 2.25)
    assert task.approach_yaw == pytest.approx(math.pi / 4.0)
    assert task.visible_reveal_gain == 3.5
    assert task.local_ordering_score == 4.5
    assert task.mrtsp_route_rank == 7
    assert task.mrtsp_route_generation == 8
    assert task.mrtsp_solver == 'test-solver'
    assert task.local_path_valid is True
    assert task.local_path_length_m == 2.75
    assert task.local_path == ((0.0, 0.0), (1.0, 1.0))
    assert task.path_heading_cost_rad == 0.4


def test_preconditions_delegate_once_with_all_boundary_arguments():
    nav = _NavMock()
    task = navigation.to_physical_task(_candidate(), 'robot1')
    callback = Mock()
    path = ((0.0, 0.0), (0.5, 0.5))

    result = navigation.check_preconditions(
        nav, task, True, callback, path, 'shared_map',
    )

    assert result is None
    assert nav.precondition_calls == [(task, True, callback, path, 'shared_map')]
    callback.assert_not_called()


def test_send_makes_one_call_and_passes_callback_and_ordinary_failure_through():
    outcome = NavigationOutcome(
        accepted=False,
        status=0,
        error_code=0,
        error_message='NavigateToPose goal rejected',
        failure_class=FailureClass.ACTION_REJECTION,
        duration_s=0.2,
        travelled_distance_m=0.0,
        recoveries=0,
    )
    nav = _NavMock(send_result=True)
    callback = Mock()
    task = navigation.to_physical_task(_candidate(), 'robot1')
    diagnostic_path = ((0.0, 0.0), (1.0, 1.0))

    sent = navigation.send(nav, task, callback, diagnostic_path)
    assert sent is True
    assert nav.send_calls == [(task, callback, diagnostic_path)]

    forwarded_callback = nav.send_calls[0][1]
    forwarded_callback(outcome)
    callback.assert_called_once_with(outcome)


def test_ordinary_failure_does_not_trigger_resend_or_recovery():
    outcome = NavigationOutcome(
        accepted=True,
        status=6,
        error_code=106,
        error_message='controller failed to make progress',
        failure_class=FailureClass.CONTROLLER_NO_PROGRESS,
        duration_s=4.0,
        travelled_distance_m=0.3,
        recoveries=1,
    )
    nav = _NavMock(send_result=True)
    callback = Mock()
    task = navigation.to_physical_task(_candidate(), 'robot1')

    assert navigation.send(nav, task, callback) is True
    nav.send_calls[0][1](outcome)

    assert nav.send_calls == [(task, callback, ())]
    assert nav.cancel_calls == 0
    callback.assert_called_once_with(outcome)


def test_rejected_send_returns_false_without_a_duplicate_attempt():
    nav = _NavMock(send_result=False)
    callback = Mock()
    task = navigation.to_physical_task(_candidate(), 'robot2')

    assert navigation.send(nav, task, callback) is False
    assert nav.send_calls == [(task, callback, ())]
    callback.assert_not_called()


def test_cancel_delegates_once_and_preserves_result():
    nav = _NavMock(cancel_result=True)

    assert navigation.cancel(nav) is True
    assert nav.cancel_calls == 1
