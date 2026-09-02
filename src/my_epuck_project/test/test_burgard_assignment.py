"""Pure deterministic regressions for the production Burgard adaptation."""

from types import SimpleNamespace

from my_epuck_project.distributed_assignment.burgard_assignment import (
    choose_burgard_assignment,
    line_of_sight_clear,
)
from my_epuck_project.distributed_assignment.canonical import build_canonical_union
from my_epuck_project.distributed_assignment.models import Bid, BidBatch, Bounds, PhysicalTask
from my_epuck_project.distributed_assignment.scoring import decisions_match


def _task(robot, signature, centroid, *, local_id=1, gain=0.2):
    return PhysicalTask(
        robot, '1' * 32 if robot == 'robot1' else '2' * 32, 1, 1,
        signature, local_id, centroid, Bounds(
            (centroid[0] - 0.1, centroid[1] - 0.1),
            (centroid[0] + 0.1, centroid[1] + 0.1),
        ), centroid, visible_reveal_gain=gain, local_path_valid=True,
    )


def _batch(robot, union, lengths):
    return BidBatch(
        'round', union.union_hash, robot,
        '1' * 32 if robot == 'robot1' else '2' * 32, 1, 3.0,
        tuple(Bid(task_id, True, length, length, path=((0.0, 0.0), (length, 0.0)))
              for task_id, length in lengths.items()),
    )


def _ids(union):
    return {member.physical_signature: task.canonical_id
            for task in union.tasks for member in task.members}


def _grid(*, width=8, height=8, resolution=1.0, origin=(0.0, 0.0), yaw=0.0, data=None):
    orientation = SimpleNamespace(
        x=0.0, y=0.0, z=__import__('math').sin(yaw / 2.0),
        w=__import__('math').cos(yaw / 2.0),
    )
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width, height=height, resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin[0], y=origin[1]),
                orientation=orientation,
            ),
        ),
        data=list(data if data is not None else [0] * (width * height)),
    )


def _solve(union, first, second, **kwargs):
    return choose_burgard_assignment(
        'round', union, first, second, shared_map=_grid(), **kwargs,
    )


def test_one_robot_one_task_assigns_and_other_is_idle():
    task = _task('robot1', 'only', (2.0, 0.0))
    union = build_canonical_union([task], [])
    task_id = union.tasks[0].canonical_id
    decision = _solve(
        union, _batch('robot1', union, {task_id: 2.0}),
        _batch('robot2', union, {}),
    )
    assert decision.robot1_task_id == task_id
    assert decision.robot2_task_id == ''
    assert decision.diagnostics.strategy == 'burgard'
    assert decision.diagnostics.beta == 1.0


def test_two_robots_choose_obvious_nearest_tasks():
    west, east = _task('robot1', 'west', (-2.0, 0.0)), _task('robot2', 'east', (2.0, 0.0))
    union = build_canonical_union([west], [east])
    ids = _ids(union)
    decision = _solve(
        union,
        _batch('robot1', union, {ids['west']: 1.0, ids['east']: 10.0}),
        _batch('robot2', union, {ids['west']: 10.0, ids['east']: 1.0}),
    )
    assert (decision.robot1_task_id, decision.robot2_task_id) == (ids['west'], ids['east'])


def test_exact_pair_search_avoids_greedy_same_task_collision():
    """The bounded pair solver evaluates the complete team before commit."""
    first = _task('robot1', 'first', (-20.0, 0.0))
    second = _task('robot2', 'second', (20.0, 0.0))
    union = build_canonical_union([first], [second])
    ids = _ids(union)
    decision = _solve(
        union,
        _batch('robot1', union, {ids['first']: 1.0, ids['second']: 2.0}),
        _batch('robot2', union, {ids['first']: 1.01, ids['second']: 10.0}),
    )
    # A greedy first pick of ``first`` leaves Robot 2 with the very costly
    # ``second``.  The exact two-robot search keeps both useful assignments.
    assert (decision.robot1_task_id, decision.robot2_task_id) == (
        ids['second'], ids['first'])
    assert decision.diagnostics.strategy == 'burgard'


def test_reduction_after_shared_preference_selects_distinct_task():
    shared, alternate = _task('robot1', 'shared', (0.0, 0.0)), _task('robot2', 'alternate', (1.0, 0.0))
    union = build_canonical_union([shared], [alternate])
    ids = _ids(union)
    decision = _solve(
        union,
        _batch('robot1', union, {ids['shared']: 1.0, ids['alternate']: 8.0}),
        _batch('robot2', union, {ids['shared']: 1.2, ids['alternate']: 2.0}),
    )
    assert decision.robot1_task_id == ids['shared']
    assert decision.robot2_task_id == ids['alternate']
    reduction = decision.diagnostics.burgard_trace[0]['reductions'][0]
    assert reduction['reduction'] > 0.0
    assert reduction['utility_after'] < reduction['utility_before']


def test_los_obstacle_prevents_reduction_and_rotated_origin_is_respected():
    data = [0] * 64
    data[1 * 8 + 2] = 100
    rotated = _grid(origin=(-2.0, -2.0), yaw=__import__('math').pi / 2.0, data=data)
    # Local cell line y=1.5 becomes a vertical world line after the rotated origin.
    assert not line_of_sight_clear(rotated, (-3.5, -1.5), (-3.5, 1.5), 50)
    first, second = _task('robot1', 'first', (1.0, 1.0)), _task('robot2', 'second', (3.0, 1.0))
    union = build_canonical_union([first], [second])
    ids = _ids(union)
    blocked = [0] * 64
    blocked[1 * 8 + 2] = 100
    decision = choose_burgard_assignment(
        'round', union,
        _batch('robot1', union, {ids['first']: 1.0}),
        _batch('robot2', union, {ids['second']: 1.0}),
        shared_map=_grid(data=blocked), sensor_max_range_m=11.98,
    )
    assert decision.diagnostics.burgard_trace[0]['reductions'][0]['reduction'] == 0.0
    assert not decision.diagnostics.burgard_trace[0]['reductions'][0]['line_of_sight_clear']


def test_out_of_range_task_receives_no_reduction():
    first, second = _task('robot1', 'first', (0.0, 0.0)), _task('robot2', 'second', (20.0, 0.0))
    union = build_canonical_union([first], [second])
    ids = _ids(union)
    decision = _solve(
        union,
        _batch('robot1', union, {ids['first']: 1.0}),
        _batch('robot2', union, {ids['second']: 1.0}),
    )
    assert decision.diagnostics.burgard_trace[0]['reductions'][0]['reduction'] == 0.0


def test_hard_failure_suppression_excludes_pair():
    failed, useful = _task('robot1', 'failed', (1.0, 0.0)), _task('robot2', 'useful', (4.0, 0.0))
    union = build_canonical_union([failed], [useful])
    ids = _ids(union)
    decision = _solve(
        union,
        _batch('robot1', union, {ids['failed']: 1.0, ids['useful']: 2.0}),
        _batch('robot2', union, {ids['failed']: 1.0, ids['useful']: 2.0}),
        hard_failed_tasks=frozenset({ids['failed']}),
    )
    assert ids['failed'] not in (decision.robot1_task_id, decision.robot2_task_id)


def test_same_canonical_task_is_never_double_assigned():
    first = _task('robot1', 'opening-a', (0.0, 0.0))
    second = _task('robot2', 'opening-b', (0.1, 0.0))
    union = build_canonical_union([first], [second])
    assert len(union.tasks) == 1
    task_id = union.tasks[0].canonical_id
    decision = _solve(
        union, _batch('robot1', union, {task_id: 1.0}),
        _batch('robot2', union, {task_id: 1.0}),
    )
    assert sum(bool(value) for value in (decision.robot1_task_id, decision.robot2_task_id)) == 1


def test_equal_score_tie_and_replica_result_are_deterministic():
    first, second = _task('robot1', 'a', (-2.0, 0.0)), _task('robot2', 'b', (2.0, 0.0))
    union = build_canonical_union([first], [second])
    ids = _ids(union)
    one = _solve(
        union,
        _batch('robot1', union, {ids['a']: 1.0, ids['b']: 1.0}),
        _batch('robot2', union, {ids['a']: 1.0, ids['b']: 1.0}),
    )
    two = _solve(
        union,
        _batch('robot1', union, {ids['b']: 1.0, ids['a']: 1.0}),
        _batch('robot2', union, {ids['b']: 1.0, ids['a']: 1.0}),
    )
    assert decisions_match(one, two)
    assert one.robot1_task_id == min(ids.values())


def test_normal_production_launch_defaults_to_burgard_beta_one_and_traffic_deferred():
    """Keep the normal launch on Burgard with unvalidated traffic deferred."""
    source = open(
        'src/my_epuck_project/launch/two_robots_distributed_assignment_launch.py',
        encoding='utf-8',
    ).read()
    assert "'assignment_strategy', default_value='frontier_mrtsp'" in source
    assert "'frontier_cost_only', 'frontier_mrtsp'" in source
    assert "DeclareLaunchArgument('burgard_beta', default_value='1.0')" in source
    assert "DeclareLaunchArgument('traffic_scheduler_enabled', default_value='false'" in source
    assert "'maximum_solo_path_m': 18.0" not in source
    assert "path_cost_scale_m" in source
