import json
import signal
import threading

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from my_epuck_interfaces.msg import ExplorationEvent, ExplorationStatus
from tf2_ros import TransformException

from my_epuck_project.cooperative_experiment_logger import (
    CooperativeExperimentLogger, create_logger_executor,
    is_shutdown_conversion_error)


@pytest.fixture
def observer(tmp_path):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"output_root:={tmp_path}",
            "-p",
            "enable_console_status:=false",
            # These lifecycle/frontend tests do not provide local occupancy
            # snapshots.  Keep optional forensic capture off here so a bare
            # logger test exercises its own contract rather than failing on
            # deliberately absent map evidence.
            "-p",
            "enable_local_map_capture:=false",
        ]
    )
    node = CooperativeExperimentLogger()
    yield node
    if not node.finalized:
        node.finalize(False)
    node.destroy_node()
    rclpy.shutdown()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_events(observer):
    observer.events.flush()
    return [
        json.loads(line)
        for line in (observer.directory / "events.jsonl").read_text(
            encoding="utf-8"
        ).splitlines()
    ]


def test_goal_decision_ledger_mirrors_navigation_decision_events(observer):
    observer.event(
        'NAV_GOAL_SENT', 'local goal dispatched', 'robot1',
        canonical_task_id='task-1', physical_task_signature='physical-1',
        path_length_m=2.5, map_revision=7,
    )
    observer.goal_decisions.flush()
    rows = [
        json.loads(line)
        for line in (observer.directory / 'goal_decision_ledger.jsonl').read_text(
            encoding='utf-8').splitlines()
    ]
    assert len(rows) == 1
    assert rows[0]['decision_stage'] == 'NAV_GOAL_SENT'
    assert rows[0]['robot_id'] == 'robot1'
    assert rows[0]['canonical_task_id'] == 'task-1'
    assert rows[0]['physical_task_signature'] == 'physical-1'
    assert rows[0]['path_length_m'] == 2.5


def test_goal_accounting_exposes_active_and_terminal_goals_per_robot(observer):
    fields = dict(round_id='round-1', canonical_task_id='task-1',
                  physical_task_signature='physical-1')
    observer.event('NAV_GOAL_SENT', 'sent', 'robot1', **fields)
    observer.event('NAV_GOAL_ACCEPTED', 'accepted', 'robot1', **fields)
    observer.event('NAVIGATION_SUCCEEDED', 'done', 'robot1', **fields)
    observer.event('NAV_GOAL_SENT', 'sent', 'robot1', round_id='round-2',
                   canonical_task_id='task-2', physical_task_signature='physical-2')
    observer.latest['robot1']['navigation_active'] = True
    accounting = observer.goal_accounting_summary()
    assert accounting['by_robot']['robot1']['dispatched'] == 2
    assert accounting['by_robot']['robot1']['categories'] == {
        'SUCCEEDED': 1, 'STILL_ACTIVE_AT_MISSION_END': 1}
    assert accounting['by_robot']['robot2']['dispatched'] == 0


def test_goal_accounting_handles_acceptance_before_send(observer):
    fields = dict(round_id='round-before-send', canonical_task_id='task-before-send',
                  physical_task_signature='physical-before-send')
    observer.event('NAV_GOAL_ACCEPTED', 'accepted before dispatch record',
                   'robot2', **fields)
    observer.event('NAV_GOAL_SENT', 'sent after acceptance record',
                   'robot2', **fields)
    record = observer.goal_accounting_summary()['records'][0]
    assert record['accepted'] is True
    assert record['terminal_category'] == 'UNKNOWN_OR_UNACCOUNTED'


def test_odom_motion_accounting_uses_local_odom_before_shared_tf(observer,
                                                                  monkeypatch):
    def missing_transform(*args, **kwargs):
        raise TransformException('shared frame unavailable before handoff')

    monkeypatch.setattr(observer.tf_buffer, 'lookup_transform',
                        missing_transform)
    first = Odometry()
    first.header.frame_id = 'robot1/odom'
    first.pose.pose.position.x = 0.0
    first.pose.pose.position.y = 0.0
    first.twist.twist.linear.x = 0.12
    first.twist.twist.angular.z = 0.20
    second = Odometry()
    second.header.frame_id = 'robot1/odom'
    second.pose.pose.position.x = 0.3
    second.pose.pose.position.y = 0.4
    second.twist.twist.linear.x = 0.08
    second.twist.twist.angular.z = -0.10
    observer.odom('robot1', first)
    observer.odom('robot1', second)
    assert observer.latest['robot1']['pose'][:2] == (0.3, 0.4)
    assert observer.latest['robot1']['speed'] == (0.08, -0.10)
    assert observer.latest['robot1']['pose_frame'] == 'robot1/odom'
    assert observer.local_trajectory.total_distance['robot1'] == pytest.approx(0.5)


def test_logger_uses_stable_executor_for_shutdown_pybind_regression():
    """The experimental EventsExecutor conversion crash is not used."""
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    try:
        executor = create_logger_executor()
        assert isinstance(executor, SingleThreadedExecutor)
        executor.shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_logger_executor_is_bound_to_explicit_context():
    from rclpy.context import Context
    context = Context()
    rclpy.init(context=context)
    try:
        executor = create_logger_executor(context)
        assert executor.context is context
        executor.shutdown()
    finally:
        if context.ok():
            context.shutdown()


def test_logger_conversion_signature_is_only_accepted_after_context_loss():
    error = RuntimeError('Unable to convert call argument')
    assert is_shutdown_conversion_error(error, True, False)
    assert not is_shutdown_conversion_error(error, False, False)
    assert not is_shutdown_conversion_error(error, True, True)
    assert not is_shutdown_conversion_error(
        RuntimeError('application callback failure'), True, False)


def test_forensic_child_keyboard_interrupt_does_not_abort_finalization(observer, monkeypatch):
    class Child:
        returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, value):
            assert value == signal.SIGINT
            self.returncode = 0

        def terminate(self):
            pass

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            del timeout
            return self.returncode

    child = Child()
    observer.ground_truth_process = child
    observer.stop_forensic_ground_truth()
    assert child.returncode == 0


def controller_error():
    message = Log()
    message.level = Log.ERROR
    message.name = "robot2.controller_server"
    message.msg = "Failed to make progress"
    return message


def test_controller_error_after_info_does_not_change_call_site_severity(observer):
    observer.event("STACK_READY", "prior info event", console=True)
    observer.rosout(controller_error())
    events = read_events(observer)
    assert events[-1]["event_type"] == "CONTROLLER_WARNING"
    assert events[-1]["message"] == "Failed to make progress"
    observer.nav2_diagnostics.flush()
    diagnostics = [
        json.loads(line)
        for line in (observer.directory / "nav2_diagnostics.jsonl").read_text(
            encoding="utf-8").splitlines()
    ]
    assert diagnostics[-1]["category"] == "CONTROLLER_OR_PLANNER_ERROR"
    assert diagnostics[-1]["message"] == "Failed to make progress"


def test_csv_timing_schema_accepts_wall_and_sim_elapsed_fields(observer):
    """Every CSV writer accepts the dual-clock row emitted by the logger."""
    observer.sample_telemetry()
    observer.sample_coverage()
    observer.sample_health()
    assert observer.internal_errors == {}
    observer.flush()
    for name in ("robot1_timeseries.csv", "robot2_timeseries.csv",
                 "coverage.csv", "topic_health.csv"):
        header = (observer.directory / name).read_text(
            encoding="utf-8").splitlines()[0].split(",")
        assert "elapsed_s" in header
        assert "wall_elapsed_s" in header


def test_scan_pipeline_records_nav_source_age_and_gap(observer):
    observer.p['initial_configuration_json'] = json.dumps({
        'slam_runtime_parameters': {'throttle_scans': 1},
    })
    values = observer.scan_pipeline['robot1']
    values['scan_d500_fixed_stamps'].extend([1.0, 2.0, 3.0])
    values['scan_d500_nav_stamps'].extend([1.0, 2.0, 4.0])
    values['scan_d500_nav_ages_s'].extend([0.2, 0.4, 2.1])
    observer.write_scan_pipeline_diagnostic()
    diagnostic = read_json(observer.directory / 'scan_pipeline_diagnostic.json')
    nav = diagnostic['robots']['robot1']
    assert nav['nav_scan_messages'] == 3
    assert nav['nav_scan_max_source_gap_s'] == 2.0
    assert nav['nav_scan_source_age_s']['p95'] == 2.1


def test_nonfatal_subsystem_error_is_counted_and_logging_continues(observer):
    def malformed():
        raise ValueError("malformed diagnostic")

    assert observer.safe_call("warning_normalization", malformed) is None
    observer.event("STACK_READY", "continued after malformed input")
    assert sum(observer.internal_errors.values()) == 1
    assert [event["event_type"] for event in read_events(observer)][-2:] == [
        "LOGGER_INTERNAL_ERROR",
        "STACK_READY",
    ]


def test_double_finalization_and_late_callback_are_safe(observer):
    assert observer.finalize(False)
    before = (observer.directory / "events.jsonl").stat().st_size
    called = []
    observer.safe_call("late_callback", lambda: called.append(True))
    assert observer.finalize(False) is False
    assert called == []
    assert (observer.directory / "events.jsonl").stat().st_size == before
    assert read_json(observer.directory / "summary.json")["run"]["clean_shutdown"] is False


def test_partial_and_terminal_robot_states_are_retained(observer):
    observer.latest["robot1"].update(
        claim_state="SUCCEEDED", claim_id=1, navigation_active=False
    )
    observer.latest["robot2"].update(
        claim_state="NAVIGATING", claim_id=2, navigation_active=True
    )
    summary = observer.summary(False)
    assert summary["robot_terminal_state"]["robot1"]["claim_state"] == "SUCCEEDED"
    assert summary["robot_terminal_state"]["robot2"]["navigation_active"] is True
    observer.latest["robot2"].update(claim_state="SUCCEEDED", navigation_active=False)
    summary = observer.summary(True)
    assert all(
        not state["navigation_active"]
        for state in summary["robot_terminal_state"].values()
    )


def test_missing_shared_map_is_explicitly_unavailable_not_zero(observer):
    """A no-handoff observer run must not encode missing coverage as zero."""
    summary = observer.summary(False)
    mapping = summary["mapping"]
    assert mapping["available"] is False
    assert mapping["initial_known_cells"] is None
    assert mapping["final_known_cells"] is None
    assert mapping["coverage_gain_cells"] is None
    assert mapping["coverage_gain_per_metre_travelled"] is None
    assert "unavailable" in mapping["reason"]


def test_clean_finalization_closes_files_after_internal_error(observer):
    observer.safe_call("optional_metric", lambda: 1 / 0)
    assert observer.finalize(True)
    assert observer.events.closed
    assert all(stream.closed for stream in observer.files)
    assert read_json(observer.directory / "run_manifest.json")["clean_shutdown"] is True
    summary = read_json(observer.directory / "summary.json")
    assert summary["system"]["internal_logger_error_count"] == 1


def test_finalization_writes_explicit_complete_artifact_contract(observer):
    assert observer.finalize(True)
    status = read_json(observer.directory / "artifact_finalization.json")
    assert status["complete"] is True
    assert status["missing"] == []
    assert read_json(observer.directory / "summary.json")[
        "artifact_finalization"]["complete"] is True


def test_missing_forensic_artifact_fails_closed(observer):
    class FakeForensic:
        def flush(self):
            pass

        def close(self):
            pass

        def manifest(self):
            return {"files": []}

    observer.forensic = FakeForensic()
    observer.forensic_snapshot = lambda force=False: None
    assert observer.finalize(True) is False
    status = read_json(observer.directory / "artifact_finalization.json")
    assert status["complete"] is False
    assert sorted(status["missing"]) == sorted([
        "forensic/transforms.csv",
        "forensic/maps/robot1_map_final.npz",
        "forensic/maps/robot2_map_final.npz",
    ])
    assert read_json(observer.directory / "mission_result.json")[
        "artifact_finalization"]["complete"] is False


def test_unknown_pose_requires_frontend_diagnostics_in_artifact_contract(observer):
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    status = observer.required_artifact_status(False)
    assert status['complete'] is False
    assert sorted(status['missing']) == sorted([
        'frontend/robot1_unknown_pose_frontend.json',
        'frontend/robot1_consensus_diagnostics.jsonl',
        'frontend/robot1_physical_evidence_diagnostics.jsonl',
        'frontend/robot2_unknown_pose_frontend.json',
        'frontend/robot2_consensus_diagnostics.jsonl',
        'frontend/robot2_physical_evidence_diagnostics.jsonl',
    ])


def test_unknown_pose_artifacts_resolve_logger_run_id_collision(observer):
    """Frontend files may be created before logger collision suffixing."""
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    sibling = observer.directory.parent / f'{observer.run_id}-01' / 'frontend'
    sibling.mkdir(parents=True)
    for robot in ('robot1', 'robot2'):
        for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl'):
            (sibling / f'{robot}_{suffix}').write_text('{}\n')
    status = observer.required_artifact_status(False)
    assert status['complete'] is True
    assert status['missing'] == []
    assert status['frontend_directory'] == f'{observer.run_id}-01/frontend'


def test_unknown_pose_artifacts_resolve_unsuffixed_frontend_run(observer):
    """A suffixed logger finds the launcher's unsuffixed frontend sibling."""
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    base_run_id = observer.run_id
    observer.run_id = f'{base_run_id}-01'
    sibling = observer.directory.parent / base_run_id / 'frontend'
    sibling.mkdir(parents=True)
    for robot in ('robot1', 'robot2'):
        for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl'):
            (sibling / f'{robot}_{suffix}').write_text('{}\n')
    status = observer.required_artifact_status(False)
    assert status['complete'] is True
    assert status['missing'] == []
    assert status['frontend_directory'] == f'{base_run_id}/frontend'


def test_unknown_pose_artifacts_resolve_frontend_created_at_observer_root(observer):
    """The runner may create observer/frontend before logger run allocation."""
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    frontend = observer.directory.parent / 'frontend'
    frontend.mkdir(parents=True)
    for robot in ('robot1', 'robot2'):
        for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl'):
            (frontend / f'{robot}_{suffix}').write_text('{}\n')
    status = observer.required_artifact_status(False)
    assert status['complete'] is True
    assert status['missing'] == []
    assert status['frontend_directory'] == 'frontend'


def test_no_handoff_does_not_require_shared_map_exports(observer):
    """Shared exports are required only after a shared-map topic is seen."""
    class PassiveForensic:
        def save_final_maps(self, *args, **kwargs):
            pass

        def flush(self):
            pass

        def close(self):
            pass

        def manifest(self):
            return {'files': []}

        def record_transform(self, *args, **kwargs):
            pass

    observer.forensic = PassiveForensic()
    observer.p['initial_configuration_json'] = json.dumps({
        'unknown_initial_pose': True,
    })
    status = observer.required_artifact_status(False)
    assert 'forensic/maps/robot1_shared_map_final.npz' not in status['required']
    assert 'forensic/maps/robot2_shared_map_final.npz' not in status['required']
    observer.shared_map_seen.add('robot1')
    status = observer.required_artifact_status(False)
    assert 'forensic/maps/robot1_shared_map_final.npz' in status['required']
    assert 'forensic/maps/robot2_shared_map_final.npz' in status['required']


def test_large_occupancy_grid_is_converted_and_counted_once(observer):
    message = OccupancyGrid()
    message.info.width = 320
    message.info.height = 320
    message.info.resolution = 0.01
    message.data = [-1] * 10000 + [0] * 90000 + [100] * 2400
    observer.mark("robot1", "map", message)
    assert observer.map_counts("robot1", "map") == (92400, 90000, 2400)
    first = observer._map_cache[("robot1", "map")]
    assert observer.map_counts("robot1", "map") == (92400, 90000, 2400)
    assert observer._map_cache[("robot1", "map")] is first


def test_concurrent_updates_and_finalization_do_not_write_closed_files(observer):
    failures = []

    def update(index):
        try:
            for sequence in range(100):
                observer.safe_call(
                    f"thread_{index}",
                    observer.event,
                    "NAVIGATION_PROGRESS",
                    f"{index}:{sequence}",
                )
                observer.safe_call("rosout", observer.rosout, controller_error())
        except Exception as exc:  # Test must expose errors outside fault boundaries.
            failures.append(exc)

    threads = [threading.Thread(target=update, args=(index,)) for index in range(3)]
    for thread in threads:
        thread.start()
    observer.finalize(False)
    for thread in threads:
        thread.join()
    assert failures == []
    assert read_json(observer.directory / "summary.json")["run"]["clean_shutdown"] is False


def test_continuous_cycle_status_and_suppression_are_reconciled(observer):
    started = ExplorationEvent()
    started.header.frame_id = "shared_map"
    started.source_robot_id = "robot1"
    started.event_type = "EXPLORATION_CYCLE_STARTED"
    started.cycle_number = 1
    started.claim_id = 1
    started.frontier_id = 42
    observer.coordinator_event("robot1", started)
    observer.trajectory.total_distance["robot1"] = 0.4
    ended = ExplorationEvent()
    ended.header.frame_id = "shared_map"
    ended.source_robot_id = "robot1"
    ended.event_type = "EXPLORATION_CYCLE_ENDED"
    ended.cycle_number = 1
    ended.claim_id = 1
    ended.frontier_id = 42
    ended.terminal_result = "UNKNOWN_NAV2_FAILURE"
    observer.coordinator_event("robot1", ended)
    suppression = ExplorationEvent()
    suppression.event_type = "FAILURE_SUPPRESSION_CREATED"
    suppression.claim_id = 1
    suppression.frontier_id = 42
    suppression.reason = "UNKNOWN_NAV2_FAILURE"
    observer.coordinator_event("robot1", suppression)
    exhausted = ExplorationStatus()
    exhausted.state = ExplorationStatus.NO_ELIGIBLE_CANDIDATES
    exhausted.reason = "locally_exhausted"
    observer.status("robot1", exhausted)
    summary = observer.summary(False)
    robot = summary["continuous_exploration"]["robot1"]
    assert robot["exploration_cycles"] == 1
    assert robot["failed_goals"] == 1
    assert robot["maximum_equivalent_region_attempt_count"] == 1
    assert summary["events"]["FAILURE_SUPPRESSION_CREATED"] == 1
