import json
import threading

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Log
from my_epuck_interfaces.msg import ExplorationEvent, ExplorationStatus

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

        def terminate(self):
            pass

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            del timeout
            raise KeyboardInterrupt

    child = Child()
    observer.ground_truth_process = child
    observer.stop_forensic_ground_truth()
    assert child.returncode == -9


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


def test_clean_finalization_closes_files_after_internal_error(observer):
    observer.safe_call("optional_metric", lambda: 1 / 0)
    assert observer.finalize(True)
    assert observer.events.closed
    assert all(stream.closed for stream in observer.files)
    assert read_json(observer.directory / "run_manifest.json")["clean_shutdown"] is True
    summary = read_json(observer.directory / "summary.json")
    assert summary["system"]["internal_logger_error_count"] == 1


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
