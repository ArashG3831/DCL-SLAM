"""Thin raw recorder/coordinator for complete-evidence Condition C runs.

The mission-time component owns only raw rosbag2 capture, external Supervisor
GT/contact capture, provenance, and bounded shutdown.  Coverage, trajectory,
idle, map and cooperation metrics are intentionally not computed here.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import time
from dataclasses import dataclass

from .passive_rosbag import (
    start_recorder,
    stop_recorder,
    validate_raw_bag,
)
from .raw_evidence_contract import (
    SCHEMA_VERSION,
    contract_metadata,
    required_nonempty_topics,
)


def _atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    os.replace(temporary, path)


def _controller_host(environment: dict) -> str:
    mode = environment.get("MY_EPUCK_WEBOTS_NETWORK_MODE", "").strip().lower()
    if mode in ("mirrored", "loopback"):
        return "127.0.0.1"
    if mode in ("nat", "subnet", "wsl_nat"):
        result = subprocess.run(
            ["ip", "route", "show", "default"], check=True,
            capture_output=True, text=True, timeout=2.0, env=environment)
        for line in result.stdout.splitlines():
            fields = line.split()
            if fields and fields[0] == "default" and "via" in fields:
                return fields[fields.index("via") + 1]
        raise RuntimeError("no WSL NAT gateway found")
    raise RuntimeError("MY_EPUCK_WEBOTS_NETWORK_MODE must be explicit")


@dataclass
class ThinEvidenceSession:
    observer_root: Path
    run_id: str
    robots: tuple[str, ...]
    bag_process: object | None = None
    bag_log: object | None = None
    bag_command: list[str] | None = None
    gt_process: object | None = None
    gt_log: object | None = None
    gt_ready_file: Path | None = None
    gt_runtime_directory: Path | None = None
    ground_truth_integrated: bool = False
    started_monotonic: float = 0.0
    stopped: bool = False

    @property
    def run_directory(self) -> Path:
        return self.observer_root / self.run_id

    @property
    def bag_directory(self) -> Path:
        return self.run_directory / "passive_rosbag"

    @property
    def raw_validation_path(self) -> Path:
        return self.run_directory / "raw_bag_validation.json"

    @property
    def manifest_path(self) -> Path:
        return self.run_directory / "raw_evidence_manifest.json"

    @property
    def finalization_path(self) -> Path:
        return self.run_directory / "raw_evidence_finalization.json"

    @property
    def runtime_boundary_path(self) -> Path:
        return self.run_directory / "runtime_boundary.json"

    def _handoff_artifact_status(self) -> dict:
        """Locate structured accepted-handoff evidence without parsing logs."""
        frontend = self.run_directory / "frontend"
        files = sorted(frontend.glob("*_unknown_pose_frontend.json"))
        accepted = {}
        for path in files:
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            accepted[path.name.split("_unknown_pose_frontend.json", 1)[0]] = (
                payload.get("accepted") is True and
                isinstance(payload.get("accepted_hypothesis"), dict) and
                bool(payload["accepted_hypothesis"].get("evidence_set_hash")) and
                payload.get("accepted_ros_time_s") is not None)
        return {
            "handoff_structured": all(
                accepted.get(robot, False) for robot in self.robots),
            "accepted_by_robot": accepted,
            "files": [str(path) for path in files],
        }

    @classmethod
    def start(cls, observer_root: Path, run_id: str, robots, environment: dict,
              provenance: dict, webots_port: int, sample_period_s: float,
              contact_enabled: bool, contact_sampling_period_ms: int,
              driver_prefix: str, horizon_s: float, condition=None,
              unknown_initial_pose=False, assignment_strategy=None):
        robots = tuple(str(robot) for robot in robots)
        session = cls(Path(observer_root), str(run_id), robots,
                       started_monotonic=time.monotonic())
        session.ground_truth_integrated = True
        session.run_directory.mkdir(parents=True, exist_ok=True)
        (session.run_directory / "forensic").mkdir(parents=True, exist_ok=True)
        metadata = contract_metadata(
            robots, condition=condition,
            unknown_initial_pose=unknown_initial_pose,
            assignment_strategy=assignment_strategy)
        _atomic_json(session.manifest_path, {
            "schema_version": SCHEMA_VERSION,
            "status": "RUNNING",
            "run_id": run_id,
            "contract": metadata,
            "provenance": provenance,
            "simulation_horizon_s": float(horizon_s),
            "condition": condition,
            "assignment_strategy": assignment_strategy,
            "unknown_initial_pose": bool(unknown_initial_pose),
            "webots_controller_endpoint": None,
            "started_monotonic_wall_s": session.started_monotonic,
        })
        try:
            (session.bag_process, session.bag_log, session.bag_command,
             _) = start_recorder(
                session.bag_directory, robots, environment=environment,
                include_offloaded=False, include_scientific_raw=True)
            forensic = session.run_directory / "forensic"
            runtime = forensic / "runtime"
            session.gt_ready_file = forensic / "supervisor_ready.json"
            session.gt_runtime_directory = runtime
            output = forensic / "supervisor_ground_truth.csv"
            contact_output = forensic / "contact_points.csv"
            controller_endpoint = (
                f"tcp://{_controller_host(environment)}:{int(webots_port)}/"
                "ForensicGroundTruthSupervisor")
            session.manifest_path  # keep the property visible to debuggers
            # Thin mode captures GT/contact through the already-running paced
            # Webots Supervisor.  This preserves the 20 ms evidence cadence
            # without opening a second Webots controller connection.
            environment["MY_EPUCK_THIN_INTEGRATED_GT"] = "1"
            environment["MY_EPUCK_THIN_GT_OUTPUT"] = str(output)
            environment["MY_EPUCK_THIN_GT_READY_FILE"] = str(
                session.gt_ready_file)
            environment["MY_EPUCK_THIN_GT_RUNTIME_DIRECTORY"] = str(runtime)
            environment["MY_EPUCK_THIN_GT_SAMPLE_PERIOD_S"] = str(
                float(sample_period_s))
            environment["MY_EPUCK_THIN_CONTACT_PERIOD_MS"] = str(
                int(contact_sampling_period_ms))
            if contact_enabled:
                environment["MY_EPUCK_THIN_CONTACT_OUTPUT"] = str(
                    contact_output)
            else:
                environment.pop("MY_EPUCK_THIN_CONTACT_OUTPUT", None)
            session.manifest_path  # keep the property visible to debuggers
            _atomic_json(session.manifest_path, {
                **json.loads(session.manifest_path.read_text(encoding="utf-8")),
                "webots_controller_endpoint": controller_endpoint,
                "recorder_pid": session.bag_process.pid,
                "ground_truth_mode": "paced_ros2_supervisor_in_process",
                "ground_truth_pid": None,
            })
        except Exception:
            session.stop(timeout_s=3.0)
            raise
        return session

    def add_to_watchdog(self, watchdog):
        for process in (self.bag_process, self.gt_process):
            if process is not None:
                watchdog.add_process(process)

    def _stop_ground_truth(self, timeout_s: float):
        process = self.gt_process
        if self.ground_truth_integrated:
            request = self.run_directory / "forensic" / "runtime" / "shutdown.requested"
            request.parent.mkdir(parents=True, exist_ok=True)
            request.write_text("shutdown_requested\n", encoding="utf-8")
            return 0
        if process is None:
            return None
        if process.poll() is None:
            request = self.run_directory / "forensic" / "runtime" / "shutdown.requested"
            request.parent.mkdir(parents=True, exist_ok=True)
            request.write_text("shutdown_requested\n", encoding="utf-8")
            try:
                process.wait(timeout=max(0.1, timeout_s))
            except subprocess.TimeoutExpired:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    if process.poll() is None:
                        process.kill()
                    process.wait(timeout=3.0)
        if self.gt_log is not None:
            self.gt_log.flush()
            self.gt_log.close()
        return process.returncode

    def stop(self, timeout_s: float = 8.0, recorder_timeout_s: float = 8.0):
        if self.stopped:
            return json.loads(self.finalization_path.read_text()) \
                if self.finalization_path.exists() else {}
        self.stopped = True
        gt_return = self._stop_ground_truth(timeout_s)
        bag_return = stop_recorder(self.bag_process, self.bag_log,
                                   timeout_s=recorder_timeout_s)
        validation_error = None
        metadata = None
        semantic_artifacts = self._handoff_artifact_status()
        condition = None
        unknown_initial_pose = False
        manifest = {}
        try:
            manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
            condition = manifest.get("condition")
            unknown_initial_pose = bool(manifest.get("unknown_initial_pose", False))
        except (OSError, ValueError):
            pass
        try:
            metadata = validate_raw_bag(
                self.bag_directory, self.robots,
                include_offloaded=False, include_scientific_raw=True,
                required_topics=required_nonempty_topics(self.robots),
                condition=condition, unknown_initial_pose=unknown_initial_pose,
                semantic_artifacts=semantic_artifacts,
                assignment_strategy=manifest.get('assignment_strategy'))
            metadata["recorder_return_code"] = bag_return
            metadata["ground_truth_return_code"] = gt_return
            _atomic_json(self.raw_validation_path, metadata)
        except Exception as exc:
            validation_error = f"{type(exc).__name__}: {exc}"
        runtime_metrics = self.run_directory / "forensic" / "runtime" / "runtime_metrics.json"
        gt_finalized = False
        if runtime_metrics.exists():
            try:
                gt_finalized = bool(json.loads(runtime_metrics.read_text()).get(
                    "finalized"))
            except (OSError, ValueError, TypeError):
                gt_finalized = False
        counts = (metadata or {}).get("message_counts", {})
        missing = [topic for topic in required_nonempty_topics(self.robots)
                   if int(counts.get(topic, 0)) <= 0]
        semantic_contract = (metadata or {}).get("semantic_contract", {})
        complete = bool(
            validation_error is None and metadata and metadata.get("complete") and
            bag_return == 0 and gt_return == 0 and gt_finalized and not missing)
        result = {
            "schema_version": SCHEMA_VERSION,
            "status": "COMPLETE" if complete else "INCOMPLETE",
            "complete": complete,
            "missing": missing + (["raw_bag_validation"]
                                   if validation_error else []),
            "bag_return_code": bag_return,
            "ground_truth_return_code": gt_return,
            "ground_truth_finalized": gt_finalized,
            "raw_validation_error": validation_error,
            "semantic_contract": semantic_contract,
            "handoff_artifacts": semantic_artifacts,
            "offline_evaluation": {
                "complete": False,
                "deferred_until_after_process_cleanup": True,
                "path": str(self.run_directory / 'thin_metrics.json'),
            },
            "message_counts": counts,
            "required_nonempty_topics": list(
                required_nonempty_topics(self.robots)),
            "finalized_monotonic_wall_s": time.monotonic(),
        }
        _atomic_json(self.finalization_path, result)
        _atomic_json(self.manifest_path, {
            **json.loads(self.manifest_path.read_text(encoding="utf-8")),
            "status": result["status"],
            "finalization": result,
        })
        return result

    def refresh_post_shutdown_semantics(self):
        """Re-evaluate artifact semantics after launch-side writers exit.

        The recorder and GT process must stop before the launch graph is torn
        down, but the unknown-pose frontend may finish its structured handoff
        artifact during that teardown.  Revalidate the immutable bag against
        the now-complete handoff artifacts; this changes only evidence status,
        never robot or allocator behavior.
        """
        if not self.raw_validation_path.exists() or not self.finalization_path.exists():
            return None
        try:
            manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
            previous = json.loads(self.finalization_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        condition = manifest.get("condition")
        unknown_initial_pose = bool(manifest.get("unknown_initial_pose", False))
        semantic_artifacts = self._handoff_artifact_status()
        metadata = validate_raw_bag(
            self.bag_directory, self.robots,
            include_offloaded=False, include_scientific_raw=True,
            required_topics=required_nonempty_topics(self.robots),
            condition=condition, unknown_initial_pose=unknown_initial_pose,
            semantic_artifacts=semantic_artifacts,
            assignment_strategy=manifest.get("assignment_strategy"))
        metadata["recorder_return_code"] = previous.get("bag_return_code")
        metadata["ground_truth_return_code"] = previous.get(
            "ground_truth_return_code")
        _atomic_json(self.raw_validation_path, metadata)
        required = required_nonempty_topics(self.robots)
        counts = metadata.get("message_counts", {})
        missing = [topic for topic in required
                   if int(counts.get(topic, 0)) <= 0]
        gt_finalized = bool(previous.get("ground_truth_finalized"))
        runtime_metrics = (self.run_directory / "forensic" / "runtime" /
                           "runtime_metrics.json")
        if runtime_metrics.exists():
            try:
                gt_finalized = bool(json.loads(
                    runtime_metrics.read_text(encoding="utf-8")).get(
                        "finalized"))
            except (OSError, ValueError, TypeError):
                pass
        complete = bool(
            metadata.get("complete") and
            previous.get("bag_return_code") == 0 and
            previous.get("ground_truth_return_code") == 0 and
            gt_finalized and not missing)
        result = {
            **previous,
            "status": "COMPLETE" if complete else "INCOMPLETE",
            "complete": complete,
            "missing": missing,
            "semantic_contract": metadata.get("semantic_contract", {}),
            "handoff_artifacts": semantic_artifacts,
            "message_counts": counts,
        }
        _atomic_json(self.finalization_path, result)
        _atomic_json(self.manifest_path, {
            **manifest,
            "status": result["status"],
            "finalization": result,
        })
        return result

    def record_runtime_boundary(self, boundary: dict) -> None:
        """Persist one final simulation/wall-clock boundary record."""
        payload = {
            "schema_version": "thin_runtime_boundary_1.0",
            "run_id": self.run_id,
            **boundary,
        }
        _atomic_json(self.runtime_boundary_path, payload)
        if self.manifest_path.exists():
            try:
                manifest = json.loads(self.manifest_path.read_text(
                    encoding="utf-8"))
                manifest["runtime_boundary"] = payload
                _atomic_json(self.manifest_path, manifest)
            except (OSError, ValueError, TypeError):
                pass
        if self.finalization_path.exists():
            try:
                finalization = json.loads(self.finalization_path.read_text(
                    encoding="utf-8"))
                finalization["runtime_boundary"] = payload
                _atomic_json(self.finalization_path, finalization)
            except (OSError, ValueError, TypeError):
                pass

    def evaluate_offline(self):
        """Replay raw evidence after all live processes have stopped."""
        from .offline_evidence_replay import evaluate_run

        evaluation = evaluate_run(self.run_directory, self.robots)
        result = json.loads(self.finalization_path.read_text(encoding='utf-8'))
        result['offline_evaluation'] = {
            'complete': bool(evaluation.get('complete')),
            'path': str(self.run_directory / 'thin_metrics.json'),
        }
        _atomic_json(self.finalization_path, result)
        _atomic_json(self.manifest_path, {
            **json.loads(self.manifest_path.read_text(encoding='utf-8')),
            'finalization': result,
        })
        return evaluation
