"""Bounded forensic evidence writer for one cooperative exploration run.

This module is deliberately passive.  It serializes selected ROS messages and
TF lookups for offline comparison; it never publishes, calls a service, or
changes robot state.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path

import numpy as np


def _stamp_value(stamp):
    return int(stamp.sec) + int(stamp.nanosec) * 1e-9


def _stamp_text(stamp):
    return f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"


def _quat_dict(q):
    return {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)}


def _origin_dict(origin):
    return {
        "position": {
            "x": float(origin.position.x),
            "y": float(origin.position.y),
            "z": float(origin.position.z),
        },
        "orientation": _quat_dict(origin.orientation),
    }


def _grid_metadata(message, topic, robot, received_ros, received_wall):
    info = message.info
    return {
        "schema_version": "forensic_evidence_1.0",
        "topic": topic,
        "robot_id": robot,
        "frame_id": message.header.frame_id,
        "header_stamp": _stamp_text(message.header.stamp),
        "header_stamp_s": _stamp_value(message.header.stamp),
        "map_load_time": _stamp_text(info.map_load_time),
        "received_ros_time_s": float(received_ros),
        "received_wall_elapsed_s": float(received_wall),
        "width": int(info.width),
        "height": int(info.height),
        "resolution": float(info.resolution),
        "origin": _origin_dict(info.origin),
        "dtype": "int8",
        "data_length": len(message.data),
    }


def _atomic_npz(path, occupancy, metadata):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("wb") as stream:
        np.savez_compressed(
            stream,
            occupancy=np.asarray(occupancy, dtype=np.int8),
            metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
        )
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _safe_name(value):
    return str(value).replace("/", "_").replace(" ", "_")


class ForensicEvidenceWriter:
    """Stream bounded maps, PeerMap records, raw odometry and TF evidence."""

    MAP_KEYS = ("map", "shared_map")

    def __init__(self, directory, robots, interval_s=15.0):
        self.root = Path(directory) / "forensic"
        self.maps_dir = self.root / "maps"
        self.peer_dir = self.root / "peer_maps"
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        self.peer_dir.mkdir(parents=True, exist_ok=True)
        self.robots = tuple(robots)
        self.interval_s = max(5.0, float(interval_s))
        self.last_snapshot_ros = None
        self.last_map_digest = {}
        self.last_peer_revision = {robot: -1 for robot in self.robots}
        self.map_snapshot_count = 0
        self.peer_record_count = 0
        self._closed = False
        self._odom_files = {}
        self._odom_writers = {}
        odom_fields = [
            "robot_id", "received_ros_time_s", "received_wall_elapsed_s",
            "header_stamp", "frame_id", "pose_x", "pose_y", "pose_z",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w",
            "twist_linear_x", "twist_linear_y", "twist_linear_z",
            "twist_angular_x", "twist_angular_y", "twist_angular_z",
        ]
        for robot in self.robots:
            stream = (self.root / f"{robot}_odom.csv").open(
                "w", newline="", encoding="utf-8")
            writer = csv.DictWriter(stream, fieldnames=odom_fields)
            writer.writeheader()
            self._odom_files[robot] = stream
            self._odom_writers[robot] = writer
        self.peer_file = (self.root / "peer_map_records.csv").open(
            "w", newline="", encoding="utf-8")
        self.peer_writer = csv.DictWriter(self.peer_file, fieldnames=[
            "robot_id", "source_robot_id", "revision", "export_stamp",
            "occupancy_header_stamp", "local_evidence_only", "accepted",
            "received_ros_time_s", "received_wall_elapsed_s", "npz_path",
            "width", "height", "resolution", "frame_id",
        ])
        self.peer_writer.writeheader()
        self.tf_file = (self.root / "transforms.csv").open(
            "w", newline="", encoding="utf-8")
        self.tf_writer = csv.DictWriter(self.tf_file, fieldnames=[
            "query_ros_time_s", "query_wall_elapsed_s", "target_frame",
            "source_frame", "transform_stamp", "transform_age_s", "available",
            "translation_x", "translation_y", "translation_z",
            "rotation_x", "rotation_y", "rotation_z", "rotation_w", "error",
        ])
        self.tf_writer.writeheader()
        self._flush_counter = 0

    @staticmethod
    def _message_digest(message):
        info = message.info
        payload = np.asarray(message.data, dtype=np.int8).tobytes()
        digest = hashlib.sha256()
        digest.update(message.header.frame_id.encode('utf-8'))
        digest.update(str((int(info.width), int(info.height),
                           float(info.resolution),
                           float(info.origin.position.x),
                           float(info.origin.position.y),
                           float(info.origin.orientation.z),
                           float(info.origin.orientation.w))).encode('ascii'))
        digest.update(payload)
        return digest.hexdigest()

    def _map_path(self, robot, key, stamp, final=False):
        stem = f"{_safe_name(robot)}_{_safe_name(key)}"
        if final:
            return self.maps_dir / f"{stem}_final.npz"
        return self.maps_dir / (
            f"sim_{stamp.sec}_{stamp.nanosec:09d}_{stem}.npz")

    def save_map(self, robot, key, message, received_ros, received_wall,
                 final=False, force=False):
        if self._closed or message is None:
            return None
        digest = self._message_digest(message)
        cache_key = (robot, key)
        if not force and self.last_map_digest.get(cache_key) == digest:
            return None
        path = self._map_path(robot, key, message.header.stamp, final=final)
        metadata = _grid_metadata(
            message, f"/{robot}/{key}", robot, received_ros, received_wall)
        metadata.update({"snapshot_kind": "final" if final else "periodic",
                         "content_hash": digest})
        data = np.asarray(message.data, dtype=np.int8).reshape(
            int(message.info.height), int(message.info.width))
        _atomic_npz(path, data, metadata)
        self.last_map_digest[cache_key] = digest
        self.map_snapshot_count += 1
        return str(path)

    def capture_maps(self, latest, received_ros, received_wall, force=False):
        if self._closed:
            return []
        if not force and self.last_snapshot_ros is not None and (
                received_ros - self.last_snapshot_ros < self.interval_s):
            return []
        paths = []
        for robot in self.robots:
            for key in self.MAP_KEYS:
                message = latest.get(robot, {}).get(key)
                path = self.save_map(robot, key, message, received_ros,
                                     received_wall, force=force)
                if path:
                    paths.append(path)
        self.last_snapshot_ros = received_ros
        return paths

    def save_final_maps(self, latest, received_ros, received_wall):
        paths = []
        for robot in self.robots:
            for key in self.MAP_KEYS:
                message = latest.get(robot, {}).get(key)
                path = self.save_map(robot, key, message, received_ros,
                                     received_wall, final=True, force=True)
                if path:
                    paths.append(path)
        return paths

    def record_peer_map(self, robot, message, received_ros, received_wall):
        if self._closed or message is None:
            return False
        source = str(message.source_robot_id)
        revision = int(message.revision)
        accepted = (source == robot and bool(message.local_evidence_only)
                    and revision > self.last_peer_revision[robot])
        path = ""
        grid = message.occupancy_grid
        if accepted:
            self.last_peer_revision[robot] = revision
            path_obj = self.peer_dir / f"{robot}_revision_{revision:06d}.npz"
            metadata = _grid_metadata(
                grid, f"/cslam/{robot}/local_map", robot, received_ros,
                received_wall)
            metadata.update({
                "source_robot_id": source,
                "revision": revision,
                "export_stamp": _stamp_text(message.export_stamp),
                "local_evidence_only": bool(message.local_evidence_only),
                "accepted": True,
            })
            data = np.asarray(grid.data, dtype=np.int8).reshape(
                int(grid.info.height), int(grid.info.width))
            _atomic_npz(path_obj, data, metadata)
            path = str(path_obj)
            self.peer_record_count += 1
        self.peer_writer.writerow({
            "robot_id": robot, "source_robot_id": source, "revision": revision,
            "export_stamp": _stamp_text(message.export_stamp),
            "occupancy_header_stamp": _stamp_text(grid.header.stamp),
            "local_evidence_only": bool(message.local_evidence_only),
            "accepted": accepted, "received_ros_time_s": received_ros,
            "received_wall_elapsed_s": received_wall, "npz_path": path,
            "width": int(grid.info.width), "height": int(grid.info.height),
            "resolution": float(grid.info.resolution),
            "frame_id": grid.header.frame_id,
        })
        return accepted

    def record_odom(self, robot, message, received_ros, received_wall):
        if self._closed:
            return
        pose = message.pose.pose
        twist = message.twist.twist
        self._odom_writers[robot].writerow({
            "robot_id": robot, "received_ros_time_s": received_ros,
            "received_wall_elapsed_s": received_wall,
            "header_stamp": _stamp_text(message.header.stamp),
            "frame_id": message.header.frame_id,
            "pose_x": pose.position.x, "pose_y": pose.position.y,
            "pose_z": pose.position.z, "orientation_x": pose.orientation.x,
            "orientation_y": pose.orientation.y, "orientation_z": pose.orientation.z,
            "orientation_w": pose.orientation.w,
            "twist_linear_x": twist.linear.x, "twist_linear_y": twist.linear.y,
            "twist_linear_z": twist.linear.z, "twist_angular_x": twist.angular.x,
            "twist_angular_y": twist.angular.y, "twist_angular_z": twist.angular.z,
        })

    def record_transform(self, query_ros, query_wall, target, source,
                         transform=None, error=""):
        row = {"query_ros_time_s": query_ros, "query_wall_elapsed_s": query_wall,
               "target_frame": target, "source_frame": source,
               "transform_stamp": "", "transform_age_s": None,
               "available": transform is not None,
               "translation_x": None, "translation_y": None,
               "translation_z": None, "rotation_x": None, "rotation_y": None,
               "rotation_z": None, "rotation_w": None, "error": error}
        if transform is not None:
            stamp = transform.header.stamp
            t = transform.transform.translation
            q = transform.transform.rotation
            row.update({"transform_stamp": _stamp_text(stamp),
                        "transform_age_s": query_ros - _stamp_value(stamp),
                        "translation_x": t.x, "translation_y": t.y,
                        "translation_z": t.z, "rotation_x": q.x,
                        "rotation_y": q.y, "rotation_z": q.z, "rotation_w": q.w})
        self.tf_writer.writerow(row)

    def flush(self):
        if self._closed:
            return
        streams = [*self._odom_files.values(), self.peer_file, self.tf_file]
        for stream in streams:
            stream.flush()
        self._flush_counter += 1

    def close(self):
        if self._closed:
            return
        self.flush()
        for stream in [*self._odom_files.values(), self.peer_file, self.tf_file]:
            stream.close()
        self._closed = True

    def manifest(self):
        return {
            "schema_version": "forensic_evidence_1.0",
            "root": str(self.root),
            "map_snapshot_interval_s": self.interval_s,
            "map_snapshot_count": self.map_snapshot_count,
            "accepted_peer_map_count": self.peer_record_count,
            "files": sorted(str(path.relative_to(self.root))
                             for path in self.root.rglob("*") if path.is_file()),
        }
