"""Deferred shared-frame trajectory replay for the legacy observer.

This module replays the native rosbag2 stream through the same tf2 Buffer
semantics used by the live observer.  It is intentionally limited to the
shared trajectory/overlap metric family; it does not define a second metric.
"""

from __future__ import annotations

import math
from pathlib import Path

from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
from rosidl_runtime_py.utilities import get_message

from .experiment_metrics import TrajectoryOverlap


def _yaw(quaternion):
    return math.atan2(
        2.0 * (float(quaternion.w) * float(quaternion.z)),
        1.0 - 2.0 * (float(quaternion.y) ** 2 +
                     float(quaternion.z) ** 2),
    )


def _finite_pose(x, y, yaw):
    return all(math.isfinite(float(value)) for value in (x, y, yaw))


def replay_shared_trajectory_from_bag(
        bag_directory: Path, robots, global_frame: str,
        bin_size: float = .05, exclusion_radius: float = .15):
    """Replay shared trajectory points in rosbag arrival order.

    The bag is the authoritative raw source.  TF and TF-static messages are
    inserted as they occur in the bag, then each odometry message performs the
    same zero-timeout tf2 lookup used by the live callback.  Missing transforms
    are recorded as skipped evidence, never converted into a valid point.
    """
    import rosbag2_py

    bag_directory = Path(bag_directory)
    if not bag_directory.is_dir():
        raise FileNotFoundError(bag_directory)
    robots = tuple(str(robot) for robot in robots)
    topics = {f'/{robot}/odom': robot for robot in robots}
    tf_topic = '/tf'
    tf_static_topic = '/tf_static'
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    type_map = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    required_types = {
        tf_topic: 'tf2_msgs/msg/TFMessage',
        tf_static_topic: 'tf2_msgs/msg/TFMessage',
    }
    required_types.update({
        topic: 'nav_msgs/msg/Odometry' for topic in topics})
    missing = [topic for topic, message_type in required_types.items()
               if type_map.get(topic) != message_type]
    if missing:
        raise ValueError(f'raw trajectory topics missing or mistyped: {missing}')

    tf_message_type = get_message('tf2_msgs/msg/TFMessage')
    odometry_type = get_message('nav_msgs/msg/Odometry')
    buffer = Buffer()
    trajectory = TrajectoryOverlap(
        bin_size=float(bin_size), exclusion_radius=float(exclusion_radius))
    accepted = 0
    skipped = 0
    deserialized = 0
    errors = []
    while reader.has_next():
        topic, serialized, _bag_timestamp = reader.read_next()
        if topic == tf_topic or topic == tf_static_topic:
            message = deserialize_message(serialized, tf_message_type)
            deserialized += 1
            for transform in message.transforms:
                try:
                    if topic == tf_static_topic:
                        buffer.set_transform_static(transform, 'deferred')
                    else:
                        buffer.set_transform(transform, 'deferred')
                except Exception as exc:  # fail closed at the evidence boundary
                    errors.append(f'{topic}:{type(exc).__name__}:{exc}')
            continue
        robot = topics.get(topic)
        if robot is None:
            continue
        message = deserialize_message(serialized, odometry_type)
        deserialized += 1
        pose = message.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        local_yaw = _yaw(pose.orientation)
        if not _finite_pose(x, y, local_yaw):
            raise ValueError(f'non-finite odometry pose on {topic}')
        try:
            transform = buffer.lookup_transform(
                global_frame, message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            skipped += 1
            continue
        translation = transform.transform.translation
        heading = _yaw(transform.transform.rotation)
        cosine, sine = math.cos(heading), math.sin(heading)
        shared_x = float(translation.x) + cosine * x - sine * y
        shared_y = float(translation.y) + sine * x + cosine * y
        shared_yaw = (heading + local_yaw + math.pi) % (2.0 * math.pi) - math.pi
        if not _finite_pose(shared_x, shared_y, shared_yaw):
            raise ValueError(f'non-finite transformed pose on {topic}')
        trajectory.add(robot, shared_x, shared_y)
        accepted += 1
    return {
        'summary': trajectory.summary(),
        'trajectory': trajectory,
        'accepted_samples': accepted,
        'skipped_transform_samples': skipped,
        'deserialized_messages': deserialized,
        'errors': errors,
        'global_frame': str(global_frame),
        'robots': list(robots),
    }
