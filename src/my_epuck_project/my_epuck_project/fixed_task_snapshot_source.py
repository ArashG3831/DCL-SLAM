"""Deterministic fixed physical-task source for replicated allocator tests."""

import hashlib
import math

from geometry_msgs.msg import Point

from my_epuck_interfaces.msg import PhysicalTask, TaskSnapshot

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .distributed_assignment.ros_conversion import seconds_to_duration, text_to_uuid


SESSIONS = {
    'robot1': '11111111111111111111111111111111',
    'robot2': '22222222222222222222222222222222',
}


def _point(x: float, y: float) -> Point:
    return Point(x=x, y=y, z=0.0)


def _task(
        robot_id: str, session_id: str, epoch: int, local_id: int,
        signature: str, approach: tuple[float, float],
        centroid: tuple[float, float], bounds: tuple[float, float, float, float],
        gain: float) -> PhysicalTask:
    message = PhysicalTask()
    message.source_robot_id = robot_id
    message.source_session_id = text_to_uuid(session_id)
    message.source_snapshot_epoch = epoch
    message.source_map_revision = 1
    message.physical_signature = signature
    message.local_frontier_id = local_id
    message.centroid = _point(*centroid)
    message.bounding_box_min = _point(bounds[0], bounds[1])
    message.bounding_box_max = _point(bounds[2], bounds[3])
    message.approach_pose.header.frame_id = 'shared_map'
    message.approach_pose.pose.position = _point(*approach)
    yaw = math.atan2(centroid[1] - approach[1], centroid[0] - approach[0])
    message.approach_pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.approach_pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.frontier_geometry = [
        _point(bounds[0], bounds[1]), _point(bounds[0], bounds[3]),
        _point(bounds[2], bounds[1]), _point(bounds[2], bounds[3]),
        _point(*centroid),
    ]
    message.visible_reveal_gain = gain
    message.local_ordering_score = gain
    message.local_path_valid = True
    return message


def alternate_hallway_tasks(
        robot_id: str, session_id: str, epoch: int) -> list[PhysicalTask]:
    """Return the direct same-hallway versus alternate-branch regression."""
    if robot_id == 'robot1':
        return [
            _task(
                robot_id, session_id, epoch, 101, 'north-near',
                (0.0, 5.0), (0.0, 5.2), (-0.2, 5.0, 0.2, 5.4), 5.0,
            ),
            _task(
                robot_id, session_id, epoch, 102, 'shared-attractive',
                (2.5, 2.5), (2.7, 2.7), (2.4, 2.4, 3.0, 3.0), 2.0,
            ),
        ]
    return [
        _task(
            robot_id, session_id, epoch, 201, 'north-far',
            (0.9, 5.0), (0.9, 5.2), (0.7, 5.0, 1.1, 5.4), 5.0,
        ),
        _task(
            robot_id, session_id, epoch, 202, 'east-branch',
            (5.0, 0.0), (5.2, 0.0), (5.0, -0.2, 5.4, 0.2), 4.5,
        ),
        _task(
            robot_id, session_id, epoch, 203, 'shared-attractive-copy',
            (2.71, 2.5), (2.72, 2.68), (2.42, 2.42, 3.02, 3.02), 2.0,
        ),
    ]


class FixedTaskSnapshotSource(Node):
    """Publish one transient-local deterministic snapshot per configured robot."""

    def __init__(self):
        """Configure a bounded synthetic source with no navigation interfaces."""
        super().__init__('fixed_task_snapshot_source')
        self._robot_id = self.declare_parameter('robot_id', '').value
        if self._robot_id not in SESSIONS:
            raise ValueError('robot_id must be robot1 or robot2')
        self._scenario = self.declare_parameter(
            'scenario', 'alternate_hallway',
        ).value
        self._validity_s = float(self.declare_parameter('validity_s', 30.0).value)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(TaskSnapshot, 'task_snapshot', qos)
        self._published = False
        self._message = None
        self._timer = self.create_timer(1.0, self._publish_snapshot)

    def _publish_snapshot(self) -> None:
        """Republish the same immutable epoch until every late peer discovers it."""
        if self._scenario != 'alternate_hallway':
            raise ValueError('only the bounded alternate_hallway scenario is supported')
        if self._message is not None:
            self._publisher.publish(self._message)
            return
        session_id = SESSIONS[self._robot_id]
        message = TaskSnapshot()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'shared_map'
        message.source_robot_id = self._robot_id
        message.source_session_id = text_to_uuid(session_id)
        message.source_snapshot_epoch = 1
        message.source_map_revision = 1
        message.source_map_fingerprint = hashlib.sha256(
            ('fixed:' + self._scenario).encode(),
        ).hexdigest()[:24]
        message.task_generation_stamp = message.header.stamp
        # Fixed protocol fixtures use one immutable candidate-generation
        # provenance value for their one snapshot.
        message.candidate_generation_id = 1
        message.validity = seconds_to_duration(self._validity_s)
        message.tasks = alternate_hallway_tasks(self._robot_id, session_id, 1)
        for task in message.tasks:
            task.generation_stamp = message.header.stamp
            task.approach_pose.header.stamp = message.header.stamp
        self._message = message
        self._publisher.publish(message)
        if not self._published:
            self._published = True
            self.get_logger().info(
                'FIXED_TASK_SNAPSHOT robot=%s session=%s epoch=1 tasks=%d' % (
                    self._robot_id, session_id, len(message.tasks),
                )
            )


def main(args=None):
    """Run a fixed-task snapshot source."""
    rclpy.init(args=args)
    node = FixedTaskSnapshotSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
