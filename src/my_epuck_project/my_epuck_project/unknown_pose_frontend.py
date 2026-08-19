"""Decentralized descriptor-first unknown relative-pose front end."""

from __future__ import annotations

from collections import OrderedDict
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from my_epuck_interfaces.msg import (
    LocalMapCrop,
    LocalMapCropRequest,
    LocalMapDescriptor,
    PeerMap,
    RelativePoseHypothesis,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .unknown_pose_frontend_core import (
    GridCrop,
    compare_descriptors,
    crop_grid,
    descriptor_checksum,
    polar_descriptor,
    register_crops,
    should_accept_hypothesis,
    temporal_consistency,
)


class UnknownPoseFrontend(Node):
    """Run bounded peer-to-peer place recognition and map registration."""

    def __init__(self):
        super().__init__('unknown_pose_frontend')
        self.declare_parameter('robot_id', self.get_namespace().strip('/'))
        self.declare_parameter('peer_robot_id', '')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('descriptor_topic', '/cslam/relative_pose/descriptors')
        self.declare_parameter('crop_request_topic', '/cslam/relative_pose/crop_requests')
        self.declare_parameter('crop_topic', '/cslam/relative_pose/crops')
        self.declare_parameter('hypothesis_topic', '/cslam/relative_pose/hypotheses')
        self.declare_parameter('peer_map_topic', '/cslam/unknown_pose/local_map')
        self.declare_parameter('descriptor_period_s', 2.0)
        self.declare_parameter('crop_size_m', 8.0)
        self.declare_parameter('max_keyframes', 12)
        self.declare_parameter('descriptor_similarity_gate', 0.72)
        self.declare_parameter('descriptor_margin_gate', 0.005)
        self.declare_parameter('minimum_keyframe_confirmations', 2)
        self.declare_parameter('confirmation_window_s', 8.0)
        self.declare_parameter('shared_frame', 'shared_map')
        self.declare_parameter('diagnostic_output', '')

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.peer_robot_id = str(self.get_parameter('peer_robot_id').value)
        if not self.robot_id or not self.peer_robot_id or self.robot_id == self.peer_robot_id:
            raise ValueError('robot_id and distinct peer_robot_id are required')
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.descriptor_topic = str(self.get_parameter('descriptor_topic').value)
        self.crop_request_topic = str(self.get_parameter('crop_request_topic').value)
        self.crop_topic = str(self.get_parameter('crop_topic').value)
        self.hypothesis_topic = str(self.get_parameter('hypothesis_topic').value)
        self.peer_map_topic = str(self.get_parameter('peer_map_topic').value)
        self.descriptor_period_s = max(0.2, float(
            self.get_parameter('descriptor_period_s').value))
        self.crop_size_m = max(2.0, float(self.get_parameter('crop_size_m').value))
        self.max_keyframes = max(2, int(self.get_parameter('max_keyframes').value))
        self.similarity_gate = float(
            self.get_parameter('descriptor_similarity_gate').value)
        self.margin_gate = float(self.get_parameter('descriptor_margin_gate').value)
        self.minimum_confirmations = max(2, int(
            self.get_parameter('minimum_keyframe_confirmations').value))
        self.confirmation_window_ns = int(max(1.0, float(
            self.get_parameter('confirmation_window_s').value)) * 1.0e9)
        self.shared_frame = str(self.get_parameter('shared_frame').value)
        self.diagnostic_output = str(self.get_parameter('diagnostic_output').value)
        self.counters = {
            'map_messages_received': 0,
            'descriptors_published': 0,
            'descriptors_received': 0,
            'candidate_comparisons': 0,
            'cheap_candidates': 0,
            'cheap_rejections': 0,
            'crop_requests_sent': 0,
            'crop_requests_received': 0,
            'crops_sent': 0,
            'crops_received': 0,
            'registrations': 0,
            'proposals_published': 0,
            'acks_published': 0,
            'accepted_hypotheses': 0,
            'rejected_hypotheses': 0,
            'tf_handoffs': 0,
            'peer_maps_published': 0,
            'merge_handoff_started': 0,
        }
        self.best_similarity = 0.0
        self.best_margin = 0.0
        self.best_known_fraction = 0.0
        self.merge_handoff_logged = False

        qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.descriptor_pub = self.create_publisher(
            LocalMapDescriptor, self.descriptor_topic, qos)
        self.request_pub = self.create_publisher(
            LocalMapCropRequest, self.crop_request_topic, qos)
        self.crop_pub = self.create_publisher(LocalMapCrop, self.crop_topic, qos)
        self.hypothesis_pub = self.create_publisher(
            RelativePoseHypothesis, self.hypothesis_topic, qos)
        self.peer_map_pub = self.create_publisher(PeerMap, self.peer_map_topic, map_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.descriptor_sub = self.create_subscription(
            LocalMapDescriptor, self.descriptor_topic,
            self.descriptor_callback, qos)
        self.request_sub = self.create_subscription(
            LocalMapCropRequest, self.crop_request_topic,
            self.request_callback, qos)
        self.crop_sub = self.create_subscription(
            LocalMapCrop, self.crop_topic, self.crop_callback, qos)
        self.hypothesis_sub = self.create_subscription(
            RelativePoseHypothesis, self.hypothesis_topic,
            self.hypothesis_callback, qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_map = None
        self.map_revision = 0
        self.last_descriptor_wall = 0.0
        self.keyframes = OrderedDict()
        self.peer_descriptors = OrderedDict()
        self.matches = {}
        self.confirmations = {}
        self.pending_requests = set()
        self.pending_proposals = {}
        self.peer_proposals = {}
        self.pending_target_proposal = False
        self.negotiation_started = False
        self.accepted = None
        self.last_export_wall = 0.0
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'Unknown-pose front end {self.robot_id}<->{self.peer_robot_id}; '
            'no transform is published before mutual acceptance')

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

    def map_callback(self, message):
        self.counters['map_messages_received'] += 1
        self.latest_map = message
        self.map_revision += 1
        if self.accepted is not None and time.monotonic() - self.last_export_wall > 1.0:
            self.publish_local_map()

    def _map_crop(self):
        if self.latest_map is None:
            return None
        message = self.latest_map
        values = np.asarray(message.data, dtype=np.int16).reshape(
            (message.info.height, message.info.width))
        center_x = center_y = None
        try:
            transform = self.tf_buffer.lookup_transform(
                message.header.frame_id,
                f'{self.robot_id}/base_footprint',
                rclpy.time.Time(), timeout=Duration(seconds=0.02))
            center_x = transform.transform.translation.x
            center_y = transform.transform.translation.y
        except TransformException:
            pass
        origin_yaw = self._yaw(message.info.origin.orientation)
        if abs(origin_yaw) > 1e-3:
            self.get_logger().debug('Rotated map origin uses crop-center fallback')
        return crop_grid(
            values, float(message.info.resolution),
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
            center_x=center_x, center_y=center_y,
            size_m=self.crop_size_m)

    def publish_descriptor(self):
        crop = self._map_crop()
        if crop is None:
            return
        descriptor = polar_descriptor(crop)
        keyframe_id = f'{self.robot_id}-{self.map_revision:08d}'
        checksum = descriptor_checksum(descriptor)
        message = LocalMapDescriptor()
        message.header = self.latest_map.header
        message.source_robot_id = self.robot_id
        message.keyframe_id = keyframe_id
        message.map_epoch = self.map_revision
        message.resolution = float(crop.resolution)
        message.ring_count = 12
        message.sector_count = 24
        message.descriptor_version = 1
        message.descriptor_bytes = list(descriptor)
        message.crop_origin_x = float(crop.origin_x)
        message.crop_origin_y = float(crop.origin_y)
        message.crop_width = int(crop.values.shape[1])
        message.crop_height = int(crop.values.shape[0])
        message.checksum = checksum
        self.descriptor_pub.publish(message)
        self.counters['descriptors_published'] += 1
        self.keyframes[keyframe_id] = (message, crop)
        while len(self.keyframes) > self.max_keyframes:
            self.keyframes.popitem(last=False)
        self.last_descriptor_wall = time.monotonic()
        self._compare_peer_descriptors()

    def tick(self):
        if time.monotonic() - self.last_descriptor_wall >= self.descriptor_period_s:
            self.publish_descriptor()
        self._compare_peer_descriptors()

    def descriptor_callback(self, message):
        if message.source_robot_id != self.peer_robot_id:
            return
        if message.descriptor_version != 1 or not message.descriptor_bytes:
            return
        self.counters['descriptors_received'] += 1
        key = message.keyframe_id
        self.peer_descriptors[key] = message
        while len(self.peer_descriptors) > self.max_keyframes:
            self.peer_descriptors.popitem(last=False)
        self._compare_peer_descriptors()

    @staticmethod
    def _stamp_ns(message):
        return (int(message.header.stamp.sec) * 1_000_000_000 +
                int(message.header.stamp.nanosec))

    def _temporal_support(self, peer_key, own_key, match):
        """Count distinct nearby cheap matches for this candidate.

        A descriptor/keyframe pair is advertised once, so counting repeated
        comparisons of that same pair can never provide confirmation.  Use
        distinct nearby own/peer keyframes instead; geometric registration is
        still the authoritative gate after this cheap temporal check.
        """
        own_entry = self.keyframes.get(own_key)
        peer_message = self.peer_descriptors.get(peer_key)
        if own_entry is None or peer_message is None:
            return 0
        own_stamp = self._stamp_ns(own_entry[0])
        peer_stamp = self._stamp_ns(peer_message)
        support = set()
        for (other_peer_key, other_own_key), other_match in self.matches.items():
            if (other_peer_key, other_own_key) == (peer_key, own_key):
                support.add((other_peer_key, other_own_key))
                continue
            if (other_match.similarity < self.similarity_gate or
                    other_match.margin < self.margin_gate or
                    other_match.known_fraction < 0.12):
                continue
            other_own_entry = self.keyframes.get(other_own_key)
            other_peer_message = self.peer_descriptors.get(other_peer_key)
            if other_own_entry is None or other_peer_message is None:
                continue
            if (abs(self._stamp_ns(other_own_entry[0]) - own_stamp) >
                    self.confirmation_window_ns or
                    abs(self._stamp_ns(other_peer_message) - peer_stamp) >
                    self.confirmation_window_ns):
                continue
            shift_delta = abs(int(other_match.sector_shift) -
                              int(match.sector_shift))
            shift_delta = min(shift_delta, 24 - shift_delta)
            if shift_delta <= 2:
                support.add((other_peer_key, other_own_key))
        return len(support)

    def _compare_peer_descriptors(self):
        if not self.keyframes or not self.peer_descriptors:
            return
        for peer_key, peer in list(self.peer_descriptors.items()):
            for own_key, (own, _) in list(self.keyframes.items()):
                pair = (peer_key, own_key)
                try:
                    match = compare_descriptors(
                        bytes(own.descriptor_bytes), bytes(peer.descriptor_bytes),
                        int(own.ring_count), int(own.sector_count))
                except ValueError:
                    continue
                self.counters['candidate_comparisons'] += 1
                self.matches[pair] = match
                self.best_similarity = max(self.best_similarity, match.similarity)
                self.best_margin = max(self.best_margin, match.margin)
                self.best_known_fraction = max(
                    self.best_known_fraction, match.known_fraction)
                if (match.similarity < self.similarity_gate or
                        match.margin < self.margin_gate or
                        match.known_fraction < 0.12):
                    self.counters['cheap_rejections'] += 1
                    continue
                self.counters['cheap_candidates'] += 1
                confirmations = self.confirmations.setdefault(pair, set())
                confirmations.add((own_key, peer_key))
                support_count = self._temporal_support(peer_key, own_key, match)
                if support_count < self.minimum_confirmations:
                    continue
                if self.robot_id > self.peer_robot_id:
                    continue
                if self.negotiation_started:
                    continue
                request_key = (peer_key, int(peer.checksum))
                if request_key in self.pending_requests:
                    continue
                self.pending_requests.add(request_key)
                self.negotiation_started = True
                request = LocalMapCropRequest()
                request.header = own.header
                request.requester_robot_id = self.robot_id
                request.source_robot_id = self.peer_robot_id
                request.keyframe_id = peer_key
                request.descriptor_checksum = int(peer.checksum)
                self.request_pub.publish(request)
                self.counters['crop_requests_sent'] += 1

    def request_callback(self, request):
        if request.source_robot_id != self.robot_id:
            return
        self.counters['crop_requests_received'] += 1
        stored = self.keyframes.get(request.keyframe_id)
        if stored is None:
            return
        descriptor, crop = stored
        if request.descriptor_checksum and int(descriptor.checksum) != int(
                request.descriptor_checksum):
            return
        crop_message = LocalMapCrop()
        crop_message.header = descriptor.header
        crop_message.source_robot_id = self.robot_id
        crop_message.keyframe_id = request.keyframe_id
        crop_message.map_epoch = descriptor.map_epoch
        crop_message.descriptor_checksum = descriptor.checksum
        crop_message.occupancy_grid = self._crop_message(crop, descriptor.header)
        self.crop_pub.publish(crop_message)
        self.counters['crops_sent'] += 1

    @staticmethod
    def _crop_message(crop, header):
        message = OccupancyGrid()
        message.header = header
        message.info.resolution = float(crop.resolution)
        message.info.width = int(crop.values.shape[1])
        message.info.height = int(crop.values.shape[0])
        message.info.origin.position.x = float(crop.origin_x)
        message.info.origin.position.y = float(crop.origin_y)
        message.info.origin.orientation.w = 1.0
        message.data = [int(value) for value in crop.values.ravel()]
        return message

    def crop_callback(self, message):
        if message.source_robot_id != self.peer_robot_id:
            return
        self.counters['crops_received'] += 1
        if self.robot_id > self.peer_robot_id:
            proposal = self.peer_proposals.get(message.keyframe_id)
            if proposal is None:
                return
            target_entry = self.keyframes.get(proposal.target_keyframe_id)
            if target_entry is None:
                return
            source_crop = self._grid_crop_from_message(message.occupancy_grid)
            self.counters['registrations'] += 1
            result = register_crops(source_crop, target_entry[1])
            tx, ty, yaw = result.transform
            proposed_tx = proposal.source_to_target.translation.x
            proposed_ty = proposal.source_to_target.translation.y
            proposed_yaw = math.atan2(
                2.0 * (proposal.source_to_target.rotation.w *
                        proposal.source_to_target.rotation.z),
                1.0 - 2.0 * proposal.source_to_target.rotation.z ** 2)
            translation_error = math.hypot(tx - proposed_tx, ty - proposed_ty)
            yaw_error = abs(math.atan2(
                math.sin(yaw - proposed_yaw), math.cos(yaw - proposed_yaw)))
            mutually_consistent = translation_error <= 0.25 and yaw_error <= 0.15
            accepted = result.accepted and mutually_consistent
            response = self._ack_message(
                proposal, result, accepted,
                '' if accepted else 'MUTUAL_TRANSFORM_INCONSISTENT')
            self.hypothesis_pub.publish(response)
            self.counters['acks_published'] += 1
            if accepted:
                self.counters['accepted_hypotheses'] += 1
            else:
                self.counters['rejected_hypotheses'] += 1
                self.pending_target_proposal = False
            return
        peer_key = message.keyframe_id
        peer_descriptor = self.peer_descriptors.get(peer_key)
        if peer_descriptor is None:
            return
        own_candidates = [
            (own_key, own, crop)
            for own_key, (own, crop) in self.keyframes.items()
            if (peer_key, own_key) in self.confirmations]
        if not own_candidates:
            return
        own_key, own_descriptor, own_crop = own_candidates[-1]
        peer_crop = self._grid_crop_from_message(message.occupancy_grid)
        self.counters['registrations'] += 1
        result = register_crops(own_crop, peer_crop)
        self.pending_proposals[(own_key, peer_key)] = result
        proposal = self._hypothesis_message(
            own_descriptor, peer_descriptor, result,
            status='PROPOSED' if result.accepted else 'REJECTED',
            accepted=False,
            rejection_reason='' if result.accepted else result.reason)
        self.hypothesis_pub.publish(proposal)
        self.counters['proposals_published'] += 1
        if not result.accepted:
            self.counters['rejected_hypotheses'] += 1
            self.negotiation_started = False

    @staticmethod
    def _grid_crop_from_message(message):
        values = np.asarray(message.data, dtype=np.int16).reshape(
            (message.info.height, message.info.width))
        return GridCrop(
            values=values, resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y))

    @staticmethod
    def _transform_delta(first, second):
        translation = math.hypot(
            float(first.transform.translation.x - second.transform.translation.x),
            float(first.transform.translation.y - second.transform.translation.y))
        yaw_first = math.atan2(
            2.0 * (first.transform.rotation.w * first.transform.rotation.z),
            1.0 - 2.0 * first.transform.rotation.z ** 2)
        yaw_second = math.atan2(
            2.0 * (second.transform.rotation.w * second.transform.rotation.z),
            1.0 - 2.0 * second.transform.rotation.z ** 2)
        yaw = abs(math.atan2(math.sin(yaw_first - yaw_second),
                             math.cos(yaw_first - yaw_second)))
        return translation, yaw

    def _hypothesis_message(self, own, peer, result, status, accepted, rejection_reason):
        message = RelativePoseHypothesis()
        message.header = own.header
        message.source_robot_id = self.robot_id
        message.target_robot_id = self.peer_robot_id
        message.source_keyframe_id = own.keyframe_id
        message.target_keyframe_id = peer.keyframe_id
        message.source_to_target.translation.x = float(result.transform[0])
        message.source_to_target.translation.y = float(result.transform[1])
        message.source_to_target.rotation.z = math.sin(result.transform[2] / 2.0)
        message.source_to_target.rotation.w = math.cos(result.transform[2] / 2.0)
        message.covariance = list(result.covariance)
        match = self.matches.get((peer.keyframe_id, own.keyframe_id))
        message.descriptor_similarity = float(match.similarity if match else 0.0)
        message.descriptor_margin = float(match.margin if match else 0.0)
        message.geometric_inlier_ratio = float(result.inlier_ratio)
        message.registration_residual_m = float(result.residual_m)
        message.occupied_free_agreement = float(result.occupied_free_agreement)
        message.overlap_fraction = float(result.overlap_fraction)
        message.temporal_consistency = float(temporal_consistency(
            [own.header.stamp.sec * 1_000_000_000 + own.header.stamp.nanosec,
             peer.header.stamp.sec * 1_000_000_000 + peer.header.stamp.nanosec]))
        message.final_confidence = float(max(0.0, min(1.0,
            0.30 * message.descriptor_similarity +
            0.15 * message.descriptor_margin +
            0.25 * message.geometric_inlier_ratio +
            0.15 * message.occupied_free_agreement +
            0.15 * message.overlap_fraction)))
        message.status = status
        message.rejection_reason = rejection_reason
        message.accepted = bool(accepted)
        return message

    def hypothesis_callback(self, message):
        if {message.source_robot_id, message.target_robot_id} != {
                self.robot_id, self.peer_robot_id}:
            return
        if self.accepted is not None:
            return
        if message.status == 'PROPOSED' and self.robot_id == message.target_robot_id:
            if self.pending_target_proposal:
                return
            self.pending_target_proposal = True
            self.peer_proposals[message.source_keyframe_id] = message
            self._request_source_for_confirmation(message)
            return
        if message.status == 'REJECTED':
            if self.robot_id == message.source_robot_id:
                self.negotiation_started = False
            return
        if not should_accept_hypothesis(
                self.accepted, message.status, message.accepted,
                message.final_confidence):
            return
        if self.robot_id != message.source_robot_id:
            self.accepted = message
            self.publish_local_map()
            return
        proposal = self.pending_proposals.get(
            (message.source_keyframe_id, message.target_keyframe_id))
        if proposal is None or not proposal.accepted:
            return
        own = self.keyframes.get(message.source_keyframe_id)
        peer = self.peer_descriptors.get(message.target_keyframe_id)
        if own is None or peer is None:
            return
        final = self._hypothesis_message(
            own[0], peer, proposal, status='ACCEPTED', accepted=True,
            rejection_reason='')
        self.hypothesis_pub.publish(final)
        self.counters['accepted_hypotheses'] += 1
        self.accepted = final
        self.publish_accepted_tf()
        self.publish_local_map()

    def _ack_message(self, proposal, result, accepted, rejection_reason):
        message = RelativePoseHypothesis()
        message.header = proposal.header
        message.source_robot_id = proposal.source_robot_id
        message.target_robot_id = proposal.target_robot_id
        message.source_keyframe_id = proposal.source_keyframe_id
        message.target_keyframe_id = proposal.target_keyframe_id
        message.source_to_target.translation.x = float(result.transform[0])
        message.source_to_target.translation.y = float(result.transform[1])
        message.source_to_target.rotation.z = math.sin(result.transform[2] / 2.0)
        message.source_to_target.rotation.w = math.cos(result.transform[2] / 2.0)
        message.covariance = list(result.covariance)
        message.descriptor_similarity = proposal.descriptor_similarity
        message.descriptor_margin = proposal.descriptor_margin
        message.geometric_inlier_ratio = float(result.inlier_ratio)
        message.registration_residual_m = float(result.residual_m)
        message.occupied_free_agreement = float(result.occupied_free_agreement)
        message.overlap_fraction = float(result.overlap_fraction)
        message.temporal_consistency = proposal.temporal_consistency
        message.final_confidence = float(max(0.0, min(1.0,
            0.30 * message.descriptor_similarity +
            0.15 * message.descriptor_margin +
            0.25 * message.geometric_inlier_ratio +
            0.15 * message.occupied_free_agreement +
            0.15 * message.overlap_fraction)))
        message.status = 'ACCEPTED' if accepted else 'REJECTED'
        message.rejection_reason = rejection_reason
        message.accepted = bool(accepted)
        return message

    def _request_source_for_confirmation(self, proposal):
        key = (proposal.source_keyframe_id, self.peer_robot_id)
        if key in self.pending_requests:
            return
        self.pending_requests.add(key)
        request = LocalMapCropRequest()
        request.header = proposal.header
        request.requester_robot_id = self.robot_id
        request.source_robot_id = proposal.source_robot_id
        request.keyframe_id = proposal.source_keyframe_id
        request.descriptor_checksum = 0
        self.request_pub.publish(request)
        self.counters['crop_requests_sent'] += 1

    def publish_accepted_tf(self):
        if self.accepted is None or self.robot_id != min(self.robot_id, self.peer_robot_id):
            return
        now = self.get_clock().now().to_msg()
        identity = TransformStamped()
        identity.header.stamp = now
        identity.header.frame_id = self.shared_frame
        identity.child_frame_id = f'{self.robot_id}/local_world'
        identity.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(identity)
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.shared_frame
        transform.child_frame_id = f'{self.peer_robot_id}/local_world'
        transform.transform = self.accepted.source_to_target
        self.tf_broadcaster.sendTransform(transform)
        self.counters['tf_handoffs'] += 1

    def publish_local_map(self):
        if self.latest_map is None or self.accepted is None:
            return
        if not self.merge_handoff_logged:
            self.merge_handoff_logged = True
            self.counters['merge_handoff_started'] += 1
            self.get_logger().info(
                'UNKNOWN_POSE_MERGE_HANDOFF_START '
                f'confidence={self.accepted.final_confidence:.6f} '
                f'descriptor_similarity={self.accepted.descriptor_similarity:.6f} '
                f'geometric_inlier_ratio={self.accepted.geometric_inlier_ratio:.6f} '
                f'residual_m={self.accepted.registration_residual_m:.6f} '
                f'overlap_fraction={self.accepted.overlap_fraction:.6f} '
                f'source_keyframe={self.accepted.source_keyframe_id} '
                f'target_keyframe={self.accepted.target_keyframe_id} '
                f'robot={self.robot_id}')
        message = PeerMap()
        message.source_robot_id = self.robot_id
        message.revision = self.map_revision
        message.export_stamp = self.get_clock().now().to_msg()
        message.local_evidence_only = True
        message.occupancy_grid = self.latest_map
        self.peer_map_pub.publish(message)
        self.counters['peer_maps_published'] += 1
        self.last_export_wall = time.monotonic()

    def finalize(self):
        """Persist bounded diagnostics without affecting navigation behavior."""
        if not self.diagnostic_output:
            return
        path = Path(self.diagnostic_output)
        path.mkdir(parents=True, exist_ok=True)
        output = path / f'{self.robot_id}_unknown_pose_frontend.json'
        payload = {
            'robot_id': self.robot_id,
            'peer_robot_id': self.peer_robot_id,
            'map_revision': self.map_revision,
            'keyframes_retained': len(self.keyframes),
            'peer_descriptors_retained': len(self.peer_descriptors),
            'accepted': bool(self.accepted is not None),
            'best_similarity': self.best_similarity,
            'best_margin': self.best_margin,
            'best_known_fraction': self.best_known_fraction,
            'accepted_confidence': (
                None if self.accepted is None else self.accepted.final_confidence),
            'accepted_descriptor_similarity': (
                None if self.accepted is None else self.accepted.descriptor_similarity),
            'accepted_geometric_inlier_ratio': (
                None if self.accepted is None else self.accepted.geometric_inlier_ratio),
            'accepted_registration_residual_m': (
                None if self.accepted is None else self.accepted.registration_residual_m),
            'accepted_overlap_fraction': (
                None if self.accepted is None else self.accepted.overlap_fraction),
            'counters': self.counters,
        }
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UnknownPoseFrontend()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.finalize()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
