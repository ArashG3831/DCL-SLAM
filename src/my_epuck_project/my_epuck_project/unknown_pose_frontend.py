"""Decentralized descriptor-first unknown relative-pose front end."""

from __future__ import annotations

from collections import Counter, OrderedDict, deque
import json
import hashlib
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
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)

from .unknown_pose_frontend_core import (
    DedicatedDiagnosticJsonl,
    BoundedVerificationBatchController,
    candidate_reuses_accepted_physical_view,
    candidate_views_are_spatially_separated,
    GridCrop,
    compare_descriptors,
    compare_descriptor_pairs,
    confirmation_window_for_cadence,
    crop_grid,
    crop_batch_is_ready,
    accumulate_physical_candidates,
    bounded_candidate_verification_order,
    deduplicate_physical_candidates,
    descriptor_checksum,
    evidence_batch_is_spatially_diverse,
    evidence_candidates_for_pool,
    evidence_pairs_for_selection,
    invert_se2,
    polar_descriptor,
    physical_crop_identity,
    physical_candidate_geometry_identity,
    physical_candidate_identity,
    register_crops,
    register_crop_set,
    should_accept_hypothesis,
    temporal_support_count,
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
        # Descriptor/crop requests can arrive after several descriptor periods
        # under the existing synchronous DDS path.  Keep a larger but bounded
        # history so an advertised keyframe remains requestable; this is not a
        # full-map exchange and does not make the history unbounded.
        self.declare_parameter('max_keyframes', 64)
        self.declare_parameter('descriptor_similarity_gate', 0.72)
        self.declare_parameter('descriptor_margin_gate', 0.005)
        self.declare_parameter('minimum_keyframe_confirmations', 2)
        self.declare_parameter('confirmation_window_s', 8.0)
        self.declare_parameter('min_consistent_constraints', 3)
        self.declare_parameter('max_evidence_constraints', 5)
        self.declare_parameter('candidate_verification_budget', 8)
        self.declare_parameter('max_verification_batches', 4)
        self.declare_parameter('verification_lifetime_s', 600.0)
        self.declare_parameter('verification_novelty_spacing_m', 0.40)
        self.declare_parameter('evidence_acquisition_window_s', 8.0)
        self.declare_parameter('target_map_radius_m', 40.0)
        self.declare_parameter('max_projected_registration_error_m', 0.20)
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
        self.confirmation_cadence_factor = 2.5
        self.min_consistent_constraints = max(2, int(
            self.get_parameter('min_consistent_constraints').value))
        self.max_evidence_constraints = max(
            self.min_consistent_constraints, int(
                self.get_parameter('max_evidence_constraints').value))
        self.candidate_verification_budget = max(
            self.min_consistent_constraints, int(
                self.get_parameter('candidate_verification_budget').value))
        self.max_verification_batches = max(1, int(
            self.get_parameter('max_verification_batches').value))
        self.verification_lifetime_s = max(1.0, float(
            self.get_parameter('verification_lifetime_s').value))
        self.verification_novelty_spacing_m = max(0.01, float(
            self.get_parameter('verification_novelty_spacing_m').value))
        self.evidence_acquisition_window_s = max(0.5, float(
            self.get_parameter('evidence_acquisition_window_s').value))
        self.target_map_radius_m = max(1.0, float(
            self.get_parameter('target_map_radius_m').value))
        self.max_projected_registration_error_m = max(0.01, float(
            self.get_parameter('max_projected_registration_error_m').value))
        self.shared_frame = str(self.get_parameter('shared_frame').value)
        self.diagnostic_output = str(self.get_parameter('diagnostic_output').value)
        self.consensus_diagnostics = None
        if self.diagnostic_output:
            self.consensus_diagnostics = DedicatedDiagnosticJsonl(
                Path(self.diagnostic_output) /
                f'{self.robot_id}_consensus_diagnostics.jsonl')
            self.physical_evidence_diagnostics = DedicatedDiagnosticJsonl(
                Path(self.diagnostic_output) /
                f'{self.robot_id}_physical_evidence_diagnostics.jsonl',
                # Candidate observations can be numerous in a single
                # diagnostic motion fixture.  Keep this stream independent
                # from the protocol buffer and large enough that selection,
                # crop, and state-transition records are not evicted before
                # shutdown finalization.
                max_records=1_000_000)
        else:
            self.physical_evidence_diagnostics = None
        self.counters = {
            'map_messages_received': 0,
            'descriptors_published': 0,
            'descriptors_received': 0,
            'candidate_comparisons': 0,
            'cheap_candidates': 0,
            'cheap_rejections': 0,
            'crop_requests_sent': 0,
            'crop_requests_queued': 0,
            'crop_request_duplicates_suppressed': 0,
            'crop_requests_received': 0,
            'crops_sent': 0,
            'crops_received': 0,
            'crop_responses_accepted': 0,
            'crop_response_rejections': 0,
            'registrations': 0,
            'registration_callback_entries': 0,
            'registration_callback_exits': 0,
            'registration_callback_exceptions': 0,
            'proposals_published': 0,
            'acks_published': 0,
            'accepted_hypotheses': 0,
            'rejected_hypotheses': 0,
            'tf_handoffs': 0,
            'peer_maps_published': 0,
            'merge_handoff_started': 0,
            'multi_constraint_attempts': 0,
            'multi_constraint_rejections': 0,
            'candidate_selections': 0,
            'candidate_selection_deferrals': 0,
            'physical_candidate_duplicates_suppressed': 0,
            'physical_geometry_rejections_suppressed': 0,
            'physical_evidence_duplicates_suppressed': 0,
            'spatial_diversity_deferrals': 0,
            'pending_candidate_additions': 0,
            'pending_candidate_removals': 0,
            'constraints_accumulated': 0,
            'evidence_sets_formed': 0,
            'temporal_gate_rejections': 0,
            'temporal_support_evaluations': 0,
            'temporal_support_max': 0,
            'candidate_verification_attempts': 0,
            'candidate_verification_accepted': 0,
            'candidate_verification_rejected': 0,
            'candidate_verification_budget_exhausted': 0,
            'verification_batches_opened': 0,
            'verification_batches_exhausted': 0,
            'verification_batch_reentries': 0,
            'verification_novelty_deferrals': 0,
            'verification_lifetime_expired': 0,
            'stale_verification_batch_responses': 0,
            'diagnostic_write_failures': 0,
        }
        self.gate_rejection_counts = Counter()
        self.temporal_gate_rejection_counts = Counter()
        self.crop_response_rejection_counts = Counter()
        self.consensus_gate_rejection_counts = Counter()
        self.descriptor_gate_survivors = set()
        self.temporal_gate_survivors = set()
        self.diagnostic_events = []
        self.diagnostic_event_drops = 0
        self.callback_stats = {}
        self.callback_started = 0
        self.callback_completed = 0
        self.callback_inflight = 0
        self.max_callback_inflight = 0
        self.max_backlog_estimate = 0
        self.cpu_samples = []
        self._last_cpu_wall = time.monotonic()
        self._last_cpu_process = time.process_time()
        self.best_similarity = 0.0
        self.best_margin = 0.0
        self.best_known_fraction = 0.0
        self.merge_handoff_logged = False
        # Bounded diagnostic timing/size state.  These values are never used
        # by descriptor matching, registration, or merger acceptance.
        self.first_map_wall = None
        self.first_candidate_wall = None
        self.accepted_wall = None
        self.descriptor_bytes = 0
        self.crop_cells_sent = 0
        self.crop_cells_received = 0
        self.active_candidate_pairs = []
        self.pending_candidate_pairs = {}
        self.request_own_by_peer_key = {}
        self.request_own_by_request_key = {}
        self.evidence_pairs = {}
        self.evidence_physical_keys = {}
        self.evidence_physical_geometry_keys = set()
        self.candidate_verification_attempted = set()
        self.candidate_verification_results = {}
        # Acquisition-only history used to keep one repeatedly rejected
        # physical view from monopolising later verification batches.  Exact
        # pair rejection and accepted-evidence deduplication remain separate
        # and authoritative; this counter changes ordering only.
        self.attempted_physical_view_reuse_counts = {}
        self.request_candidate_by_request_key = {}
        self.request_metadata_by_request_key = {}
        self.completed_request_keys = set()
        self.candidate_verification_attempts = 0
        self.candidate_verification_batch_attempts = 0
        self.verification_attempt_sequence = 0
        self.rejected_physical_evidence_keys = set()
        # Exact physical identities retain epoch/checksum freshness.  This
        # companion set prevents a rejected crop footprint from being retried
        # under a later map revision when its geometry is unchanged.
        self.rejected_physical_geometry_keys = set()
        self._diagnosed_physical_candidates = set()
        # Repeated observations of an already-pending physical candidate are
        # represented by the counter below.  Persisting one diagnostic record
        # per repetition can dominate the single-threaded executor and delay
        # the actual crop/registration callbacks; the estimator state itself
        # remains unchanged and every request/result/rejection is still
        # recorded.
        self._diagnosed_duplicate_physical_candidates = set()
        self.received_peer_crops = {}
        self.batch_proposal_published = False
        self.pending_target_proposal = False
        self.registration_callback_depth = 0
        self.evidence_acquisition_deadline_wall = None
        self.evidence_acquisition_started = False
        self.verification_batches = BoundedVerificationBatchController(
            budget=self.candidate_verification_budget,
            max_batches=self.max_verification_batches,
            lifetime_s=self.verification_lifetime_s,
            novelty_spacing_m=self.verification_novelty_spacing_m)

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
            OccupancyGrid, self.map_topic,
            lambda message: self._timed_callback(
                'map_callback', self.map_callback, message), map_qos)
        self.descriptor_sub = self.create_subscription(
            LocalMapDescriptor, self.descriptor_topic,
            lambda message: self._timed_callback(
                'descriptor_callback', self.descriptor_callback, message), qos)
        self.request_sub = self.create_subscription(
            LocalMapCropRequest, self.crop_request_topic,
            lambda message: self._timed_callback(
                'request_callback', self.request_callback, message), qos)
        self.crop_sub = self.create_subscription(
            LocalMapCrop, self.crop_topic,
            lambda message: self._timed_callback(
                'crop_callback', self.crop_callback, message), qos)
        self.hypothesis_sub = self.create_subscription(
            RelativePoseHypothesis, self.hypothesis_topic,
            lambda message: self._timed_callback(
                'hypothesis_callback', self.hypothesis_callback, message), qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # The accepted alignment is immutable for the lifetime of this
        # frontend. Publish it on /tf_static so a post-handoff fusion process
        # can start later and still resolve the canonical map chain; a
        # one-shot dynamic /tf sample expires from a late listener's buffer.
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.latest_map = None
        self.map_revision = 0
        self.keyframe_sequence = 0
        self.last_descriptor_wall = 0.0
        self.keyframes = OrderedDict()
        self.peer_descriptors = OrderedDict()
        self.own_descriptor_stamps_ns = deque(maxlen=16)
        self.peer_descriptor_stamps_ns = deque(maxlen=16)
        self.effective_confirmation_window_ns = self.confirmation_window_ns
        self.matches = {}
        self.compared_pairs = set()
        # Descriptor subscriptions are serviced by the ROS executor.  Keep
        # their callbacks bounded: comparison work is drained by the existing
        # timer one keyframe at a time so a busy peer cannot starve receipt of
        # newer descriptors.
        self.pending_peer_descriptor_keys = deque(maxlen=self.max_keyframes)
        self.pending_own_descriptor_keys = deque(maxlen=self.max_keyframes)
        self.pending_descriptor_pair_keys = deque(maxlen=8192)
        self.pending_descriptor_pair_key_set = set()
        self.descriptor_pair_budget_per_tick = 16
        self.descriptor_gate_status = {}
        self.temporal_support_cache = {}
        self.temporal_gate_rejected_pairs = set()
        self.confirmations = {}
        self.pending_requests = set()
        self.pending_proposals = {}
        self.peer_proposals = {}
        self.pending_target_proposal = False
        self.negotiation_started = False
        self.accepted = None
        self.last_export_wall = 0.0
        self.timer = self.create_timer(
            0.2, lambda: self._timed_callback('timer_tick', self.tick))
        self.get_logger().info(
            f'Unknown-pose front end {self.robot_id}<->{self.peer_robot_id}; '
            'no transform is published before mutual acceptance')

    def _write_consensus_diagnostic(self, record_type, **fields):
        """Stream consensus records independently of bounded protocol events."""
        if self.consensus_diagnostics is None:
            return
        try:
            record = {
                'record_type': str(record_type),
                'robot_id': self.robot_id,
                'peer_robot_id': self.peer_robot_id,
                'wall_monotonic_s': time.monotonic(),
                'ros_time_s': self.get_clock().now().nanoseconds / 1.0e9,
                'acquisition_batch_id': int(self.verification_batches.batch_id),
                'verification_batch_attempts': int(
                    self.candidate_verification_batch_attempts),
            }
            record.update(fields)
            self.consensus_diagnostics.write(record)
        except Exception as error:  # diagnostics must never kill estimation
            self.counters['diagnostic_write_failures'] += 1
            self.get_logger().error(
                'UNKNOWN_POSE_DIAGNOSTIC_WRITE_FAILURE record=%s error=%s' %
                (record_type, error))

    def _write_physical_evidence_diagnostic(self, record_type, **fields):
        """Stream formation evidence independently of protocol-event storage."""
        if self.physical_evidence_diagnostics is None:
            return
        try:
            record = {
                'record_type': str(record_type),
                'robot_id': self.robot_id,
                'peer_robot_id': self.peer_robot_id,
                'wall_monotonic_s': time.monotonic(),
                'ros_time_s': self.get_clock().now().nanoseconds / 1.0e9,
                'acquisition_batch_id': int(self.verification_batches.batch_id),
                'verification_batch_attempts': int(
                    self.candidate_verification_batch_attempts),
            }
            record.update(fields)
            self.physical_evidence_diagnostics.write(record)
        except Exception as error:  # diagnostics must never kill estimation
            self.counters['diagnostic_write_failures'] += 1
            self.get_logger().error(
                'UNKNOWN_POSE_DIAGNOSTIC_WRITE_FAILURE record=%s error=%s' %
                (record_type, error))

    @staticmethod
    def _descriptor_geometry(descriptor):
        resolution = float(descriptor.resolution)
        width = int(descriptor.crop_width)
        height = int(descriptor.crop_height)
        origin = [float(descriptor.crop_origin_x),
                  float(descriptor.crop_origin_y), 0.0]
        center = [origin[0] + 0.5 * width * resolution,
                  origin[1] + 0.5 * height * resolution]
        return {
            'origin': origin,
            'center': center,
            'width': width,
            'height': height,
            'resolution': resolution,
            'orientation_yaw': 0.0,
            'map_epoch': int(descriptor.map_epoch),
            'checksum': int(descriptor.checksum),
            'keyframe_id': str(descriptor.keyframe_id),
            'keyframe_creation_timestamp_ns': int(
                UnknownPoseFrontend._stamp_ns(descriptor)),
        }

    @staticmethod
    def _crop_geometry(crop, keyframe_id='', map_epoch=0, checksum=0):
        height, width = crop.values.shape[:2]
        origin = [float(crop.origin_x), float(crop.origin_y),
                  float(crop.origin_yaw)]
        centre = [
            float(crop.origin_x + 0.5 * width * crop.resolution),
            float(crop.origin_y + 0.5 * height * crop.resolution)]
        return {
            'origin': origin,
            'center': centre,
            'width': int(width),
            'height': int(height),
            'resolution': float(crop.resolution),
            'orientation_yaw': float(crop.origin_yaw),
            'map_epoch': int(map_epoch),
            'checksum': int(checksum),
            'keyframe_id': str(keyframe_id),
        }

    def _candidate_diagnostic(self, candidate, status=None, reason=None,
                              compact=False):
        if len(candidate) == 5:
            _, peer_key, own_key, peer, own = candidate
        else:
            peer_key, own_key, peer, own = candidate
        own_entry = self.keyframes.get(own_key)
        own_crop = None if own_entry is None else own_entry[1]
        if own_entry is None or own_crop is None:
            return {
                'own_keyframe_id': str(own_key),
                'peer_keyframe_id': str(peer_key),
                'status': status,
                'rejection_reason': reason,
            }
        match = self.matches.get((peer_key, own_key))
        common = {
            'own_keyframe_id': str(own_key),
            'peer_keyframe_id': str(peer_key),
            'peer_descriptor': self._descriptor_geometry(peer),
            'own_crop': self._crop_geometry(
                own_crop, own.keyframe_id, own.map_epoch, own.checksum),
            'physical_identity': list(physical_candidate_identity(
                candidate, {own_key: own_crop}) ),
            'descriptor_similarity': (
                None if match is None else float(match.similarity)),
            'descriptor_margin': (
                None if match is None else float(match.margin)),
            'status': status,
            'rejection_reason': reason,
        }
        if compact:
            # Selection/formation diagnostics can contain many candidates.
            # Keep every field needed to reconstruct physical identity and
            # geometry without repeating the redundant own descriptor object.
            common['own_keyframe_creation_timestamp_ns'] = int(
                self._stamp_ns(own))
            return common
        common['own_descriptor'] = self._descriptor_geometry(own)
        return common

    def _candidate_diagnostic_reference(self, candidate, status=None,
                                        reason=None):
        """Return a small reference for repeated selection-attempt records."""
        if len(candidate) == 5:
            _, peer_key, own_key, peer, own = candidate
        else:
            peer_key, own_key, peer, own = candidate
        match = self.matches.get((peer_key, own_key))
        own_entry = self.keyframes.get(own_key)
        own_crop = None if own_entry is None else own_entry[1]
        return {
            'own_keyframe_id': str(own_key),
            'peer_keyframe_id': str(peer_key),
            'physical_identity': list(self._candidate_physical_key(candidate)),
            'own_crop': None if own_crop is None else self._crop_geometry(
                own_crop, own_key, own.map_epoch, own.checksum),
            'peer_crop': self._descriptor_geometry(peer),
            'own_map_epoch': int(own.map_epoch),
            'own_checksum': int(own.checksum),
            'peer_map_epoch': int(peer.map_epoch),
            'peer_checksum': int(peer.checksum),
            'descriptor_similarity': (
                None if match is None else float(match.similarity)),
            'descriptor_margin': (
                None if match is None else float(match.margin)),
            'status': status,
            'rejection_reason': reason,
        }

    def _record_diagnostic_event(self, event_type, **fields):
        """Retain a bounded wall/ROS timestamped protocol trace."""
        if len(self.diagnostic_events) >= 512:
            self.diagnostic_event_drops += 1
            return
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        event = {
            'event': event_type,
            'wall_monotonic_s': now,
            'ros_time_s': ros_now / 1.0e9,
        }
        event.update(fields)
        self.diagnostic_events.append(event)

    def _timed_callback(self, name, callback, *args):
        """Measure callback service time without changing callback behavior."""
        started = time.perf_counter()
        self.callback_started += 1
        self.callback_inflight += 1
        self.max_callback_inflight = max(
            self.max_callback_inflight, self.callback_inflight)
        self.max_backlog_estimate = max(
            self.max_backlog_estimate,
            max(0, self.callback_started - self.callback_completed - 1))
        try:
            return callback(*args)
        finally:
            duration_ms = (time.perf_counter() - started) * 1000.0
            stats = self.callback_stats.setdefault(name, {
                'count': 0, 'total_ms': 0.0, 'max_ms': 0.0,
                'samples_ms': [],
            })
            stats['count'] += 1
            stats['total_ms'] += duration_ms
            stats['max_ms'] = max(stats['max_ms'], duration_ms)
            if len(stats['samples_ms']) < 256:
                stats['samples_ms'].append(duration_ms)
            self.callback_inflight -= 1
            self.callback_completed += 1

    def _sample_cpu(self):
        now = time.monotonic()
        process = time.process_time()
        elapsed = now - self._last_cpu_wall
        if elapsed < 1.0:
            return
        self.cpu_samples.append({
            'wall_monotonic_s': now,
            'process_cpu_percent_one_core': (
                100.0 * (process - self._last_cpu_process) / elapsed),
        })
        self.cpu_samples = self.cpu_samples[-256:]
        self._last_cpu_wall = now
        self._last_cpu_process = process

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

    def _update_confirmation_cadence(self, stamps, stamp_ns):
        """Track message-clock cadence without using wall time as evidence."""
        stamp_ns = int(stamp_ns)
        if stamps and stamp_ns <= stamps[-1]:
            return
        if stamps:
            interval = stamp_ns - stamps[-1]
            if interval > 0:
                stamps.append(stamp_ns)
        else:
            stamps.append(stamp_ns)
        intervals = []
        for values in (self.own_descriptor_stamps_ns,
                       self.peer_descriptor_stamps_ns):
            intervals.extend(
                right - left for left, right in zip(values, list(values)[1:])
                if right > left)
        self.effective_confirmation_window_ns = (
            confirmation_window_for_cadence(
                self.confirmation_window_ns, intervals,
                self.confirmation_cadence_factor))

    def map_callback(self, message):
        self.counters['map_messages_received'] += 1
        if self.first_map_wall is None:
            self.first_map_wall = time.monotonic()
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
        return crop_grid(
            values, float(message.info.resolution),
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
            center_x=center_x, center_y=center_y,
            size_m=self.crop_size_m, origin_yaw=origin_yaw)

    def _allocate_keyframe_id(self):
        """Allocate an identity for each materially sampled descriptor.

        ``map_revision`` remains the map epoch, but it is not a keyframe
        identity: with a slower SLAM publication interval the robot can move
        and produce new spatial views between two map revisions.
        """
        self.keyframe_sequence += 1
        return f'{self.robot_id}-{self.keyframe_sequence:08d}'

    def publish_descriptor(self):
        crop = self._map_crop()
        if crop is None:
            return
        descriptor = polar_descriptor(crop)
        self.descriptor_bytes = len(descriptor)
        keyframe_id = self._allocate_keyframe_id()
        checksum = descriptor_checksum(descriptor)
        message = LocalMapDescriptor()
        message.header.frame_id = self.latest_map.header.frame_id
        # The map epoch identifies the source map revision.  The descriptor
        # timestamp identifies when this crop/keyframe was actually sampled,
        # which remains meaningful when SLAM publishes maps less frequently.
        message.header.stamp = self.get_clock().now().to_msg()
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
        self._update_confirmation_cadence(
            self.own_descriptor_stamps_ns, self._stamp_ns(message))
        self.descriptor_pub.publish(message)
        self.counters['descriptors_published'] += 1
        self._record_diagnostic_event(
            'DESCRIPTOR_PUBLISHED', keyframe_id=keyframe_id,
            map_revision=self.map_revision, descriptor_bytes=len(descriptor))
        self.keyframes[keyframe_id] = (message, crop)
        while len(self.keyframes) > self.max_keyframes:
            self.keyframes.popitem(last=False)
        self._prune_expired_descriptor_state()
        self.last_descriptor_wall = time.monotonic()
        self.pending_own_descriptor_keys.append(keyframe_id)

    def tick(self):
        self._sample_cpu()
        if time.monotonic() - self.last_descriptor_wall >= self.descriptor_period_s:
            self.publish_descriptor()
        # Drain at most one descriptor key per timer tick.  Descriptor
        # callbacks only retain messages and enqueue keys, preventing the
        # single-threaded executor from losing current peer views while
        # descriptor/temporal work is performed.
        if self.pending_descriptor_pair_keys:
            self._compare_peer_descriptors(
                new_peer_key=None, new_own_key=None)
        elif self.pending_peer_descriptor_keys:
            key = self.pending_peer_descriptor_keys.popleft()
            if key in self.peer_descriptors:
                self._compare_peer_descriptors(new_peer_key=key)
        elif self.pending_own_descriptor_keys:
            key = self.pending_own_descriptor_keys.popleft()
            if key in self.keyframes:
                self._compare_peer_descriptors(new_own_key=key)
        self._maybe_finalize_evidence_acquisition()

    def descriptor_callback(self, message):
        if message.source_robot_id != self.peer_robot_id:
            return
        if message.descriptor_version != 1 or not message.descriptor_bytes:
            return
        self.counters['descriptors_received'] += 1
        self._record_diagnostic_event(
            'DESCRIPTOR_RECEIVED', keyframe_id=message.keyframe_id,
            map_epoch=int(message.map_epoch), descriptor_bytes=len(
                message.descriptor_bytes))
        key = message.keyframe_id
        self._update_confirmation_cadence(
            self.peer_descriptor_stamps_ns, self._stamp_ns(message))
        self.peer_descriptors[key] = message
        while len(self.peer_descriptors) > self.max_keyframes:
            self.peer_descriptors.popitem(last=False)
        self._prune_expired_descriptor_state()
        self.pending_peer_descriptor_keys.append(key)

    @staticmethod
    def _stamp_ns(message):
        return (int(message.header.stamp.sec) * 1_000_000_000 +
                int(message.header.stamp.nanosec))

    def _prune_expired_descriptor_state(self):
        """Discard pair caches whose bounded descriptor history expired.

        Descriptor matching and temporal confirmation only use currently
        retained own/peer keyframes.  Keeping match/cache entries for every
        historical Cartesian pair made each later descriptor callback scan an
        ever-growing set, even though the configured history is bounded.
        Physical verification/rejection history is intentionally not touched:
        it is the separate safety record that prevents retrying an old pair.
        """
        live_own = set(self.keyframes)
        live_peer = set(self.peer_descriptors)
        live_pairs = {
            (peer_key, own_key)
            for peer_key in live_peer
            for own_key in live_own
        }
        for name in (
                'matches', 'compared_pairs', 'descriptor_gate_status',
                'temporal_support_cache', 'temporal_gate_rejected_pairs',
                'confirmations'):
            values = getattr(self, name)
            if isinstance(values, dict):
                for pair in tuple(values):
                    if pair not in live_pairs:
                        values.pop(pair, None)
            else:
                values.intersection_update(live_pairs)
        for name in ('descriptor_gate_survivors', 'temporal_gate_survivors'):
            getattr(self, name).intersection_update(live_pairs)

    def _temporal_observations(self):
        """Snapshot valid cached matches once for one descriptor callback."""
        observations = []
        for (other_peer_key, other_own_key), other_match in self.matches.items():
            other_own_entry = self.keyframes.get(other_own_key)
            other_peer_message = self.peer_descriptors.get(other_peer_key)
            if other_own_entry is None or other_peer_message is None:
                continue
            observations.append((
                (other_peer_key, other_own_key),
                self._stamp_ns(other_own_entry[0]),
                self._stamp_ns(other_peer_message),
                other_match.similarity, other_match.margin,
                other_match.known_fraction, other_match.sector_shift))
        return observations

    def _temporal_support(self, peer_key, own_key, match, observations=None):
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
        if observations is None:
            observations = self._temporal_observations()
        return temporal_support_count(
            (peer_key, own_key), own_stamp, peer_stamp,
            match.sector_shift, observations, self.similarity_gate,
            self.margin_gate, 0.12, self.effective_confirmation_window_ns)

    def _temporal_candidate_affected(self, pair_key, changed_pair_keys):
        """Return whether a newly queued descriptor can change this gate."""
        if not changed_pair_keys or pair_key in changed_pair_keys:
            return bool(changed_pair_keys)
        peer_key, own_key = pair_key
        own_entry = self.keyframes.get(own_key)
        peer = self.peer_descriptors.get(peer_key)
        if own_entry is None or peer is None:
            return False
        own_stamp = self._stamp_ns(own_entry[0])
        peer_stamp = self._stamp_ns(peer)
        for changed_peer_key, changed_own_key in changed_pair_keys:
            changed_peer = self.peer_descriptors.get(changed_peer_key)
            changed_own_entry = self.keyframes.get(changed_own_key)
            if changed_peer is None or changed_own_entry is None:
                continue
            if (abs(self._stamp_ns(changed_own_entry[0]) - own_stamp) <=
                    self.effective_confirmation_window_ns and
                    abs(self._stamp_ns(changed_peer) - peer_stamp) <=
                    self.effective_confirmation_window_ns):
                return True
        return False

    def _update_temporal_support_cache(self, changed_pair_keys):
        """Update temporal support incrementally for newly computed pairs.

        A new descriptor can add evidence to existing anchors, but it cannot
        change the relationship between two already cached pairs.  The old
        implementation re-ran every affected anchor against the complete
        Cartesian observation list, which became quadratic as descriptor
        history accumulated.  Compute the new anchors exactly once, then add
        only the newly arrived valid observations to existing cached counts.
        This preserves the same timestamp, quality, and circular-sector gates.
        """
        changed = []
        changed_keys = set(changed_pair_keys)
        for pair_key in changed_keys:
            match = self.matches.get(pair_key)
            if match is None or self.descriptor_gate_status.get(pair_key) is not None:
                continue
            peer_key, own_key = pair_key
            own_entry = self.keyframes.get(own_key)
            peer = self.peer_descriptors.get(peer_key)
            if own_entry is None or peer is None:
                continue
            changed.append((
                pair_key,
                self._stamp_ns(own_entry[0]),
                self._stamp_ns(peer),
                int(match.sector_shift),
            ))
        if not changed:
            return

        observations = self._temporal_observations()
        for pair_key, own_stamp, peer_stamp, sector_shift in changed:
            match = self.matches[pair_key]
            self.temporal_support_cache[pair_key] = temporal_support_count(
                pair_key, own_stamp, peer_stamp, sector_shift, observations,
                self.similarity_gate, self.margin_gate, 0.12,
                self.effective_confirmation_window_ns)

        existing = []
        for pair_key in self.temporal_support_cache:
            if pair_key in changed_keys or pair_key not in self.matches:
                continue
            if self.descriptor_gate_status.get(pair_key) is not None:
                continue
            peer_key, own_key = pair_key
            own_entry = self.keyframes.get(own_key)
            peer = self.peer_descriptors.get(peer_key)
            if own_entry is None or peer is None:
                continue
            existing.append((
                pair_key,
                self._stamp_ns(own_entry[0]),
                self._stamp_ns(peer),
                int(self.matches[pair_key].sector_shift),
            ))
        if not existing:
            return
        anchor_own = np.asarray([item[1] for item in existing], dtype=np.int64)
        anchor_peer = np.asarray([item[2] for item in existing], dtype=np.int64)
        anchor_shift = np.asarray([item[3] for item in existing], dtype=np.int16)
        new_own = np.asarray([item[1] for item in changed], dtype=np.int64)
        new_peer = np.asarray([item[2] for item in changed], dtype=np.int64)
        new_shift = np.asarray([item[3] for item in changed], dtype=np.int16)
        within_time = (
            (np.abs(anchor_own[:, None] - new_own[None, :]) <=
             self.effective_confirmation_window_ns) &
            (np.abs(anchor_peer[:, None] - new_peer[None, :]) <=
             self.effective_confirmation_window_ns))
        shift_delta = np.abs(anchor_shift[:, None] - new_shift[None, :])
        shift_delta = np.minimum(shift_delta, 24 - shift_delta)
        additions = np.sum(within_time & (shift_delta <= 2), axis=1)
        for item, addition in zip(existing, additions):
            self.temporal_support_cache[item[0]] += int(addition)

    def _compare_peer_descriptors(self, new_peer_key=None, new_own_key=None):
        if not self.keyframes or not self.peer_descriptors:
            return
        if self.robot_id > self.peer_robot_id or self.batch_proposal_published:
            return
        if new_peer_key is not None or new_own_key is not None:
            peer_items = list(self.peer_descriptors.items())
            own_items = list(self.keyframes.items())
            if new_peer_key is not None:
                peer_items = [(new_peer_key,
                               self.peer_descriptors[new_peer_key])]
            if new_own_key is not None:
                own_items = [(new_own_key,
                              self.keyframes[new_own_key])]
            new_pairs = [
                (peer_key, own_key, peer, own)
                for peer_key, peer in peer_items
                for own_key, own in own_items]
            for peer_key, own_key, _, _ in new_pairs:
                pair_key = (peer_key, own_key)
                if (pair_key not in self.compared_pairs and
                        pair_key not in self.pending_descriptor_pair_key_set):
                    self.pending_descriptor_pair_keys.append(pair_key)
                    self.pending_descriptor_pair_key_set.add(pair_key)

        uncomputed_pairs = []
        while (self.pending_descriptor_pair_keys and
               len(uncomputed_pairs) < self.descriptor_pair_budget_per_tick):
            pair_key = self.pending_descriptor_pair_keys.popleft()
            self.pending_descriptor_pair_key_set.discard(pair_key)
            if pair_key in self.compared_pairs:
                continue
            peer_key, own_key = pair_key
            peer = self.peer_descriptors.get(peer_key)
            own_entry = self.keyframes.get(own_key)
            if peer is None or own_entry is None:
                continue
            uncomputed_pairs.append((peer_key, own_key, peer, own_entry))
        changed_pair_keys = {
            (peer_key, own_key)
            for peer_key, own_key, _, _ in uncomputed_pairs}
        for peer_key, own_key, _, _ in uncomputed_pairs:
            self.compared_pairs.add((peer_key, own_key))
        if uncomputed_pairs:
            first_descriptors = [
                bytes(own_entry[0].descriptor_bytes)
                for _, _, _, own_entry in uncomputed_pairs]
            second_descriptors = [
                bytes(peer.descriptor_bytes)
                for _, _, peer, _ in uncomputed_pairs]
            try:
                matches = compare_descriptor_pairs(
                    first_descriptors, second_descriptors,
                    int(uncomputed_pairs[0][3][0].ring_count),
                    int(uncomputed_pairs[0][3][0].sector_count))
            except ValueError:
                # Preserve the historical per-pair rejection behavior for a
                # malformed descriptor without affecting valid pairs in the
                # same bounded callback batch.
                matches = []
                for _, _, peer, own_entry in uncomputed_pairs:
                    own = own_entry[0]
                    try:
                        matches.append(compare_descriptors(
                            bytes(own.descriptor_bytes),
                            bytes(peer.descriptor_bytes),
                            int(own.ring_count), int(own.sector_count)))
                    except ValueError:
                        matches.append(None)
            for (peer_key, own_key, peer, own_entry), match in zip(
                    uncomputed_pairs, matches):
                if match is None:
                    continue
                self.counters['candidate_comparisons'] += 1
                self.matches[(peer_key, own_key)] = match
                self.best_similarity = max(
                    self.best_similarity, match.similarity)
                self.best_margin = max(self.best_margin, match.margin)
                self.best_known_fraction = max(
                    self.best_known_fraction, match.known_fraction)
                rejection_reason = None
                if match.similarity < self.similarity_gate:
                    rejection_reason = 'SIMILARITY_BELOW_GATE'
                elif match.margin < self.margin_gate:
                    rejection_reason = 'MARGIN_BELOW_GATE'
                elif match.known_fraction < 0.12:
                    rejection_reason = 'KNOWN_FRACTION_BELOW_GATE'
                self.descriptor_gate_status[
                    (peer_key, own_key)] = rejection_reason
                if rejection_reason is not None:
                    self.counters['cheap_rejections'] += 1
                    self.gate_rejection_counts[rejection_reason] += 1
                    continue
                self.counters['cheap_candidates'] += 1
                self.descriptor_gate_survivors.add((peer_key, own_key))
                self._record_diagnostic_event(
                    'DESCRIPTOR_GATE_SURVIVED', peer_key=peer_key,
                    own_key=own_key, similarity=float(match.similarity),
                    margin=float(match.margin),
                    known_fraction=float(match.known_fraction))

        eligible = []
        self._update_temporal_support_cache(changed_pair_keys)
        for (peer_key, own_key), match in list(self.matches.items()):
            rejection_reason = self.descriptor_gate_status.get(
                (peer_key, own_key))
            if rejection_reason is not None:
                continue
            peer = self.peer_descriptors.get(peer_key)
            own_entry = self.keyframes.get(own_key)
            if peer is None or own_entry is None:
                continue
            own = own_entry[0]
            pair_key = (peer_key, own_key)
            support_count = self.temporal_support_cache.get(pair_key)
            if support_count is None:
                continue
            confirmations = self.confirmations.setdefault(
                pair_key, set())
            confirmations.add((own_key, peer_key))
            if pair_key in changed_pair_keys:
                self.counters['temporal_support_evaluations'] += 1
            self.counters['temporal_support_max'] = max(
                self.counters['temporal_support_max'], support_count)
            if support_count < self.minimum_confirmations:
                if pair_key not in self.temporal_gate_rejected_pairs:
                    self.temporal_gate_rejected_pairs.add(pair_key)
                    self.counters['temporal_gate_rejections'] += 1
                    self.temporal_gate_rejection_counts[
                        'INSUFFICIENT_CONFIRMATIONS'] += 1
                continue
            self.temporal_gate_survivors.add((peer_key, own_key))
            if self.first_candidate_wall is None:
                self.first_candidate_wall = time.monotonic()
            eligible.append((
                -float(match.similarity), peer_key, own_key, peer, own))
        if not eligible:
            return
        eligible.sort(key=lambda value: (
            0 if (value[2], value[1]) in self.evidence_pairs else 1,
            value[0], value[1], value[2]))
        own_crops = {key: entry[1] for key, entry in self.keyframes.items()}
        physical_eligible = deduplicate_physical_candidates(
            eligible, own_crops)
        self.counters['physical_candidate_duplicates_suppressed'] += (
            len(eligible) - len(physical_eligible))
        for candidate in physical_eligible:
            physical_key = self._candidate_physical_key(candidate)
            if physical_key in self._diagnosed_physical_candidates:
                continue
            self._diagnosed_physical_candidates.add(physical_key)
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_OBSERVED',
                candidate=self._candidate_diagnostic(candidate, compact=True))
        added, duplicates = accumulate_physical_candidates(
            self.pending_candidate_pairs, physical_eligible, own_crops)
        for identity, candidate in added:
            self.counters['pending_candidate_additions'] += 1
            self._write_physical_evidence_diagnostic(
                'PENDING_CANDIDATE_ADDED',
                identity=list(identity), candidate=self._candidate_diagnostic(
                    candidate, status='PENDING', compact=True))
        for identity, candidate in duplicates:
            self.counters['physical_candidate_duplicates_suppressed'] += 1
            if identity in self._diagnosed_duplicate_physical_candidates:
                continue
            self._diagnosed_duplicate_physical_candidates.add(identity)
            self._write_physical_evidence_diagnostic(
                'PENDING_CANDIDATE_DUPLICATE_SUPPRESSED',
                identity=list(identity), duplicate_record='first_observation',
                candidate=self._candidate_diagnostic(
                    candidate, status='DUPLICATE',
                    reason='IDENTICAL_PHYSICAL_EVIDENCE', compact=True))
        for identity, candidate in list(self.pending_candidate_pairs.items()):
            if len(candidate) == 5:
                _, peer_key, own_key, _, _ = candidate
            else:
                peer_key, own_key, _, _ = candidate
            if peer_key in self.peer_descriptors and own_key in self.keyframes:
                continue
            self.pending_candidate_pairs.pop(identity, None)
            self.counters['pending_candidate_removals'] += 1
            self._write_physical_evidence_diagnostic(
                'PENDING_CANDIDATE_REMOVED', identity=list(identity),
                candidate=self._candidate_diagnostic(
                    candidate, status='REMOVED', reason='KEYFRAME_EXPIRED',
                    compact=True))
        pending_candidates = list(self.pending_candidate_pairs.values())
        pending_candidates.sort(key=lambda value: (
            0 if (value[2], value[1]) in self.evidence_pairs else 1,
            value[0], value[1], value[2]))
        selected = []
        own_centres = []
        peer_centres = []
        # Requests are serialized at publication time but their responses can
        # be delayed.  Treat already-in-flight physical views as part of the
        # current acquisition set so a newly observed candidate cannot be
        # selected next to a view whose registration is still pending.
        inflight_own_centres = []
        inflight_peer_centres = []
        for inflight in self.request_candidate_by_request_key.values():
            inflight_peer_key, inflight_own_key, inflight_peer, _ = (
                self._candidate_fields(inflight))
            inflight_entry = self.keyframes.get(inflight_own_key)
            if inflight_entry is None:
                continue
            inflight_crop = inflight_entry[1]
            inflight_own_centres.append(np.array([
                inflight_crop.origin_x +
                inflight_crop.values.shape[1] * inflight_crop.resolution / 2.0,
                inflight_crop.origin_y +
                inflight_crop.values.shape[0] * inflight_crop.resolution / 2.0]))
            inflight_peer_centres.append(np.array([
                float(inflight_peer.crop_origin_x),
                float(inflight_peer.crop_origin_y)]))
        for _, peer_key, own_key, peer, own in pending_candidates:
            if any(item[1] == peer_key or item[2] == own_key
                   for item in selected):
                continue
            own_crop = self.keyframes[own_key][1]
            own_centre = np.array([
                own_crop.origin_x + own_crop.values.shape[1] * own_crop.resolution / 2.0,
                own_crop.origin_y + own_crop.values.shape[0] * own_crop.resolution / 2.0])
            peer_centre = np.array([float(peer.crop_origin_x),
                                    float(peer.crop_origin_y)])
            if not candidate_views_are_spatially_separated(
                    own_centre, peer_centre,
                    own_centres + inflight_own_centres,
                    peer_centres + inflight_peer_centres):
                continue
            selected.append((peer_key, own_key, peer, own))
            own_centres.append(own_centre)
            peer_centres.append(peer_centre)
            if len(selected) >= self.max_evidence_constraints:
                break
        self.active_candidate_pairs = selected
        self.counters['candidate_selections'] += 1
        self._record_diagnostic_event(
            'CANDIDATE_SELECTION', selected_count=len(selected),
            candidate_count=len(pending_candidates),
            minimum_constraints=self.min_consistent_constraints,
            selected_pairs=[{'peer_key': item[0], 'own_key': item[1]}
                            for item in selected])
        selected_identity = []
        for candidate in selected:
            selected_identity.append(self._candidate_diagnostic_reference(
                candidate, status='SELECTED'))
        centres = []
        for candidate in selected:
            own_crop = self.keyframes[candidate[1]][1]
            centres.append(self._crop_geometry(
                own_crop, candidate[1], self.keyframes[candidate[1]][0].map_epoch,
                self.keyframes[candidate[1]][0].checksum)['center'])
        baseline = 0.0
        if len(centres) > 1:
            baseline = float(np.max(np.linalg.norm(
                np.asarray(centres)[:, None, :] - np.asarray(centres)[None, :, :],
                axis=2)))
        selected_pair_ids = {(candidate[0], candidate[1])
                             for candidate in selected}
        self._write_physical_evidence_diagnostic(
            'SELECTION_ATTEMPT',
            candidate_pool=[self._candidate_diagnostic_reference(
                candidate, status='PENDING')
                for candidate in pending_candidates],
            selected_candidates=selected_identity,
            pending_candidates=[self._candidate_diagnostic_reference(
                candidate, status='PENDING')
                for candidate in pending_candidates
                if (candidate[1], candidate[2]) not in selected_pair_ids],
            measured_spatial_baseline_m=baseline,
            minimum_spatial_baseline_m=0.75,
            selected_count=len(selected),
            candidate_pool_count=len(pending_candidates),
            state_transition='SELECTED' if selected else 'PENDING')

        batch_ready = crop_batch_is_ready(
            len(selected), self.min_consistent_constraints)
        if not batch_ready:
            self.counters['candidate_selection_deferrals'] += 1
            self._record_diagnostic_event(
                'CANDIDATE_SELECTION_DEFERRED',
                reason='INSUFFICIENT_SPATIAL_CONSTRAINTS',
                selected_count=len(selected),
                minimum_constraints=self.min_consistent_constraints)
        # A partial selection is intentionally requestable.  It remains
        # pending, while later descriptor callbacks can add a physically
        # distinct candidate to the same bounded selection.
        self.negotiation_started = bool(selected)
        # Verification is deliberately serialized.  A crop that is merely a
        # descriptor survivor must not consume one of the consensus slots
        # before its unchanged geometric quality gate has passed.
        if selected:
            self._begin_evidence_acquisition()
            # A batch can open before the next spatially distinct candidate
            # arrives.  Keep the active batch requestable on subsequent map /
            # descriptor selection callbacks; otherwise the batch remains
            # stuck in WAITING after its initial empty request attempt.
            self._schedule_active_evidence_request()

    def _schedule_active_evidence_request(self):
        """Request newly available evidence without reopening a batch."""
        if (self.evidence_acquisition_started and
                not self.batch_proposal_published):
            self._request_next_candidate_verification()

    @staticmethod
    def _candidate_fields(candidate):
        if len(candidate) == 5:
            _, peer_key, own_key, peer, own = candidate
        else:
            peer_key, own_key, peer, own = candidate
        return peer_key, own_key, peer, own

    def _batch_snapshot(self):
        own = {}
        for key, (descriptor, crop) in self.keyframes.items():
            geometry = self._crop_geometry(
                crop, key, descriptor.map_epoch, descriptor.checksum)
            own[str(key)] = {
                'timestamp_ns': self._stamp_ns(descriptor),
                'center': geometry['center'],
            }
        peer = {
            str(key): {
                'timestamp_ns': self._stamp_ns(descriptor),
                'center': self._descriptor_geometry(descriptor)['center'],
            }
            for key, descriptor in self.peer_descriptors.items()}
        return own, peer

    def _candidate_physical_key(self, candidate):
        peer_key, own_key, peer, own = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return None
        own_crop = own_entry[1]
        return (
            physical_crop_identity(
                own_crop, int(own_entry[0].map_epoch),
                int(own_entry[0].checksum)),
            physical_crop_identity(
                GridCrop(
                    values=np.empty((int(peer.crop_height), int(peer.crop_width)),
                                   dtype=np.int16),
                    resolution=float(peer.resolution),
                    origin_x=float(peer.crop_origin_x),
                    origin_y=float(peer.crop_origin_y)),
                int(peer.map_epoch), int(peer.checksum)))

    def _candidate_physical_geometry_key(self, candidate):
        """Return the revision-independent geometry of one candidate pair."""
        own_crops = {key: value[1] for key, value in self.keyframes.items()}
        return physical_candidate_geometry_identity(candidate, own_crops)

    def _candidate_is_novel_for_reentry(self, candidate):
        peer_key, own_key, peer, own = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return False
        own_geometry = self._crop_geometry(
            own_entry[1], own_key, own_entry[0].map_epoch,
            own_entry[0].checksum)
        peer_geometry = self._descriptor_geometry(peer)
        physical_key = self._candidate_physical_key(candidate)
        return self.verification_batches.is_novel(
            own_key, peer_key,
            self._stamp_ns(own_entry[0]), self._stamp_ns(peer),
            own_geometry['center'], peer_geometry['center'],
            attempted_pairs=self.candidate_verification_attempted,
            rejected_physical=(
                physical_key in self.rejected_physical_evidence_keys or
                self._candidate_physical_geometry_key(candidate) in
                self.rejected_physical_geometry_keys))

    def _queue_crop_request(self, peer_key, own_key, peer, own,
                            batch_id, correlation_id):
        request_key = (peer_key, int(peer.checksum))
        if request_key in self.pending_requests:
            self.counters['crop_request_duplicates_suppressed'] += 1
            return False
        self.pending_requests.add(request_key)
        self.counters['crop_requests_queued'] += 1
        self.request_own_by_peer_key[peer_key] = own_key
        self.request_own_by_request_key[request_key] = own_key
        self.request_candidate_by_request_key[request_key] = (
            peer_key, own_key, peer, own)
        self.request_metadata_by_request_key[request_key] = {
            'acquisition_batch_id': int(batch_id),
            'candidate_correlation_id': str(correlation_id),
            'verification_attempt_id': str(correlation_id),
            'own_keyframe_id': str(own_key),
            'peer_keyframe_id': str(peer_key),
            'request_timestamp_ns': int(self._stamp_ns(own)),
            'own_keyframe_creation_timestamp_ns': int(self._stamp_ns(own)),
            'peer_keyframe_creation_timestamp_ns': int(self._stamp_ns(peer)),
        }
        request = LocalMapCropRequest()
        request.header = own.header
        request.requester_robot_id = self.robot_id
        request.source_robot_id = self.peer_robot_id
        request.keyframe_id = peer_key
        request.descriptor_checksum = int(peer.checksum)
        self.request_pub.publish(request)
        self.counters['crop_requests_sent'] += 1
        self._write_physical_evidence_diagnostic(
            'CROP_REQUEST_SENT',
            request_key=[peer_key, int(peer.checksum)],
            **self.request_metadata_by_request_key[request_key],
            candidate=self._candidate_diagnostic(
                (peer_key, own_key, peer, own), status='REQUESTED',
                compact=True))
        self._record_diagnostic_event(
            'CROP_REQUEST_PUBLISHED', peer_key=peer_key,
            own_key=own_key, descriptor_checksum=int(peer.checksum),
            peer_epoch=int(peer.map_epoch),
            request_header_stamp_ns=self._stamp_ns(request))
        return True

    def _candidate_is_distinct_from_evidence(self, candidate):
        """Apply the existing physical-spacing rule to a next candidate."""
        peer_key, own_key, peer, own = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return False
        own_center = np.asarray(self._crop_geometry(
            own_entry[1], own_key, own_entry[0].map_epoch,
            own_entry[0].checksum)['center'], dtype=np.float64)
        peer_center = np.asarray(
            self._descriptor_geometry(peer)['center'], dtype=np.float64)
        own_centers = []
        peer_centers = []
        for evidence_own_key, evidence_peer_key in self.evidence_pairs:
            evidence_own = self.keyframes.get(evidence_own_key)
            evidence_peer = self.peer_descriptors.get(evidence_peer_key)
            if evidence_own is None or evidence_peer is None:
                continue
            own_centers.append(np.asarray(self._crop_geometry(
                evidence_own[1], evidence_own_key,
                evidence_own[0].map_epoch, evidence_own[0].checksum)[
                    'center'], dtype=np.float64))
            peer_centers.append(np.asarray(
                self._descriptor_geometry(evidence_peer)['center'],
                dtype=np.float64))
        # The production spatial-baseline gate is measured from source/own
        # crop centres.  Do not spend verification attempts pairing one local
        # view with many peer keyframes: a local crop that is near any
        # accepted source view cannot increase that baseline.  This is an
        # acquisition-order filter only; registration and consensus gates
        # remain unchanged.
        if own_centers and any(np.linalg.norm(own_center - prior) < 0.40
                               for prior in own_centers):
            return False
        if peer_centers and all(np.linalg.norm(peer_center - prior) < 0.40
                               for prior in peer_centers):
            return False
        return True

    def _candidate_spatial_novelty_key(self, candidate):
        """Prefer displaced views when choosing the next crop request."""
        peer_key, own_key, peer, _ = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return (0.0, 0.0, 0.0, str(peer_key), str(own_key))
        own_center = np.asarray(self._crop_geometry(
            own_entry[1], own_key, own_entry[0].map_epoch,
            own_entry[0].checksum)['center'], dtype=np.float64)
        peer_center = np.asarray(
            self._descriptor_geometry(peer)['center'], dtype=np.float64)
        reference_own = []
        reference_peer = []
        for evidence_own_key, evidence_peer_key in self.evidence_pairs:
            evidence_own = self.keyframes.get(evidence_own_key)
            evidence_peer = self.peer_descriptors.get(evidence_peer_key)
            if evidence_own is not None:
                reference_own.append(np.asarray(self._crop_geometry(
                    evidence_own[1], evidence_own_key,
                    evidence_own[0].map_epoch,
                    evidence_own[0].checksum)['center'], dtype=np.float64))
            if evidence_peer is not None:
                reference_peer.append(np.asarray(
                    self._descriptor_geometry(evidence_peer)['center'],
                    dtype=np.float64))
        own_distance = min(
            (float(np.linalg.norm(own_center - reference))
             for reference in reference_own), default=0.0)
        peer_distance = min(
            (float(np.linalg.norm(peer_center - reference))
             for reference in reference_peer), default=0.0)
        similarity = float(candidate[0]) if len(candidate) == 5 else 0.0
        # A useful cross-robot evidence view must move on both sides of the
        # candidate pair.  Prioritising own_distance first selected very far
        # local crops whose peer crop was still inside the existing physical
        # baseline, starving the consensus pool of independent views.  Rank
        # by the weaker displacement first; this changes acquisition order
        # only and leaves every geometric/consensus gate unchanged.
        balanced_distance = min(own_distance, peer_distance)
        total_distance = max(own_distance, peer_distance)
        geometry_key = self._candidate_physical_geometry_key(candidate)
        reuse_count = 0
        if geometry_key is not None:
            reuse_count = sum(
                int(self.attempted_physical_view_reuse_counts.get(view, 0))
                for view in geometry_key)
        # If a high-scoring descriptor repeatedly leads to geometric
        # rejection, ranking another pair with the same local or peer crop
        # first can starve genuinely new views.  Prefer never-attempted
        # physical views, then retain the existing balanced spatial novelty
        # ordering.  This is scheduling only: all registration and consensus
        # gates and all exact duplicate/rejection sets are unchanged.
        return (reuse_count, -balanced_distance, -total_distance, similarity,
                str(peer_key), str(own_key))

    def _rank_next_verification_candidate(self):
        candidates = []
        for candidate in self.pending_candidate_pairs.values():
            peer_key, own_key, peer, own = self._candidate_fields(candidate)
            pair_key = (own_key, peer_key)
            request_key = (peer_key, int(peer.checksum))
            if pair_key in self.candidate_verification_attempted:
                continue
            if pair_key in self.evidence_pairs:
                continue
            if request_key in self.pending_requests:
                continue
            physical_key = self._candidate_physical_key(candidate)
            if physical_key in self.rejected_physical_evidence_keys:
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_EVIDENCE_PREVIOUSLY_REJECTED',
                        compact=True),
                    reason='PHYSICAL_EVIDENCE_PREVIOUSLY_REJECTED')
                continue
            geometry_key = self._candidate_physical_geometry_key(candidate)
            if candidate_reuses_accepted_physical_view(
                    candidate, self.evidence_physical_geometry_keys,
                    {key: value[1] for key, value in self.keyframes.items()}):
                self.counters['physical_evidence_duplicates_suppressed'] += 1
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_VIEW_ALREADY_ACCEPTED', compact=True),
                    reason='PHYSICAL_VIEW_ALREADY_ACCEPTED')
                continue
            if geometry_key in self.evidence_physical_geometry_keys:
                self.counters['physical_evidence_duplicates_suppressed'] += 1
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_GEOMETRY_ALREADY_ACCEPTED',
                        compact=True),
                    reason='PHYSICAL_GEOMETRY_ALREADY_ACCEPTED')
                continue
            if geometry_key in self.rejected_physical_geometry_keys:
                self.counters['physical_geometry_rejections_suppressed'] += 1
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_GEOMETRY_PREVIOUSLY_REJECTED',
                        compact=True),
                    reason='PHYSICAL_GEOMETRY_PREVIOUSLY_REJECTED')
                continue
            if not self._candidate_is_distinct_from_evidence(candidate):
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='TOO_CLOSE_TO_ACCEPTED_EVIDENCE', compact=True),
                    reason='TOO_CLOSE_TO_ACCEPTED_EVIDENCE')
                continue
            candidates.append(candidate)
        ranked = bounded_candidate_verification_order(
            candidates, self.candidate_verification_attempted,
            len(candidates))
        ranked.sort(key=self._candidate_spatial_novelty_key)
        return None if not ranked else ranked[0]

    def _request_next_candidate_verification(self):
        if self.batch_proposal_published or not self.evidence_acquisition_started:
            return False
        if self.candidate_verification_batch_attempts >= self.candidate_verification_budget:
            self.counters['candidate_verification_budget_exhausted'] += 1
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_VERIFICATION_BUDGET_EXHAUSTED',
                attempts=self.candidate_verification_attempts,
                batch_attempts=self.candidate_verification_batch_attempts,
                acquisition_batch_id=self.verification_batches.batch_id,
                accepted_count=len(self.evidence_pairs),
                budget=self.candidate_verification_budget)
            self._end_evidence_acquisition('BUDGET_EXHAUSTED')
            return False
        candidate = self._rank_next_verification_candidate()
        if candidate is None:
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_VERIFICATION_WAITING',
                attempts=self.candidate_verification_attempts,
                accepted_count=len(self.evidence_pairs))
            return False
        peer_key, own_key, peer, own = self._candidate_fields(candidate)
        pair_key = (own_key, peer_key)
        geometry_key = self._candidate_physical_geometry_key(candidate)
        if geometry_key is not None:
            for view in geometry_key:
                self.attempted_physical_view_reuse_counts[view] = (
                    int(self.attempted_physical_view_reuse_counts.get(view, 0))
                    + 1)
        self.candidate_verification_attempted.add(pair_key)
        self.candidate_verification_attempts += 1
        self.candidate_verification_batch_attempts += 1
        self.verification_batches.batch_attempts += 1
        self.verification_attempt_sequence += 1
        correlation_id = (
            f'{self.robot_id}-b{self.verification_batches.batch_id:04d}-'
            f'a{self.candidate_verification_batch_attempts:04d}-'
            f'c{self.verification_attempt_sequence:06d}')
        self.counters['candidate_verification_attempts'] = (
            self.candidate_verification_attempts)
        self._write_physical_evidence_diagnostic(
            'CANDIDATE_VERIFICATION_REQUESTED',
            candidate=self._candidate_diagnostic(
                candidate, status='REQUESTED', compact=True),
            attempt=self.candidate_verification_attempts,
            batch_attempt=self.candidate_verification_batch_attempts,
            budget=self.candidate_verification_budget,
            acquisition_batch_id=self.verification_batches.batch_id,
            candidate_correlation_id=correlation_id,
            verification_attempt_id=correlation_id)
        return self._queue_crop_request(
            peer_key, own_key, peer, own,
            self.verification_batches.batch_id, correlation_id)

    def _request_additional_evidence_candidates(self):
        """Request one more candidate; retained as a compatibility wrapper."""
        if not self.evidence_acquisition_started:
            return 0
        return int(self._request_next_candidate_verification())

    def _begin_evidence_acquisition(self):
        if self.evidence_acquisition_started or self.batch_proposal_published:
            return
        own_snapshot, peer_snapshot = self._batch_snapshot()
        now = time.monotonic()
        initial = self.verification_batches.batch_id == 0
        if not initial and not self.verification_batches.waiting_for_novelty:
            return
        if not initial:
            novel = any(
                self._candidate_is_novel_for_reentry(candidate)
                for candidate in self.pending_candidate_pairs.values())
            if not novel:
                self.counters['verification_novelty_deferrals'] += 1
                self._write_physical_evidence_diagnostic(
                    'VERIFICATION_BATCH_WAITING_FOR_NOVELTY',
                    acquisition_batch_id=self.verification_batches.batch_id,
                    pending_candidate_count=len(self.pending_candidate_pairs),
                    reason='NO_MATERIAL_NEW_SPATIAL_EVIDENCE')
                return
        batch_id = self.verification_batches.open(
            now, own_snapshot, peer_snapshot, initial=initial)
        if batch_id is None:
            if self.verification_batches.lifetime_expired:
                self.counters['verification_lifetime_expired'] += 1
            return
        if not initial:
            self.counters['verification_batch_reentries'] += 1
        self.counters['verification_batches_opened'] += 1
        self.candidate_verification_batch_attempts = 0
        self.evidence_acquisition_started = True
        self.evidence_acquisition_deadline_wall = (
            time.monotonic() + self.evidence_acquisition_window_s)
        self._record_diagnostic_event(
            'EVIDENCE_ACQUISITION_STARTED',
            acquisition_batch_id=batch_id,
            constraints_accumulated=len(self.evidence_physical_keys),
            window_s=self.evidence_acquisition_window_s)
        self._write_physical_evidence_diagnostic(
            'VERIFICATION_BATCH_OPENED',
            acquisition_batch_id=batch_id,
            batch_attempts=0,
            constraints_accumulated=len(self.evidence_physical_keys),
            window_s=self.evidence_acquisition_window_s,
            deadline_wall=self.evidence_acquisition_deadline_wall)
        self._request_next_candidate_verification()

    def _end_evidence_acquisition(self, reason):
        if not self.evidence_acquisition_started:
            return
        own_snapshot, peer_snapshot = self._batch_snapshot()
        self.evidence_acquisition_started = False
        self.evidence_acquisition_deadline_wall = None
        self.verification_batches.exhaust(
            time.monotonic(), own_snapshot, peer_snapshot)
        self.counters['verification_batches_exhausted'] += 1
        self._write_physical_evidence_diagnostic(
            'VERIFICATION_BATCH_WAITING',
            acquisition_batch_id=self.verification_batches.batch_id,
            batch_attempts=self.candidate_verification_batch_attempts,
            constraints_accumulated=len(self.evidence_physical_keys),
            reason=str(reason),
            state='WAITING_FOR_NOVEL_EVIDENCE')
        if len(self.evidence_physical_keys) >= self.min_consistent_constraints:
            self._publish_multi_constraint_proposal()

    def _maybe_finalize_evidence_acquisition(self):
        if not self.evidence_acquisition_started:
            return
        if (self.evidence_acquisition_deadline_wall is None or
                time.monotonic() < self.evidence_acquisition_deadline_wall):
            return
        self._end_evidence_acquisition('WINDOW_EXPIRED')
        self._record_diagnostic_event(
            'EVIDENCE_ACQUISITION_TIMEOUT',
            acquisition_batch_id=self.verification_batches.batch_id,
            constraints_accumulated=len(self.evidence_physical_keys))
        self._write_physical_evidence_diagnostic(
            'EVIDENCE_ACQUISITION_TIMEOUT',
            acquisition_batch_id=self.verification_batches.batch_id,
            constraints_accumulated=len(self.evidence_physical_keys),
            state_transition='FINALIZE')

    def request_callback(self, request):
        if request.source_robot_id != self.robot_id:
            return
        self.counters['crop_requests_received'] += 1
        self._record_diagnostic_event(
            'CROP_REQUEST_RECEIVED', keyframe_id=request.keyframe_id,
            descriptor_checksum=int(request.descriptor_checksum),
            request_header_stamp_ns=self._stamp_ns(request))
        stored = self.keyframes.get(request.keyframe_id)
        if stored is None:
            self._record_crop_rejection('KEYFRAME_NOT_RETAINED', request)
            return
        self._write_physical_evidence_diagnostic(
            'CROP_REQUEST_RECEIVED',
            request_key=[request.keyframe_id,
                         int(request.descriptor_checksum)],
            source_descriptor=self._descriptor_geometry(stored[0]),
            requester_robot_id=str(request.requester_robot_id),
            source_robot_id=str(request.source_robot_id),
            status='RECEIVED')
        descriptor, crop = stored
        if request.descriptor_checksum and int(descriptor.checksum) != int(
                request.descriptor_checksum):
            self._record_crop_rejection('CHECKSUM_MISMATCH', request)
            return
        crop_message = LocalMapCrop()
        crop_message.header = descriptor.header
        crop_message.source_robot_id = self.robot_id
        crop_message.keyframe_id = request.keyframe_id
        crop_message.map_epoch = descriptor.map_epoch
        crop_message.descriptor_checksum = descriptor.checksum
        crop_message.occupancy_grid = self._crop_message(crop, descriptor.header)
        self.crop_cells_sent = max(
            self.crop_cells_sent, len(crop_message.occupancy_grid.data))
        self.crop_pub.publish(crop_message)
        self.counters['crops_sent'] += 1
        self._write_physical_evidence_diagnostic(
            'CROP_RESPONSE_SENT',
            keyframe_id=str(request.keyframe_id),
            map_epoch=int(descriptor.map_epoch),
            checksum=int(descriptor.checksum),
            crop=self._crop_geometry(
                crop, request.keyframe_id, descriptor.map_epoch,
                descriptor.checksum),
            status='SENT')
        self.counters['crop_responses_accepted'] += 1
        self._record_diagnostic_event(
            'CROP_RESPONSE_PUBLISHED', keyframe_id=request.keyframe_id,
            map_epoch=int(descriptor.map_epoch),
            descriptor_checksum=int(descriptor.checksum))

    def _record_crop_rejection(self, reason, message, **fields):
        self.counters['crop_response_rejections'] += 1
        self.crop_response_rejection_counts[str(reason)] += 1
        self._write_physical_evidence_diagnostic(
            'CROP_RESPONSE_REJECTED',
            keyframe_id=str(getattr(message, 'keyframe_id', '')),
            map_epoch=int(getattr(message, 'map_epoch', 0)),
            checksum=int(getattr(message, 'descriptor_checksum', 0)),
            reason=str(reason), status='REJECTED', **fields)
        self._record_diagnostic_event(
            'CROP_RESPONSE_REJECTED', reason=str(reason),
            keyframe_id=getattr(message, 'keyframe_id', ''),
            descriptor_checksum=int(getattr(message, 'descriptor_checksum', 0)))

    def _verify_candidate_crop(self, pair_key, candidate, own_crop,
                               received_crop, metadata=None):
        """Run the unchanged single-pair geometric gate before consensus."""
        peer_key, _, _, _ = self._candidate_fields(candidate)
        self.counters['registrations'] += 1
        self.counters['registration_callback_entries'] += 1
        self.registration_callback_depth += 1
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_ENTRY', source='candidate_verification',
            keyframe_id=peer_key, constraint_count=1)
        try:
            result = register_crops(own_crop, received_crop)
        except Exception as exc:
            self.counters['registration_callback_exceptions'] += 1
            self.consensus_gate_rejection_counts['REGISTRATION_EXCEPTION'] += 1
            self._record_diagnostic_event(
                'REGISTRATION_CALLBACK_EXCEPTION',
                source='candidate_verification', keyframe_id=peer_key,
                exception=repr(exc))
            raise
        finally:
            self.registration_callback_depth -= 1
            self.counters['registration_callback_exits'] += 1
        self.candidate_verification_results[pair_key] = result
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_EXIT', source='candidate_verification',
            keyframe_id=peer_key, constraint_count=1,
            accepted=bool(result.accepted), reason=str(result.reason),
            residual_m=float(result.residual_m),
            inlier_ratio=float(result.inlier_ratio),
            projected_error_m=float(result.projected_error_m))
        self._write_physical_evidence_diagnostic(
            'CANDIDATE_VERIFICATION_RESULT',
            **(metadata or {}),
            candidate=self._candidate_diagnostic(
                candidate, status='GEOMETRIC_ACCEPTED' if result.accepted
                else 'GEOMETRIC_REJECTED', reason=str(result.reason),
                compact=True),
            transform=[float(value) for value in result.transform],
            residual_m=float(result.residual_m),
            median_residual_m=float(result.median_residual_m),
            p95_residual_m=float(result.p95_residual_m),
            inlier_ratio=float(result.inlier_ratio),
            occupied_free_agreement=float(result.occupied_free_agreement),
            overlap_fraction=float(result.overlap_fraction),
            translation_uncertainty_m=float(result.translation_uncertainty_m),
            yaw_uncertainty_rad=float(result.yaw_uncertainty_rad),
            condition_number=float(result.condition_number),
            projected_error_m=float(result.projected_error_m),
            accepted_geometric=bool(result.accepted),
            rejection_reason='' if result.accepted else str(result.reason))
        if result.accepted:
            self.counters['candidate_verification_accepted'] += 1
        else:
            self.counters['candidate_verification_rejected'] += 1
        return result

    @staticmethod
    def _crop_message(crop, header):
        message = OccupancyGrid()
        message.header = header
        message.info.resolution = float(crop.resolution)
        message.info.width = int(crop.values.shape[1])
        message.info.height = int(crop.values.shape[0])
        message.info.origin.position.x = float(crop.origin_x)
        message.info.origin.position.y = float(crop.origin_y)
        message.info.origin.orientation.z = math.sin(crop.origin_yaw / 2.0)
        message.info.origin.orientation.w = math.cos(crop.origin_yaw / 2.0)
        message.data = [int(value) for value in crop.values.ravel()]
        return message

    def crop_callback(self, message):
        if message.source_robot_id != self.peer_robot_id:
            return
        self.counters['crops_received'] += 1
        self._record_diagnostic_event(
            'CROP_RECEIVED', keyframe_id=message.keyframe_id,
            map_epoch=int(message.map_epoch),
            descriptor_checksum=int(message.descriptor_checksum))
        self.crop_cells_received = max(
            self.crop_cells_received, len(message.occupancy_grid.data))
        received_crop = self._grid_crop_from_message(message.occupancy_grid)
        expected_peer = self.peer_descriptors.get(message.keyframe_id)
        self._write_physical_evidence_diagnostic(
            'CROP_RESPONSE_RECEIVED',
            keyframe_id=str(message.keyframe_id),
            map_epoch=int(message.map_epoch),
            checksum=int(message.descriptor_checksum),
            crop=self._crop_geometry(
                received_crop, message.keyframe_id, message.map_epoch,
                message.descriptor_checksum),
            peer_descriptor=(None if expected_peer is None else
                             self._descriptor_geometry(expected_peer)),
            status='RECEIVED')
        self.received_peer_crops[message.keyframe_id] = received_crop
        if self.robot_id > self.peer_robot_id:
            proposal = self.peer_proposals.get(message.keyframe_id)
            if proposal is None:
                self._record_crop_rejection('UNMATCHED_TARGET_PROPOSAL', message)
                return
            evidence_sources = list(getattr(
                proposal, 'evidence_source_keyframe_ids', []))
            evidence_targets = list(getattr(
                proposal, 'evidence_target_keyframe_ids', []))
            if not evidence_sources:
                evidence_sources = [proposal.source_keyframe_id]
                evidence_targets = [proposal.target_keyframe_id]
            if len(evidence_sources) != len(evidence_targets):
                self._record_crop_rejection('EVIDENCE_KEY_LENGTH_MISMATCH', message)
                return
            evidence_pairs = []
            for source_key, target_key in zip(evidence_sources, evidence_targets):
                target_entry = self.keyframes.get(target_key)
                source_crop = self.received_peer_crops.get(source_key)
                if target_entry is None or source_crop is None:
                    self._record_crop_rejection('INCOMPLETE_EVIDENCE_SET', message)
                    return
                evidence_pairs.append((source_crop, target_entry[1]))
            result = self._run_registration(
                evidence_pairs, 'target_confirmation', message.keyframe_id)
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
            mutually_consistent = translation_error <= 0.12 and yaw_error <= 0.04
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
        response_request_key = (
            str(message.keyframe_id), int(message.descriptor_checksum))
        request_metadata = self.request_metadata_by_request_key.get(
            response_request_key)
        if response_request_key not in self.pending_requests:
            self.counters['stale_verification_batch_responses'] += 1
            self._record_crop_rejection(
                'STALE_VERIFICATION_BATCH', message,
                metadata=request_metadata,
                response_request_key=list(response_request_key))
            return
        peer_key = message.keyframe_id
        request_key = (peer_key, int(message.descriptor_checksum))
        own_key = self.request_own_by_request_key.get(request_key)
        if own_key is None:
            # Backwards-compatible fallback for confirmation requests, which
            # intentionally carry checksum zero and are keyed by source ID.
            own_key = self.request_own_by_peer_key.get(peer_key)
        if own_key is None or own_key not in self.keyframes:
            self._record_crop_rejection('UNMATCHED_REQUEST_KEY', message)
            return
        expected_peer = self.peer_descriptors.get(peer_key)
        if expected_peer is None:
            self._record_crop_rejection('PEER_DESCRIPTOR_NOT_RETAINED', message)
            return
        if int(message.descriptor_checksum) != int(expected_peer.checksum):
            self._record_crop_rejection('RESPONSE_CHECKSUM_MISMATCH', message)
            return
        if int(message.map_epoch) != int(expected_peer.map_epoch):
            self._record_crop_rejection('RESPONSE_MAP_EPOCH_MISMATCH', message)
            return
        pair_key = (own_key, peer_key)
        physical_key = (
            physical_crop_identity(
                self.keyframes[own_key][1],
                int(self.keyframes[own_key][0].map_epoch),
                int(self.keyframes[own_key][0].checksum)),
            physical_crop_identity(
                received_crop, int(message.map_epoch),
                int(message.descriptor_checksum)))
        if physical_key in self.evidence_physical_keys.values():
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            self._write_physical_evidence_diagnostic(
                'CROP_RESPONSE_DUPLICATE_SUPPRESSED',
                physical_identity=list(physical_key),
                own_keyframe_id=str(own_key),
                peer_keyframe_id=str(peer_key),
                reason='IDENTICAL_PHYSICAL_EVIDENCE')
            self._record_diagnostic_event(
                'CROP_DUPLICATE_PHYSICAL_EVIDENCE_SUPPRESSED',
                own_key=own_key, peer_key=peer_key,
                descriptor_checksum=int(message.descriptor_checksum))
            return
        candidate = self.request_candidate_by_request_key.get(
            request_key, (peer_key, own_key, expected_peer,
                          self.keyframes[own_key][0]))
        candidate_geometry_key = self._candidate_physical_geometry_key(candidate)
        # A response can arrive after another in-flight request has already
        # accepted the same local or peer crop footprint.  Exact pair
        # deduplication is insufficient here: reusing either physical view
        # creates a second, non-independent registration constraint and can
        # poison the unchanged multi-constraint consensus gate.  Apply the
        # existing acquisition-only physical-view rule again at response
        # time, when the accepted-evidence set is authoritative.
        if candidate_reuses_accepted_physical_view(
                candidate, self.evidence_physical_geometry_keys,
                {key: value[1] for key, value in self.keyframes.items()}):
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            self._write_physical_evidence_diagnostic(
                'CROP_RESPONSE_DUPLICATE_SUPPRESSED',
                physical_identity=list(physical_key),
                own_keyframe_id=str(own_key),
                peer_keyframe_id=str(peer_key),
                reason='PHYSICAL_VIEW_ALREADY_ACCEPTED')
            self.pending_requests.discard(request_key)
            self.completed_request_keys.add(request_key)
            self.request_candidate_by_request_key.pop(request_key, None)
            self.request_metadata_by_request_key.pop(request_key, None)
            self._request_next_candidate_verification()
            return
        if candidate_geometry_key in self.evidence_physical_geometry_keys:
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            self._write_physical_evidence_diagnostic(
                'CROP_RESPONSE_DUPLICATE_SUPPRESSED',
                physical_identity=list(physical_key),
                own_keyframe_id=str(own_key),
                peer_keyframe_id=str(peer_key),
                reason='PHYSICAL_GEOMETRY_ALREADY_ACCEPTED')
            self.pending_requests.discard(request_key)
            self.completed_request_keys.add(request_key)
            self.request_candidate_by_request_key.pop(request_key, None)
            self._request_next_candidate_verification()
            return
        self.pending_requests.discard(request_key)
        self.completed_request_keys.add(request_key)
        request_metadata = self.request_metadata_by_request_key.get(
            request_key, {})
        candidate = self.request_candidate_by_request_key.pop(
            request_key, candidate)
        self.counters['crop_responses_accepted'] += 1
        accepted_metadata = dict(request_metadata or {})
        # Keep the request metadata schema, while making the response's
        # canonical identities authoritative without passing duplicate Python
        # keyword arguments.
        accepted_metadata.update({
            'own_keyframe_id': str(own_key),
            'peer_keyframe_id': str(peer_key),
        })
        self._write_physical_evidence_diagnostic(
            'CROP_RESPONSE_ACCEPTED',
            **accepted_metadata,
            physical_identity=list(physical_key),
            own_crop=self._crop_geometry(
                self.keyframes[own_key][1], own_key,
                self.keyframes[own_key][0].map_epoch,
                self.keyframes[own_key][0].checksum),
            peer_crop=self._crop_geometry(
                received_crop, peer_key, message.map_epoch,
                message.descriptor_checksum),
            constraints_accumulated=len(self.evidence_pairs),
            status='ACCEPTED')
        result = self._verify_candidate_crop(
            pair_key, candidate, self.keyframes[own_key][1], received_crop,
            metadata=request_metadata)
        if not result.accepted:
            self.rejected_physical_evidence_keys.add(physical_key)
            self.rejected_physical_geometry_keys.add(
                self._candidate_physical_geometry_key(candidate))
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_REJECTED_BEFORE_CONSENSUS',
                **request_metadata,
                candidate=self._candidate_diagnostic(
                    candidate, status='REJECTED', reason=str(result.reason),
                    compact=True),
                rejection_reason=str(result.reason))
            self._request_next_candidate_verification()
            return
        self.evidence_pairs[pair_key] = (
            self.keyframes[own_key][1], received_crop)
        self.evidence_physical_keys[pair_key] = physical_key
        self.evidence_physical_geometry_keys.add(candidate_geometry_key)
        self.counters['constraints_accumulated'] = len(
            self.evidence_physical_keys)
        self._record_diagnostic_event(
            'CROP_ACCEPTED', own_key=own_key, peer_key=peer_key,
            map_epoch=int(message.map_epoch),
            descriptor_checksum=int(message.descriptor_checksum),
            constraints_accumulated=len(self.evidence_physical_keys))
        if len(self.evidence_physical_keys) < self.min_consistent_constraints:
            self._record_diagnostic_event(
                'EVIDENCE_SET_WAITING',
                constraints_accumulated=len(self.evidence_physical_keys),
                minimum_constraints=self.min_consistent_constraints)
            self._request_next_candidate_verification()
            return
        self.counters['evidence_sets_formed'] += 1
        self._record_diagnostic_event(
            'EVIDENCE_SET_FORMED',
            constraints_accumulated=len(self.evidence_physical_keys))
        evidence_items = list(self.evidence_pairs.items())
        pairs = [evidence for _, evidence in evidence_items]
        cached_results = [
            self.candidate_verification_results.get(pair_key)
            for pair_key, _ in evidence_items]
        consensus = self._run_registration(
            pairs, 'incremental_consensus', peer_key,
            individual_results=cached_results)
        if consensus.accepted:
            self.evidence_acquisition_started = False
            self.evidence_acquisition_deadline_wall = None
            self.verification_batches.mark_completed()
            self._publish_multi_constraint_proposal(result=consensus)
            return
        self._write_physical_evidence_diagnostic(
            'INCREMENTAL_CONSENSUS_REJECTED',
            accepted_constraint_count=len(self.evidence_pairs),
            reason=str(consensus.reason))
        # The current pool remains intact.  Continue with the next ranked,
        # physically distinct candidate until the bounded budget/window ends.
        self._request_next_candidate_verification()

    def _run_registration(self, evidence_pairs, source, keyframe_id='',
                          individual_results=None):
        self.counters['registrations'] += 1
        self.counters['registration_callback_entries'] += 1
        self.registration_callback_depth += 1
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_ENTRY', source=source,
            keyframe_id=keyframe_id, constraint_count=len(evidence_pairs))
        try:
            result = register_crop_set(
                evidence_pairs,
                target_map_radius_m=self.target_map_radius_m,
                min_consistent_constraints=self.min_consistent_constraints,
                max_projected_registration_error_m=(
                    self.max_projected_registration_error_m),
                individual_results=individual_results)
        except Exception as exc:
            self.counters['registration_callback_exceptions'] += 1
            self.consensus_gate_rejection_counts['REGISTRATION_EXCEPTION'] += 1
            self._record_diagnostic_event(
                'REGISTRATION_CALLBACK_EXCEPTION', source=source,
                keyframe_id=keyframe_id, exception=repr(exc))
            raise
        finally:
            self.registration_callback_depth -= 1
            self.counters['registration_callback_exits'] += 1
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_EXIT', source=source,
            keyframe_id=keyframe_id, constraint_count=len(evidence_pairs),
            accepted=bool(result.accepted), reason=str(result.reason),
            residual_m=float(result.residual_m),
            consistent_constraint_count=int(result.consistent_constraint_count))
        for diagnostic in getattr(result, 'consensus_diagnostics', ()):
            event_name = {
                'constraint': 'CONSENSUS_CONSTRAINT_DIAGNOSTIC',
                'pairwise_comparison': 'CONSENSUS_PAIRWISE_COMPARISON',
                'subset_comparison': 'CONSENSUS_SUBSET_COMPARISON',
            }.get(diagnostic.get('kind'), 'CONSENSUS_DIAGNOSTIC')
            fields = {key: value for key, value in diagnostic.items()
                      if key != 'kind'}
            self._record_diagnostic_event(
                event_name, source=source, keyframe_id=keyframe_id,
                **fields)
            self._write_consensus_diagnostic(
                event_name, source=source, keyframe_id=keyframe_id,
                **fields)
        self._write_consensus_diagnostic(
            'REGISTRATION_RESULT', source=source, keyframe_id=keyframe_id,
            accepted=bool(result.accepted), reason=str(result.reason),
            transform=[float(value) for value in result.transform],
            constraint_count=int(result.constraint_count),
            consistent_constraint_count=int(
                result.consistent_constraint_count),
            residual_m=float(result.residual_m),
            inlier_ratio=float(result.inlier_ratio),
            translation_uncertainty_m=float(
                result.translation_uncertainty_m),
            yaw_uncertainty_rad=float(result.yaw_uncertainty_rad),
            condition_number=float(result.condition_number),
            projected_error_m=float(result.projected_error_m))
        if not result.accepted:
            self.counters['multi_constraint_rejections'] += 1
            self.consensus_gate_rejection_counts[str(result.reason)] += 1
        return result

    def _proposal_candidate_pool(self, result=None):
        """Return the candidate view of the evidence used for a proposal.

        When an incremental consensus result is supplied, its evidence has
        already been removed from ``pending_candidate_pairs`` by the crop
        verification path.  Reconstruct that bounded pool from the canonical
        hashable evidence-pair identities instead of silently losing the
        just-accepted constraints at proposal publication time.
        """
        if result is not None:
            pool = []
            for own_key, peer_key in self.evidence_pairs:
                own_entry = self.keyframes.get(own_key)
                peer_descriptor = self.peer_descriptors.get(peer_key)
                if own_entry is None or peer_descriptor is None:
                    continue
                pool.append((
                    peer_key, own_key, peer_descriptor, own_entry[0]))
            return pool
        candidate_pool = list(self.pending_candidate_pairs.values())
        if not candidate_pool:
            candidate_pool = list(self.active_candidate_pairs)
        return candidate_pool

    def _publish_multi_constraint_proposal(self, result=None):
        if self.batch_proposal_published or self.robot_id > self.peer_robot_id:
            return
        if self.evidence_acquisition_started and result is None:
            return
        pairs = []
        selected_pairs = []
        physical_keys = set()
        # Evidence can arrive over several selection callbacks.  The active
        # selection is only the latest snapshot; use the bounded pending pool
        # so accepted earlier pairs are reconsidered together.  A completed
        # incremental consensus must instead use the evidence pairs that were
        # just verified, because those candidates have already left pending.
        candidate_pool = self._proposal_candidate_pool(result)
        for candidate in evidence_candidates_for_pool(
                candidate_pool, self.evidence_pairs):
            pair_key = (candidate[1], candidate[0])
            evidence = self.evidence_pairs.get(pair_key)
            if evidence is None:
                continue
            physical_key = self.evidence_physical_keys.get(pair_key)
            if physical_key is None:
                physical_key = (
                    physical_crop_identity(
                        evidence[0], int(candidate[3].map_epoch),
                        int(candidate[3].checksum)),
                    physical_crop_identity(
                        evidence[1], int(candidate[2].map_epoch),
                        int(candidate[2].checksum)))
            if physical_key in physical_keys:
                self.counters['physical_evidence_duplicates_suppressed'] += 1
                continue
            physical_keys.add(physical_key)
            selected_pairs.append(candidate)
            pairs.append(evidence)
            if len(pairs) >= self.candidate_verification_budget:
                break
        if len(pairs) < self.min_consistent_constraints:
            return
        if not evidence_batch_is_spatially_diverse(
                [pair[0] for pair in pairs], min_spatial_baseline_m=0.75):
            self.counters['spatial_diversity_deferrals'] += 1
            self._record_diagnostic_event(
                'EVIDENCE_SET_DEFERRED',
                reason='INSUFFICIENT_SPATIAL_BASELINE',
                constraints_accumulated=len(pairs),
                minimum_spatial_baseline_m=0.75)
            return
        for index, (candidate, evidence) in enumerate(
                zip(selected_pairs, pairs)):
            peer_key, own_key, peer_descriptor, own_descriptor = candidate
            own_crop = self.keyframes[own_key][1]
            peer_crop = evidence[1]
            match = self.matches.get((peer_key, own_key))
            constraint_fields = {
                'index': index,
                'own_keyframe_id': own_key, 'peer_keyframe_id': peer_key,
                'own_timestamp_ns': self._stamp_ns(own_descriptor),
                'peer_timestamp_ns': self._stamp_ns(peer_descriptor),
                'own_map_epoch': int(own_descriptor.map_epoch),
                'peer_map_epoch': int(peer_descriptor.map_epoch),
                'own_descriptor_checksum': int(own_descriptor.checksum),
                'peer_descriptor_checksum': int(peer_descriptor.checksum),
                'descriptor_similarity': (
                    0.0 if match is None else float(match.similarity)),
                'descriptor_margin': (
                    0.0 if match is None else float(match.margin)),
                'own_crop_origin': [float(own_crop.origin_x),
                                 float(own_crop.origin_y),
                                 float(own_crop.origin_yaw)],
                'peer_crop_origin': [float(peer_crop.origin_x),
                                  float(peer_crop.origin_y),
                                  float(peer_crop.origin_yaw)],
                'own_crop_center': [float(value) for value in (
                    own_crop.origin_x + 0.5 * own_crop.values.shape[1] *
                    own_crop.resolution,
                    own_crop.origin_y + 0.5 * own_crop.values.shape[0] *
                    own_crop.resolution)],
                'peer_crop_center': [float(value) for value in (
                    peer_crop.origin_x + 0.5 * peer_crop.values.shape[1] *
                    peer_crop.resolution,
                    peer_crop.origin_y + 0.5 * peer_crop.values.shape[0] *
                    peer_crop.resolution)]
            }
            self._record_diagnostic_event(
                'CONSENSUS_INPUT_CONSTRAINT', **constraint_fields)
            self._write_consensus_diagnostic(
                'CONSENSUS_INPUT_CONSTRAINT', **constraint_fields)
        self.counters['multi_constraint_attempts'] += 1
        if result is None:
            result = self._run_registration(
                pairs, 'initiator_batch', selected_pairs[0][0])
        own_key = selected_pairs[0][1]
        peer_key = selected_pairs[0][0]
        own_descriptor = self.keyframes[own_key][0]
        peer_descriptor = self.peer_descriptors[peer_key]
        self.pending_proposals[(own_key, peer_key)] = result
        source_ids = [pair[1] for pair in selected_pairs]
        target_ids = [pair[0] for pair in selected_pairs]
        self._publish_local_evidence_crops(source_ids)
        proposal = self._hypothesis_message(
            own_descriptor, peer_descriptor, result,
            status='PROPOSED' if result.accepted else 'REJECTED',
            accepted=False,
            rejection_reason='' if result.accepted else result.reason,
            evidence_source_keyframe_ids=source_ids,
            evidence_target_keyframe_ids=target_ids)
        self.hypothesis_pub.publish(proposal)
        self.counters['proposals_published'] += 1
        self.batch_proposal_published = bool(result.accepted)
        if result.accepted:
            self.verification_batches.mark_completed()
        else:
            self.counters['rejected_hypotheses'] += 1
            self.negotiation_started = False
            self._write_physical_evidence_diagnostic(
                'VERIFICATION_BATCH_WAITING',
                acquisition_batch_id=self.verification_batches.batch_id,
                constraints_accumulated=len(self.evidence_physical_keys),
                reason='CONSENSUS_REJECTED',
                state='WAITING_FOR_NOVEL_EVIDENCE')

    def _publish_local_evidence_crops(self, keyframe_ids):
        """Make the initiator's bounded evidence available for peer verification."""
        for keyframe_id in keyframe_ids:
            stored = self.keyframes.get(keyframe_id)
            if stored is None:
                continue
            descriptor, crop = stored
            message = LocalMapCrop()
            message.header = descriptor.header
            message.source_robot_id = self.robot_id
            message.keyframe_id = keyframe_id
            message.map_epoch = descriptor.map_epoch
            message.descriptor_checksum = descriptor.checksum
            message.occupancy_grid = self._crop_message(crop, descriptor.header)
            self.crop_pub.publish(message)
            self.counters['crops_sent'] += 1
            self._write_physical_evidence_diagnostic(
                'CROP_RESPONSE_SENT',
                keyframe_id=str(keyframe_id),
                map_epoch=int(descriptor.map_epoch),
                checksum=int(descriptor.checksum),
                crop=self._crop_geometry(
                    crop, keyframe_id, descriptor.map_epoch,
                    descriptor.checksum),
                status='SENT')

    @staticmethod
    def _grid_crop_from_message(message):
        values = np.asarray(message.data, dtype=np.int16).reshape(
            (message.info.height, message.info.width))
        return GridCrop(
            values=values, resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            origin_yaw=UnknownPoseFrontend._yaw(message.info.origin.orientation))

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

    def _hypothesis_message(
            self, own, peer, result, status, accepted, rejection_reason,
            evidence_source_keyframe_ids=None,
            evidence_target_keyframe_ids=None):
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
        descriptor_confidence = (
            0.30 * message.descriptor_similarity +
            0.15 * min(1.0, message.descriptor_margin / 0.10) +
            0.20 * message.geometric_inlier_ratio +
            0.15 * message.occupied_free_agreement +
            0.10 * message.overlap_fraction +
            0.10 * message.temporal_consistency)
        geometric_confidence = float(result.final_confidence)
        message.final_confidence = float(max(0.0, min(1.0,
            0.45 * descriptor_confidence +
            0.55 * geometric_confidence
            if result.constraint_count > 1 else descriptor_confidence)))
        message.status = status
        message.rejection_reason = rejection_reason
        message.accepted = bool(accepted)
        source_ids = list(evidence_source_keyframe_ids or [own.keyframe_id])
        target_ids = list(evidence_target_keyframe_ids or [peer.keyframe_id])
        evidence = '|'.join(f'{source}:{target}' for source, target in zip(
            source_ids, target_ids))
        message.evidence_set_hash = hashlib.sha256(
            evidence.encode('utf-8')).hexdigest()[:16]
        message.evidence_source_keyframe_ids = source_ids
        message.evidence_target_keyframe_ids = target_ids
        message.constraint_count = int(result.constraint_count)
        message.consistent_constraint_count = int(
            result.consistent_constraint_count)
        message.spatial_baseline_m = float(result.spatial_baseline_m)
        message.angular_spread_rad = float(result.angular_spread_rad)
        message.median_registration_residual_m = float(
            result.median_residual_m)
        message.p95_registration_residual_m = float(result.p95_residual_m)
        message.projected_error_m = float(result.projected_error_m)
        message.translation_uncertainty_m = float(
            result.translation_uncertainty_m)
        message.yaw_uncertainty_rad = float(result.yaw_uncertainty_rad)
        message.condition_number = float(result.condition_number)
        return message

    def hypothesis_callback(self, message):
        self._record_diagnostic_event(
            'HYPOTHESIS_RECEIVED',
            source_robot_id=str(message.source_robot_id),
            target_robot_id=str(message.target_robot_id),
            source_keyframe_id=str(message.source_keyframe_id),
            target_keyframe_id=str(message.target_keyframe_id),
            status=str(message.status),
            accepted=bool(message.accepted),
            final_confidence=float(message.final_confidence),
            pending_target_proposal=bool(self.pending_target_proposal),
            already_accepted=bool(self.accepted is not None))
        if {message.source_robot_id, message.target_robot_id} != {
                self.robot_id, self.peer_robot_id}:
            self._record_diagnostic_event(
                'HYPOTHESIS_IGNORED_SCOPE',
                source_robot_id=str(message.source_robot_id),
                target_robot_id=str(message.target_robot_id))
            return
        if self.accepted is not None:
            self._record_diagnostic_event(
                'HYPOTHESIS_IGNORED_ALREADY_ACCEPTED',
                status=str(message.status))
            return
        if message.status == 'PROPOSED' and self.robot_id == message.target_robot_id:
            if self.pending_target_proposal:
                self._record_diagnostic_event(
                    'HYPOTHESIS_IGNORED_PENDING_TARGET',
                    source_keyframe_id=str(message.source_keyframe_id),
                    target_keyframe_id=str(message.target_keyframe_id))
                return
            self.pending_target_proposal = True
            self.peer_proposals[message.source_keyframe_id] = message
            self._record_diagnostic_event(
                'HYPOTHESIS_PROPOSAL_ACCEPTED_FOR_CONFIRMATION',
                source_keyframe_id=str(message.source_keyframe_id),
                target_keyframe_id=str(message.target_keyframe_id),
                evidence_source_keyframe_ids=[
                    str(value) for value in
                    getattr(message, 'evidence_source_keyframe_ids', [])],
                evidence_target_keyframe_ids=[
                    str(value) for value in
                    getattr(message, 'evidence_target_keyframe_ids', [])])
            self._request_source_for_confirmation(message)
            return
        if message.status == 'REJECTED':
            if self.robot_id == message.source_robot_id:
                self.negotiation_started = False
            if self.robot_id == message.target_robot_id:
                # A rejected proposal terminates the responder's pending
                # confirmation too.  Without this reset, the responder
                # remains latched forever and ignores every later proposal,
                # including a valid multi-keyframe consensus from a new
                # acquisition batch.
                self.pending_target_proposal = False
                self.peer_proposals.pop(
                    str(message.source_keyframe_id), None)
            self._record_diagnostic_event(
                'HYPOTHESIS_REJECTED',
                source_robot_id=str(message.source_robot_id),
                target_robot_id=str(message.target_robot_id),
                rejection_reason=str(message.rejection_reason))
            return
        if not should_accept_hypothesis(
                self.accepted, message.status, message.accepted,
                message.final_confidence):
            self._record_diagnostic_event(
                'HYPOTHESIS_IGNORED_ACCEPTANCE_GATE',
                status=str(message.status), accepted=bool(message.accepted),
                final_confidence=float(message.final_confidence))
            return
        if self.robot_id != message.source_robot_id:
            self.accepted = message
            self.accepted_wall = time.monotonic()
            self._record_diagnostic_event(
                'HYPOTHESIS_TARGET_ACCEPTED',
                source_keyframe_id=str(message.source_keyframe_id),
                target_keyframe_id=str(message.target_keyframe_id))
            self.publish_local_map()
            return
        proposal = self.pending_proposals.get(
            (message.source_keyframe_id, message.target_keyframe_id))
        if proposal is None or not proposal.accepted:
            self._record_diagnostic_event(
                'HYPOTHESIS_ACK_IGNORED_NO_PENDING_PROPOSAL',
                source_keyframe_id=str(message.source_keyframe_id),
                target_keyframe_id=str(message.target_keyframe_id),
                pending_proposal_count=len(self.pending_proposals))
            return
        own = self.keyframes.get(message.source_keyframe_id)
        peer = self.peer_descriptors.get(message.target_keyframe_id)
        if own is None or peer is None:
            self._record_diagnostic_event(
                'HYPOTHESIS_ACK_IGNORED_MISSING_KEYFRAME',
                source_keyframe_id=str(message.source_keyframe_id),
                target_keyframe_id=str(message.target_keyframe_id))
            return
        final = self._hypothesis_message(
            own[0], peer, proposal, status='ACCEPTED', accepted=True,
            rejection_reason='',
            evidence_source_keyframe_ids=list(getattr(
                message, 'evidence_source_keyframe_ids', [])),
            evidence_target_keyframe_ids=list(getattr(
                message, 'evidence_target_keyframe_ids', [])))
        self.hypothesis_pub.publish(final)
        self._record_diagnostic_event(
            'HYPOTHESIS_CANONICAL_ACCEPTED',
            source_keyframe_id=str(message.source_keyframe_id),
            target_keyframe_id=str(message.target_keyframe_id),
            evidence_set_hash=str(message.evidence_set_hash))
        self.counters['accepted_hypotheses'] += 1
        self.accepted = final
        self.accepted_wall = time.monotonic()
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
        message.final_confidence = float(
            proposal.final_confidence if accepted else 0.0)
        message.status = 'ACCEPTED' if accepted else 'REJECTED'
        message.rejection_reason = rejection_reason
        message.accepted = bool(accepted)
        message.evidence_set_hash = proposal.evidence_set_hash
        message.evidence_source_keyframe_ids = list(
            proposal.evidence_source_keyframe_ids)
        message.evidence_target_keyframe_ids = list(
            proposal.evidence_target_keyframe_ids)
        message.constraint_count = proposal.constraint_count
        message.consistent_constraint_count = proposal.consistent_constraint_count
        message.spatial_baseline_m = proposal.spatial_baseline_m
        message.angular_spread_rad = proposal.angular_spread_rad
        message.median_registration_residual_m = proposal.median_registration_residual_m
        message.p95_registration_residual_m = proposal.p95_registration_residual_m
        message.projected_error_m = proposal.projected_error_m
        message.translation_uncertainty_m = proposal.translation_uncertainty_m
        message.yaw_uncertainty_rad = proposal.yaw_uncertainty_rad
        message.condition_number = proposal.condition_number
        return message

    def _request_source_for_confirmation(self, proposal):
        source_ids = list(getattr(
            proposal, 'evidence_source_keyframe_ids', []))
        if not source_ids:
            source_ids = [proposal.source_keyframe_id]
        for source_keyframe_id in source_ids:
            key = (source_keyframe_id, self.peer_robot_id)
            if key in self.pending_requests:
                self.counters['crop_request_duplicates_suppressed'] += 1
                continue
            self.pending_requests.add(key)
            self.counters['crop_requests_queued'] += 1
            request = LocalMapCropRequest()
            request.header = proposal.header
            request.requester_robot_id = self.robot_id
            request.source_robot_id = proposal.source_robot_id
            request.keyframe_id = source_keyframe_id
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
        # ``register_crops(source, target)`` returns the point transform that
        # maps source-crop coordinates into target-crop coordinates.  A ROS TF
        # with parent ``shared_map`` (the source/local frame) and child
        # ``peer_robot/local_world`` needs the inverse: the child pose
        # expressed in the parent frame.  Keep the protocol hypothesis in its
        # source->target convention and invert only at this TF boundary.
        registration = self.accepted.source_to_target
        yaw = math.atan2(
            2.0 * (registration.rotation.w * registration.rotation.z +
                   registration.rotation.x * registration.rotation.y),
            1.0 - 2.0 * (registration.rotation.y ** 2 +
                         registration.rotation.z ** 2))
        inverse_x, inverse_y, inverse_yaw = invert_se2(
            (registration.translation.x, registration.translation.y, yaw))
        transform.transform.translation.x = inverse_x
        transform.transform.translation.y = inverse_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(inverse_yaw / 2.0)
        transform.transform.rotation.w = math.cos(inverse_yaw / 2.0)
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
        accepted_hypothesis = None
        if self.accepted is not None:
            accepted_hypothesis = {
                'source_robot_id': self.accepted.source_robot_id,
                'target_robot_id': self.accepted.target_robot_id,
                'source_keyframe_id': self.accepted.source_keyframe_id,
                'target_keyframe_id': self.accepted.target_keyframe_id,
                'transform_se2': [
                    float(self.accepted.source_to_target.translation.x),
                    float(self.accepted.source_to_target.translation.y),
                    float(self._yaw(self.accepted.source_to_target.rotation)),
                ],
                'covariance': [float(value) for value in self.accepted.covariance],
                'descriptor_similarity': float(
                    self.accepted.descriptor_similarity),
                'descriptor_margin': float(self.accepted.descriptor_margin),
                'temporal_consistency': float(
                    self.accepted.temporal_consistency),
                'geometric_inlier_ratio': float(
                    self.accepted.geometric_inlier_ratio),
                'registration_residual_m': float(
                    self.accepted.registration_residual_m),
                'occupied_free_agreement': float(
                    self.accepted.occupied_free_agreement),
                'overlap_fraction': float(self.accepted.overlap_fraction),
                'final_confidence': float(self.accepted.final_confidence),
                'evidence_set_hash': str(self.accepted.evidence_set_hash),
                'evidence_source_keyframe_ids': list(
                    self.accepted.evidence_source_keyframe_ids),
                'evidence_target_keyframe_ids': list(
                    self.accepted.evidence_target_keyframe_ids),
                'constraint_count': int(self.accepted.constraint_count),
                'consistent_constraint_count': int(
                    self.accepted.consistent_constraint_count),
                'spatial_baseline_m': float(self.accepted.spatial_baseline_m),
                'angular_spread_rad': float(self.accepted.angular_spread_rad),
                'median_registration_residual_m': float(
                    self.accepted.median_registration_residual_m),
                'p95_registration_residual_m': float(
                    self.accepted.p95_registration_residual_m),
                'projected_error_m': float(self.accepted.projected_error_m),
                'translation_uncertainty_m': float(
                    self.accepted.translation_uncertainty_m),
                'yaw_uncertainty_rad': float(
                    self.accepted.yaw_uncertainty_rad),
                'condition_number': float(self.accepted.condition_number),
                'status': str(self.accepted.status),
                'accepted': bool(self.accepted.accepted),
                'rejection_reason': str(self.accepted.rejection_reason),
            }
        candidate_latency = None
        if self.first_candidate_wall is not None and self.accepted_wall is not None:
            candidate_latency = max(
                0.0, self.accepted_wall - self.first_candidate_wall)
        handoff_latency = None
        if self.first_map_wall is not None and self.accepted_wall is not None:
            handoff_latency = max(0.0, self.accepted_wall - self.first_map_wall)
        callback_timing = {}
        for name, stats in self.callback_stats.items():
            samples = sorted(stats['samples_ms'])
            p95 = samples[min(len(samples) - 1, int(0.95 * (len(samples) - 1)))] if samples else 0.0
            callback_timing[name] = {
                'count': stats['count'],
                'mean_ms': (stats['total_ms'] / stats['count']
                            if stats['count'] else 0.0),
                'p95_ms': p95,
                'max_ms': stats['max_ms'],
            }
        if self.consensus_diagnostics is not None:
            self.consensus_diagnostics.close()
        if self.physical_evidence_diagnostics is not None:
            self.physical_evidence_diagnostics.close()
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
            'accepted_hypothesis': accepted_hypothesis,
            'candidate_latency_wall_s': candidate_latency,
            'map_to_accept_latency_wall_s': handoff_latency,
            'descriptor_bytes': self.descriptor_bytes,
            'crop_cells_sent_max': self.crop_cells_sent,
            'crop_cells_received_max': self.crop_cells_received,
            'multi_constraint_attempts': self.counters[
                'multi_constraint_attempts'],
            'multi_constraint_rejections': self.counters[
                'multi_constraint_rejections'],
            'counters': self.counters,
            'diagnostics': {
                'descriptor_gate_rejection_reason_counts': dict(
                    self.gate_rejection_counts),
                'temporal_gate_rejection_reason_counts': dict(
                    self.temporal_gate_rejection_counts),
                'crop_response_rejection_reason_counts': dict(
                    self.crop_response_rejection_counts),
                'consensus_gate_rejection_reason_counts': dict(
                    self.consensus_gate_rejection_counts),
                'unique_descriptor_gate_survivors': len(
                    self.descriptor_gate_survivors),
                'unique_temporal_gate_survivors': len(
                    self.temporal_gate_survivors),
                'temporal_confirmation_clock': 'message_header_stamp_ros_clock',
                'configured_confirmation_window_s': (
                    self.confirmation_window_ns / 1.0e9),
                'effective_confirmation_window_s': (
                    self.effective_confirmation_window_ns / 1.0e9),
                'observed_own_descriptor_intervals_s': [
                    (right - left) / 1.0e9 for left, right in zip(
                        self.own_descriptor_stamps_ns,
                        list(self.own_descriptor_stamps_ns)[1:])],
                'observed_peer_descriptor_intervals_s': [
                    (right - left) / 1.0e9 for left, right in zip(
                        self.peer_descriptor_stamps_ns,
                        list(self.peer_descriptor_stamps_ns)[1:])],
                'callback_timing': callback_timing,
                'callback_started': self.callback_started,
                'callback_completed': self.callback_completed,
                'max_callback_inflight': self.max_callback_inflight,
                'max_executor_backlog_estimate': self.max_backlog_estimate,
                'cpu_samples': self.cpu_samples,
                'protocol_events': self.diagnostic_events,
                'protocol_event_drops': self.diagnostic_event_drops,
                'consensus_diagnostics_artifact': (
                    None if self.consensus_diagnostics is None else
                    self.consensus_diagnostics.path.name),
                'consensus_diagnostic_records_written': (
                    0 if self.consensus_diagnostics is None else
                    self.consensus_diagnostics.records_written),
                'consensus_diagnostic_drops': (
                    0 if self.consensus_diagnostics is None else
                    self.consensus_diagnostics.dropped_records),
                'consensus_diagnostic_write_failures': (
                    0 if self.consensus_diagnostics is None else
                    self.consensus_diagnostics.write_failures),
                'physical_evidence_diagnostics_artifact': (
                    None if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.path.name),
                'physical_evidence_diagnostic_records_written': (
                    0 if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.records_written),
                'physical_evidence_diagnostic_drops': (
                    0 if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.dropped_records),
                'physical_evidence_diagnostic_write_failures': (
                    0 if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.write_failures),
                'verification_batch_state': {
                    'acquisition_batch_id': int(
                        self.verification_batches.batch_id),
                    'batch_attempts': int(
                        self.verification_batches.batch_attempts),
                    'max_batches': int(
                        self.verification_batches.max_batches),
                    'budget': int(self.verification_batches.budget),
                    'waiting_for_novelty': bool(
                        self.verification_batches.waiting_for_novelty),
                    'lifetime_expired': bool(
                        self.verification_batches.lifetime_expired),
                    'completed': bool(self.verification_batches.completed),
                    'lifetime_s': float(
                        self.verification_batches.lifetime_s),
                    'novelty_spacing_m': float(
                        self.verification_batches.novelty_spacing_m),
                },
                'pending_candidate_pool_size': len(
                    self.pending_candidate_pairs),
                'finalized_wall_monotonic_s': time.monotonic(),
            },
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
