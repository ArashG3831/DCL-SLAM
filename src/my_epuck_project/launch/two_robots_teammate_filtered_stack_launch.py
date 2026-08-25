#!/usr/bin/env python3

import os
import shutil
import subprocess
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from my_epuck_project.cooperative_profiles import profile_for_world
from nav2_common.launch import RewrittenYaml


LIFECYCLE_NODES = [
    'controller_server', 'smoother_server', 'planner_server', 'route_server',
    'behavior_server', 'velocity_smoother', 'collision_monitor',
    'bt_navigator', 'waypoint_follower',
]


def _is_large_world_profile(selected):
    """Return whether a resolved profile uses the large-world geometry.

    Unknown-pose and physics-timestep variants retain the ``large`` prefix;
    comparing only against the literal base profile would select the small
    world progress-checker contract for them.
    """
    return str(selected.get('name', '')).startswith('large')


def _progress_movement_time_allowance(selected):
    """Return the bounded progress allowance for the resolved world profile.

    Unknown-pose large profiles use RPP's intentional rotate-to-heading phase.
    The observed e-puck angular command (about 0.2--0.3 rad/s) can require
    more than the ordinary large-world 18 s allowance for a 2--4 rad heading
    change.  Keep the existing checker and raise only this bounded allowance
    for those profiles; small and ordinary large-world behavior is unchanged.
    """
    name = str(selected.get('name', ''))
    if name.startswith('large_unknown_pose'):
        return '30.0'
    if _is_large_world_profile(selected):
        return '18.0'
    return '10.0'


def _user_systemd_scope_available():
    """Return whether a user systemd scope can actually be launched.

    WSL images commonly ship ``systemd-run`` without a user bus.  Passing a
    systemd prefix in that environment makes the fusion process fail before
    it creates its ROS node (``Failed to connect to bus: No medium found``).
    A process-limit wrapper is optional; the map-fusion node itself is not.
    """
    if not (
            shutil.which('systemd-run') is not None
            and os.path.isdir('/run/systemd/system')
            and bool(os.environ.get('DBUS_SESSION_BUS_ADDRESS'))):
        return False
    try:
        # A socket and environment variable can exist in WSL even when the
        # user manager is not actually serving requests.  Probe the exact
        # wrapper once, with no ROS process attached, before using it.
        subprocess.run(
            ['systemd-run', '--user', '--scope', '--quiet', '--wait', 'true'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return True


def _fusion_process_prefix(
        fusion_process_nice, diagnostic_mode, quota_enabled, quota_percent):
    """Build a safe optional prefix for the map-fusion process.

    ``nice`` is inherited from the campaign supervisor and is sufficient for
    the normal WSL validation path.  Use a CPUQuota scope only when a live
    user systemd bus is provably available; never make fusion startup depend
    on an unavailable service manager.
    """
    if fusion_process_nice != 0 and not (diagnostic_mode and quota_enabled):
        return 'nice -n ' + str(fusion_process_nice)
    if diagnostic_mode and quota_enabled and _user_systemd_scope_available():
        return (
            'systemd-run --user --scope --quiet -p CPUQuota='
            + str(quota_percent) + '%')
    return ''


# Historical diagnostic baseline.  Production YAML now contains RPP; keeping
# this block here preserves explicit ``controller_variant:=dwb`` reproduction
# without reintroducing DWB into the active production configuration.
_DWB_DIAGNOSTIC_FOLLOW_PATH = {
    'plugin': 'dwb_core::DWBLocalPlanner',
    'debug_trajectory_details': True,
    'min_vel_x': 0.0,
    'min_vel_y': 0.0,
    'max_vel_x': 0.13,
    'max_vel_y': 0.0,
    'max_vel_theta': 0.35,
    'min_speed_xy': 0.0,
    'max_speed_xy': 0.13,
    'min_speed_theta': 0.0,
    'acc_lim_x': 0.20,
    'acc_lim_y': 0.0,
    'acc_lim_theta': 1.2,
    'decel_lim_x': -0.20,
    'decel_lim_y': 0.0,
    'decel_lim_theta': -1.2,
    'vx_samples': 6,
    'vy_samples': 5,
    'vtheta_samples': 21,
    'sim_time': 1.7,
    'linear_granularity': 0.01,
    'angular_granularity': 0.025,
    'transform_tolerance': 1.5,
    'xy_goal_tolerance': 0.06,
    'trans_stopped_velocity': 0.04,
    'short_circuit_trajectory_evaluation': True,
    'stateful': True,
    'critics': [
        'RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
        'PathAlign', 'PathDist', 'GoalDist',
    ],
    'BaseObstacle.scale': 0.20,
    'PathAlign.scale': 8.0,
    'PathAlign.forward_point_distance': 0.0025,
    'GoalAlign.scale': 0.0,
    'GoalAlign.forward_point_distance': 0.0025,
    'PathDist.scale': 24.0,
    'GoalDist.scale': 24.0,
    'RotateToGoal.scale': 0.0,
    'RotateToGoal.slowing_factor': 5.0,
    'RotateToGoal.lookahead_time': -1.0,
}


def _diagnostic_params(source, robot, variant, node_prefix='',
                       use_sim_time_value=None):
    """Create a temporary controller variant; production YAML stays untouched."""
    # RPP is the authoritative production YAML.  Returning it directly keeps
    # the normal launch free of generated diagnostic files or overrides.
    if variant == 'rpp' and not node_prefix and use_sim_time_value is None:
        return source
    with open(source, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if node_prefix:
        # The same Nav2 parameter document serves the normal nodes and the
        # pre-handoff local nodes.  The latter deliberately have a ``local_``
        # name so their lifecycle can be switched independently; rewrite only
        # the YAML node keys so the existing frozen parameters are actually
        # applied to those renamed nodes instead of falling back to Nav2's
        # defaults.
        document = {
            (node_prefix + key if key in LIFECYCLE_NODES else key): value
            for key, value in document.items()
        }
    if use_sim_time_value is not None:
        sim_time = str(use_sim_time_value).lower() == 'true'

        def rewrite_clock(value):
            if isinstance(value, dict):
                if isinstance(value.get('ros__parameters'), dict):
                    value['ros__parameters']['use_sim_time'] = sim_time
                for child in value.values():
                    rewrite_clock(child)
            elif isinstance(value, list):
                for child in value:
                    rewrite_clock(child)

        rewrite_clock(document)
    controller = document[
        node_prefix + 'controller_server' if node_prefix else 'controller_server'
    ]['ros__parameters']
    original = dict(controller['FollowPath'])
    if variant == 'dwb':
        controller['FollowPath'] = dict(_DWB_DIAGNOSTIC_FOLLOW_PATH)
    elif variant == 'rotation_shim_dwb':
        primary = dict(_DWB_DIAGNOSTIC_FOLLOW_PATH)
        primary.pop('plugin', None)
        shim = {
            'plugin': 'nav2_rotation_shim_controller::RotationShimController',
            'primary_controller': 'dwb_core::DWBLocalPlanner',
            'angular_dist_threshold': 0.785,
            'angular_disengage_threshold': 0.3925,
            'forward_sampling_distance': 0.20,
            'rotate_to_heading_angular_vel': 0.35,
            'max_angular_accel': 0.20,
            'simulate_ahead_time': 1.0,
            'rotate_to_goal_heading': False,
            'rotate_to_heading_once': False,
            'closed_loop': True,
        }
        # The Jazzy RotationShim creates the primary controller with the same
        # plugin name (FollowPath).  Keep the primary DWB parameters at that
        # namespace; only the wrapper's plugin and primary_controller fields
        # are special.  A nested mapping is not a ROS parameter namespace.
        shim.update(primary)
        controller['FollowPath'] = shim
    elif variant == 'rpp':
        # Prefix-only rewrite: retain the frozen production RPP parameters.
        pass
    else:
        raise ValueError(f'unknown controller_variant={variant}')
    handle = tempfile.NamedTemporaryFile(
        mode='w', suffix=f'_{robot}_{variant}.yaml', delete=False,
        encoding='utf-8')
    with handle:
        yaml.safe_dump(document, handle, sort_keys=False)
    return handle.name


def nav2_nodes(package_dir, robot, selected, controller_variant,
               *, node_prefix='', global_frame='shared_map',
               map_topic=None, autostart=None, use_sim_time_value=None):
    source = os.path.join(
        package_dir, 'resource', f'nav2_{robot}_shared_map.yaml'
    )
    source = _diagnostic_params(
        source, robot, controller_variant, node_prefix=node_prefix,
        use_sim_time_value=use_sim_time_value)
    parameter_node = lambda name: node_prefix + name if node_prefix else name
    diagnostic_rewrites = {
        f'{parameter_node("controller_server")}.ros__parameters.FollowPath.publish_evaluation':
            LaunchConfiguration('diagnostic_mode'),
        f'{parameter_node("controller_server")}.ros__parameters.FollowPath.publish_local_plan':
            LaunchConfiguration('diagnostic_mode'),
        f'{parameter_node("controller_server")}.ros__parameters.FollowPath.publish_global_plan':
            LaunchConfiguration('diagnostic_mode'),
        f'{parameter_node("controller_server")}.ros__parameters.FollowPath.publish_transformed_global_plan':
            LaunchConfiguration('diagnostic_mode'),
        f'{parameter_node("controller_server")}.ros__parameters.FollowPath.publish_cost_grid_pc':
            LaunchConfiguration('diagnostic_mode'),
    } if controller_variant == 'dwb' else {}
    parameter_rewrites = {
        f'{parameter_node("bt_navigator")}.ros__parameters.global_frame':
            global_frame,
        'global_costmap.global_costmap.ros__parameters.global_frame':
            global_frame,
        'global_costmap.global_costmap.ros__parameters.static_layer.map_topic':
            map_topic or f'/{robot}/shared_map',
        f'{parameter_node("controller_server")}.ros__parameters.progress_checker.required_movement_radius':
            # All large-world variants, including the close/far unknown-pose
            # fixtures, need the large-world progress contract.  Comparing
            # only against the literal ``large`` profile silently applied the
            # small-world 0.5 m/10 s checker to those fixtures; RPP then
            # failed while it was still rotating to a frontier heading.
            '0.08' if _is_large_world_profile(selected) else '0.5',
        f'{parameter_node("controller_server")}.ros__parameters.progress_checker.movement_time_allowance':
            _progress_movement_time_allowance(selected),
        f'{parameter_node("collision_monitor")}.ros__parameters.state_topic':
            'collision_monitor_state',
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }
    # The Nav2 parameter files carry ``use_sim_time`` inside every node's
    # ros__parameters block (and inside both costmaps).  Rewriting only the
    # document-level key leaves the pre-handoff ``local_*`` stack on its
    # default wall clock.  In simulation that makes stamped Webots scans look
    # millions of seconds old to collision_monitor, which then emits a safety
    # stop forever.  Rewrite each actual node parameter explicitly so local
    # and shared stacks use the same clock selected by the launch.
    for lifecycle_name in LIFECYCLE_NODES:
        parameter_rewrites[
            f'{parameter_node(lifecycle_name)}.ros__parameters.use_sim_time'
        ] = LaunchConfiguration('use_sim_time')
    for costmap_name in ('local_costmap.local_costmap',
                         'global_costmap.global_costmap'):
        parameter_rewrites[
            f'{costmap_name}.ros__parameters.use_sim_time'
        ] = LaunchConfiguration('use_sim_time')
    parameter_rewrites.update(diagnostic_rewrites)
    parameters = ParameterFile(
        RewrittenYaml(
            source_file=source,
            root_key=robot,
            param_rewrites={
                'local_costmap.local_costmap.ros__parameters.resolution':
                    str(selected['local_costmap_resolution']),
                'global_costmap.global_costmap.ros__parameters.resolution':
                    str(selected['global_costmap_resolution']),
                **parameter_rewrites,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    common_remaps = [('tf', '/tf'), ('tf_static', '/tf_static')]
    specifications = [
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', []),
        ('nav2_planner', 'planner_server', []),
        ('nav2_route', 'route_server', []),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_collision_monitor', 'collision_monitor', []),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', []),
    ]
    nodes = [
        Node(
            package=package,
            executable=executable,
            name=node_prefix + executable,
            namespace=robot,
            output='screen',
            parameters=[parameters],
            remappings=common_remaps + extra_remaps,
        )
        for package, executable, extra_remaps in specifications
    ]
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=node_prefix + 'lifecycle_manager_navigation',
        namespace=robot,
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': (LaunchConfiguration('nav2_autostart')
                          if autostart is None else autostart),
            'node_names': [node_prefix + name for name in LIFECYCLE_NODES],
        }],
    ))
    return nodes


def launch_setup(context):
    package_dir = get_package_share_directory('my_epuck_project')
    world_path = LaunchConfiguration('world_path').perform(context)
    selected = profile_for_world(
        LaunchConfiguration('world_profile').perform(context),
        os.path.dirname(world_path) if world_path else os.path.join(package_dir, 'worlds'),
        explicit_world_path=world_path,
    )
    diagnostic_mode = (
        LaunchConfiguration('diagnostic_mode').perform(context) == 'true')
    fusion_quota = LaunchConfiguration(
        'fusion_cpu_quota_percent').perform(context)
    fusion_process_nice = int(LaunchConfiguration(
        'fusion_process_nice').perform(context))
    unknown_initial_pose = (
        LaunchConfiguration('unknown_initial_pose').perform(context).lower()
        == 'true')
    phase_already_aligned = (
        LaunchConfiguration('phase_already_aligned').perform(context).lower()
        == 'true')
    launch_mapping = (
        LaunchConfiguration('launch_mapping').perform(context).lower()
        == 'true')
    requested_shared_stack = (
        LaunchConfiguration('launch_shared_stack').perform(context).lower()
        == 'true')
    requested_shared_fusion = (
        LaunchConfiguration('launch_shared_fusion').perform(context).lower()
        == 'true')
    # Unknown-pose pre-handoff is structurally local-only.  The explicit
    # phase marker is required in addition to the request flag so a nested
    # launch default or scope collision cannot instantiate shared Nav2/fusion
    # before the canonical handoff.
    launch_shared_stack = requested_shared_stack and (
        not unknown_initial_pose or phase_already_aligned)
    # Keep only the fusion processes resident before handoff.  They are
    # explicitly handoff-gated and therefore have no map subscriptions,
    # timer, or TF listener until an accepted hypothesis arrives.  This
    # avoids delaying fusion behind the large post-handoff Nav2 launch.
    launch_shared_fusion = requested_shared_fusion and (
        not unknown_initial_pose or phase_already_aligned or
        not launch_shared_stack)
    handoff_gated = unknown_initial_pose and not phase_already_aligned
    try:
        quota_enabled = float(fusion_quota) > 0.0
    except ValueError:
        quota_enabled = False
    filtered_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_dir, 'launch',
            'two_robots_teammate_filtered_dual_slam_launch.py',
        )),
        launch_arguments={
            'world_profile': selected['name'],
            'webots_port': LaunchConfiguration('webots_port'),
            'webots_controller_port': LaunchConfiguration(
                'webots_controller_port'),
            'webots_mode': LaunchConfiguration('webots_mode'),
            'webots_gui': LaunchConfiguration('webots_gui'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'scan_input_reliability': LaunchConfiguration(
                'scan_input_reliability'),
            'world_path': world_path,
            'use_scan_matching': LaunchConfiguration('use_scan_matching'),
            'do_loop_closing': LaunchConfiguration('do_loop_closing'),
            'slam_tf_publish_probe_library': LaunchConfiguration(
                'slam_tf_publish_probe_library'),
            'slam_tf_publish_probe_log': LaunchConfiguration(
                'slam_tf_publish_probe_log'),
            'slam_tf_publication_mode': LaunchConfiguration(
                'slam_tf_publication_mode'),
            # The D500 scan-plane silhouette is the measured 26 mm housing,
            # not the historical 35 mm body-radius approximation.
            'teammate_geometry_radius_m': '0.026',
            'unknown_initial_pose': LaunchConfiguration(
                'unknown_initial_pose'),
            'launch_mapping': LaunchConfiguration('launch_mapping'),
            'phase_already_aligned': LaunchConfiguration(
                'phase_already_aligned'),
        }.items(),
    )
    relative = selected['world_metadata']['relative_transform']
    alignment = [] if unknown_initial_pose else [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='shared_to_robot1_map',
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0',
                '--frame-id', 'shared_map', '--child-frame-id', 'robot1/map',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='shared_to_robot2_map',
            output='screen',
            arguments=[
                '--x', str(relative[0]), '--y', str(relative[1]),
                '--z', '0.0', '--yaw', str(relative[2]),
                '--frame-id', 'shared_map', '--child-frame-id', 'robot2/map',
            ],
        ),
    ]
    exchange = []
    for robot, peer in (('robot1', 'robot2'), ('robot2', 'robot1')):
        if not unknown_initial_pose:
            exchange.append(Node(
                package='my_epuck_project', executable='map_exporter',
                name='map_exporter', namespace=robot, output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'source_robot_id': robot,
                    'input_topic': f'/{robot}/map',
                    'output_topic': f'/cslam/{robot}/local_map',
                    'export_rate_hz': 1.0,
                }],
            ))
        if launch_shared_fusion:
            exchange.append(Node(
                package='my_epuck_project',
                executable='source_aware_map_fusion',
                name='map_fusion',
                namespace=robot,
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'local_map_topic': f'/{robot}/map',
                    'remote_peer_topic': (
                        f'/cslam/unknown_pose/{peer}/local_map'
                        if unknown_initial_pose else
                        f'/cslam/{peer}/local_map'),
                    'expected_remote_source': peer,
                    'output_topic': 'shared_map',
                    'metadata_topic': 'shared_map_metadata',
                    'output_frame': 'shared_map',
                    'resolution': selected['fusion_resolution'],
                    'live_robot_frames': [
                        'robot1/base_footprint',
                        'robot2/base_footprint',
                    ],
                    'live_footprint_radius_m': 0.037,
                    'live_footprint_uncertainty_cells': 1,
                    'live_pose_max_age_s': 0.5,
                    'min_fusion_rebuild_period_s': LaunchConfiguration(
                        'fusion_rebuild_period_s'),
                    # Coalesce local and peer map callbacks behind one bounded
                    # timer. Callbacks only mark source state dirty.
                    'publish_on_callback': False,
                    'handoff_gated': handoff_gated,
                    # A peer silhouette can be observed while its scan-frame
                    # transform is delayed.  Clear both fresh live robot
                    # footprints in every shared map so that transient peer
                    # evidence cannot make either robot its own obstacle.
                    'sanitize_live_footprints': True,
                }],
                prefix=_fusion_process_prefix(
                    fusion_process_nice,
                    diagnostic_mode,
                    quota_enabled,
                    LaunchConfiguration(
                        'fusion_cpu_quota_percent').perform(context)),
                ))
    # The pre-handoff mapping launch owns the robot-local frame anchors, but
    # those processes are intentionally terminated with the local Nav2 stack
    # at handoff.  Recreate the non-estimating local_world -> map anchors in
    # the shared launch so shared_map -> local_world (from the accepted
    # protocol TF relay) remains connected to each robot's SLAM map/odom/base
    # chain.  These anchors are identity edges within one robot and contain no
    # inter-robot pose information.
    shared_frame_anchors = []
    if unknown_initial_pose and launch_shared_stack:
        for robot in ('robot1', 'robot2'):
            shared_frame_anchors.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='shared_phase_local_map_frame_anchor',
                namespace=robot,
                output='screen',
                arguments=[
                    '--x', '0.0', '--y', '0.0', '--z', '0.0', '--yaw', '0.0',
                    '--frame-id', f'{robot}/local_world',
                    '--child-frame-id', f'{robot}/map',
                ],
            ))
    nav2_actions = []
    for robot in ('robot1', 'robot2'):
        if unknown_initial_pose and launch_mapping:
            nav2_actions.extend(nav2_nodes(
                package_dir, robot, selected,
                LaunchConfiguration('controller_variant').perform(context),
                node_prefix='local_', global_frame=f'{robot}/map',
                map_topic=f'/{robot}/map',
                use_sim_time_value=LaunchConfiguration(
                    'use_sim_time').perform(context),
                # The bounded runner explicitly disables automatic lifecycle
                # startup so its readiness gate can start local Nav2 only
                # after both wheel controllers have produced odom/TF.
                autostart=LaunchConfiguration('nav2_autostart')))
        if unknown_initial_pose and launch_shared_stack:
            nav2_actions.extend(nav2_nodes(
                package_dir, robot, selected,
                LaunchConfiguration('controller_variant').perform(context),
                node_prefix='', global_frame='shared_map',
                map_topic=f'/{robot}/shared_map', autostart=False,
                use_sim_time_value=LaunchConfiguration(
                    'use_sim_time').perform(context)))
        elif not unknown_initial_pose:
            nav2_actions.extend(nav2_nodes(
                package_dir, robot, selected,
                LaunchConfiguration('controller_variant').perform(context),
                use_sim_time_value=LaunchConfiguration(
                    'use_sim_time').perform(context)))
    return [filtered_slam, *alignment, *exchange,
            *shared_frame_anchors, *nav2_actions]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='small',
            choices=['large', 'small', 'large_unknown_pose',
                     'large_unknown_pose_16m',
                     'large_unknown_pose_close_start',
                     'large_unknown_pose_close_start_20ms',
                     'large_unknown_pose_close_start_20ms_scan_matching',
                     'large_unknown_pose_far_start_20ms_scan_matching'],
        ),
        DeclareLaunchArgument('webots_port', default_value='23000'),
        DeclareLaunchArgument(
            'webots_controller_port', default_value=''),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('webots_mode', default_value='realtime'),
        DeclareLaunchArgument('webots_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_scan_matching', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('do_loop_closing', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('slam_tf_publish_probe_library', default_value=''),
        DeclareLaunchArgument('slam_tf_publish_probe_log', default_value=''),
        DeclareLaunchArgument('slam_tf_publication_mode', default_value='',
                              choices=['', 'SYNCHRONOUS', 'ASYNCHRONOUS']),
        DeclareLaunchArgument('sensor_profile', default_value='full',
                              choices=['full', 'throughput']),
        DeclareLaunchArgument('scan_input_reliability', default_value='best_effort',
                              choices=['reliable', 'best_effort']),
        DeclareLaunchArgument('diagnostic_mode', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('fusion_cpu_quota_percent', default_value='30'),
        DeclareLaunchArgument('fusion_process_nice', default_value='0'),
        DeclareLaunchArgument('fusion_rebuild_period_s', default_value='1.0'),
        DeclareLaunchArgument('nav2_autostart', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('controller_variant', default_value='rpp',
                              choices=['dwb', 'rotation_shim_dwb', 'rpp']),
        DeclareLaunchArgument('unknown_initial_pose', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_mapping', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_shared_stack', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('launch_shared_fusion', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('phase_already_aligned', default_value='false',
                              choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
