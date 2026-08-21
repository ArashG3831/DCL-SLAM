#!/usr/bin/env python3

import os
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


def _diagnostic_params(source, robot, variant, node_prefix=''):
    """Create a temporary controller variant; production YAML stays untouched."""
    # RPP is the authoritative production YAML.  Returning it directly keeps
    # the normal launch free of generated diagnostic files or overrides.
    if variant == 'rpp' and not node_prefix:
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
               map_topic=None, autostart=None):
    source = os.path.join(
        package_dir, 'resource', f'nav2_{robot}_shared_map.yaml'
    )
    source = _diagnostic_params(
        source, robot, controller_variant, node_prefix=node_prefix)
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
            '0.08' if selected['name'] == 'large' else '0.5',
        f'{parameter_node("controller_server")}.ros__parameters.progress_checker.movement_time_allowance':
            '18.0' if selected['name'] == 'large' else '10.0',
        f'{parameter_node("collision_monitor")}.ros__parameters.state_topic':
            'collision_monitor_state',
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }
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
    # Unknown-pose pre-handoff is structurally local-only.  The explicit
    # phase marker is required in addition to the request flag so a nested
    # launch default or scope collision cannot instantiate shared Nav2/fusion
    # before the canonical handoff.
    launch_shared_stack = requested_shared_stack and (
        not unknown_initial_pose or phase_already_aligned)
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
        if launch_shared_stack:
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
                prefix=(
                    'nice -n ' + str(fusion_process_nice)
                    if fusion_process_nice != 0 and not (
                        diagnostic_mode and quota_enabled) else
                    'systemd-run --user --scope --quiet '
                    '-p CPUQuota=' + LaunchConfiguration(
                        'fusion_cpu_quota_percent').perform(context) + '%'
                    if diagnostic_mode and quota_enabled else ''),
                ))
    nav2_actions = []
    for robot in ('robot1', 'robot2'):
        if unknown_initial_pose and launch_mapping:
            nav2_actions.extend(nav2_nodes(
                package_dir, robot, selected,
                LaunchConfiguration('controller_variant').perform(context),
                node_prefix='local_', global_frame=f'{robot}/map',
                map_topic=f'/{robot}/map', autostart=True))
        if unknown_initial_pose and launch_shared_stack:
            nav2_actions.extend(nav2_nodes(
                package_dir, robot, selected,
                LaunchConfiguration('controller_variant').perform(context),
                node_prefix='', global_frame='shared_map',
                map_topic=f'/{robot}/shared_map', autostart=False))
        elif not unknown_initial_pose:
            nav2_actions.extend(nav2_nodes(
                package_dir, robot, selected,
                LaunchConfiguration('controller_variant').perform(context)))
    return [filtered_slam, *alignment, *exchange, *nav2_actions]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_profile',
            default_value='small',
            choices=['large', 'small', 'large_unknown_pose',
                     'large_unknown_pose_16m',
                     'large_unknown_pose_close_start'],
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
        DeclareLaunchArgument('phase_already_aligned', default_value='false',
                              choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
