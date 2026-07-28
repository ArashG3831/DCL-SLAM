#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generator(robot):
    return Node(
        package='my_epuck_frontier_candidates', executable='frontier_candidate_generator',
        name='frontier_candidate_generator', namespace=robot, output='screen',
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        parameters=[{
            'robot_id': robot,
            'map_topic': f'/{robot}/shared_map',
            'global_costmap_topic': f'/{robot}/global_costmap/costmap',
            'global_frame': 'shared_map',
            'robot_base_frame': f'{robot}/base_footprint',
            'compute_path_action': f'/{robot}/compute_path_to_pose',
            'candidate_topic': f'/{robot}/frontier_candidates',
            'marker_topic': f'/{robot}/frontier_candidate_markers',
            'processing_rate_hz': 0.5,
            'minimum_frontier_cells': 5,
            'minimum_frontier_length_m': 0.05,
            'stable_id_quantization_m': 0.05,
            'approach_clearance_m': 0.06,
            'minimum_robot_distance_m': 0.08,
            'maximum_candidates_before_path_check': 8,
            'maximum_path_queries_per_cycle': 5,
            'path_query_timeout_s': 1.0,
            'planner_id': 'GridBased',
        }],
    )

def generate_launch_description():
    project = get_package_share_directory('my_epuck_project')
    stack = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        project, 'launch', 'two_robots_teammate_filtered_stack_launch.py')))
    return LaunchDescription([stack, generator('robot1'), generator('robot2')])
