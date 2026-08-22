from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_epuck_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.wbt') + glob('worlds/.*.wbproj')),
        (os.path.join('share', package_name, 'protos', 'e-puck'),
         glob('protos/e-puck/*.proto')),
        (os.path.join('share', package_name, 'protos', 'arena'),
         glob('protos/arena/*.proto')),
        (os.path.join('share', package_name, 'protos', 'e-puck', 'textures'),
         glob('protos/e-puck/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arash',
    maintainer_email='arash.ganjei92@gmail.com',
    description='Local Webots e-puck project package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd500_scan_fix = my_epuck_project.d500_scan_fix:main',
            'decentralized_map_fusion = '
            'my_epuck_project.decentralized_map_fusion:main',
            'map_exporter = my_epuck_project.map_exporter:main',
            'teammate_scan_filter = '
            'my_epuck_project.teammate_scan_filter:main',
            'source_aware_map_fusion = '
            'my_epuck_project.source_aware_map_fusion:main',
            'unknown_pose_frontend = '
            'my_epuck_project.unknown_pose_frontend:main',
            'unknown_pose_phase_manager = '
            'my_epuck_project.unknown_pose_phase_manager:main',
            'unknown_pose_shared_stack_activation = '
            'my_epuck_project.unknown_pose_shared_stack_activation:main',
            'unknown_pose_motion_fixture = '
            'my_epuck_project.unknown_pose_motion_fixture:main',
            'controller_startup_guard = '
            'my_epuck_project.controller_startup_guard:main',
            'controller_readiness_gate = '
            'my_epuck_project.controller_readiness_gate:main',
            'twist_stamper = my_epuck_project.twist_stamper:main',
            'cooperative_experiment_logger = '
            'my_epuck_project.cooperative_experiment_logger:main',
            'cooperative_trial_collector = '
            'my_epuck_project.cooperative_trial_collector:main',
            'run_cooperative_regression = '
            'my_epuck_project.cooperative_regression:main',
            'run_cooperative_trial_fast = '
            'my_epuck_project.cooperative_trial_fast:main',
            'rviz_map_overlay = my_epuck_project.rviz_map_overlay:main',
            'run_cooperative_campaign = '
            'my_epuck_project.cooperative_campaign_wrapper:main',
            'nav2_frontier_diagnostic = '
            'my_epuck_project.nav2_frontier_diagnostic:main',
            'export_cooperative_maps_png = '
            'my_epuck_project.cooperative_map_png_export:main',
            'controller_pipeline_diagnostics = '
            'my_epuck_project.controller_pipeline_diagnostics:main',
            'frontier_proposal_adapter = '
            'my_epuck_project.frontier_proposal_adapter:main',
            'distributed_frontier_assignment = '
            'my_epuck_project.distributed_frontier_assignment:main',
            'fixed_task_snapshot_source = '
            'my_epuck_project.fixed_task_snapshot_source:main',
            'motion_characterization_node = '
            'my_epuck_project.motion_characterization_node:main',
            'motion_course_node = '
            'my_epuck_project.motion_course_node:main',
            'motion_course_supervisor = '
            'my_epuck_project.motion_course_supervisor:main',
            'fixed_follow_path_driver = '
            'my_epuck_project.fixed_follow_path_driver:main',
            'hard_failure_runtime_fixture = '
            'my_epuck_project.hard_failure_runtime_fixture:main',
            'terminal_finalization_runtime_fixture = '
            'my_epuck_project.terminal_finalization_runtime_fixture:main',
            'motion_scan_branch_relay = '
            'my_epuck_project.motion_scan_branch_relay:main',
        ],
    },
)
