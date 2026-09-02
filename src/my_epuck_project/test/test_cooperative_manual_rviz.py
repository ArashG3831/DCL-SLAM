"""Validate the passive manual cooperative-exploration RViz preset."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


CONFIG = (
    Path(__file__).resolve().parents[1]
    / 'resource'
    / 'cooperative_manual_exploration.rviz'
)
INSTALLED_CONFIG = (
    Path(get_package_share_directory('my_epuck_project'))
    / 'resource'
    / 'cooperative_manual_exploration.rviz'
)


def configuration():
    """Load the source-space RViz YAML configuration."""
    return yaml.safe_load(CONFIG.read_text(encoding='utf-8'))


def test_default_layout_orbit_camera_and_tools_are_retained():
    """The adapted Jazzy default keeps its standard 3D layout and tools."""
    source = CONFIG.read_text(encoding='utf-8')
    installed = INSTALLED_CONFIG.read_text(encoding='utf-8')
    assert yaml.safe_load(source) == yaml.safe_load(installed)
    config = yaml.safe_load(source)
    manager = config['Visualization Manager']
    panels = [panel['Name'] for panel in config['Panels']]
    assert panels == [
        'Displays', 'Selection', 'Tool Properties', 'Views', 'Time']
    assert manager['Global Options']['Fixed Frame'] == 'viz/world'
    assert manager['Global Options']['Frame Rate'] == 10
    view = manager['Views']['Current']
    assert view['Class'] == 'rviz_default_plugins/Orbit'
    assert view['Target Frame'] == 'viz/world'
    assert 2.5 <= view['Distance'] <= 3.5
    assert 'TopDownOrtho' not in CONFIG.read_text(encoding='utf-8')
    tools = {tool['Class'] for tool in manager['Tools']}
    assert 'TopDownOrtho' not in installed
    assert {
        'rviz_default_plugins/FocusCamera',
        'rviz_default_plugins/Measure',
        'rviz_default_plugins/SetInitialPose',
        'rviz_default_plugins/SetGoal',
        'rviz_default_plugins/PublishPoint',
    } <= tools
    geometry = config['Window Geometry']
    assert geometry['Hide Left Dock'] is False
    assert geometry['Hide Right Dock'] is False
    assert geometry['Time']['collapsed'] is False


def test_clean_robot_pose_axes_replace_initial_full_tf_tree():
    """Only the requested passive displays are initially visible."""
    displays = configuration()['Visualization Manager']['Displays']
    by_name = {display['Name']: display for display in displays}
    enabled = {
        display['Name'] for display in displays if display['Enabled']}
    assert {
        'Shared Map (single replica)',
        'Robot1 Pose (visualization)', 'Robot2 Pose (visualization)',
        'Handoff Status', 'Robot1 Traveled Path', 'Robot2 Traveled Path',
    } <= enabled
    assert by_name['Grid']['Enabled'] is False
    assert by_name['Grid']['Value'] is False
    assert by_name['Robot1 Pre-Handoff Map Overlay']['Topic']['Value'] == '/viz/robot1_map'
    assert by_name['Robot1 Pre-Handoff Map Overlay']['Enabled'] is False
    assert by_name['Robot2 Pre-Handoff Map Overlay']['Topic']['Value'] == '/viz/robot2_map'
    assert by_name['Robot2 Pre-Handoff Map Overlay']['Enabled'] is False
    transform = by_name['TF']
    assert transform['Class'] == 'rviz_default_plugins/TF'
    assert transform['Enabled'] is False
    assert transform['Show Names'] is False
    assert transform['Show Arrows'] is False
    assert 0.08 <= transform['Marker Scale'] <= 0.16
    expected_frames = {
        'Robot1 Pose (visualization)': 'viz/robot1/base_footprint',
        'Robot2 Pose (visualization)': 'viz/robot2/base_footprint',
    }
    for name, frame in expected_frames.items():
        axes = by_name[name]
        assert axes['Class'] == 'rviz_default_plugins/Axes'
        assert axes['Enabled'] is True
        assert axes['Reference Frame'] == frame
        assert 0.12 <= axes['Length'] <= 0.15
        assert 0.008 <= axes['Radius'] <= 0.012
    assert by_name['Robot1 Model']['Enabled'] is False
    assert by_name['Robot2 Model']['Enabled'] is False


def test_single_rviz_preset_has_two_visualization_map_displays():
    """One RViz process can render both aliased local maps."""
    manager = configuration()['Visualization Manager']
    enabled_maps = [
        display for display in manager['Displays']
        if display['Class'] == 'rviz_default_plugins/Map'
        and display['Enabled']
    ]
    assert {display['Topic']['Value'] for display in enabled_maps} == {
        '/viz/shared_map'}
    assert manager['Global Options']['Fixed Frame'] == 'viz/world'


def test_namespaced_optional_display_topics_are_exact():
    """Disabled diagnostics use topics from the committed namespaced stack."""
    displays = configuration()['Visualization Manager']['Displays']
    topics = {}
    for display in displays:
        topic = display.get('Topic', {}).get('Value')
        topic = topic or display.get('Description Topic', {}).get('Value')
        if topic:
            topics[display['Name']] = topic
    assert topics == {
        'Robot1 Shared Map': '/robot1/shared_map_visualization',
        'Robot2 Shared Map (raw replica)': '/robot2/shared_map_visualization',
        'Robot1 Pre-Handoff Map Overlay': '/viz/robot1_map',
        'Robot2 Pre-Handoff Map Overlay': '/viz/robot2_map',
        'Robot1 LaserScan': '/robot1/scan_d500_fixed',
        'Robot2 LaserScan': '/robot2/scan_d500_fixed',
        'Robot1 Global Costmap': '/robot1/global_costmap/costmap',
        'Robot1 Local Costmap': '/robot1/local_costmap/costmap',
        'Robot2 Global Costmap': '/robot2/global_costmap/costmap',
        'Robot2 Local Costmap': '/robot2/local_costmap/costmap',
        'Robot1 Global Plan': '/robot1/plan',
        'Robot1 Local Plan': '/robot1/local_plan',
        'Robot2 Global Plan': '/robot2/plan',
        'Robot2 Local Plan': '/robot2/local_plan',
        'Robot1 Frontier Candidates':
            '/robot1/frontier_candidate_markers',
        'Robot2 Frontier Candidates':
            '/robot2/frontier_candidate_markers',
        'Shared Map (single replica)': '/viz/shared_map',
        'Handoff Status': '/viz/handoff_status',
        'Robot1 Traveled Path': '/viz/robot1_traveled_path',
        'Robot2 Traveled Path': '/viz/robot2_traveled_path',
    }
    optional = [
        display for display in displays
        if display['Name'] not in {
            'Grid', 'Robot1 Shared Map', 'Robot2 Shared Map (raw replica)',
            'Robot1 Pre-Handoff Map Overlay', 'Robot2 Pre-Handoff Map Overlay',
            'Shared Map (single replica)', 'TF',
            'Robot1 Pose (visualization)', 'Robot2 Pose (visualization)',
            'Handoff Status', 'Robot1 Traveled Path', 'Robot2 Traveled Path',
        }]
    assert all(display['Enabled'] is False for display in optional)


def test_shared_map_visualization_uses_incremental_update_topics():
    """Verify RViz uses the bounded visualization stream, not canonical maps."""
    displays = configuration()['Visualization Manager']['Displays']
    by_name = {display['Name']: display for display in displays}
    for robot in ('Robot1', 'Robot2'):
        name = f'{robot} Shared Map'
        if robot == 'Robot2':
            name += ' (raw replica)'
        display = by_name[name]
        assert display['Topic']['Value'] == (
            f'/{robot.lower()}/shared_map_visualization')
        assert display['Update Topic']['Value'] == (
            f'/{robot.lower()}/shared_map_visualization_updates')
    assert by_name['Shared Map (single replica)']['Topic']['Value'] == '/viz/shared_map'
    assert by_name['Shared Map (single replica)']['Color Scheme'] == 'map'
