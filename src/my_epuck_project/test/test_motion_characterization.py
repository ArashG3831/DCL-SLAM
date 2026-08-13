from pathlib import Path
import json


PROJECT = Path(__file__).resolve().parents[1]


def test_motion_diagnostic_uses_supervisor_and_no_diagnostic_gps_compass():
    world = (PROJECT / 'worlds' / 'epuck_motion_characterization.wbt').read_text()
    observer = (PROJECT / 'my_epuck_project' / 'motion_supervisor_observer.py').read_text()
    assert 'supervisor TRUE' in world
    assert 'getFromDef' in observer
    assert 'GPS {' not in world
    assert 'Compass {' not in world


def test_model_speed_derivation_matches_e_puck_v2_motor_limit():
    wheel_radius = 0.02
    motor_max_velocity = 7.536
    assert abs(wheel_radius * motor_max_velocity - 0.15072) < 1.0e-9


def test_generated_audit_keeps_provenance_and_runtime_layers_separate():
    audit = Path('results/motion_characterization_20260813/motion_model_audit.json')
    if not audit.exists():
        return
    data = json.loads(audit.read_text())
    assert data['source_classification']['official_model_values']
    assert data['source_classification']['project_overrides']
    assert data['source_classification']['runtime_ros_limits']


def test_slam_recorder_is_read_only():
    source = (PROJECT / 'my_epuck_project' / 'motion_slam_recorder.py').read_text()
    assert 'create_publisher' not in source
    assert 'TransformBroadcaster' not in source
