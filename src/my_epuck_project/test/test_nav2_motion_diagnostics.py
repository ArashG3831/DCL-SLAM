from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def test_navigation_evidence_separates_linear_from_rotation_only_commands():
    source = (PROJECT / 'my_epuck_project'
              / 'nav2_frontier_diagnostic.py').read_text(encoding='utf-8')
    for field in ('nonzero_linear_cmd_count', 'nonzero_angular_cmd_count',
                  'max_linear_cmd_mps', 'max_angular_cmd_radps',
                  'NAVIGATION_RECOVERY_TRANSITION'):
        assert field in source
