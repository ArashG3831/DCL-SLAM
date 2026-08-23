from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _launch_text(name: str) -> str:
    return (ROOT / 'launch' / name).read_text(encoding='utf-8')


def test_production_frontier_generators_query_the_bounded_candidate_set():
    """The eight admitted candidates must all receive a path-query slot."""
    for name in (
        'two_robots_frontier_candidates_launch.py',
        'two_robots_decentralized_exploration_launch.py',
    ):
        text = _launch_text(name)
        assert "'maximum_candidates_before_path_check': 8" in text
        assert "'maximum_path_queries_per_cycle': 8" in text

