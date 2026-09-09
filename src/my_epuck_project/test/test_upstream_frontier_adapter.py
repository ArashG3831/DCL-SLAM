"""Focused contract tests for the non-dispatching upstream frontier adapter."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GENERATOR = (ROOT / "my_epuck_frontier_candidates" / "src" /
             "frontier_candidate_generator.cpp")
CMAKE = ROOT / "my_epuck_frontier_candidates" / "CMakeLists.txt"
FRONTIER_LAUNCH = ROOT / "my_epuck_project" / "launch" / (
    "two_robots_frontier_candidates_launch.py")


def _source():
    return GENERATOR.read_text(encoding="utf-8")


def test_adapter_constructs_upstream_core_and_uses_snapshot():
    source = _source()
    assert "FrontierExplorerCoreParams" in source
    assert "FrontierExplorerCoreCallbacks" in source
    assert "FrontierExplorerCore>(" in source
    assert "get_frontier_snapshot" in source
    assert "get_frontier(" not in source


def test_condition_c_plumbs_pinned_decision_map_defaults_without_changing_minimum():
    source = _source()
    launch = FRONTIER_LAUNCH.read_text(encoding="utf-8")
    for declaration in (
            "P(bool, frontier_map_optimization_enabled, true)",
            "P(double, sigma_s, 2.0)",
            "P(double, sigma_r, 30.0)",
            "P(int, dilation_kernel_radius_cells, 1)"):
        assert declaration in source
    for assignment in (
            "params.frontier_map_optimization_enabled = frontier_map_optimization_enabled_;",
            "params.sigma_s = sigma_s_;",
            "params.sigma_r = sigma_r_;",
            "params.dilation_kernel_radius_cells = dilation_kernel_radius_cells_;"):
        assert assignment in source
    assert "'frontier_map_optimization_enabled': True" in launch
    assert "'sigma_s': 2.0" in launch
    assert "'sigma_r': 30.0" in launch
    assert "'dilation_kernel_radius_cells': 1" in launch
    assert "'minimum_frontier_cells': minimum_frontier_cells" in launch
    assert "frontier_map_optimization_enabled = false" not in source


def test_adapter_is_non_dispatching_and_allocator_owns_goals():
    source = _source()
    assert "core_->exploration_enabled = false" in source
    assert "UPSTREAM_AUTONOMOUS_DISPATCH_BLOCKED" in source
    assert "dispatch_goal_request =" in source


def test_adapter_preserves_rotated_cells_and_bounds_diagnostics():
    source = _source()
    assert "diagnostic.region.cells" in source
    assert "frontier_world_bounds" in source
    assert "FrontierExplorerCore::Frontier" not in source


def test_project_links_established_upstream_core_target():
    cmake = CMAKE.read_text(encoding="utf-8")
    assert "find_package(frontier_exploration_ros2 REQUIRED)" in cmake
    assert "frontier_exploration_ros2::frontier_exploration_ros2_core" in cmake


def test_evaluation_cache_has_explicit_capacity_and_pruning():
    source = _source()
    assert "maximum_evaluation_records" in source
    assert "prune_evaluation_cache" in source
    assert "FRONTIER_EVALUATION_CACHE_BOUND" in source
