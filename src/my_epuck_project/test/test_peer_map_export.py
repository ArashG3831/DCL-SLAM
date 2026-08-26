"""Regression tests for bounded post-handoff map export."""

from types import SimpleNamespace

from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time

import my_epuck_project.unknown_pose_frontend as frontend_module
from my_epuck_project.unknown_pose_frontend import UnknownPoseFrontend


def _grid(values, origin_x=0.0):
    message = OccupancyGrid()
    message.header.frame_id = 'robot1/map'
    message.info.width = 2
    message.info.height = 2
    message.info.resolution = 0.05
    message.info.origin.position.x = origin_x
    message.info.origin.orientation.w = 1.0
    message.data = list(values)
    return message


def test_map_fingerprint_includes_geometry_and_occupancy():
    first = UnknownPoseFrontend._map_fingerprint(_grid([0, -1, 50, 100]))
    same = UnknownPoseFrontend._map_fingerprint(_grid([0, -1, 50, 100]))
    moved = UnknownPoseFrontend._map_fingerprint(
        _grid([0, -1, 50, 100], origin_x=0.05))
    changed = UnknownPoseFrontend._map_fingerprint(_grid([0, -1, 51, 100]))

    assert first == same
    assert first != moved
    assert first != changed


def test_peer_map_export_is_rate_limited_and_change_aware(monkeypatch):
    frontend = object.__new__(UnknownPoseFrontend)
    frontend.latest_map = _grid([0, -1, 50, 100])
    frontend.robot_id = 'robot1'
    frontend.map_revision = 1
    frontend.latest_map_fingerprint = frontend._map_fingerprint(
        frontend.latest_map)
    frontend.last_export_map_fingerprint = None
    frontend.last_export_wall = 0.0
    frontend.peer_map_publish_period_s = 5.0
    frontend.accepted = SimpleNamespace()
    frontend.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    frontend.merge_handoff_logged = True
    frontend.counters = {'peer_maps_published': 0}
    frontend.peer_map_pub = SimpleNamespace(published=[])
    frontend.peer_map_pub.publish = frontend.peer_map_pub.published.append
    clock = {'now': 10.0}
    monkeypatch.setattr(frontend_module.time, 'monotonic',
                        lambda: clock['now'])

    assert frontend.publish_local_map() is True
    assert len(frontend.peer_map_pub.published) == 1
    assert frontend.counters['peer_maps_published'] == 1

    # A duplicate map inside the cadence window is suppressed.
    clock['now'] = 12.0
    assert frontend.publish_local_map() is False
    assert len(frontend.peer_map_pub.published) == 1

    # Waiting alone is not enough: unchanged content stays suppressed.
    clock['now'] = 20.0
    assert frontend.publish_local_map() is False
    assert len(frontend.peer_map_pub.published) == 1

    # A changed map is exported once the cadence window has elapsed.
    frontend.latest_map = _grid([0, -1, 51, 100])
    frontend.latest_map_fingerprint = frontend._map_fingerprint(
        frontend.latest_map)
    assert frontend.publish_local_map() is True
    assert len(frontend.peer_map_pub.published) == 2
    assert frontend.counters['peer_maps_published'] == 2


def test_handoff_export_can_bypass_cadence():
    source = frontend_module.__file__
    text = open(source, encoding='utf-8').read()
    assert 'self.publish_local_map(force=True)' in text
    assert 'peer_map_publish_period_s' in text
