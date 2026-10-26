"""The mission feed (ui/feeds/mission.py): status, decisions and hypotheses into the TUI and the map."""
import json
from pathlib import Path

import pytest

from mhseals_testing.ui.feeds.mission import FRESH, MissionState
from mhseals_testing.ui.minimap.state import MapState

FIXTURES = Path(__file__).parent / 'fixtures'


def load(name):
    return json.loads((FIXTURES / name).read_text())


def feed():
    clock = [100.0]
    state = MissionState(MapState(), clock=lambda: clock[0], wall=lambda: 1761400000.0 + clock[0])
    return state, clock


def test_status_is_held_and_its_goal_goes_to_the_map():
    mission, clock = feed()
    assert not mission.fresh() and mission.age() == float('inf')
    mission.ingest('status', load('mission_status.json'))
    assert mission.fresh() and mission.status['phase'] == 'search'
    assert mission.map_state.mission_goal == (18.5, -6.0, 1.57, 'search the east field for the speed gate')
    assert mission.ladder()[2] == ('docking', 'failed', 3, 3)
    clock[0] += FRESH
    assert not mission.fresh()
    mission.ingest('status', dict(load('mission_status.json'), goal=None))
    assert mission.map_state.mission_goal is None


def test_a_status_with_only_the_older_keys_still_reads():
    mission, _ = feed()
    mission.ingest('status', {'task': 'docking', 'status': 'RUNNING', 'elapsed': 3.0, 'tip': 'approach'})
    assert mission.fresh() and mission.ladder() == [] and mission.recent() == []
    assert mission.map_state.mission_goal is None


def test_only_decision_events_are_logged_and_status_recent_stands_in_until_one_arrives():
    mission, _ = feed()
    mission.ingest('status', load('mission_status.json'))
    assert [e['kind'] for _, e in mission.recent()][-1] == 'search' and mission.recent()[0][0] is None
    mission.ingest('event', {'event': 'goal', 'x': 1.0, 'y': 2.0, 'why': 'gate centre'})
    mission.ingest('event', {'event': 'decision', 'kind': 'start', 'stage': 'nav_channel', 'reason': 'buoy pair'})
    assert [(t, e['reason']) for t, e in mission.recent()] == [(1761400100.0, 'buoy pair')]
    for i in range(60):
        mission.ingest('event', {'event': 'decision', 'kind': 'retry', 'stage': 'dock', 'reason': str(i)})
    assert len(mission.decisions) == 50 and mission.recent(2)[-1][1]['reason'] == '59'


def test_hypotheses_become_map_tasks_and_bad_input_is_ignored():
    mission, _ = feed()
    mission.ingest('hypotheses', load('task_hypotheses.json'))
    assert [t.state for t in mission.map_state.tasks] == ['done', 'failed', 'hypothesis', 'confirmed']
    mission.ingest('hypotheses', ['not', 'a', 'dict'])
    assert len(mission.map_state.tasks) == 4
    with pytest.raises(ValueError):
        mission.ingest('nope', {})


def test_a_decision_keeps_its_own_finite_time():
    mission, _ = feed()
    for stamp in (1761300000.5, float('nan'), float('inf'), 'noon', -5, None):
        mission.ingest('event', {'event': 'decision', 'kind': 'start', 'reason': 'r', 'time': stamp})
    assert [t for t, _ in mission.decisions] == [1761300000.5] + [1761400100.0] * 5


def test_another_status_schema_is_ignored_with_a_notice():
    mission, _ = feed()
    mission.ingest('status', dict(load('mission_status.json'), schema='mhseals.mission_status/2'))
    assert not mission.status and 'mission_status/2' in mission.notice
    mission.ingest('status', {k: v for k, v in load('mission_status.json').items() if k != 'schema'})
    assert mission.fresh()


FUZZ = [{'stages': [1, 2]}, {'stages': 'x'}, {'stages': {'a': {'state': 'done', 'attempts': 'x'}}},
        {'stages': {'a': {'attempts': float('nan'), 'failures': float('inf')}}}, {'stages': {'a': {'state': ['run']}}},
        {'stages': {'a': {'state': {'x': 1}}, 'b': 5}}, {'stage': ['search']}, {'stage': {'a': 1}},
        {'recent': {'a': 1}}, {'recent': 7}, {'recent': [1, 'x', {'kind': ['k'], 'reason': None, 'stage': {}}]},
        {'goal': [1, 2]}, {'goal': {'x': float('nan'), 'y': 1e308, 'why': 3}}, {'phase': 3, 'behaviour': ['b']},
        {'elapsed': float('nan')}, {'stages': {'search': {'elapsed': 'long'}}, 'stage': 'search'}]


@pytest.mark.parametrize('bad', FUZZ)
def test_a_malformed_status_still_renders(bad):
    from rich.console import Console
    from mhseals_testing.ui.panels import mission as panels
    mission, _ = feed()
    mission.ingest('status', dict(load('mission_status.json'), **bad))
    mission.ingest('event', {'event': 'decision', 'kind': ['k'], 'stage': {'s': 1}, 'reason': 5})
    console = Console(width=80, color_system=None, file=open('/dev/null', 'w'))
    for renderable in (panels.strip({'mission': mission}), panels.pane({'mission': mission}),
                       panels.decisions({'mission': mission}, 8)):
        console.print(renderable)
    assert all(isinstance(name, str) and isinstance(state, str) for name, state, *_ in mission.ladder())


def test_the_ros_feed_drops_any_bad_message_and_logs_once():
    pytest.importorskip('rclpy')
    from mhseals_testing.ros.mission import MissionFeed
    logged = []
    node = type('N', (), {'create_subscription': lambda *a: None,
                          'get_logger': lambda self: type('L', (), {'warning': lambda s, m: logged.append(m)})()})()
    mission, _ = feed()
    ros = MissionFeed(node, mission)
    for data in ('{"hypotheses": 5}', '{"schema": "mhseals.task_hypotheses/1", "hypotheses": [[1]]}', 'not json'):
        ros._take('hypotheses', data)
    assert ros.drops.get('hypotheses', 0) <= 3 and len(logged) <= 1


def _watcher_module(monkeypatch):
    """mhseals_testing.ros.mission, its rclpy and std_msgs imports stubbed when absent."""
    import importlib
    import sys
    from types import ModuleType, SimpleNamespace
    if importlib.util.find_spec('rclpy') is None:   # stub the two imports; nothing spins
        qos = ModuleType('rclpy.qos')
        qos.DurabilityPolicy = qos.ReliabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1, RELIABLE=1)
        qos.QoSProfile = lambda **kw: kw
        msg = ModuleType('std_msgs.msg')
        msg.String = object
        for name, module in (('rclpy', ModuleType('rclpy')), ('rclpy.qos', qos),
                             ('std_msgs', ModuleType('std_msgs')), ('std_msgs.msg', msg)):
            monkeypatch.setitem(sys.modules, name, module)
        monkeypatch.setitem(sys.modules, 'mhseals_testing.ros.mission', None)   # dropped on undo
        monkeypatch.delitem(sys.modules, 'mhseals_testing.ros.mission')
    from mhseals_testing.ros import mission
    return mission


def test_a_runner_gone_without_a_final_status_stops_reading_running(monkeypatch):
    from types import SimpleNamespace
    TaskWatcher = _watcher_module(monkeypatch).TaskWatcher
    publishers = [1]
    node = SimpleNamespace(create_subscription=lambda *a: None,
                           count_publishers=lambda topic: publishers[0] if topic == '/task/status' else 0)
    watcher = TaskWatcher(SimpleNamespace(node=node, monitor=SimpleNamespace(clock=lambda: 1.0)))
    watcher._status(SimpleNamespace(data='{"task": "gate", "status": "RUNNING"}'))
    assert watcher.status['status'] == 'RUNNING'
    publishers[0] = 0                   # the runner crashed: its latched RUNNING is stale
    assert watcher.status == {}
    watcher._status(SimpleNamespace(data='{"task": "gate", "status": "SUCCEEDED"}'))
    assert watcher.status['status'] == 'SUCCEEDED'   # a final status stays after the runner exits
    assert [s['status'] for s in watcher.statuses] == ['RUNNING', 'SUCCEEDED']   # each one recorded
    assert watcher.statuses[0]['received'] == 1.0
    watcher.reset()
    assert list(watcher.statuses) == []


def test_the_watcher_keeps_a_bounded_window_of_statuses(monkeypatch):
    from types import SimpleNamespace
    mission = _watcher_module(monkeypatch)
    monkeypatch.setattr(mission, 'STATUS_KEEP', 3)
    node = SimpleNamespace(create_subscription=lambda *a: None, count_publishers=lambda topic: 1)
    watcher = mission.TaskWatcher(SimpleNamespace(node=node, monitor=SimpleNamespace(clock=lambda: 1.0)))
    for i in range(5):
        watcher._status(SimpleNamespace(data=f'{{"status": "RUNNING", "phase": "p{i}"}}'))
    assert [s['phase'] for s in watcher.statuses] == ['p2', 'p3', 'p4']   # the newest kept


def test_the_feed_view_is_itself():
    mission, _ = feed()
    assert mission.view() == {'mission': mission}
