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


def test_the_feed_view_is_itself():
    mission, _ = feed()
    assert mission.view() == {'mission': mission}
