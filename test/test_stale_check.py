"""Session close records tracking; the calibration probe turns the stored verdict into calib.stale."""

import json

from mhseals_testing import calibration
from mhseals_testing.analysis.tracking import load_settings
from mhseals_testing.health.config import load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.probes import staleness_facts
from mhseals_testing.ui.feeds.tracking import TrackingFeed


def close_sessions(path, means):
    verdict = {}
    for mean in means:
        feed = TrackingFeed(load_settings(), path)
        for score in (mean, mean):
            feed.add('surge', score, 0.9, 4)
        verdict = feed.close()['surge']
    return verdict


def status(path):
    spec = next(s for s in load(user=False).specs if s.id == 'calib.stale')
    facts = staleness_facts(calibration.load(path))
    return spec, Engine([spec]).tick(facts, {}).results.get('calib.stale')


def test_three_falling_sessions_flag_calib_stale_as_info(tmp_path):
    path = tmp_path / 'current.json'
    assert close_sessions(path, [0.6, 0.55, 0.5])['flag']
    spec, result = status(path)
    assert spec.severity == 'info' and spec.missing == 'skip'
    assert result.status == 'info' and 'surge' in result.detail + str(result.data)
    assert json.loads(path.read_text())['provenance']['tracking']['surge']['sessions'][-1]['n'] == 2


def test_one_bad_session_never_flags(tmp_path):
    path = tmp_path / 'current.json'
    assert not close_sessions(path, [0.9, 0.9, 0.3])['flag']
    assert status(path)[1].status == 'ok'


def test_no_recorded_sessions_publish_nothing_and_close_writes_nothing(tmp_path):
    assert staleness_facts({}) == {}
    path = tmp_path / 'current.json'
    assert TrackingFeed(load_settings(), path).close() == {} and not path.exists()


def test_recharacterization_clears_the_flag(tmp_path):
    path = tmp_path / 'current.json'
    assert close_sessions(path, [0.6, 0.55, 0.5])['flag'] and status(path)[1].status == 'info'
    calibration.record('d1', {'id': 'F2', 'config': 'C1'}, {}, path=path)
    assert status(path)[1] is None      # no verdict for the new fit: skipped, not raised
    assert len(calibration.load(path)['provenance']['tracking']['surge']['sessions']) == 3


def test_malformed_files_publish_nothing():
    for current in ([], {'provenance': []}, {'provenance': {'tracking': []}},
                    {'provenance': {'tracking': {'surge': [], 'sway': {'staleness': 3}}}}):
        assert staleness_facts(current) == {}


def test_health_tick_survives_a_malformed_current_json(tmp_path, monkeypatch):
    from mhseals_testing.health.advisories import Health
    monkeypatch.setattr(calibration, 'load', lambda: [])
    health = Health(None, tmp_path, host=lambda: {})
    snapshot = {'sensors': {'imu': {'status': 'OK', 'stamp_age': 0.2}}}
    health.tick(snapshot)
    facts, probes = health.facts.read()
    assert facts['clock.stamp_offset'].value == 0.2 and 'calibration.stale' not in facts
