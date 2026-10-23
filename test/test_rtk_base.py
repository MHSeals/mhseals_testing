"""rtk_base argv, config errors and the status document; no ROS or hardware."""

from types import SimpleNamespace

import pytest
import yaml

from mhseals_testing import rtk
from mhseals_testing.rtk import StatusTracker, main, parse


@pytest.fixture
def conf(tmp_path, monkeypatch):
    path = tmp_path / 'rtk.yaml'
    path.write_text(yaml.safe_dump(dict(device='/dev/ttyACM0', positions={
        'dock': dict(latitude=30.0, longitude=-97.0, height=100.0)})))
    monkeypatch.setenv('MHSEALS_RTK_CONFIG', str(path))
    return path


def test_defaults_resume_with_the_configured_device(conf):
    config, survey, position = parse([])
    assert config['device'] == '/dev/ttyACM0' and not survey and position is None


def test_port_overrides_and_survey_or_known_select_the_mode(conf):
    assert parse(['/dev/ttyUSB1', '--survey'])[0]['device'] == '/dev/ttyUSB1'
    assert parse(['--survey'])[1] is True
    assert parse(['--known', 'dock'])[2]['latitude'] == 30.0
    assert parse(['--resume'])[1:] == (False, None)


def test_config_flag_beats_the_environment(conf, tmp_path):
    other = tmp_path / 'other.yaml'
    other.write_text('device: /dev/other\n')
    assert parse(['--config', str(other)])[0]['device'] == '/dev/other'


def test_unknown_position_and_missing_device_are_errors(conf, tmp_path):
    with pytest.raises(ValueError, match='dock'):
        parse(['--known', 'nowhere'])
    empty = tmp_path / 'empty.yaml'
    empty.write_text('survey_seconds: 60\n')
    with pytest.raises(ValueError, match='no base receiver'):
        parse(['--config', str(empty)])


def test_main_exits_cleanly_on_a_bad_config_and_serves_otherwise(conf, tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('survey_seconds: 0\n')
    with pytest.raises(SystemExit, match='rtk_base: survey duration'):
        main(['--config', str(bad)], serve=lambda *a: None)
    with pytest.raises(SystemExit):
        main(['--survey', '--resume'], serve=lambda *a: None)     # exclusive modes
    seen = []
    main(['--known', 'dock'], serve=lambda *a: seen.append(a))
    assert seen[0][1] is False and seen[0][2]['height'] == 100.0


class FakeSession:
    config = dict(survey_seconds=120, survey_accuracy=2.0)

    def __init__(self, **state):
        self.state = dict(phase='ready', detail='', model='M8P', base_satellites=12, survey=None,
                          base_position=None, received=5, delivered=5, errors=0)
        self.state.update(state)

    def snapshot(self):
        return dict(self.state)


def test_status_has_age_rates_and_survey_progress():
    now = [100.0]
    sink = SimpleNamespace(counts=dict(sent=0, stale=0, oversize=0), bytes=0, last=None)
    session = FakeSession(survey=dict(seconds=60, accuracy=1.0, observations=60, valid=False, active=True))
    session.state['phase'] = 'surveying'
    tracker = StatusTracker(session, sink, pubs=lambda: 1, clock=lambda: now[0])
    first = tracker.status()
    assert first['schema'] == 'mhseals.rtk_status/1' and first['base'] == 'surveying'
    assert first['age'] is None and 0 < first['survey'] < 1 and first['svin']['seconds'] == 60

    session.state['phase'] = 'ready'
    now[0] = 102.0
    sink.counts.update(sent=4, stale=1)
    sink.bytes, sink.last = 800, 101.5
    status = tracker.status()
    assert status['base'] == 'streaming' and status['age'] == 0.5 and status['drops'] == 1
    assert status['rates'] == dict(frames_per_s=2.0, bytes_per_s=400) and status['pubs'] == 1
    now[0] = 170.0
    assert tracker.status()['drops'] == 0            # stale drops age out after 60 s


def test_survey_start_accuracy_is_the_peak_of_the_first_ten_seconds():
    sink = SimpleNamespace(counts=dict(sent=0, stale=0, oversize=0), bytes=0, last=None)
    session = FakeSession()
    tracker = StatusTracker(session, sink)

    def progress(seconds, accuracy):
        session.state['survey'] = dict(seconds=seconds, accuracy=accuracy, observations=seconds, valid=False, active=True)
        return tracker.status()['survey']

    for seconds, accuracy in ((2, 4.0), (6, 8.0), (9, 5.0), (30, 20.0)):
        progress(seconds, accuracy)
    assert tracker.first_accuracy == 8.0             # peak within 10 s; later worse readings do not move a0
    session.state['survey'] = None
    tracker.status()
    assert progress(40, 6.0) is not None and tracker.first_accuracy == 6.0   # resumed past 10 s: first seen


def test_console_script_is_registered():
    import pathlib
    setup = (pathlib.Path(rtk.__file__).parents[1] / 'setup.py').read_text()
    assert "'rtk_base = mhseals_testing.rtk:main'" in setup


def test_status_keys_line_up_with_rtk_view():
    import dataclasses
    from mhseals_testing.analysis import geo
    from mhseals_testing.ui.rtk_row import RtkView
    sink = SimpleNamespace(counts=dict(sent=0, stale=0, oversize=0), bytes=0, last=None)
    pos = dict(latitude=40.0, longitude=-75.0, height=20.0)
    session = FakeSession(survey=dict(seconds=30, accuracy=1.0, observations=30, valid=False, active=True))
    session.state.update(phase='surveying', base_position=pos)
    status = StatusTracker(session, sink).status()
    fields = {f.name for f in dataclasses.fields(RtkView)} & set(status)
    assert {'base', 'age', 'drops', 'pubs', 'survey', 'survey_limit', 'survey_left', 'survey_acc',
            'survey_target'} <= fields
    view = RtkView(**{k: status[k] for k in fields})
    assert isinstance(view.survey, float) and view.survey_left == 90 and view.survey_limit == 'time'
    assert status['base_ecef'] == pytest.approx(geo.ecef(40.0, -75.0, 20.0), abs=1e-3)
    assert set(status['svin']) == {'seconds', 'min_seconds', 'accuracy', 'observations', 'valid', 'active'}
    session.state.update(survey=None, base_position=None)
    idle = StatusTracker(session, sink).status()
    assert idle['survey'] == 0.0 and idle['base_ecef'] is None
