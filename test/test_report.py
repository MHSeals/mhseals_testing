import json
from datetime import datetime, timezone

import pytest

from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile
from mhseals_testing.report import build_report, write_report


def test_report_round_trips_and_refuses_nan(tmp_path):
    monitor = BoatMonitor(clock=lambda: 2)
    monitor.event('trial start')
    monitor.record_result(dict(procedure='axis', verdict='SIGN MATCH'))
    report = build_report(load_profile('boat'), monitor.snapshot(), bag=tmp_path / 'bag',
                          now=datetime(2026, 1, 1, tzinfo=timezone.utc))
    path = write_report(report, tmp_path / 'out' / 'session.json')
    loaded = json.loads(path.read_text())
    assert loaded['profile']['name'] == 'boat'
    assert loaded['results'][0]['verdict'] == 'SIGN MATCH'
    assert loaded['events'] == [dict(time=2, message='trial start')]
    report['results'].append({'response': float('nan')})
    with pytest.raises(ValueError):
        write_report(report, path)
    assert json.loads(path.read_text()) == loaded  # previous report intact
    assert [p.name for p in path.parent.iterdir()] == ['session.json']
