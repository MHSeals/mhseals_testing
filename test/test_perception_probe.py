"""The perception probe: /perception/status and /diagnostics fixtures to perc.<source>.* facts."""
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import Fact
from mhseals_testing.perception.probe import Probe
from mhseals_testing.ros import perception as feed

SETTINGS = feed.settings(user=False)

FIXTURE = Path(__file__).parent / 'fixtures' / 'perception_status.json'


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def status(name, values, level=0, message='', hw='blastoise-jetson'):
    return NS(name=name, level=level, message=message, hardware_id=hw,
              values=[NS(key=k, value=v) for k, v in values])


def replay(probe, clock, messages=None):
    """Feed the recorded arrays at their own times, as the ROS callbacks would."""
    for m in messages or json.loads(FIXTURE.read_text()):
        clock.t = m['t']
        array = NS(status=[status(s['name'], s['values'], s['level'], s['message'], s['hardware_id'])
                           for s in m['status']])
        if m['topic'] == '/perception/status':
            info = {'source_timestamp': int(m['source_ts'] * 1e9), 'received_timestamp': int(m['recv_ts'] * 1e9)}
            feed.observe_status(probe, array, info)
        else:
            feed.observe_diagnostics(probe, array)


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def facts(clock):
    probe = Probe('zed_native', clock, SETTINGS)
    replay(probe, clock)
    return probe.facts()


def test_every_fact_comes_from_the_fixture(facts):
    s = 'perc.zed_native.'
    assert facts[s + 'hz'].value == 14.8
    assert facts[s + 'latency'].value == pytest.approx(0.088)
    assert facts[s + 'latency'].data['p95'] == pytest.approx(0.131)
    assert facts[s + 'clock_offset'].value == pytest.approx(0.012, abs=1e-6)
    assert facts[s + 'inference_ms'].value == pytest.approx(52.0)
    assert facts[s + 'stages'].value == {'camera': {'hz': 14.9, 'proc_ms': 12.0}, 'sdk': {'hz': 14.6, 'proc_ms': 52.0},
                                         'backend': {'hz': 14.8}, 'tracker': {'hz': 10.0}}
    # 2 drops per second against 14.8 frames per second
    assert facts[s + 'funnel_loss'].value == pytest.approx(2 / 16.8)
    assert facts[s + 'funnel_loss'].data['drops'] == {'stale': 10.0, 'reorder': 0.0, 'malformed': 0.0}
    assert facts[s + 'tracker_hz'].value == 10.0
    assert facts[s + 'tracker_latency'].value == pytest.approx(0.112)
    assert facts[s + 'split'].value == 'S12 M1 U2 P8'
    assert facts[s + 'classes'].value == 2 and set(facts[s + 'classes'].data) == {
        'red_buoy', 'green_buoy', 'black_buoy'}
    assert (facts[s + 'class_conf'].value, facts[s + 'class_conf'].label) == (0.68, 'green_buoy')
    assert facts[s + 'heading'].value == 'ok' and facts[s + 'heading'].data['lag_s'] == '0.03'
    assert facts[s + 'costmap_age'].value == pytest.approx(0.21)
    assert facts[s + 'ids'].value == 'true' and facts[s + 'ids'].data['id_assoc_share'] == 0.93


def test_the_shipped_checks_evaluate_on_the_fixture_and_none_is_skipped(facts):
    from mhseals_testing.health.config import load
    from mhseals_testing.profiles import load_profile
    config = load(profile=load_profile('boat'), user=False)
    specs = [replace(s, raise_after=0.0) for s in config.specs  # fusion is off
             if s.group == 'Perception' and not s.id.startswith('perc.lidar.')]
    results = Engine(specs).tick(facts, {}).results
    assert {s.id for s in specs} == set(results)            # nothing skipped
    assert {r.status for i, r in results.items() if i != 'perc.zed_native.split'} == {'ok'}
    for check in ('inference', 'clock', 'funnel'):
        assert results[f'perc.zed_native.{check}'].status == 'ok'


def check(source, fact_name, facts, profile='boat'):
    from mhseals_testing.health.config import load
    from mhseals_testing.profiles import load_profile
    config = load(profile=load_profile(profile), user=False)
    spec = replace(next(s for s in config.specs if s.id == f'perc.{source}.{fact_name}'), raise_after=0.0)
    return Engine([spec]).tick(facts, {}).results[spec.id].status


def test_silent_source_reads_zero_hz_after_the_rate_window(clock):
    probe = Probe('zed_native', clock, SETTINGS)
    assert probe.facts() == {}
    clock.t = 4.0
    assert probe.facts() == {}                              # still inside the window
    clock.t = 5.5
    facts = probe.facts()
    assert set(facts) == {f'perc.zed_native.{n}' for n in ('hz', 'tracker_hz', 'costmap_age')}
    assert facts['perc.zed_native.hz'].value == 0 and facts['perc.zed_native.tracker_hz'].value == 0
    assert facts['perc.zed_native.costmap_age'].value == pytest.approx(5.5)
    for name in ('rate', 'tracker_rate', 'costmap'):
        assert check('zed_native', name, facts) == 'fail'


def test_a_tracker_never_seen_fails_while_the_backend_runs(clock):
    probe = Probe('sim', clock, SETTINGS)
    for t in range(8):
        clock.t = float(t)
        probe.status(status('perception/sim/backend', [('hz', '10')]))
    facts = probe.facts()
    assert facts['perc.sim.hz'].value == 10 and facts['perc.sim.tracker_hz'].value == 0
    assert facts['perc.sim.costmap_age'].value == pytest.approx(7.0)
    assert check('sim', 'tracker_rate', facts, 'crane_full_stack') == 'fail'
    assert check('sim', 'costmap', facts, 'crane_full_stack') == 'fail'


def test_a_source_that_goes_quiet_drops_to_zero_and_its_costmap_ages(clock, facts):
    probe = Probe('zed_native', clock, SETTINGS)
    replay(probe, clock)
    clock.t += 7.0
    facts = probe.facts()
    s = 'perc.zed_native.'
    assert facts[s + 'hz'].value == 0 and facts[s + 'tracker_hz'].value == 0
    for gone in ('latency', 'clock_offset', 'inference_ms', 'funnel_loss', 'heading', 'split'):
        assert s + gone not in facts
    assert facts[s + 'costmap_age'].value > 1.0
    assert check('zed_native', 'costmap', facts) == 'fail'
    assert check('zed_native', 'tracker_rate', facts) == 'fail'


def test_costmap_age_holds_through_the_status_period_then_grows(clock):
    probe = Probe('sim', clock, SETTINGS)
    probe.status(status('perception/sim/tracker', [('grid_age_s', '0.2'), ('hz', '10')]))
    clock.t = 1.2
    assert probe.facts()['perc.sim.costmap_age'].value == pytest.approx(0.2)
    clock.t = 2.5
    assert probe.facts()['perc.sim.costmap_age'].value == pytest.approx(1.2)


def test_missing_keys_leave_their_facts_out(clock):
    probe = Probe('sim', clock, SETTINGS)
    probe.status(status('perception/sim/backend', [('hz', '9.5')]))
    probe.status(status('perception/sim/tracker', [('hz', 'nan')]))
    probe.status(status('perception/sim/inject', [('case', 'none')]))
    probe.status(status('not/a/perception/stage', [('hz', '1')]))
    assert set(probe.facts()) == {'perc.sim.hz', 'perc.sim.stages'}


def test_heading_levels_map_to_pass_warn_fail(clock):
    def level(n, message='lag'):
        probe = Probe('zed_native', clock, SETTINGS)
        probe.status(status('perception/zed_native/heading', [('lag_s', '0.2'), ('rho', '0.8')], n, message))
        return probe.facts()
    assert check('zed_native', 'heading', level(0)) == 'ok'
    assert check('zed_native', 'heading', level(1)) == 'warn'
    error = level(2, 'heading lags 0.20 s')
    assert error['perc.zed_native.heading'].value == 'error'
    assert check('zed_native', 'heading', error) == 'fail'


def test_fusion_stage_rows_and_a_dead_fusion_node_fails(clock):
    """The lidar_fusion stage feeds its four rows; silence after it fails."""
    probe = Probe('zed_native', clock, SETTINGS)
    values = [('hz', '14.5'), ('lidar_share', '0.62'), ('proc_ms_p95', '4.4'),
              ('extrinsic_deg', '0.10 0.20 0.00')]
    probe.status(status('perception/lidar/fusion', values))
    facts = probe.facts()
    assert {k for k in facts if k.startswith('perc.lidar.')} == {
        f'perc.lidar.fusion_{n}' for n in ('hz', 'share', 'p95_ms', 'state')}
    for row in ('fusion_rate', 'fusion_share', 'fusion_p95', 'fusion_state'):
        assert check('lidar', row, facts) == 'ok'
    probe.status(status('perception/lidar/fusion', values, 2, 'beyond 1.5 deg'))
    assert check('lidar', 'fusion_state', probe.facts()) == 'fail'
    clock.t = 7.0
    facts = probe.facts()
    assert facts['perc.lidar.fusion_hz'].value == 0
    assert 'perc.lidar.fusion_state' not in facts
    assert check('lidar', 'fusion_rate', facts) == 'fail'


def test_badly_clocked_or_lossy_camera_fails(clock):
    messages = json.loads(FIXTURE.read_text())
    for m in messages:
        if m['topic'] == '/perception/status':
            m['source_ts'] += 0.2                            # Orin clock 0.2 s ahead
            for s in m['status']:
                if s['name'].endswith('backend'):
                    s['values'] = [[k, str(int(m['t']) * 40) if k == 'drop_stale' else v] for k, v in s['values']]
        else:
            m['status'][0]['values'][3] = ['Object detection', 'Processing Time: 0.2 sec']
    probe = Probe('zed_native', clock, SETTINGS)
    replay(probe, clock, messages)
    facts = probe.facts()
    assert check('zed_native', 'clock', facts) == 'fail'
    assert check('zed_native', 'funnel', facts) == 'warn'          # loss and slow inference only warn
    assert check('zed_native', 'inference', facts) == 'warn'


def test_backend_restart_resets_the_funnel(clock):
    probe = Probe('sim', clock, SETTINGS)
    for t, drops in ((0, 50), (1, 60), (2, 0), (3, 1)):
        clock.t = t
        probe.status(status('perception/sim/backend', [('hz', '10'), ('drop_stale', str(drops))]))
    assert probe.facts()['perc.sim.funnel_loss'].value == pytest.approx(1 / 11)


def test_weak_class_warns(clock):
    probe = Probe('sim', clock, SETTINGS)
    probe.status(status('perception/sim/tracker', [('conf_red_buoy', '0.3'), ('n_red_buoy', '2')]))
    facts = probe.facts()
    assert check('sim', 'class_conf', facts, 'crane_full_stack') == 'warn'
    assert facts['perc.sim.class_conf'] == Fact(0.3, 'red_buoy', {}, 0.0)


def test_the_yaml_timings_take_effect(tmp_path, clock):
    from mhseals_testing.health.config import HEALTH_DIR
    text = (HEALTH_DIR / 'perception.yaml').read_text()
    assert 'silence_s: 5.0, status_grace_s: 1.5' in text
    (tmp_path / 'perception.yaml').write_text(text.replace('silence_s: 5.0, status_grace_s: 1.5',
                                                           'silence_s: 2.0, status_grace_s: 0.5'))
    probe = Probe('sim', clock, feed.settings(directory=tmp_path, user=False))
    probe.status(status('perception/sim/tracker', [('grid_age_s', '0.2'), ('hz', '10')]))
    clock.t = 1.0
    assert probe.facts()['perc.sim.costmap_age'].value == pytest.approx(0.7)   # grace 0.5, not 1.5
    clock.t = 2.5
    assert probe.facts()['perc.sim.hz'].value == 0                              # silent after 2 s, not 5
