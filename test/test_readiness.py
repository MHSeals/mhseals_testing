from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile
from mhseals_testing.readiness import BEAT_STALE_S, advisories, liveness_refusal, motion_blockers


def test_a_watchdog_beat_silent_past_its_staleness_refuses_and_one_never_heard_is_left_to_the_graph():
    required = ('/link_watchdog', '/fcu_watchdog', '/fence_watchdog')
    beats = {'/link_watchdog': 10.0, '/fcu_watchdog': 10.0 - BEAT_STALE_S}
    assert liveness_refusal(required, beats, 10.0) == ''                       # exactly at the limit
    beats['/link_watchdog'] = 10.1
    assert liveness_refusal(required, beats, 10.1) == ''           # no run known: never proof
    assert liveness_refusal(required, beats, 10.1, runs={'/link_watchdog': 0.0}) == (
        'safety watchdog silent: /fcu_watchdog 2.1 s (crashed, or the link to the boat is down)')
    assert liveness_refusal(required, beats, 12.2) == ''      # every beat silent: a dropout, not a crash
    assert liveness_refusal(required, {'/fcu_watchdog': 0.0}, 9.0) == ''      # none to compare with


def test_the_first_beat_back_after_a_dropout_is_no_proof_the_others_died():
    required = ('/link_watchdog', '/fcu_watchdog', '/fence_watchdog')
    beats = {node: 100.0 for node in required}
    runs = {node: 0.0 for node in required}
    beats['/link_watchdog'] = runs['/link_watchdog'] = 107.05      # the link is back: one beat so far
    assert liveness_refusal(required, beats, 107.1, runs=runs) == ''
    beats['/link_watchdog'] = 109.0                               # its run lasts; fcu never returns
    beats['/fence_watchdog'], runs['/fence_watchdog'] = 109.0, 107.3
    assert liveness_refusal(required, beats, 109.0, runs=runs) == ''
    assert liveness_refusal(required, beats, 109.1, runs=runs) == (
        'safety watchdog silent: /fcu_watchdog 9.1 s (crashed, or the link to the boat is down)')
    assert liveness_refusal(required, {}, 99.0) == ''                         # an older build: no beat
    assert liveness_refusal(required, {}, 99.0, ['Safety watchdogs on blastoise-odroid']) == (
        'safety watchdogs reported off: Safety watchdogs on blastoise-odroid')


def ready_monitor(profile):
    monitor = BoatMonitor(clock=lambda: 1)
    for sensor in profile.sensors:
        monitor.observe(sensor.name, stamp_age=0)
    for parent, child in profile.required_tf:
        monitor.transform(parent, child, authority=parent)
    return monitor


def test_live_required_sensors_and_tf_allow_motion():
    profile = load_profile('boat')
    snapshot = ready_monitor(profile).snapshot(profile.required_tf)
    assert motion_blockers(snapshot, profile) == []


def test_missing_required_sensor_and_tf_block_but_optional_do_not():
    profile = load_profile('boat')
    monitor = BoatMonitor(clock=lambda: 1)
    optional = [s for s in profile.sensors if not s.required]
    for sensor in optional:
        monitor.observe(sensor.name, stamp_age=0)
    blockers = motion_blockers(monitor.snapshot(profile.required_tf), profile)
    required = [s.name for s in profile.sensors if s.required]
    assert all(any(b.startswith(name) for b in blockers) for name in required)
    assert not any(b.startswith(s.name + ':') for s in optional for b in blockers)
    assert sum(b.startswith('TF ') for b in blockers) == len(profile.required_tf)
    assert motion_blockers(monitor.snapshot(profile.required_tf), profile, bench_override=True) == []


def test_advisories_never_block_and_sort_worst_first():
    profile = load_profile('boat')
    monitor = ready_monitor(profile)
    monitor.advise('battery', 'FAIL', 'below cutoff')
    monitor.advise('clock', 'OK')
    monitor.advise('cpu', 'WARN', '95%')
    snapshot = monitor.snapshot(profile.required_tf)
    assert motion_blockers(snapshot, profile) == []
    assert [name for name, _, _ in advisories(snapshot)] == ['battery', 'cpu', 'clock']


def test_subscribed_advisories_go_stale_but_polled_ones_do_not():
    now = [1]
    monitor = BoatMonitor(clock=lambda: now[0])
    monitor.advise('flight controller', 'OK')
    monitor.advise('battery', 'WARN', 'low')
    monitor.advise('clock', 'OK')
    assert dict((n, l) for n, l, _ in advisories(monitor.snapshot()))['flight controller'] == 'OK'
    now[0] += 5
    rows = {name: (level, detail) for name, level, detail in advisories(monitor.snapshot())}
    assert rows['flight controller'] == ('N/A', 'stale (5s)')
    assert rows['battery'][0] == 'N/A'
    assert rows['clock'][0] == 'OK'
