from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile
from mhseals_testing.readiness import advisories, motion_blockers


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
