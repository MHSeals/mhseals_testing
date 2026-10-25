"""popup.inspect: goldens for the sensor, TF, compass and GPS views, the sparks, and Enter on a row."""
import io
import math
from pathlib import Path

import pytest
from rich.console import Console, Group

from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout, sparks, theme
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import inspect

GOLDEN = Path(__file__).parent / 'golden' / 'inspect'
needs_unicode = pytest.mark.skipif(theme.ASCII, reason='goldens are the Unicode glyph set')
SPECS = load_profile('boat').sensors
MAG = 470e-7   # the site field in tesla


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def scene(opened=None):
    """70 s of a boat at rest heading about 247°: a 20 Hz IMU with one 3 s dropout, 5 Hz RTK float
    GPS raw wandering a few cm, a compass, odometry and a TF tree with a second map->odom publisher.
    ``opened(monitor, now)`` runs every 0.25 s from t = 5 s, as the popup refresh does while open."""
    clock = Clock()
    monitor = BoatMonitor(clock=clock, deadlines={s.name: s.deadline for s in SPECS if s.deadline})
    heading = math.radians(247 - 2.8)        # magnetic
    yaw = math.radians(90 - 249)             # ENU
    t = 0.0
    while t < 70.0:
        clock.t = 1000.0 + t
        step = round(t * 100)
        if step % 5 == 0 and not 30 <= t < 33:
            monitor.observe('IMU', dict(wz=0.002 * math.sin(t), summary='wz'), stamp_age=0.012)
        if step % 20 == 0:
            k = step // 20
            monitor.observe('GPS raw', dict(fix=5, sats=17, h_acc=0.06, vel_acc=0.03, speed=0.02, cog=None,
                                            lat=42.0 + 3e-7 * math.sin(k * 0.7), lon=-71.0 + 4e-7 * math.cos(k * 1.3),
                                            alt=12.0, summary='float'), stamp_age=0.05)
            monitor.sensors['GPS raw']['values']['h_acc'] = 0.06 + 0.015 * math.sin(k / 40)
            monitor.observe('compass', dict(x=MAG * 0.5 * math.cos(heading), y=MAG * 0.5 * math.sin(heading),
                                            z=-MAG * 0.86, compass_gyro=1.5, magcal_residual=3.0, yaw_sigma=0.05))
            monitor.observe('odometry', dict(vx=0.0, vy=0.0, wz=0.0, x=1.0, y=2.0, yaw=yaw, summary='odom'))
            monitor.transform('map', 'odom', 0.05, authority='ekf_global')
            monitor.transform('odom', 'base_link', 0.02, authority='ekf_local')
            if t > 60:
                monitor.transform('map', 'odom', 0.04, authority='rogue_node')
        if opened and t >= 5 and step % 25 == 0:
            opened(monitor, clock.t)
        t += 0.01
    monitor.transform('base_link', 'lidar_link', 0.0, static=True, authority='robot_state_publisher')
    return monitor, clock


def popup_for(name, source=None, publishers=('mavros',), compact=False):
    row = dict(name=name, source=source, group='')
    state = inspect.Inspect(row, SPECS, publishers=list(publishers), compact=compact,
                            fix_changes=[(5000.0 - 252, 5)], wall=lambda: 5000.0)

    def opened(monitor, now):
        state.monitor = monitor
        state.update(monitor.snapshot(), now)
    monitor, clock = scene(opened)
    snapshot = monitor.snapshot()
    state.update(snapshot, clock())
    row = next(r for r in status_rows(snapshot, SPECS) if r['name'] == name and r.get('source') == source)
    edges = monitor.tf_edges() if source == 'tf' else ()
    modal = dict(kind='message', title=inspect.title(row, state.watched() if source == 'sensor' else None),
                 width=inspect.WIDTH, hint='Any key closes', body=inspect.body(state, snapshot, inspect.WIDTH - 4, edges))
    return modal


def paint(modal, cols=120, rows=40):
    panel, width = layout.popup(modal, cols, rows)
    out = io.StringIO()
    Console(width=width, file=out, color_system=None).print(panel)
    return out.getvalue()


CASES = {'imu': ('IMU', 'sensor'), 'tf': (f'map {theme.ARROW} odom', 'tf'), 'compass': ('Yaw', None),
         'gps': ('GPS', None)}


@needs_unicode
@pytest.mark.parametrize('case', sorted(CASES))
def test_golden(case, request):
    out = paint(popup_for(*CASES[case]))
    path = GOLDEN / f'{case}_120x40.txt'
    if request.config.getoption('--update-goldens'):
        GOLDEN.mkdir(exist_ok=True)
        path.write_text(out)
    assert out == path.read_text()


def test_sensor_strip_marks_the_dropout_and_reports_rate_and_publishers():
    out = paint(popup_for('IMU', 'sensor'))
    assert '20 Hz · 0–20' in out and 'max gap 3.0' in out and 'expected 20 Hz' in out
    assert theme.pick('┄', '.') in out and '1 publisher (mavros)' in out


def test_tf_diagram_flags_two_publishers_and_names_ages():
    out = paint(popup_for(f'map {theme.ARROW} odom', 'tf'))
    assert '! 2 publishers' in out and 'rogue_node' in out and 'base_link' not in out.split('\n')[1]


def test_tf_missing_chain_is_crossed():
    lines = inspect.tf_part([], 'map', 'odom', 72)
    assert 'odom' in lines[0].plain and theme.pick('✗', 'x') in lines[0].plain


def test_tf_missing_hop_keeps_the_partial_chain_and_names_the_expected_publisher():
    edge = dict(authority='', seen=0.1, stamp_age=0.02, static=False, fresh=True)
    edges = [dict(edge, parent='map', child='odom', authority='ekf_global'),
             dict(edge, parent='imu_link', child='lidar_link', authority='robot_state_publisher',
                  seen=41.0, fresh=False)]
    line, who, age = (t.plain for t in inspect.tf_part(edges, 'map', 'lidar_link', 72)[:3])
    assert line.split() == ['map', theme.pick('━━▶', '-->'), 'odom', theme.pick('╌✗╌', '-x-'), 'lidar_link']
    assert 'ekf_global' in who and 'expected: robot_state_publisher' in who and 'last seen 41 s ago' in age


def test_advisory_strip_follows_the_detail_while_open():
    monitor = BoatMonitor(clock=Clock())
    state = inspect.Inspect(dict(name='compute', source=None, group='System'), SPECS, monitor)
    for t, temp in ((0, 60), (1, 70), (2, 80)):
        monitor.clock.t = 1000.0 + t
        monitor.advise('compute', 'OK', f'load 1.0/8, {temp}°C')
        state.update(monitor.snapshot(), monitor.clock())
    assert [v for _, v in state.series['value']] == [60, 70, 80]


def test_compass_reads_heading_cardinal_and_yaw():
    out = paint(popup_for('Yaw'))
    assert '247° WSW' in out and '249° WSW' in out and 'stationary' in out and '470 mG' in out


def test_gps_shows_accuracy_in_cm_ladder_and_time_at_level():
    out = paint(popup_for('GPS'))
    assert '7 cm' in out and 'RTK float' in out and 'RTK float for 4 min 12 s' in out and 'CEP50' in out


def test_lengths_and_bearings_read_naturally():
    assert [inspect.length(m) for m in (0.001, 0.004, 0.02, 0.0996, 0.15, 0.4, 9.96, 12.3)] == \
        ['1 cm', '1 cm', '2 cm', '0.1 m', '0.15 m', '0.4 m', '10 m', '12 m']
    assert inspect.duration(252) == '4 min 12 s' and inspect.duration(3720) == '1 h 02 min'
    assert inspect.cardinal(247) == 'WSW' and inspect.cardinal(359) == 'N'
    assert inspect.nice(0.07) == 0.1 and inspect.nice(3) == 5


def test_compact_fits_80x24_by_scrolling():
    modal = popup_for('GPS', compact=True)
    paint(modal, 80, 24)
    assert modal['clipped']


def test_sparks_shapes():
    rows = sparks.spark([0, 1, 2, None, 4], 5, 0, 4, band=(1, 3), gaps=[0, 0, 0, 1, 0])
    assert len(rows) == 2 and all(r.cell_len == 5 for r in rows)
    assert sparks.hbar(5, 0, 10, 8).plain.startswith(theme.FILL * 4)
    assert 'FIXED' in sparks.ladder(('none', 'FIXED'), 'FIXED').plain


def test_enter_on_a_plain_row_opens_inspect(tmp_path):
    from test_app import make_app
    app, _ = make_app(tmp_path)
    seen = []
    app.popup = lambda modal, handle, refresh=None: seen.append(modal)
    app.focus = 'status'
    rows = status_rows(app.snapshot(), app.profile.sensors)
    app.cursors['status'] = next(i for i, r in enumerate(rows)
                                 if r.get('source') == 'sensor' and r['name'] != 'odometry')
    app.on_status('enter')
    assert seen and seen[0]['body'] is not None and 'Detail' in paint(seen[0])


def test_gps_reads_the_shared_map_accuracy_without_gps_raw():
    """No GPSRAW sensor: the navsat covariance reading of ``MapState.accuracy()`` still draws the circle."""
    specs = [s for s in SPECS if s.kind != 'gpsraw']
    monitor = BoatMonitor(clock=Clock())
    state = inspect.Inspect(dict(name='GPS', source=None), specs, monitor)
    state.accuracy = lambda: (0.4, '3d')
    lines = inspect.gps_part(state, monitor.snapshot(), 72)
    out = paint(dict(kind='message', title='GPS', width=76, body=Group(*lines)))
    assert '0.4 m' in out and '3D' in out
