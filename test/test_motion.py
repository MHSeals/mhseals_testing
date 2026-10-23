"""Motion plots: command history and gyro in the monitor, braille axes at 120 and 80 columns."""

import importlib

import pytest
import yaml

from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import PLOT_SCALE, PROFILE_DIR, load_profile, parse_profile
from mhseals_testing.ui import layout
from mhseals_testing.ui.panels.motion import motion_panel
from test_dashboard import text, view

BRAILLE = {chr(c) for c in range(0x2801, 0x2900)}


def driven(vx=0.2, wz=0.1, command=(0.25, 0.0, 0.1), seconds=10, gyro_issue=''):
    """A monitor with ``seconds`` of 20 Hz odometry, gyro and command samples."""
    now = [0.0]
    m = BoatMonitor(clock=lambda: now[0], history_sensors=('odometry', 'IMU'), gyro_sensor='IMU')
    for _ in range(seconds * 20):
        now[0] += 0.05
        m.observe('odometry', {'vx': vx, 'vy': 0.0, 'wz': 9.0}, stamp_age=0)  # odometry wz is not the plot's
        m.observe('IMU', {'wz': wz}, issue=gyro_issue, stamp_age=0)
        m.command(command)
    return m


def page(m, width, height, **overrides):
    profile = load_profile('boat')
    return text(layout.render(m.snapshot(profile.required_tf), view(profile, **overrides), width, height),
                width, height)


def test_monitor_windows_the_plot_series_and_takes_yaw_from_the_gyro():
    plots = driven(seconds=15).snapshot()['plots']
    assert plots['window'] == 10.0 and plots['meas']['wz'][0][0] >= plots['now'] - 10.0
    assert {v for _, v in plots['meas']['wz']} == {0.1}    # not odometry's 9.0
    assert plots['cmd'][-1][1] == (0.25, 0.0, 0.1) and plots['cmd'][0][0] >= plots['now'] - 10.0


def test_gyro_samples_with_an_orientation_issue_still_plot():
    plots = driven(gyro_issue='IMU orientation unavailable').snapshot()['plots']
    assert plots['meas']['wz'] and plots['meas']['vx']


@pytest.mark.parametrize('width,height', [(120, 40), (106, 34)])
def test_drive_page_draws_three_braille_axes(width, height):
    out = page(driven(), width, height, units='effort', command=(0.25, 0.0, 0.1))
    rows = [line for line in out.splitlines() if ('meas' in line and 'cmd' in line) or ' → ' in line]
    for axis in ('Surge', 'Sway', 'Yaw'):
        assert any(axis in line for line in out.splitlines()), axis
    assert BRAILLE & set(out) and '…' not in out.split('Procedures')[0]
    assert any('meas +0.20' in line or '→ +0.20' in line for line in out.splitlines())
    assert rows


@pytest.mark.parametrize('width,rows', [(46, 4), (50, 1)])
def test_motion_panel_at_the_80_column_stack_and_the_wide_form(width, rows):
    """Motion is 50 wide in the 80-column stack (compact) and 46 in the tall 120-column form."""
    m = driven()
    snapshot = m.snapshot()
    height = 20 if rows == 4 else 10
    out = text(motion_panel(snapshot, view(load_profile('boat'), units='velocity', command=(0.25, 0, 0.1)),
                            height), width, height)
    lines = out.splitlines()
    assert max(len(line) for line in lines) <= width and '…' not in out
    assert any('Yaw rad/s gyro  cmd +0.10  meas +0.10' in line for line in lines)
    assert ('+0.6' in out and '-0.6' in out and '-10s' in out) == (rows == 4)
    assert sum(any(ch in BRAILLE for ch in line) for line in lines) >= 3


def test_clipped_samples_mark_the_label():
    out = page(driven(vx=1.5), 120, 40)
    surge = next(line for line in out.splitlines() if 'Surge' in line)
    assert '▲' in surge and '→ +1.50' in surge
    assert '▲' not in next(line for line in out.splitlines() if 'Sway' in line)


def test_narrow_panel_uses_the_short_header_and_nothing_cut():
    out = text(motion_panel(driven().snapshot(), view(load_profile('boat'), units='effort',
                                                       command=(0.6, 0, 0)), 10), 36, 10)
    assert 'Surge  +60% → +0.20' in out and 'cmd' not in out and '…' not in out


def test_no_samples_reads_dashes_and_draws_nothing():
    out = text(motion_panel(BoatMonitor().snapshot(), view(load_profile('boat')), 10), 50, 10)
    assert 'meas —' in out and not BRAILLE & set(out)


def test_plot_scale_defaults_and_profile_override():
    assert load_profile('boat').plot_scale == PLOT_SCALE == (0.6, 0.3, 0.5)
    data = yaml.safe_load((PROFILE_DIR / 'boat.yaml').read_text())
    assert parse_profile({**data, 'plot_scale': [1, 2, 3]}).plot_scale == (1.0, 2.0, 3.0)
    with pytest.raises(ValueError, match='plot_scale'):
        parse_profile({**data, 'plot_scale': [1, 0, 3]})
    m = driven(vx=0.5)
    wide = text(motion_panel(m.snapshot(), view(load_profile('boat'), plot_scale=(0.3, 0.3, 0.5)), 10), 50, 10)
    assert '▲' in wide


def test_zero_label_sits_on_the_zero_line_row():
    out = text(motion_panel(driven().snapshot(), view(load_profile('boat'), units='velocity'), 20), 46, 20)
    surge = out.splitlines()[1:6]    # frame row, header, then the four braille rows
    zero = next(i for i, line in enumerate(surge) if line.lstrip('│ ').startswith('0 '))
    assert zero == 3 and '+0.6' in surge[1]


def test_two_decimal_scale_widens_the_gutter_without_wrapping():
    out = text(motion_panel(driven().snapshot(), view(load_profile('boat'), plot_scale=(0.25, 0.25, 0.25)),
                            20), 46, 20)
    lines = out.splitlines()
    assert '+0.25' in out and max(len(line) for line in lines) <= 46 and len(lines) == 18   # 16 rows of plot and legend


def test_dashboard_shim_is_gone():
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module('mhseals_testing.ui.dashboard')
