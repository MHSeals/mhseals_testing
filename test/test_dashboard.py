import io

import pytest
from rich.console import Console

from mhseals_hardware.configuration import load_defaults
from mhseals_hardware.thruster_mixer import THRUSTER_MIXER
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui.dashboard import PANELS, render, short, status_rows


def view(profile, **overrides):
    hardware = load_defaults()
    base = dict(
        focus='status', cursors={panel: 0 for panel in PANELS}, modal=None, running=None,
        activity='results', now=10.0, notice='Ready', position='1/9',
        hints=[('Tab', 'Focus'), ('a', 'Arm'), ('Space', 'Stop'), ('?', 'Help'), ('q', 'Quit')],
        armed=False, arm_label='disarmed', command=(0.0, 0.0, 0.0), profile=profile.name,
        units=profile.command_units, outputs=profile.outputs, specs=profile.sensors,
        host='boat', domain='42', recording=True, bench_override=False, hardware=hardware,
        draft=None, grabbed=None,
        matrix=THRUSTER_MIXER, test_pulses=None, pwm_readback=['—'] * 4,
        config_note='defaults',
        procedures=[dict(key='1', title='Surge trial', description='both ways', reason='')])
    base.update(overrides)
    return base


def monitor():
    m = BoatMonitor(clock=lambda: 10)
    m.observe('odometry', {'vx': .2, 'vy': 0., 'wz': -.1, 'child': 'base_link'}, stamp_age=0)
    m.observe('IMU', issue='IMU orientation unavailable', stamp_age=0)
    m.transform('map', 'odom', static=True, authority='g')
    m.advise('compute', 'WARN', 'load 7.8/20, 78°C, 688 GB free')
    m.record_result(dict(procedure='surge', direction=1, verdict='SIGN MATCH', response=.2))
    m.event('armed')
    return m


def text(renderable, width, height):
    console = Console(file=io.StringIO(), width=width, height=height, color_system=None)
    console.print(renderable)
    return console.file.getvalue()


@pytest.mark.parametrize('width,height', [(80, 24), (120, 40)])
@pytest.mark.parametrize('focus', PANELS)
@pytest.mark.parametrize('name', ['boat', 'crane_full_stack'])
def test_every_focus_fits_and_keeps_stop_visible(width, height, focus, name):
    if focus == 'motion' and width < 100:
        pytest.skip('Motion folds away, and takes no focus, below 100 columns')
    profile = load_profile(name)
    page = text(render(monitor().snapshot(profile.required_tf),
                       view(profile, focus=focus), width, height), width, height)
    lines = page.splitlines()
    assert len(lines) <= height and all(len(line) <= width for line in lines)
    assert 'Space Stop' in page and 'boat_test' in page
    assert '╭─' in page and '┏' not in page  # one border style: rounded
    for title in ('Status', 'Thrusters', 'Procedures', 'Activity'):
        assert title in page
    assert ('─ Motion' in page or '▸ Motion' in page) == (width >= 100)


def test_tf_and_health_live_inside_the_page():
    profile = load_profile('boat')
    page = text(render(monitor().snapshot(profile.required_tf), view(profile), 120, 40), 120, 40)
    assert 'map → odom' in page and 'odom → base_link' in page
    assert 'compute 78°C' in page  # health in the header strip
    assert '× tf' in page            # odom -> base_link is missing


def test_popup_overlays_the_page():
    profile = load_profile('boat')
    modal = dict(kind='choice', title='Arm', message='Real outputs.', options=['Cancel', 'Arm'],
                 keys=['n', 'y'], selected=1, hint='Enter choose')
    page = text(render(monitor().snapshot(), view(profile, modal=modal), 100, 30), 100, 30)
    assert '› y Arm' in page and 'n Cancel' in page and 'Status' in page


def test_status_rows_and_short_values():
    profile = load_profile('boat')
    rows = status_rows(monitor().snapshot(profile.required_tf), profile.sensors)
    assert [row['group'] for row in rows].count('Frames') == 2
    assert next(r for r in rows if r['name'] == 'IMU')['state'] == 'WARN'
    assert short('load 7.8/20, 78°C, 688 GB free') == '78°C'
    assert short('worst GPS +0.04s') == '+0.04s'
    assert short('/bags/session') == ''


def test_drift_arrows_are_drawn_bow_up_in_the_body_frame():
    from mhseals_testing.ui.dashboard import drift_arrow
    assert drift_arrow(0.3, 0.0) == '↑'     # forward
    assert drift_arrow(0.0, 0.3) == '←'     # port
    assert drift_arrow(-0.3, 0.0) == '↓'
    assert drift_arrow(0.0, -0.3) == '→'    # starboard
    assert drift_arrow(0.2, 0.2) == '↖' and drift_arrow(-0.2, -0.2) == '↘'
    assert drift_arrow(0.01, 0.0) == '·'


def drift_text(command, bias, vx=0.1, vy=0.1, yaw=0.0, drift=None, live=None):
    from mhseals_testing.ui.dashboard import drift_line
    m = BoatMonitor(clock=lambda: 10)
    m.observe('odometry', {'vx': vx, 'vy': vy, 'wz': 0.0, 'yaw': yaw}, stamp_age=0)
    if live is not None:
        m.estimate_drift(*live)
    hardware = dict(load_defaults())
    characterization = {}
    if bias is not None:
        characterization['mechanical_bias'] = dict(zip(('surge', 'sway', 'yaw'), bias))
    if drift is not None:
        characterization['drift'] = dict(x=drift[0], y=drift[1], separated=True)
    hardware['characterization'] = characterization
    return drift_line(m.snapshot(), dict(command=command, hardware=hardware)).plain


def test_drift_is_live_with_a_bearing_and_compared_with_the_fit():
    assert drift_text((0, 0, 0), None) == ('Drift  ↖ 0.14 m/s  45° port\n'
                                           '       fit not characterized')
    assert 'stbd' in drift_text((0, 0, 0), None, vx=-0.1, vy=-0.3)
    assert 'ahead' in drift_text((0, 0, 0), None, vx=0.3, vy=0.0)
    agree = drift_text((0, 0, 0), (0.1, 0.12, 0.0))
    assert 'fit ↖ 0.16 m/s' in agree and '● Δ 0.02' in agree
    assert '× Δ' in drift_text((0, 0, 0), (-0.2, 0.0, 0.0))   # fit points the wrong way
    driving = drift_text((0.2, 0, 0), (0.1, 0.1, 0.0))
    assert driving.startswith('Vel    ↖') and 'Δ' not in driving


def test_header_badge_says_armed_or_disarmed():
    profile = load_profile('boat')
    snapshot = monitor().snapshot(profile.required_tf)
    disarmed = text(render(snapshot, view(profile), 120, 40), 120, 40)
    armed = text(render(snapshot, view(profile, armed=True), 120, 40), 120, 40)
    assert disarmed.splitlines()[0].rstrip().endswith('● Disarmed')
    assert armed.splitlines()[0].rstrip().endswith('● Armed')
    assert '● Armed · odroid' in armed  # the footer uses the same indicator


def test_unsaved_mapping_is_marked():
    profile = load_profile('crane_full_stack')
    draft = dict(channel_map=[2, 1, 3, 4], reversed=[True, False, False, False])
    page = text(render(monitor().snapshot(), view(profile, focus='thrusters', draft=draft,
                                                  grabbed=0), 120, 40), 120, 40)
    assert 'topic · unsaved' in page and 'FL ⇅' in page and '↺' in page


def test_world_drift_is_turned_into_the_body_by_the_heading():
    import math
    # A current setting north (+y in odom): heading north, it pushes the boat ahead.
    north = drift_text((0, 0, 0), None, vx=0.2, vy=0.0, yaw=math.pi / 2, drift=(0.0, 0.2))
    assert 'fit ↑ 0.20 m/s' in north and '● Δ 0.00' in north
    # Heading east, the same current pushes the boat to port.
    east = drift_text((0, 0, 0), None, vx=0.0, vy=0.2, yaw=0.0, drift=(0.0, 0.2))
    assert 'fit ← 0.20 m/s' in east
    # The controller's live estimate wins over the characterization.
    live = drift_text((0, 0, 0), None, vx=0.0, vy=0.1, yaw=0.0, drift=(0.0, 0.2), live=(0.0, 0.1))
    assert 'live ← 0.10 m/s' in live
