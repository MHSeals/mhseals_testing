import io

import pytest

from rich.console import Console
from mhseals_hardware.thruster_mixer import THRUSTER_MIXER
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui.dashboard import VIEWS, render_dashboard


def render(context, width=100, height=32, monitor=None):
    stream = io.StringIO()
    console = Console(file=stream, width=width, height=height, color_system=None)
    console.print(render_dashboard((monitor or BoatMonitor()).snapshot(), context, width, height))
    return stream.getvalue()


@pytest.mark.parametrize('width,height', [(80, 24), (120, 40)])
@pytest.mark.parametrize('view', VIEWS)
@pytest.mark.parametrize('profile', ['boat', 'crane_direct'])
def test_all_views_keep_safety_controls_visible(width, height, view, profile):
    profile = load_profile(profile)
    monitor = BoatMonitor()
    monitor.observe('odometry', {'vx': .2, 'vy': 0., 'wz': -.1, 'child': 'base_link'})
    monitor.advise('battery', 'WARN', '14.1 V')
    monitor.record_result(dict(axis='surge', direction=1, verdict='SIGN MATCH', samples=30,
                               response=.2))
    context = dict(view=view, specs=profile.sensors, profile=profile.name,
                   units=profile.command_units, outputs=profile.outputs,
                   channel_map=[2, 1, 4, 3], matrix=THRUSTER_MIXER,
                   chips=['/sys/pwm/one', '/sys/pwm/two', '/sys/pwm/three', '/sys/pwm/four']
                   if profile.outputs == 'odroid' else None,
                   channels=[0] * 4, host='test-boat', domain='42', notice='Ready')
    text = render(context, width, height, monitor)
    assert 'DISARMED' in text and 'Space STOP' in text and 'Q quit' in text
    assert len(text.splitlines()) <= height
    assert all(len(line) <= width for line in text.splitlines())
    if view == 'overview':
        assert 'Measured' in text and 'BOW' in text
    if view == 'health':
        assert 'battery' in text and '14.1 V' in text


def test_manual_view_does_not_advertise_inactive_dashboard_keys():
    text = render(dict(manual=True, specs=(), matrix=THRUSTER_MIXER))
    assert 'M/X/Esc return' in text
    assert 'H arm/disarm' not in text
