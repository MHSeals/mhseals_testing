"""The Config row reports what the thruster driver did with a hardware edit."""

from types import SimpleNamespace

import pytest
import yaml

from mhseals_hardware.configuration import config_sha, load_config, load_defaults
from mhseals_testing.harness import bags
from mhseals_testing.ui.popups import config as config_popup

from test_app import make_app


class Ros:
    """Fake graph whose driver status follows the file the driver would read."""
    fault = ''

    def __init__(self, path):
        self.path, self.up, self.error, self.pending = path, True, '', []

    @property
    def driver_status(self):
        return dict(requested_config_sha=config_sha(load_config(self.path)), error=self.error,
                    pending_restart_keys=self.pending)

    def nodes(self):
        return ['/thruster_pwm_node'] if self.up else []

    def emit(self, *args, **kwargs):
        pass


@pytest.fixture
def rig(tmp_path, monkeypatch):
    operator = tmp_path / 'operator.yaml'
    operator.write_text(yaml.safe_dump(load_defaults()))
    app, _ = make_app(tmp_path, ['--profile', 'crane_full_stack', '--config', str(operator)])
    app.ros = Ros(app.paths.hardware)
    app.arming = SimpleNamespace(armed=False, hardware_config=None,
                                 disarm=lambda: setattr(app.arming, 'armed', False))
    monkeypatch.setattr('mhseals_testing.ui.app.time.sleep', lambda s: None)
    return app, operator


def edit(operator, **changes):
    document = yaml.safe_load(operator.read_text())
    document.update(changes)
    operator.write_text(yaml.safe_dump(document))


def row(app):
    return config_popup.rows(app.watcher, 0)[0][1]


def test_disarmed_edit_is_saved_for_the_next_arm(rig):
    app, operator = rig
    edit(operator, command_timeout=0.9)
    assert app.watcher.reload('hardware').ok
    assert app.args.hardware_config['command_timeout'] == 0.9
    assert app.arming.hardware_config['command_timeout'] == 0.9
    assert row(app).startswith(config_popup.GLYPH['ok'])


def test_armed_edit_is_confirmed_by_the_driver_readback(rig):
    app, operator = rig
    app.arming.armed = True
    edit(operator, command_timeout=0.9)
    assert app.watcher.reload('hardware').ok
    assert row(app).startswith(config_popup.GLYPH['ok'])


def test_space_during_the_driver_wait_disarms(rig):
    app, operator = rig
    app.arming.armed = True
    app.ros.up = False
    app.keys = SimpleNamespace(read=lambda timeout: 'space')
    app.plant = None
    edit(operator, command_timeout=0.9)
    app.watcher.reload('hardware')
    assert not app.arming.armed


def test_driver_refusal_is_a_rejection(rig):
    app, operator = rig
    app.arming.armed = True
    app.ros.pending, app.ros.error = ['channel_map'], 'restart required: channel_map'
    edit(operator, channel_map=[4, 3, 2, 1])
    change = app.watcher.reload('hardware')
    assert not change.ok and 'channel_map' in change.error


def test_dead_driver_is_not_confirmed(rig, monkeypatch):
    app, operator = rig
    app.arming.armed = True
    app.ros.up = False
    ticks = iter(range(100))
    monkeypatch.setattr('time.monotonic', lambda: next(ticks))
    edit(operator, command_timeout=0.9)
    assert app.watcher.reload('hardware').ok
    assert 'did not confirm' in row(app) and config_popup.GLYPH['ok'] not in row(app)


def test_lean_bag_profile_records_driver_status_and_parameter_events():
    topics = bags.load_profiles()['lean']['topics']
    assert '/thruster_pwm_node/status' in topics and '/parameter_events' in topics


@pytest.mark.parametrize('section, key, value', [('controller', 'kp', [0.7, 0.8, 0.6]),
                                                 ('command_mux', 'ramp', 2.0)])
def test_armed_restart_or_idle_section_is_never_reported_live(rig, section, key, value):
    app, operator = rig
    app.arming.armed = True
    assert app.watcher.reload('hardware').ok
    edit(operator, **{section: dict(load_defaults()[section], **{key: value})})
    assert app.watcher.reload('hardware').ok
    text = row(app)
    assert 'live until re-arm' not in text and 'applied' not in text.replace('not applied', '')
    assert config_popup.GLYPH['ok'] not in text
