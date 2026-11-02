"""LiveStaging: C on the running driver over SetParameters, verified by GetParameters (contract C4)."""
import pytest
from mhseals_hardware.configuration import load_config, update_config
from mhseals_hardware.menu import Cancelled

from mhseals_testing.harness.apply import IDENTITY, LiveStaging
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.procedures.characterize import run
from test_app import arm, make_app

DEPLOYED = [1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, -0.2, 1.0]


class Driver:
    """thruster_pwm_node's parameters: set atomically or not at all, read back as stored."""

    def __init__(self, refuse='', drift=0.0):
        self.values, self.refuse, self.drift, self.sets = {}, refuse, drift, []

    def set_atomically(self, node, values):
        self.sets.append((node, values))
        if self.refuse:
            return False, self.refuse
        self.values.update({k: [v + self.drift for v in vs] for k, vs in values.items()})
        return True, ''

    def get(self, node, names):
        return {name: self.values[name] for name in names if name in self.values}


@pytest.fixture
def config(tmp_path):
    path = tmp_path / 'hardware.yaml'
    update_config({'decoupling': DEPLOYED}, path)
    return path


def staging(config, driver):
    return LiveStaging(config, config.parent / 'state.json', params=lambda: driver)


def test_a_staged_run_drives_identity_live_and_restores_the_deployed_c(config):
    driver = Driver()
    live = staging(config, driver)
    before = config.read_bytes()
    with live.staged():
        assert driver.values['decoupling'] == IDENTITY and live.blockers() == []   # its own run is not blocked
        assert config.read_bytes() == before            # the YAML keeps the deployed C
    assert driver.values['decoupling'] == DEPLOYED and live.pending() is None


def test_commit_saves_the_yaml_and_applies_it_live(config):
    driver, fitted = Driver(), [1.0, 0.05, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    live = staging(config, driver)
    with live.staged():
        live.commit(fitted, None)
    assert load_config(config)['decoupling'] == fitted
    assert driver.values == {'decoupling': fitted, 'decoupling_negative': fitted} and live.pending() is None


@pytest.mark.parametrize('driver, says', [(Driver(refuse='out of bounds'), 'refused C: out of bounds'),
                                          (Driver(drift=0.5), 'reads back')])
def test_a_refused_or_misread_stage_cancels_and_leaves_no_backup(config, driver, says):
    live = staging(config, driver)
    with pytest.raises(Cancelled, match=says):
        live.raw_correction()
    assert live.pending() is None and not live.active


def test_no_ros_link_refuses_before_anything_moves(config):
    live = LiveStaging(config, config.parent / 'state.json', params=lambda: None)
    with pytest.raises(Cancelled, match='no ROS link'):
        live.raw_correction()


def test_a_restore_the_driver_misses_stays_a_blocker_until_recovered(config):
    driver = Driver()
    live = staging(config, driver)
    with live.staged():
        driver.refuse = 'gone'
    assert live.pending() and 'gone' in live.blockers()[0]
    restarted = staging(config, Driver())
    assert restarted.blockers()                       # durable across a restart
    assert restarted.recover() and restarted.pending() is None   # the YAML still holds the backup


def test_the_procedure_is_the_survey():
    assert next(p for p in PROCEDURES if p.key == 'f').run is run


def test_characterize_without_a_driver_link_cancels_and_writes_nothing(tmp_path):
    app, _ = make_app(tmp_path)
    before = (tmp_path / 'hardware.yaml').read_bytes()
    with app.attached():
        arm(app)
        app.dispatch('f')
    assert 'no ROS link' in app.notice or 'cancelled' in app.notice
    assert (tmp_path / 'hardware.yaml').read_bytes() == before and app.staging().pending() is None


def test_the_progress_line_shows_bar_step_command_and_measured_velocity():
    from mhseals_testing.runner import Runner
    from mhseals_testing.ui import progress
    runner = Runner(None, lambda: [], None)
    runner.progress(3, 12, 'sway+')
    text = progress.line(runner.place, (0.0, 0.4, 0.0), (0.01, 0.31, -0.02), 2.5)
    assert text.startswith('▕███') and ' 25% 3/12 sway+' in text
    assert 'cmd +0.00 +0.40 +0.00' in text and 'vel +0.01 +0.31 -0.02' in text and text.endswith('2.5s')
    assert 'vel —' in progress.line((0, 0, ''), (0, 0, 0), None, 0.0)


pytestmark = pytest.mark.smoke   # tools/smoke
