"""boat_agent set across layers: the hardware YAML (whole keys, mapping leaves, matrix rows), reads, readback,
snapshots, the armed-unsafe refusal and the structured settings ask (agent/ask.py, agent/layers.py)."""

import pytest
import yaml

from mhseals_hardware.configuration import load_config, update_config

from mhseals_testing.agent import ask as ask_module, layers
from mhseals_testing.agent.ask import assignments, reads_only

from test_agent_desk import ask, claim, fence_topic, final, rig   # noqa: F401 - the desk rig and its fixtures

pytestmark = pytest.mark.smoke

YAW_ROW = [-0.0375, 0.1, 1.0]


@pytest.fixture
def hw(rig, tmp_path):
    """The rig with an operator hardware YAML and the TUI's save path (update_config, as save_hardware does)."""
    path = tmp_path / 'hardware.yaml'
    update_config({'manual_amplitude': 0.75}, path)
    rig.app.args.config = str(path)
    saved = []

    def save_hardware(**changes):
        saved.append(changes)
        update_config(changes, path)
        return True
    rig.app.save_hardware = save_hardware
    rig.app.is_armed = False
    rig.saved, rig.path = saved, path
    return rig


def test_the_decoupling_yaw_row_is_set_alone_validated_and_read_back(hw):
    c = claim(hw)
    reply = final(ask(hw, 'set', c, settings={'decoupling.yaw': '[-0.0375, 0.1, 1.0]'})['id'])
    assert reply['ok'], reply
    row = reply['result']['hardware'][0]
    assert row['old'] == [0.0, 0.0, 1.0] and row['value'] == YAW_ROW and row['layer'] == 'hardware'
    assert load_config(hw.path)['decoupling'] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, *YAW_ROW]
    assert 'decoupling.yaw=[0.0, 0.0, 1.0]' in row['revert'] and row['snapshot']
    assert yaml.safe_load(open(row['snapshot']))['manual_amplitude'] == 0.75      # the file before the write
    assert [f['key'] for p, f in hw.app.events if p == 'setting'] == ['decoupling.yaw']


def test_manual_amplitude_and_a_mapping_leaf_write_only_overrides(hw):
    c = claim(hw)
    reply = final(ask(hw, 'set', c, settings={'manual_amplitude': '0.25', 'command_mux.ramp': '0.5'})['id'])
    assert reply['ok'], reply
    assert hw.saved == [{'manual_amplitude': 0.25}, {'command_mux': {'ramp': 0.5}}]
    assert load_config(hw.path)['command_mux']['ramp'] == 0.5


@pytest.mark.parametrize('settings,hint', [
    ({'decoupling.yaw': '[1, 2]'}, 'a row is 3 numbers'), ({'decoupling.heave': '[0, 0, 1]'}, 'has no'),
    ({'manual_amplitude': '3'}, 'manual_amplitude'), ({'decoupling': '[1, 0, 0, 0, 1, 0, 0, 0, -1]'}, 'diagonal'),
    ({'no_such_key': '1'}, 'hardware key')])
def test_a_bad_hardware_value_is_refused_and_nothing_is_written(hw, settings, hint):
    c = claim(hw)
    reply = final(ask(hw, 'set', c, settings=settings)['id'])
    assert reply['error'] == 'bad_args' and hint in reply['hint'], reply
    assert not hw.saved


def test_armed_an_armed_unsafe_key_is_refused_with_the_exact_ask_and_when(hw):
    c = claim(hw)
    hw.app.is_armed = True
    reply = final(ask(hw, 'set', c, settings={'manual_amplitude': '0.25'})['id'])
    assert reply['error'] == 'refused' and not hw.saved
    found = reply['result']['ask']
    assert (found['key'], found['current'], found['target']) == ('manual_amplitude', 0.75, 0.25)
    assert 'disarmed' in found['when'] and str(hw.path) in found['where']
    assert "boat_agent set manual_amplitude=0.25 --claim ID" in reply['hint'] and '--until disarmed' in reply['hint']
    reply = final(ask(hw, 'set', c, settings={'command_mode': 'velocity'})['id'])       # the session layer too
    assert reply['error'] == 'refused' and reply['result']['ask']['current'] == 'effort'
    assert 'boat_agent set command_mode=velocity' in reply['hint']


def test_armed_c_is_live_and_set(hw):
    c = claim(hw)
    hw.app.is_armed = True
    reply = final(ask(hw, 'set', c, settings={'decoupling.yaw': str(YAW_ROW)})['id'])
    assert reply['ok'] and hw.saved, reply


def test_set_key_reads_without_a_claim(hw):
    reply = final(ask(hw, 'set', None, settings={'decoupling.yaw': None, 'command_mode': None})['id'])
    assert reply['ok'], reply
    hardware, session = reply['result']['read']
    assert hardware['value'] == [0.0, 0.0, 1.0] and str(hw.path) in hardware['where']
    assert session['value'] == 'effort' and session['armed_unsafe'] and 'setting row' in session['where']
    assert not hw.saved


def test_a_settings_set_reports_old_value_readback_and_revert(hw):
    c = claim(hw)
    reply = final(ask(hw, 'set', c, settings={'command_mode': 'velocity'})['id'])
    assert reply['ok'], reply
    row = reply['result']['rows'][0]
    assert (row['old'], row['value']) == ('effort', 'velocity') and 'command_mode=effort' in row['revert']


def test_a_non_tunable_key_names_where_and_who_changes_it(hw):
    c = claim(hw)
    reply = final(ask(hw, 'set', c, settings={'bench_override': 'on'})['id'])
    assert reply['error'] == 'not_allowed' and reply['result']['ask']['key'] == 'bench_override'
    assert 'setting row' in reply['hint'] and 'operator' in reply['hint']


def test_cli_reads_bare_keys_and_quotes_values():
    assert reads_only(['decoupling.yaw', 'command_mode']) and not reads_only(['a=1', 'b'])
    assert assignments(['decoupling.yaw'], reads=True) == ({'decoupling.yaw': None}, None)
    assert assignments(['decoupling.yaw'])[1]
    line = ask_module.set_command('decoupling.yaw', YAW_ROW, claim='c4', why='10-18 C')
    assert line == "boat_agent set 'decoupling.yaw=[-0.0375, 0.1, 1.0]' --claim c4 --why '10-18 C'"


def test_layers_split_names_only_hardware_keys():
    assert layers.split('hardware.decoupling.yaw') == ('decoupling', 'yaw')
    assert layers.split('manual_amplitude') == ('manual_amplitude', '')
    assert layers.split('amplitude') is None and layers.split('nav2.x.y') is None


@pytest.fixture
def vessel_wiring(tmp_path, monkeypatch):
    """A vessel profile whose thrusters: section remaps the outputs."""
    (tmp_path / 'boat').mkdir()
    (tmp_path / 'boat' / 'network.env').write_text('')
    (tmp_path / 'boat' / 'profile.yaml').write_text('thrusters: {channel_map: [1, 3, 2, 4]}\n')
    monkeypatch.setenv('ASTRO_NETWORK_ENV', str(tmp_path / 'boat' / 'network.env'))


def test_the_sim_hardware_file_keeps_crane_wired_canonically_under_a_vessel_env(vessel_wiring):
    from pathlib import Path
    sim = Path(__file__).resolve().parents[1] / 'config' / 'sim' / 'hardware_crane.yaml'
    assert load_config(sim)['channel_map'] == [1, 2, 3, 4]


def test_plan_validates_over_the_vessel_layer_as_the_driver_loads_it(vessel_wiring, tmp_path):
    path = tmp_path / 'hardware.yaml'
    path.write_text('')
    planned = layers.plan('channel_map', '[2, 1, 3, 4]', path)
    assert planned['old'] == [1, 3, 2, 4] and planned['change'] == {'channel_map': [2, 1, 3, 4]}
    with pytest.raises(ValueError, match='channel_map'):
        layers.plan('channel_map', '[1, 1, 2, 4]', path)
