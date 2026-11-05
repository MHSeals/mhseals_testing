"""boat_agent fcu: allowlisted FCU writes through the desk (MAVROS stubbed by a fake MavrosLink), the never list,
the snapshot, read-back, change log and ledger, revert, the disarmed restart, the 400 Hz gate, the rates check,
the CLI and the hook routes (own tmux sessions, boat_agent ask text)."""

import json

import pytest

from mhseals_testing import agent_hook as hook
from mhseals_testing.agent import config as agent_config
from mhseals_testing.agent import fcu as af
from mhseals_testing.ros.fcu import FcuParams
from test_agent_cli import Desk, agent_id, call, lines  # noqa: F401 (agent_id: fixture)
from test_agent_desk import ask, claim, final, rig  # noqa: F401 (fixtures)

WHO = 'claude-3f2a'
BOAT = 'ssh blastoise-odroid '

BOAT_AGENT = hook.DEFAULTS['boat_agent']


class Link:
    """The MavrosLink surface FcuParams uses: names, get, set, armed, command."""

    def __init__(self, armed=False, accept=True):
        self.values = {'SCHED_LOOP_RATE': 50, 'SR0_EXTRA1': 4, 'INS_GYRO_FILTER': 4, 'EK3_SRC1_YAW': 1,
                       'COMPASS_USE': 1, 'SERVO1_FUNCTION': 1, 'FS_ACTION': 2, 'BATT_FS_LOW_ACT': 2}
        self.is_armed, self.accept, self.sets, self.commands = armed, accept, [], []

    def names(self):
        return list(self.values)

    def get(self, names):
        return {n: self.values[n] for n in names if n in self.values}

    def set(self, name, value):
        self.sets.append((name, value))
        if self.accept:
            self.values[name] = value
        return self.accept

    def armed(self):
        return self.is_armed

    def command(self, command, params):
        self.commands.append((command, list(params)))
        return True


@pytest.fixture
def fcu(rig, tmp_path):
    link = Link()
    params = FcuParams(link, meta={'params': {n: {} for n in link.values}}, stable=0)
    rig.desk.fcu = af.AgentFcu(lambda: params, af.Store(tmp_path / 'store', 'blastoise'), rig.desk.config.get,
                               measure=lambda window: {'topics': {'/imu/raw': af.stats([0, 0.005, 0.01])},
                                                       'load_pct': 41.0, 'main_loop_slow': 0})
    rig.app.is_armed = False
    rig.link, rig.store = link, tmp_path / 'store'
    return rig


def test_shipped_fcu_config_lists_the_rate_params_and_never_the_safety_ones():
    fcu = af.settings(agent_config.load())
    assert {'SCHED_LOOP_RATE', 'SR0_*', 'SR1_*', 'INS_GYRO_FILTER'} <= set(fcu['set'])
    assert fcu['values']['SCHED_LOOP_RATE'] == [50, 100, 200, 400]
    for name in ('EK3_SRC1_YAW', 'EK3_SRC2_YAW', 'COMPASS_USE', 'SERVO1_FUNCTION', 'ARMING_CHECK', 'FS_ACTION',
                 'BATT_FS_LOW_ACT', 'FENCE_ENABLE', 'RC1_MIN', 'BRD_SAFETY_DEFLT', 'FS_GCS_ENABLE', 'SYSID_MYGCS'):
        assert af.never(name), name
    for name in ('SCHED_LOOP_RATE', 'SR0_EXTRA1', 'SR1_RAW_SENS', 'INS_GYRO_FILTER', 'INS_ACCEL_FILTER'):
        assert not af.never(name), name


def test_an_allowlisted_set_snapshots_reads_back_diffs_and_logs_with_attribution(fcu):
    c = claim(fcu)
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 200.0, 'SR0_EXTRA1': 25.0})['id'])
    assert reply['ok'], reply
    assert fcu.link.sets == [('SCHED_LOOP_RATE', 200), ('SR0_EXTRA1', 25)]           # ints stay ints
    result = reply['result']
    assert result['changes'] == {'SCHED_LOOP_RATE': [50, 200], 'SR0_EXTRA1': [4, 25]}
    assert result['read_back'] == {'SCHED_LOOP_RATE': 200, 'SR0_EXTRA1': 25}
    assert result['restart_needed'] == ['SCHED_LOOP_RATE'] and 'fcu restart' in reply['hint']
    snap = fcu.store / result['snapshot']
    assert snap.name.startswith('fcu_blastoise_') and 'SCHED_LOOP_RATE,50\n' in snap.read_text()  # before the write
    log = (fcu.store / af.CHANGES).read_text()
    assert f'set by agent {WHO} (claim {c})' in log and '- SCHED_LOOP_RATE: 50 -> 200' in log
    entry = json.loads((fcu.store / af.LEDGER).read_text().splitlines()[-1])
    assert entry['who'] == WHO and entry['claim'] == c and entry['snapshot'] == snap.name
    assert fcu.app.notice.startswith(f'Agent {WHO} {c}: setting FCU parameters: SCHED_LOOP_RATE 50->200')
    assert ('fcu', {'action': 'set', 'ok': True, 'by': 'agent', 'who': WHO, 'claim': c,
                    'changes': result['changes'], 'snapshot': snap.name}) in fcu.app.events
    # one snapshot per TUI session
    assert final(ask(fcu, 'fcu_set', c, changes={'INS_GYRO_FILTER': 20.0})['id'])['ok']
    assert len(list(fcu.store.glob('fcu_*.param'))) == 1


@pytest.mark.parametrize('name', ['EK3_SRC1_YAW', 'COMPASS_USE', 'SERVO1_FUNCTION', 'FS_ACTION', 'BATT_FS_LOW_ACT'])
def test_never_list_refuses_even_when_agent_yaml_allows_everything(fcu, name):
    cfg = dict(fcu.desk.config.get(), fcu={'set': ['*']})
    fcu.desk.fcu.config = lambda: cfg
    reply = final(ask(fcu, 'fcu_set', claim(fcu), changes={name: 0.0})['id'])
    assert not reply['ok'] and reply['error'] == 'not_allowed' and 'never' in reply['hint']
    assert fcu.link.sets == [] and not fcu.store.exists()                       # nothing written, no snapshot


def test_set_refusals_unlisted_value_unclaimed_armed_and_blocked(fcu):
    c = claim(fcu)
    assert final(ask(fcu, 'fcu_set', c, changes={'CRUISE_SPEED': 2.0})['id'])['error'] == 'not_allowed'
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 300.0})['id'])
    assert reply['error'] == 'not_allowed' and '50, 100, 200, 400' in reply['hint']
    assert final(ask(fcu, 'fcu_set', None, changes={'SR0_EXTRA1': 10.0})['id'])['error'] == 'unknown_claim'
    fcu.app.is_armed = True
    assert final(ask(fcu, 'fcu_set', c, changes={'SR0_EXTRA1': 10.0})['id'])['error'] == 'refused'
    fcu.app.is_armed = False
    fcu.app.settings.agents = 'block'
    assert final(ask(fcu, 'fcu_set', c, changes={'SR0_EXTRA1': 10.0})['id'])['error'] == 'blocked'
    assert fcu.link.sets == []


def test_a_write_the_fcu_does_not_take_is_reported_and_logged(fcu):
    fcu.link.accept = False
    reply = final(ask(fcu, 'fcu_set', claim(fcu), changes={'SR0_EXTRA1': 25.0})['id'])
    assert not reply['ok'] and 'SR0_EXTRA1' in reply['hint'] and reply['result']['read_back'] == {'SR0_EXTRA1': 4}
    assert 'write not acknowledged' in (fcu.store / af.CHANGES).read_text()


def test_loop_rate_400_needs_a_rates_check_at_200_with_low_load(fcu):
    c = claim(fcu)
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 400.0})['id'])
    assert reply['error'] == 'refused' and 'no fcu rates check yet' in reply['hint']
    assert final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 200.0})['id'])['ok']
    rates = final(ask(fcu, 'fcu_rates', None, window_s=5)['id'])
    assert rates['ok'] and rates['result']['loop_rate'] == 200 and rates['result']['load_pct'] == 41.0
    assert rates['result']['topics']['/imu/raw'] == {'n': 3, 'hz': 200.0, 'gap_p99_ms': 5.0, 'gap_max_ms': 5.0}
    assert rates['result']['loop_running'] and not rates['result']['after_restart']
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 400.0})['id'])      # no restart since the set
    assert reply['error'] == 'refused' and 'did not follow an FCU restart' in reply['hint']
    assert final(ask(fcu, 'fcu_restart', c)['id'])['ok']
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 400.0})['id'])      # rates came before it
    assert reply['error'] == 'refused' and 'did not follow an FCU restart' in reply['hint']
    assert final(ask(fcu, 'fcu_rates', None, window_s=5)['id'])['result']['after_restart']
    assert final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 400.0})['id'])['ok']


def test_loop_400_refuses_high_load_an_unconfirmed_loop_a_later_set_and_forged_files(fcu):
    c = claim(fcu)
    work = fcu.desk.fcu
    assert final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 200.0})['id'])['ok']
    assert final(ask(fcu, 'fcu_restart', c)['id'])['ok']
    # a forged ledger rates record grants nothing: the gate's facts live in the TUI process
    work.store.record({'kind': 'rates', 'loop_rate': 200, 'load_pct': 1.0, 'loop_running': True})
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 400.0})['id'])
    assert reply['error'] == 'refused' and 'no fcu rates check yet' in reply['hint']
    work.measure = lambda window: {'topics': {'/imu/raw': af.stats([0, 0.005])}, 'load_pct': 75.0}
    final(ask(fcu, 'fcu_rates', None)['id'])
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 400.0})['id'])
    assert reply['error'] == 'refused' and 'load 75.0%' in reply['hint']
    work.measure = lambda window: {'topics': {'/imu/raw': af.stats([0, 0.01, 0.02])}, 'load_pct': 30.0}  # 100 Hz
    assert 'unconfirmed' in final(ask(fcu, 'fcu_rates', None)['id'])['hint']
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 400.0})['id'])
    assert reply['error'] == 'refused' and 'IMU 100.0 Hz' in reply['hint']
    work.measure = lambda window: {'topics': {'/imu/raw': af.stats([0, 0.005, 0.01])}, 'load_pct': 30.0}
    final(ask(fcu, 'fcu_rates', None)['id'])
    assert final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 200.0})['id'])['ok']   # a later loop write
    reply = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 400.0})['id'])
    assert reply['error'] == 'refused' and 'did not follow an FCU restart' in reply['hint']


def test_the_fcu_arming_mid_set_stops_the_writes_and_the_partial_set_is_on_record(fcu):
    c = claim(fcu)
    states = iter([False, False, True])               # before the set, before write 1, before write 2
    fcu.link.armed = lambda: next(states)
    reply = final(ask(fcu, 'fcu_set', c, changes={'SR0_EXTRA1': 25.0, 'INS_GYRO_FILTER': 20.0})['id'])
    assert not reply['ok'] and reply['error'] == 'refused'
    assert 'partial: written SR0_EXTRA1; not applied INS_GYRO_FILTER' in reply['hint'] and 'armed' in reply['hint']
    assert fcu.link.sets == [('SR0_EXTRA1', 25)]
    result = reply['result']
    assert result['written'] == ['SR0_EXTRA1'] and result['not_written'] == ['INS_GYRO_FILTER']
    sets = [json.loads(line) for line in (fcu.store / af.LEDGER).read_text().splitlines()]
    assert [e['changes'] for e in sets if e['kind'] == 'set'] == [{'SR0_EXTRA1': [4, 25]}]
    log = (fcu.store / af.CHANGES).read_text()
    assert '- SR0_EXTRA1: 4 -> 25' in log and '- INS_GYRO_FILTER: not written' in log
    assert ('fcu', {'action': 'write', 'ok': True, 'by': 'agent', 'who': WHO, 'claim': c,
                    'changes': {'SR0_EXTRA1': [4, 25]}, 'snapshot': result['snapshot']}) in fcu.app.events
    assert 'partial: written SR0_EXTRA1' in fcu.app.notice
    fcu.link.armed = lambda: None                     # unknown before the first write: nothing written
    fcu.link.sets.clear()
    assert final(ask(fcu, 'fcu_set', c, changes={'SR0_EXTRA1': 10.0})['id'])['error'] == 'refused'
    assert fcu.link.sets == []
    fcu.link.armed = lambda: False                    # the partial set reverts
    assert final(ask(fcu, 'fcu_revert', c, to=result['snapshot'])['id'])['ok']
    assert fcu.link.values['SR0_EXTRA1'] == 4


def test_revert_restores_only_the_params_agents_changed(fcu):
    c = claim(fcu)
    snap = final(ask(fcu, 'fcu_set', c, changes={'SCHED_LOOP_RATE': 200.0, 'SR0_EXTRA1': 25.0})['id'])['result'][
        'snapshot']
    fcu.link.values['INS_GYRO_FILTER'] = 30           # changed by someone else: not the agents' to undo
    fcu.link.sets.clear()
    reply = final(ask(fcu, 'fcu_revert', c, to=snap)['id'])
    assert reply['ok'], reply
    assert sorted(fcu.link.sets) == [('SCHED_LOOP_RATE', 50), ('SR0_EXTRA1', 4)]
    assert fcu.link.values['INS_GYRO_FILTER'] == 30
    assert f'revert to {snap} by agent {WHO}' in (fcu.store / af.CHANGES).read_text()
    assert final(ask(fcu, 'fcu_revert', c, to='fcu_none.param')['id'])['error'] == 'bad_args'
    assert final(ask(fcu, 'fcu_revert', c, to=str(fcu.store / snap))['id'])['ok']    # its full store path


def test_revert_takes_only_store_snapshots_with_their_checksum_and_refusal_checks_every_value(fcu, tmp_path):
    c = claim(fcu)
    snap = final(ask(fcu, 'fcu_set', c, changes={'SR0_EXTRA1': 25.0})['id'])['result']['snapshot']
    outside = tmp_path / 'elsewhere' / snap
    outside.parent.mkdir()
    outside.write_text((fcu.store / snap).read_text())
    for path in (str(outside), f'../elsewhere/{snap}', 'fcu_x.param/../' + snap):
        reply = final(ask(fcu, 'fcu_revert', c, to=path)['id'])
        assert reply['error'] == 'bad_args', path
    link = fcu.store / 'fcu_link.param'                # a link out of the store
    link.symlink_to(outside)
    assert final(ask(fcu, 'fcu_revert', c, to=link.name)['id'])['error'] == 'bad_args'
    original = (fcu.store / snap).read_text()
    (fcu.store / snap).write_text(original.replace('SR0_EXTRA1,4', 'SR0_EXTRA1,9'))   # edited after the snapshot
    reply = final(ask(fcu, 'fcu_revert', c, to=snap)['id'])
    assert reply['error'] == 'bad_args' and 'checksum' in reply['hint']
    # a consistently forged snapshot and ledger can do no more than set: every value goes through refusal()
    forged = original.replace('EK3_SRC1_YAW,1', 'EK3_SRC1_YAW,8').replace('SCHED_LOOP_RATE,50', 'SCHED_LOOP_RATE,400')
    (fcu.store / snap).write_text(forged)
    work = fcu.desk.fcu
    work.store.record({'kind': 'snapshot', 'snapshot': snap, 'sha256': af.Store.digest(fcu.store / snap)})
    work.store.record({'kind': 'set', 'snapshot': snap, 'changes': {'EK3_SRC1_YAW': [8, 1]}})
    fcu.link.sets.clear()
    reply = final(ask(fcu, 'fcu_revert', c, to=snap)['id'])
    assert reply['error'] == 'not_allowed' and 'EK3_SRC1_YAW' in reply['hint'] and fcu.link.sets == []
    work.store.record({'kind': 'set', 'snapshot': snap, 'changes': {'SCHED_LOOP_RATE': [400, 50]}})
    fcu.link.values['EK3_SRC1_YAW'] = 8                # equal to the snapshot: skipped; the loop rate is gated
    reply = final(ask(fcu, 'fcu_revert', c, to=snap)['id'])
    assert reply['error'] == 'refused' and 'fcu rates check' in reply['hint'] and fcu.link.sets == []


def test_restart_needs_a_claim_and_a_disarmed_fcu_and_is_announced(fcu):
    c = claim(fcu)
    fcu.app.is_armed = True
    assert final(ask(fcu, 'fcu_restart', c)['id'])['error'] == 'refused'
    fcu.app.is_armed, fcu.link.is_armed = False, True                         # the FCU itself says armed
    reply = final(ask(fcu, 'fcu_restart', c)['id'])
    assert reply['error'] == 'refused' and 'armed' in reply['hint']
    fcu.link.is_armed = None                                                     # MAVROS not connected
    assert final(ask(fcu, 'fcu_restart', c)['id'])['error'] == 'refused'
    assert final(ask(fcu, 'fcu_restart', None)['id'])['error'] == 'unknown_claim'
    assert fcu.link.commands == []
    fcu.link.is_armed = False
    assert final(ask(fcu, 'fcu_restart', c)['id'])['ok']
    assert fcu.link.commands == [(af.RESTART, [1, 0, 0, 0, 0, 0, 0])]
    assert fcu.app.notice == f'Agent {WHO} {c}: restarting the FCU'
    assert ('fcu', {'action': 'restart', 'ok': True, 'by': 'agent', 'who': WHO, 'claim': c}) in fcu.app.events
    assert f'restart by agent {WHO} (claim {c})' in (fcu.store / af.CHANGES).read_text()


def test_get_reads_without_a_claim(fcu):
    reply = final(ask(fcu, 'fcu_get', None, params=['SCHED_LOOP_RATE', 'NOPE'])['id'])
    assert reply['ok'] and reply['result'] == {'values': {'SCHED_LOOP_RATE': 50}} and 'NOPE' in reply['hint']


def test_cli_fcu_verbs_send_their_ops():
    desk = Desk(lambda req: [dict(state='done', result={})])
    assert call(['fcu', 'set', 'SCHED_LOOP_RATE', '200', 'SR0_EXTRA1', '25', '--claim', 'c4'], desk)[0] == 0
    assert call(['fcu', 'get', 'SCHED_LOOP_RATE'], desk)[0] == 0
    assert call(['fcu', 'restart', '--claim', 'c4'], desk)[0] == 0
    assert call(['fcu', 'revert', '--to', 'fcu_b.param', '--claim', 'c4'], desk)[0] == 0
    assert call(['fcu', 'rates', '--window', '5'], desk)[0] == 0
    assert [(r['op'], r['args'], r['claim']) for r in lines('req')] == [
        ('fcu_set', {'changes': {'SCHED_LOOP_RATE': 200.0, 'SR0_EXTRA1': 25.0}}, 'c4'),
        ('fcu_get', {'params': ['SCHED_LOOP_RATE']}, None), ('fcu_restart', {}, 'c4'),
        ('fcu_revert', {'to': 'fcu_b.param'}, 'c4'), ('fcu_rates', {'window_s': 5.0}, None)]
    for argv in (['fcu', 'set', 'SR0_EXTRA1', '25'], ['fcu', 'restart'], ['fcu', 'set', 'SR0_EXTRA1', '--claim', 'c'],
                 ['fcu', 'set', 'SR0_EXTRA1', 'x', '--claim', 'c'], ['fcu', 'revert', '--claim', 'c'], ['fcu', 'get']):
        code, out = call(argv, desk)
        assert code == 2 and out[-1]['error'] == 'bad_args', argv


# --- hook -----------------------------------------------------------------------------------

def reason(command):
    found = hook.denied(command, hook.DEFAULTS)
    return found and hook.deny_reason(*found, who=WHO, config=hook.DEFAULTS)


REBOOT = 'sudo re' + 'boot'


@pytest.mark.parametrize('command', [
    BOAT + f'tmux kill-session -t agent-{WHO}', BOAT + f'tmux kill-session -t agent-{WHO}-odom',
    BOAT + f'"tmux kill-session -t boatops-{WHO}_odom"', BOAT + f"'tmux kill-session -t =agent-{WHO}-x'",
    BOAT + '~/astro_dock/src/mhseals_testing/tools/boat_agent ask "may I re' + 'boot the FCU?" --who c1',
    BOAT + "boat_agent note 'after the re" + "boot: loop 200' --who c1",
    BOAT + "\"boat_agent ask 'may I re" + "boot the FCU?' --who c1\"",
])
def test_own_tmux_sessions_and_ask_text_are_allowed_on_a_boat(command):
    assert hook.denied(command, hook.DEFAULTS, who=WHO) is None


@pytest.mark.parametrize('command', [
    BOAT + 'tmux kill-session -t boat', BOAT + f'tmux -L boat kill-session -t agent-{WHO}',
    BOAT + f'tmux -S /tmp/x kill-session -t agent-{WHO}', BOAT + f'tmux kill-session -t agent-{WHO} -a',
    BOAT + f'tmux kill-session -a -t agent-{WHO}', BOAT + 'tmux kill-server',
    # another agent's session (a prefix of ours, ours as a prefix of theirs, or no id at all)
    BOAT + 'tmux kill-session -t agent-c2', BOAT + 'tmux kill-session -t agent-claude',
    BOAT + f'tmux kill-session -t agent-{WHO}0', BOAT + 'tmux kill-session -t boatops',
    BOAT + "'boat_agent note \"x $(re" + "boot)\" --who c1'", BOAT + f"'boat_agent ask \"ok\" --who c1; {REBOOT}'",
    # what the remote shell runs: unquoted text, $( ) and backticks there
    f"echo \"boat_agent ask '$(ssh blastoise-odroid {REBOOT})'\"",
    "bash -c \"boat_agent ask '$(ssh blastoise-odroid sudo systemctl st" + "op mhseals-driver)'\"",
    BOAT + f"'boat_agent ask \"`{REBOOT}`\"'", BOAT + "\"boat_agent ask '${X:-x}'; " + REBOOT + '"',
    f"bash -c 'boat_agent ask 'x;ssh blastoise-odroid {REBOOT}''",
    f"ssh blastoise-odroid boat_agent ask \"x; {REBOOT}\"",
    BOAT + f"eval 'boat_agent ask\n'\"{REBOOT}\"",
])
def test_the_boat_session_other_sockets_and_hidden_commands_stay_denied(command):
    assert hook.denied(command, hook.DEFAULTS, who=WHO) is not None
    assert hook.denied(command, hook.DEFAULTS) is not None                     # no caller id: no own session


def test_fcu_param_writes_route_to_fcu_set_and_restarts_to_fcu_restart():
    mav = '/mav' + 'ros'
    assert f'{BOAT_AGENT} fcu set PARAM VALUE --claim ID --who {WHO}' in reason(
        BOAT + f'docker exec roboboat_dev ros2 param set {mav}/param SCHED_LOOP_RATE 200')
    assert 'fcu set PARAM VALUE --claim ID' in reason(f'ros2 service call {mav}/param/set x "{{}}"')
    assert 'boat_agent fcu restart' in reason(f'ros2 service call {mav}/cmd/command x "{{command: 246}}"')
    assert 'fcu restart --claim ID' in reason(BOAT + 'sudo re' + 'boot')
    text = hook.start_text(WHO, config=hook.DEFAULTS)
    assert '`fcu set PARAM VALUE... --claim ID`' in text and f'`tmux kill-session -t agent-{WHO}-...`' in text
