"""boat_agent fence (D-20261101-23) and the 11-01 sim punch list: the Outside toast, holds in status, late requests,
the first watchdog release, the characterize hint and BOAT_AGENT_TARGET=sim."""

import io
import json
import time
from types import SimpleNamespace

import pytest
import yaml

from mhseals_testing import agent_hook as hook
from mhseals_testing.harness import agent_channel as ch, agent_cli, authority, course_area
from mhseals_testing.harness.cli import build_parser
from mhseals_testing.ros.fence import FenceClient
from mhseals_testing.ui import agent_acts
from mhseals_testing.ui.feeds import agent_ops
from mhseals_testing.ui.toasts import Toasts

from test_agent_desk import final, rig  # noqa: F401 - the rig fixture
from test_course_area import BOX, Params

WORDS = [f'{lat},{lon}' for lat, lon in BOX]


def cli(argv, monkeypatch):
    monkeypatch.setenv('BOAT_AGENT', 'claude-test')
    args = build_parser().parse_args(argv)
    out, sent = io.StringIO(), []
    registry = SimpleNamespace(owner=lambda: {'pid': 1}, alive=lambda pid: True, state=lambda: {})
    agent = agent_cli.Agent(args, registry=registry, out=out, settings=agent_cli.DEFAULTS, sleep=lambda s: None)
    agent.send = lambda op, args=None, claim=None, dry=False, wait=None: sent.append((op, args, claim)) or 0
    args.who, _ = agent_cli.who_of(args)
    return agent_cli.needs_why(args), getattr(agent, args.command)(), sent, out.getvalue()


def test_the_fence_verb_checks_its_words_and_needs_why_but_no_claim(monkeypatch):
    why, code, sent, _ = cli(['fence', 'corners', *WORDS, '--why', 'today'], monkeypatch)
    assert not why and code == 0 and sent[0][0] == 'fence' and sent[0][2] is None
    assert course_area.parse(sent[0][1]['corners'])[0] == BOX
    assert cli(['fence', 'corners', *WORDS], monkeypatch)[0]                     # --why is required
    for argv in (['corners', *WORDS[:2]], ['corners', '42,x', *WORDS[1:]], ['corners', '95,0', *WORDS[1:]],
                 ['on_exit', 'drift'], ['anchor', 'now']):
        _, code, sent, out = cli(['fence', *argv, '--why', 'x'], monkeypatch)
        assert code == 2 and not sent and json.loads(out)['error'] == 'bad_args', argv
    assert cli(['fence', 'on_exit', 'return', '--why', 'x'], monkeypatch)[2] == [
        ('fence', {'action': 'on_exit', 'mode': 'return'}, None)]


@pytest.fixture
def fenced(rig, tmp_path, monkeypatch):  # noqa: F811
    monkeypatch.setattr(course_area, 'FENCE_FILE', tmp_path / 'fence.yaml')
    monkeypatch.setattr(course_area.save, '__defaults__', (None, tmp_path / 'fence.yaml', None))
    monkeypatch.setattr(agent_acts, 'in_background', lambda app, title, work, **k: work())
    client = FenceClient(node=None, params=Params())
    client.anchor = lambda: (True, 'course area fixed')
    rig.app.fence_client = lambda: client
    rig.client, rig.file = client, tmp_path / 'fence.yaml'
    return rig


def fence(rig, why='set for the day', **args):
    req = ch.request('fence', 'claude-3f2a', args, why=why)
    rig.desk.tick()
    return final(req['id'])


def test_the_desk_applies_reads_back_keeps_and_logs_the_corners(fenced):
    reply = fence(fenced, action='corners', corners=WORDS)
    assert reply['ok'] and reply['result']['read_back'] and len(reply['result']['corners']) == 4
    assert course_area.pairs(fenced.client.params.values['course.corners']) == BOX
    assert course_area.load(fenced.file) == BOX
    assert fenced.desk.recent[-1]['why'] == 'set for the day' and fenced.desk.recent[-1]['label'] == 'fence corners'
    assert ('course_area', 'claude-3f2a') in [(p, f.get('who')) for p, f in fenced.app.events]
    assert fence(fenced, action='on_exit', mode='return')['result'] == {'on_exit': 'return', 'read_back': True}
    assert yaml.safe_load(fenced.file.read_text())['course'] == {'corners': course_area.flatten(BOX),
                                                                 'on_exit': 'return'}
    assert fence(fenced, action='anchor')['result']['anchored']


def test_the_desk_refuses_a_bad_area_a_missing_why_and_blocked_agents(fenced):
    assert fence(fenced, action='corners', corners=WORDS[:2])['error'] == 'bad_args'
    assert fence(fenced, why='', action='anchor')['error'] == 'bad_args'
    fenced.client.params.keep = False                              # fence_watchdog kept other corners
    reply = fence(fenced, action='corners', corners=WORDS)
    assert reply['error'] == 'refused' and 'readback' in reply['hint'] and not fenced.file.exists()
    fenced.app.settings.agents = 'block'
    assert fence(fenced, action='anchor')['error'] == 'blocked'


def test_the_hook_routes_raw_fence_changes_to_the_verb():
    config = dict(hook.DEFAULTS)
    for command in ('ros2 param set /fence_watchdog course.corners "[1.0, 2.0]"',
                    'ros2 param set /fence_watchdog on_exit return',
                    'ros2 service call /fence/anchor std_srvs/srv/Trigger'):
        hit = hook.denied(command, config, None, 'claude-1')
        assert hit, command
        reason = hook.deny_reason(*hit, who='claude-1', config=config)
        assert 'fence corners LAT,LON' in reason and 'claim first' not in reason, reason
        assert hook.use_of(hit[0]) == 'boat_agent fence'
    assert hook.denied(f'{hook.BOAT_AGENT} fence corners {" ".join(WORDS)} --why x', config, None, 'claude-1') is None
    assert 'fence corners' in hook.start_text('claude-1', config=config)


# --- the punch list ---------------------------------------------------------------------------------------------

def test_the_outside_text_names_the_live_distance_and_u_only_with_a_hold():
    status = {'inside': False, 'distance_to_edge': 3.2, 'on_exit': 'alert'}
    assert 'U releases' in course_area.status_line(status)[1]
    text = course_area.status_line(status, held=False)[1]
    assert '3.2 m' in text and 'U releases' not in text
    toasts = Toasts()
    toasts.push('fence.outside', 'fail', 'by 3.2 m', 0.0)
    assert toasts.update('fence.outside', 'by 4.0 m') and not toasts.update('gone', 'x')
    shown = toasts.visible(1.0)
    assert [(t.text, t.count) for t in shown] == [('by 4.0 m', 1)]


def test_a_stopped_mux_lists_its_saved_holds():
    saved = [{'cause': 'geofence', 'by': 'fence_watchdog', 'has_token': False}]
    record = authority.derive(None, [], None, None, None, None, 100.0, mux_expected=False, saved_holds=saved)
    assert record['holds'] == [dict(saved[0], saved=True)]
    fresh = authority.derive({'input': 'hold', 'holds': saved, 't_ns': 100 * 10 ** 9}, [], None, None, None,
                             None, 100.0, mux_age_s=0.1, saved_holds=[{'cause': 'old'}])
    assert fresh['holds'] == saved


def test_a_request_read_while_a_handler_works_waits_and_expires(rig):  # noqa: F811
    rig.desk.working = 'set'
    req = ch.request('mark', 'claude-3f2a', {'note': 'x'}, request_s=0.05)
    rig.desk.poll()
    assert not [r for r in ch.Tail(ch.channel_path()).read()[0] if r.get('re') == req['id']]
    rig.desk.working = None
    time.sleep(0.1)
    rig.desk.poll()
    assert final(req['id'])['error'] == 'expired'


def test_a_holding_watchdog_is_watched_at_once_and_a_first_release_waits_for_its_beat(rig, monkeypatch):  # noqa: F811
    watched, beats = [], {}
    rig.app.ros.watch_beats = lambda nodes: [n for n in nodes if n not in watched and not watched.append(n)]
    rig.app.ros.nodes = lambda: ['/fence_watchdog', '/planner_server']
    rig.app.arbiter.on_mux({'input': 'hold', 'holds': [{'cause': 'geofence', 'by': 'fence_watchdog'}]})
    rig.desk.tick()
    assert watched == ['/fence_watchdog']
    watched.clear()                                          # the release subscribes now: the first beat comes late
    rig.app.ros.beats = lambda: dict(beats)
    rig.app.ros.beat_status = lambda: {n: {'ok': True} for n in beats}
    monkeypatch.setattr(agent_ops.time, 'sleep', lambda s: beats.setdefault('/fence_watchdog', time.monotonic()))
    assert agent_ops.cleared(rig.app, {'cause': 'geofence', 'by': 'fence_watchdog'}) == (True, '')


def test_agents_are_never_told_to_set_command_mode():
    text = agent_acts.agent_text('needs effort commands; use --set command_mode=effort')
    assert '--set command_mode' not in text and 'operator' in text and 'BOAT_SIM_COMMAND_MODE=effort' in text


def test_the_sim_target_is_this_machine_never_an_ssh_host(tmp_path, monkeypatch):
    monkeypatch.setenv('BOAT_AGENT_TARGET', 'sim')
    monkeypatch.setenv('BOAT_SIM_OVERLAY', str(tmp_path))
    config = hook.hook_config()
    assert hook.remote(config) is None and 'sim' not in hook.boats(config)
    assert hook.sim_channel() == tmp_path / 'data' / 'agent' / 'channel.jsonl'
    hook.sim_channel().parent.mkdir(parents=True)
    hook.deliver({'kind': 'note'}, config)
    assert json.loads(hook.sim_channel().read_text()) == {'kind': 'note'}
    assert 'sim TUI on this machine' in hook.agent_event({'agent': 'claude', 'event': 'start'}, config)['context']


pytestmark = pytest.mark.smoke   # tools/smoke
