"""The multi-client control model (harness/control.py): one controller, viewers, takeover, the tracker,
E and Space from a viewer, an agent claim against a human takeover, stale and reconnecting clients."""

import io
import os
import signal
import socket
import sys
from types import SimpleNamespace

import pytest
from rich.console import Console

from mhseals_testing.harness import control
from mhseals_testing.agent import channel as ch
from mhseals_testing.harness.authority import ClaimTable
from mhseals_testing.ros.authority import Arbiter
from mhseals_testing.ui import app as app_module, control_tracker
from mhseals_testing.ui.app import App
from mhseals_testing.ui.app.state import AppState
from mhseals_testing.ui.feeds.authority import ViewerFeed
from mhseals_testing.ui.feeds.control import ControlFeed
from mhseals_testing.ui.takeover import decide

from test_agent_desk import claim, fence_topic, final, rig  # noqa: F401 - pytest fixtures
from test_registry import World

ME = os.getpid()


def seat(pid, client, host='boat-host', started=100.0, **more):
    return {'pid': pid, 'node': f'n{pid}', 'host': host, 'client': client, 'from': '10.0.0.5',
            'started': started, **more}


def guest(pid, client):
    return {k: v for k, v in seat(pid, client).items() if k not in ('pid', 'node')}


def entry(owner=None, viewers=()):
    return {'owner': owner, 'guests': {str(v['pid']): dict(v, procedure='watch') for v in viewers}}


HUMAN_RECORD = {'owner': 'human', 'claims': []}


def agent_record(who='claude-3f2a', claim_id='c4', state='active'):
    return {'owner': 'agent', 'claims': [{'id': claim_id, 'owner': 'agent', 'who': who, 'state': state,
                                          'since_ns': int(200e9), 'host': 'dev', 'pid': 77}]}


def ask(answers):
    answers = list(answers)
    return lambda _: answers.pop(0)


def console():
    return Console(file=io.StringIO(), width=100)


# --- the tracker ------------------------------------------------------------------------------

def test_the_tracker_names_the_seat_as_the_one_human_controller_and_counts_live_viewers():
    alive = {1, 2, 3}.__contains__
    state = entry(seat(1, 'boat'), [seat(2, 'boat-v1'), seat(3, 'boat-v2'), seat(4, 'boat-v3')])
    view = control.tracker(state, HUMAN_RECORD, 2, alive, 160.0)
    assert view['controller']['kind'] == 'human' and view['controller']['name'] == 'boat@boat-host'
    assert view['controller']['since'] == 100.0 and view['controller']['from'] == '10.0.0.5'
    assert view['viewers'] == 2                       # pid 4 is gone: a stale viewer is not counted
    assert view['role'] == 'view' and control.tracker(state, HUMAN_RECORD, 1, alive, 160.0)['role'] == 'control'


def test_an_active_agent_claim_controls_via_the_seat_and_a_suspended_or_revoked_one_does_not():
    alive = {1}.__contains__
    view = control.tracker(entry(seat(1, 'boat')), agent_record(), 1, alive, 300.0)
    ctl = view['controller']
    assert (ctl['kind'], ctl['who'], ctl['claim'], ctl['via'], ctl['since']) == \
        ('agent', 'claude-3f2a', 'c4', 'boat@boat-host', 200.0)
    for state in ('suspended', 'revoked'):
        assert control.tracker(entry(seat(1, 'boat')), agent_record(state=state), 1, alive, 300.0)[
            'controller']['kind'] == 'human'


def test_a_dead_seat_is_stale_and_an_empty_one_has_no_controller():
    assert control.tracker(entry(seat(1, 'boat')), agent_record(), 2, set().__contains__, 0)[
        'controller']['kind'] == 'stale'                # a claim on a dead seat controls nothing
    assert control.tracker({}, None, 2, set().__contains__, 0)['controller'] == {'kind': 'none'}


def test_every_client_computes_the_same_controller_and_it_follows_each_change(tmp_path):
    world = World(tmp_path, alive={1, 2, 3})
    registry = world.registry()
    registry.claim(seat(1, 'boat'))
    registry.add_guest(2, 'n2', 'watch', **guest(2, 'boat-v1'))
    feeds = [ControlFeed(registry, pid, alive=world.alive.__contains__, clock=lambda: world.now)
             for pid in (1, 2)]

    def seen(record=None):
        world.now += 1.0                                # past POLL_S: the registry is re-read
        for feed in feeds:
            feed.poll(world.now)
        return [feed.update(record, world.now) for feed in feeds]

    first = seen()
    assert first[0]['controller'] == first[1]['controller'] and first[0]['viewers'] == 1
    assert [v['role'] for v in first] == ['control', 'view']
    registry.add_guest(3, 'n3', 'watch', **guest(3, 'boat-v2'))
    assert [v['viewers'] for v in seen()] == [2, 2]
    agent = seen(agent_record())
    assert {v['controller']['kind'] for v in agent} == {'agent'} and agent[0]['controller'] == agent[1]['controller']
    world.alive.discard(3)                              # a viewer killed without a goodbye
    assert [v['viewers'] for v in seen()] == [1, 1]


def test_the_owner_journals_each_controller_change_once_and_a_viewer_never():
    events = []
    alive = {1, 2}.__contains__
    owner = ControlFeed(SimpleNamespace(path='/nonexistent'), 1, alive=alive, emit=lambda *a, **k: events.append(k))
    viewer = ControlFeed(SimpleNamespace(path='/nonexistent'), 2, alive=alive, emit=lambda *a, **k: events.append(k))
    for feed in (owner, viewer):
        feed.entry = entry(seat(1, 'boat'), [seat(2, 'boat-v1')])
        feed.update(None, 0)
        feed.update(None, 1)
        feed.update(agent_record(), 2)
    assert len(events) == 1 and events[0]['controller']['who'] == 'claude-3f2a'


def test_the_tracker_item_shows_role_controller_age_and_viewers():
    view = control.tracker(entry(seat(1, 'boat'), [seat(2, 'boat-v1')]), HUMAN_RECORD, 2, {1, 2}.__contains__, 0)
    text = control_tracker.item(view, now=340.0).plain
    assert text.startswith('VIEW') and 'CTRL boat@boat-host' in text and '4m' in text
    assert '1 viewing' in text and 'O take' in text
    agent = control.tracker(entry(seat(1, 'boat')), agent_record(), 1, {1}.__contains__, 0)
    text = control_tracker.item(agent, now=230.0).plain
    assert 'claude-3f2a c4 via boat@boat-host' in text and 'VIEW' not in text
    assert len(control_tracker.item(agent, now=230.0, short=True)) < len(text)


def test_a_viewer_sees_view_and_the_controller_at_80_and_100_columns():
    import time
    from mhseals_testing.ui import chrome
    owner = seat(1, 'boat', started=time.time() - 240)
    tracker = control.tracker(entry(owner, [seat(2, 'boat-v1')]), HUMAN_RECORD, 2, {1, 2}.__contains__, 0)
    view = {'profile': 'blastoise-full-course', 'units': 'metric', 'tab': 'Drive', 'host': 'blastoise-orin',
            'domain': 42, 'deploy': None, 'authority': ('human', 'HUMAN'), 'armed': True,
            'armed_by': {'who': 'claude-3f2a', 'sim': True}, 'control_tracker': tracker}
    for width in (80, 100):
        row = chrome.top_row(view, width).plain
        assert len(row) <= width
        assert 'VIEW' in row and 'boat' in row.split('VIEW', 1)[1], (width, row)
        assert 'HUMAN' in row and 'ARMED' in row, (width, row)       # the safety marks stay
        assert 'blastoise-full-course' not in row and 'Correction' not in row, (width, row)
    wide = chrome.top_row(view, 200).plain
    assert 'blastoise-full-course' in wide and 'Correction' in wide and 'VIEW' in wide


def test_a_missing_or_stale_authority_record_shows_an_unknown_controller_never_a_human():
    alive = {1, 2}.__contains__
    state = entry(seat(1, 'boat'), [seat(2, 'boat-v1')])
    stale = ViewerFeed(clock=lambda: 100.0)
    stale.on_record(agent_record())
    stale.clock = lambda: 1000.0                     # the owner stopped publishing
    for record in (None, stale.view()['authority_record']):
        view = control.tracker(state, record, 2, alive, 160.0)
        assert view['controller']['kind'] == 'unknown' and view['controller']['name'] == 'boat@boat-host'
        text = control_tracker.item(view, now=160.0).plain
        assert 'unknown' in text and '● CTRL' not in text
        assert 'unknown' in control_tracker.item(view, short=True).plain
        assert control.summary(view).startswith('controller unknown') and 'human' not in control.summary(view)


# --- launching a viewer and taking control (takeover.decide) -----------------------------------

def test_a_viewer_starts_without_the_menu_and_waits_for_the_next_owner(tmp_path):
    world = World(tmp_path, alive={1, 2})
    world.registry().claim(seat(1, 'boat'))
    choice = decide(world.registry(), console(), seat(2, 'boat-v1'), ask=ask([]), mode='view')
    assert (choice.mode, choice.watching) == ('watch', 1)
    empty = World(tmp_path / 'b', alive={2})
    assert decide(empty.registry(), console(), seat(2, 'v'), ask=ask([]), mode='view').mode == 'quit'
    assert empty.now >= 15.0                            # it waited for an owner (a takeover in progress)


def test_take_hands_over_from_a_live_owner_naming_the_taker_and_asking_for_the_revoke(tmp_path):
    world = World(tmp_path, alive={1, 2, 9})
    owner = world.registry()
    owner.claim(seat(1, 'boat'))
    owner.set_children({'bag': {'pid': 9}})
    handovers = []
    world.responder = lambda: (handovers.append(owner.state()['handover']), owner.release(1))
    choice = decide(world.registry(), console(), seat(2, 'boat-v1'), ask=ask([]), mode='take')
    assert choice.mode == 'adopt' and choice.children == {'bag': {'pid': 9}}
    assert world.signals == [(1, signal.SIGUSR1)]
    assert handovers[0]['revoke'] is True and handovers[0]['by']['client'] == 'boat-v1'
    assert world.registry().owner()['pid'] == 2 and world.registry().state()['handover'] is None


def test_take_adopts_a_stale_owner_and_falls_back_to_the_menu_when_the_owner_holds_on(tmp_path):
    world = World(tmp_path, alive={2, 9})
    world.registry().claim(seat(1, 'boat'))
    world.registry().set_children({'bag': {'pid': 9}})
    assert decide(world.registry(), console(), seat(2, 'v'), ask=ask([]), mode='take').mode == 'adopt'
    stuck = World(tmp_path / 'b', alive={1, 2})
    stuck.registry().claim(seat(1, 'boat'))
    choice = decide(stuck.registry(), console(), seat(2, 'v'), ask=ask(['4']), mode='take')
    assert choice.mode == 'quit' and stuck.registry().owner()['pid'] == 1      # never half-owned


def test_two_takers_race_one_controller_wins(tmp_path):
    world = World(tmp_path, alive={1, 2, 3})
    owner = world.registry()
    owner.claim(seat(1, 'boat'))
    world.responder = lambda: owner.release(1)
    assert decide(world.registry(), console(), seat(2, 'v1'), ask=ask([]), mode='take').mode in ('own', 'adopt')
    world.responder = None                              # pid 2 is a live owner that ignores signals
    assert decide(world.registry(), console(), seat(3, 'v2'), ask=ask(['4']), mode='take').mode == 'quit'
    assert world.registry().owner()['pid'] == 2


# --- the leaving owner: revoke, name the taker, restart as a viewer -------------------------------

class Owner(SimpleNamespace):
    close_handover = App.close_handover


def leaving_owner(tmp_path, handover):
    world = World(tmp_path, alive={1, 2})
    registry = world.registry()
    registry.claim(seat(1, 'boat'))
    registry.update(handover=handover)
    holds = []
    arbiter = Arbiter(lambda record: None, lambda *a, **k: None, hold=lambda cause, by: holds.append(cause),
                      clock=lambda: 1000.0)
    arbiter.table = ClaimTable()
    arbiter.table.claim('agent', 'claude-3f2a', 1000.0)
    app = Owner(registry=registry, arbiter=arbiter, export=lambda: None,
                processes=SimpleNamespace(records=lambda: {}, release=lambda c: None),
                ros=SimpleNamespace(emit=lambda *a, **k: None), state=AppState())
    return app, arbiter, holds


def test_a_human_takeover_revokes_the_agent_claim_and_names_the_taker(tmp_path):
    app, arbiter, holds = leaving_owner(tmp_path, {'to': 2, 'deadline': 9, 'revoke': True, 'by': seat(2, 'boat-v1')})
    app.close_handover()
    assert arbiter.table.holder() is None and arbiter.ended == {'c1': 'revoked'} and holds == ['revoke']
    assert app.state.operator.handed_by['client'] == 'boat-v1'


def test_a_plain_handover_keeps_the_claim_for_the_successor(tmp_path):
    app, arbiter, holds = leaving_owner(tmp_path, {'to': 2, 'deadline': 9})
    app.close_handover()
    assert arbiter.table.holder()['id'] == 'c1' and not holds and app.state.operator.handed_by is None


def ending(restart_as, detaching, handed_by):
    """What restart_as reads of an App: the Loop's detach and the Operator's handover."""
    return SimpleNamespace(services=SimpleNamespace(loop=SimpleNamespace(detaching=detaching), operator=SimpleNamespace(
        restart_as=restart_as, handed_by=handed_by)))


def test_restart_as_takes_from_a_viewer_and_views_after_a_human_takeover(monkeypatch):
    monkeypatch.setattr(sys, 'orig_argv', ['python3', '/x/boat_test', '--profile', 'sim', '--view'])
    viewer = ending('take', False, None)
    assert app_module.restart_as(viewer)[0] == ['python3', '/x/boat_test', '--profile', 'sim', '--take']
    old = ending(None, True, seat(2, 'boat-v1'))
    argv, notice = app_module.restart_as(old)
    assert argv[-1] == '--view' and 'boat-v1@boat-host (10.0.0.5)' in notice
    assert app_module.restart_as(ending(None, True, None)) is None
    assert app_module.restart_as(ending(None, False, None)) is None


def test_view_and_take_are_exclusive_flags():
    assert app_module.build_parser().parse_args(['--view']).view
    try:
        app_module.build_parser().parse_args(['--view', '--take'])
    except SystemExit:
        return
    raise AssertionError('--view --take parsed')


# --- E and Space from any client ------------------------------------------------------------------

class Viewer(SimpleNamespace):
    operator_stop = App.operator_stop
    revoke_agent = App.revoke_agent
    client_name = App.client_name
    take_control = App.take_control
    set_command = App.set_command
    say = App.say


def viewer(monkeypatch):
    monkeypatch.setenv('BOAT_TUI_CLIENT', 'boat-v1')
    holds, stops = [], []
    app = Viewer(watching=1, plant=SimpleNamespace(hold=lambda cause, by='tui': holds.append((cause, by))),
                 command=None, notice='', review=None, state=AppState())
    app.stop = lambda reason, by='operator': stops.append(reason)
    return app, holds, stops


def test_space_in_a_viewer_holds_the_boat_attributed_to_the_viewer(monkeypatch):
    app, holds, stops = viewer(monkeypatch)
    app.operator_stop()
    host = socket.gethostname()
    assert holds == [('keys', f'view boat-v1@{host}')] and stops == [f'stopped by viewer boat-v1@{host}']


def test_e_in_a_viewer_holds_at_once_then_asks_the_desk_to_revoke(monkeypatch):
    app, holds, _ = viewer(monkeypatch)
    app.revoke_agent()
    assert holds and holds[0][0] == 'keys'             # the stop never waits on the owner
    req = [line for line in ch.channel_path().read_text().splitlines() if '"revoke"' in line]
    assert len(req) == 1 and f'view:boat-v1@{socket.gethostname()}' in req[0]


def test_o_in_the_owner_does_nothing(monkeypatch):
    app, _, _ = viewer(monkeypatch)
    app.watching = 0
    app.take_control()
    assert app.notice == 'You control this session' and not getattr(app, 'quit', False)


def register_viewer(rig, pid=ME):
    rig.registry.data.setdefault('guests', {})[str(pid)] = {'procedure': 'watch', 'client': 'boat-v1'}


def test_the_desk_revokes_the_agent_claim_for_a_live_viewer(rig):
    claim_id = claim(rig)
    register_viewer(rig)
    req = ch.request('revoke', 'view:boat-v1@h')
    rig.desk.tick()
    assert final(req['id'])['ok'] and rig.app.arbiter.table.holder() is None
    assert rig.app.arbiter.ended[claim_id] == 'revoked' and 'viewer boat-v1@h' in str(rig.app.notice)


def test_the_desk_refuses_a_revoke_from_an_agent_or_a_gone_viewer(rig):
    claim(rig)
    agent = ch.request('revoke', 'claude-other')        # no revoke verb for agents, and not a viewer pid
    rig.desk.tick()
    assert final(agent['id'])['error'] == 'refused'
    register_viewer(rig, pid=999999)                    # a viewer that is gone (not this pid)
    gone = ch.request('revoke', 'view:boat-v1@h')
    rig.desk.tick()
    assert final(gone['id'])['error'] == 'refused' and rig.app.arbiter.table.holder() is not None


def test_a_viewer_reads_the_owners_record_and_goes_stale_without_it():
    clock = [0.0]
    feed = ViewerFeed(clock=lambda: clock[0])
    assert feed.view()['authority_paused']
    feed.on_record(dict(agent_record(), t_ns=0))
    assert feed.view()['authority_record']['owner'] == 'agent'
    clock[0] = 10.0                                     # the owner stopped publishing (gone, handing over)
    assert feed.view() == {'authority_record': None, 'authority_paused': True}
    feed.on_record({'junk': 1})
    assert feed.view()['authority_paused']


# --- the launcher never touches the caller's tmux server -----------------------------------------

FAKE_TMUX = """#!/bin/sh
printf '%s %s %s|TMUX=%s\\n' "$1" "$2" "$3" "${TMUX:-}" >> "$LOG"
echo "$*" >> "$LOG.args"
case " $* " in
    *" has-session "*) exit 1 ;;
    *" display-message "*) echo 0 ;;
esac
exit 0
"""


def test_boat_tui_only_talks_to_its_own_socket_with_tmux_cleared(tmp_path):
    import subprocess
    from pathlib import Path
    shim = tmp_path / 'bin'
    shim.mkdir()
    (shim / 'tmux').write_text(FAKE_TMUX)
    (shim / 'tmux').chmod(0o755)
    log = tmp_path / 'tmux.log'
    script = Path(__file__).resolve().parents[1] / 'tools' / 'boat_tui'
    env = {'PATH': f"{shim}:{os.environ['PATH']}", 'LOG': str(log), 'TMUX': '/tmp/caller-server,1,0',
           'TMUX_PANE': '%1', 'BOAT_TUI_SOCKET': 'boat-test', 'BOAT_TUI_START_S': '0', 'HOME': str(tmp_path)}
    for verb in (['up', '--', '--profile', 'sim'], ['view'], ['status']):
        subprocess.run(['sh', str(script), '--container', '', *verb], env=env, capture_output=True, text=True,
                       timeout=30, stdin=subprocess.DEVNULL)
    up = subprocess.run(['sh', str(script), '--container', '', 'up'], env=env, capture_output=True, text=True,
                        timeout=30)
    calls = log.read_text().splitlines()
    assert calls and all(c.startswith('-L boat-test ') and c.endswith('|TMUX=') for c in calls), calls
    assert not any('kill-server' in c for c in calls)
    assert '-L boat-test set-option -g allow-passthrough on' in (tmp_path / 'tmux.log.args').read_text().splitlines()
    assert up.stdout.strip() == 'tmux -L boat-test attach -t boat'
    # over ssh the line names tmux by its path: the station's non-login ssh shell may not have it on PATH
    remote = subprocess.run(['sh', str(script), '--container', '', 'up'], env=dict(env, SSH_CONNECTION='1 2 3 4'),
                            capture_output=True, text=True, timeout=30)
    assert remote.stdout.strip().endswith(f'{shim}/tmux -L boat-test attach -t boat'), remote.stdout


def test_boat_tui_replaces_a_dead_pane_on_a_private_tmux_server():
    """alive() asked display-message for =boat, which prints nothing on tmux 3.8, so a dead pane counted as running:
    up said "already runs" and view attached to it (11-01). A private server: its own TMUX_TMPDIR and socket."""
    import shutil
    import subprocess
    import tempfile
    import time
    from pathlib import Path
    if shutil.which('tmux') is None:
        pytest.skip('no tmux')
    root = Path(tempfile.mkdtemp(prefix='bt', dir='/tmp'))     # short: tmux sockets cap the path length
    shim = root / 'bin'
    shim.mkdir()
    (shim / 'boat_test').write_text('#!/bin/sh\nexec sleep 300\n')
    (shim / 'boat_test').chmod(0o755)
    env = {k: v for k, v in os.environ.items() if k not in ('TMUX', 'TMUX_PANE')}
    env.update(PATH=f"{shim}:{os.environ['PATH']}", HOME=str(root), XDG_CONFIG_HOME=str(root), SHELL='/bin/sh',
               TMUX_TMPDIR=str(root), BOAT_TUI_SOCKET='bt', BOAT_TUI_START_S='0.5')
    script = Path(__file__).resolve().parents[1] / 'tools' / 'boat_tui'

    def tmux(*args):
        return subprocess.run(['tmux', '-L', 'bt', *args], env=env, capture_output=True, text=True, timeout=10)

    def boat_tui(*args):
        return subprocess.run(['sh', str(script), '--container', '', *args], env=env, capture_output=True,
                              text=True, timeout=30, stdin=subprocess.DEVNULL)

    def dead(name):
        return tmux('display-message', '-p', '-t', f'={name}:', '#{pane_dead}').stdout.strip()

    def end_pane(name):             # the pane exits and stays, as remain-on-exit keeps a failed boat_test
        tmux('respawn-pane', '-k', '-t', f'={name}:', 'true')
        for _ in range(50):
            if dead(name) == '1':
                return
            time.sleep(0.1)
        raise AssertionError(f'{name} pane never exited')

    try:
        tmux('new-session', '-d', '-s', 'boat', '-x', '80', '-y', '20')
        tmux('set-option', '-t', '=boat:', 'remain-on-exit', 'on')
        end_pane('boat')
        view = boat_tui('view')
        assert view.returncode == 4 and 'no boat session' in view.stderr, view
        up = boat_tui('up')
        assert up.returncode == 0 and 'boat_test runs in tmux session boat' in up.stderr, up
        assert dead('boat') == '0'
        view = boat_tui('view')
        assert view.returncode == 0 and 'viewer boat-v1' in view.stderr, view
        end_pane('boat-v1')
        view = boat_tui('view')                     # the dead viewer is replaced, not reused
        assert view.returncode == 0 and 'viewer boat-v1' in view.stderr and dead('boat-v1') == '0', view
        again = boat_tui('up')
        assert 'boat already runs' in again.stderr, again
    finally:
        tmux('kill-server')                         # this test's private server only
        shutil.rmtree(root, ignore_errors=True)
