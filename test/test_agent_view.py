"""Agent-control visibility (agent-control 5): banner act and lease, the working item, the Activity
agent view, timeline and strip agent points, and the Authority popup columns. Desk state is a fake
built from ``harness.agent_channel`` lines, the AgentDesk's ``lines``/``act``/``lease`` view keys.
"""

import pytest

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import authority_chip, chrome, layout, strip, theme
from mhseals_testing.ui.panels import agent_log
from mhseals_testing.ui.popups import authority as authority_popup
from mhseals_testing.ui.timeaxis import Axis
from mhseals_testing.ui.timeline import Timeline
from test_dashboard import monitor, text, view

NOW = 1_800_000_000.0
NS = int(NOW * 1e9)
WHO = 'claude-3f2a'


def ns(at):
    return NS + int(at * 1e9)


def claim(**fields):
    return dict(dict(id='c4', owner='agent', who=WHO, state='active', pid=5120, host='blastoise',
                     reason='tune MPPI', since_ns=NS - int(240e9), seen_ns=NS - int(10e9), lease_s=120),
                **fields)


def record(*claims, owner='agent'):
    return {'owner': owner, 'claims': list(claims), 't_ns': NS, 'since_ns': NS - int(240e9)}


def note(at, phase, tool_id, cmd='', **fields):
    return {'t_ns': ns(at), 'who': WHO, 'phase': phase, 'tool_id': tool_id, 'cmd': cmd, 'exit': None,
            'dur_s': None, 'target': 'blastoise', **fields}


def desk(**changes):
    """The AgentDesk view (ui/feeds/agent.py ``summary``) mid-run."""
    return {
        'act': dict(id='r2', op='run', who=WHO, claim='c4', label='run square', state='started',
                    elapsed_s=34.0, motion_lease_s=8.0, motion_s=10, cause=None),
        'queue': [],
        'recent': [dict(id='r1', op='goal', who=WHO, label='goal 12, -3', t_ns=ns(-20), ok=False,
                        error='refused: no_mux', hint='agent motion needs a muxed profile')],
        'notes': [note(-40, 'pre', 't1', 'ssh blastoise ros2 topic hz /odom'),
                  note(-38, 'post', 't1', 'ssh blastoise ros2 topic hz /odom', exit=0, dur_s=2.1),
                  note(-30, 'pre', 't2', 'ssh -t blastoise ros2 node list'),
                  note(-29, 'post', 't2', exit=1, dur_s=0.4)],
        'config_error': None, **changes}


# -- banner ----------------------------------------------------------------------------------------

def test_banner_names_the_agent_its_act_lease_and_age():
    text_ = authority_chip.banner(record(claim()), NOW, desk=desk())
    parts = text_.split(theme.SEPARATOR)
    assert parts[:4] == [f'{theme.AGENT_GLYPH} AGENT DRIVING', WHO, f'{theme.FOCUS} run square 34 s', 'lease ok']
    assert parts[-1] == '4m'


@pytest.mark.parametrize('desk_, state', [
    ({'act': None}, 'IDLE'), (None, 'DRIVING'),
    ({'act': {'op': 'run', 'label': 'run square', 'state': 'waiting_arm'}}, 'WAITING ARM'),
    ({'act': {'op': 'ask', 'label': 'ask', 'state': 'asking'}}, 'ASKING')])
def test_banner_states(desk_, state):
    assert f'AGENT {state}' in authority_chip.banner(record(claim()), NOW, desk=desk_)


def test_a_low_lease_shows_its_seconds_in_the_warn_cell():
    text_ = authority_chip.banner(record(claim(seen_ns=NS - int(100e9))), NOW, desk={'act': None})
    assert 'lease 20 s' in text_
    row = chrome.banner(text_, 120)
    start = row.plain.index('lease 20 s')
    assert any(span.style == theme.AGENT_LOW and span.start == start for span in row.spans)


def test_motion_lease_from_the_desk_wins_over_the_claim():
    assert authority_chip.lease(claim(), desk(), NOW) == ('lease ok', False)
    low = desk(act=dict(desk()['act'], motion_lease_s=2.5))
    assert authority_chip.lease(claim(), low, NOW) == ('lease 2 s', True)


def test_an_act_of_another_claim_is_not_this_claims():
    assert authority_chip.act(desk(), 'c9') is None and authority_chip.act(desk(), 'c4')['id'] == 'r2'


@pytest.mark.parametrize('width', (120, 100, 80, 60))
def test_narrow_banner_drops_pid_and_reason_before_the_act_and_keeps_the_keys(width):
    text_ = authority_chip.banner(record(claim()), NOW, desk=desk())
    row = chrome.banner(text_, width).plain
    assert len(row) <= width and row.rstrip().endswith('E revoke' + theme.SEPARATOR + 'Space stop')
    if width >= 80:
        assert WHO in row and 'run square 34 s' in row
    if width <= 80:
        assert 'pid 5120' not in row and 'tune MPPI' not in row


# -- working item ----------------------------------------------------------------------------------

def test_working_item_shows_recent_notes_only_without_a_claim():
    assert authority_chip.working(desk(), record(owner='none'), NOW) == (WHO, pytest.approx(2.0, abs=27.1))
    assert authority_chip.working(desk(), record(claim()), NOW) is None
    assert authority_chip.working(desk(), record(owner='none'), NOW + 120) is None


def test_working_item_sits_left_of_the_host():
    page = view(load_profile('boat'), host='blastoise', agent_desk=desk(), authority_record=record(owner='none'))
    item = chrome.working_item(page, NOW)
    assert item.plain == f'{theme.WORKING_GLYPH} {WHO} 29 s' and item.style == theme.AGENT_DIM
    assert chrome.working_item(dict(page, authority_record=record(claim())), NOW) is None


# -- activity agent view ---------------------------------------------------------------------------

def test_fold_pairs_notes_by_tool_id_and_puts_the_act_on_top():
    rows = agent_log.fold(desk(queue=[dict(id='r3', op='goal', who=WHO)]))
    assert [r['what'] for r in rows[:2]] == [f'{theme.FOCUS} run square', f'{theme.FOCUS} goal']
    assert (rows[0]['state'], rows[0]['result']) == ('busy', '34 s') and rows[1]['result'] == 'queued'
    refused = next(r for r in rows if 'goal 12' in r['what'])
    assert (refused['state'], refused['result']) == ('warn', 'refused')
    assert refused['hint'] == f'no_mux{theme.SEPARATOR}agent motion needs a muxed profile'
    hz = next(r for r in rows if 'hz' in r['what'])
    assert hz['what'] == 'blastoise$ ros2 topic hz /odom'
    assert (hz['state'], hz['result']) == ('good', '2.1 s') and len(hz['lines']) == 2
    failed = next(r for r in rows if 'node list' in r['what'])
    assert (failed['what'], failed['state'], failed['result']) == ('blastoise$ ros2 node list', 'bad', 'exit 1')


def test_a_waiting_or_cancelled_act_says_so():
    waiting = agent_log.fold(desk(act=dict(desk()['act'], state='waiting_arm')))[0]
    assert (waiting['state'], waiting['result']) == ('busy', 'waiting arm')
    lost = agent_log.fold(desk(act=dict(desk()['act'], cause='overridden: keys')))[0]
    assert (lost['state'], lost['result']) == ('warn', 'overridden: keys')


def test_rows_are_cached_until_something_changes():
    desk_ = desk()
    first = agent_log.rows(desk_)
    assert agent_log.rows(dict(desk_, act=dict(desk_['act'], motion_lease_s=3.0))) is first
    desk_['notes'].append(note(0, 'pre', 't3', 'ssh blastoise uptime'))
    assert any(r['what'] == 'blastoise$ uptime' for r in agent_log.rows(desk_))


def page(width, height, **extra):
    profile = load_profile('boat')
    page_ = view(profile, host='blastoise', armed=True, authority=('agent', f'AGENT {WHO} 4m'),
                 activity='agent', focus='activity', agent_desk=desk(), authority_record=record(claim()),
                 agent=authority_chip.banner(record(claim()), NOW, desk=desk()), **extra)
    return text(layout.render(monitor().snapshot(profile.required_tf), page_, width, height), width, height)


@pytest.mark.parametrize('width, height', [(120, 40), (80, 24)])
def test_agent_view_and_banner_render_in_the_page(width, height):
    out = page(width, height)
    rows = out.splitlines()
    assert all(len(r) <= width for r in rows)
    assert 'AGENT DRIVING' in rows[1] and 'E revoke' in rows[1]
    assert f'{theme.FOCUS} Activity' in out and 'agent' + theme.SEPARATOR + '4' in out
    assert 'run square' in out
    if height >= 40:
        assert 'blastoise$ ros2 topic' in out and '! refused' in out


def test_empty_agent_view_says_where_rows_come_from():
    body, subtitle = agent_log.body({'cursors': {'activity': 0}}, 5)
    assert subtitle == 'agent' and 'No agent activity' in body.plain


def test_detail_shows_every_line_of_the_row():
    shown = {}

    class App:
        def show(self, title, message=None, rows=None, width=60):
            shown.update(title=title, message=message)
    agent_log.detail(App(), desk(), 0)
    assert 'run square' in shown['message'] and '"id": "r2"' in shown['message']


# -- timeline and strip ----------------------------------------------------------------------------

def journal():
    return [
        {'phase': 'claim', 't_ns': NS, 'owner': 'agent', 'who': WHO, 'id': 'c4'},
        {'phase': 'claim', 't_ns': NS + 1, 'owner': 'procedure', 'who': 'square', 'id': 'c5'},
        {'phase': 'agent_act', 't_ns': NS + int(5e9), 'id': 'r2', 'state': 'started', 'who': WHO, 'op': 'run'},
        {'phase': 'square_start', 't_ns': NS + int(5.5e9), 'procedure': 'square', 'key': 'w'},
        {'phase': 'marker', 't_ns': NS + int(8e9), 'id': 3, 'kind': 'agent', 'note': 'gate 1 passed'},
        {'phase': 'square_stop', 't_ns': NS + int(15e9), 'procedure': 'square'},
        {'phase': 'agent_act', 't_ns': NS + int(15e9), 'id': 'r2', 'state': 'done', 'who': WHO, 'op': 'run'},
        {'phase': 'square_start', 't_ns': NS + int(16e9), 'procedure': 'square', 'key': 'w'},
        {'phase': 'claim_expire', 't_ns': NS + int(20e9), 'owner': 'agent', 'who': WHO, 'id': 'c4'},
    ]


def test_timeline_places_agent_points_and_labels_agent_runs():
    tl = Timeline()
    for e in journal():
        tl.add(e)
    points = [p for p in tl.points() if p.kind == 'agent']
    assert [p.label for p in points] == [f'claim · {WHO}', f'run started · {WHO}',
                                         'gate 1 passed', f'run done · {WHO}', f'lease expired · {WHO}']
    assert points[-1].sev == 1
    assert [s.label for s in tl.spans() if s.kind == 'proc'] == ['agent:square', 'square']
    assert [i.what for i in tl.items('markers')] == ['gate 1 passed']


@pytest.mark.parametrize('ascii_mode, glyph', [(False, '◆'), (True, 'A')])
def test_strip_draws_the_agent_glyph(ascii_mode, glyph):
    axis = Axis(NOW, NOW + 60, 40, [(NOW + 30, NOW + 30)])
    events = strip.rows(axis, points=[strip.Point(NOW + 30, 'agent')], ascii_mode=ascii_mode, color=False)[0]
    assert glyph in events.plain


# -- authority popup -------------------------------------------------------------------------------

def test_popup_claims_gain_lease_and_act_and_a_config_row():
    body, ids = authority_popup.rows(record(claim()), [], NOW, desk=desk())
    row = dict(body)['› c4' if not theme.ASCII else '> c4']
    assert 'lease 110/120 s' in row and 'run square 34 s' in row
    assert body[-1][0] == 'Agent' and 'agent.yaml' in body[-1][1]


def test_v_adds_the_agent_view_only_while_a_desk_runs():
    assert [agent_log.next_view(v, None) for v in ('results', 'log', 'agent')] == ['log', 'results', 'results']
    assert [agent_log.next_view(v, {}) for v in ('results', 'log', 'agent')] == ['log', 'agent', 'results']


def test_control_group_shows_the_agent_settings():
    from mhseals_testing.ui.panels.status import status_rows
    control = authority_chip.control(None, None, NOW, 'allow', agent_motion='confirm')
    rows = {r['name']: r for r in status_rows({**monitor().snapshot(), 'control': control},
                                              load_profile('boat').sensors) if r['group'] == 'Control'}
    assert rows['Agent motion']['setting'] == 'agent_motion' and 'confirm' in rows['Agent motion']['value']
    assert 'Agent view' not in rows


def test_agent_hues_stay_apart_from_status_class_and_trail_colours():
    """The agent roles never read as a severity, a buoy class or a trail colour (truecolor and 256)."""
    from mhseals_testing.ui import palette
    tokens = palette.load_theme(overlay=False)[0]

    def rgb(token, mode):
        value = tokens.resolve(token, mode)
        return [int(value[i:i + 2], 16) for i in (1, 3, 5)]
    others = [t for t in tokens.tokens if t in ('ok', 'warn', 'fail', 'busy', 'unknown')
              or t.startswith(('obj.', 'gps.', 'xte.', 'spd.', 'track.', 'fov.'))]
    for agent in ('auth.agent', 'auth.agent_dim'):
        for other in others:
            for mode in ('dark', 'light'):
                gap = sum((a - b) ** 2 for a, b in zip(rgb(agent, mode), rgb(other, mode))) ** .5
                assert gap >= 30, (agent, other, mode)
                assert tokens.resolve(agent, mode, '256') != tokens.resolve(other, mode, '256'), (agent, other)
