"""The session owner runs the arbiter and keeps its latest record; guests and review run none."""

import os
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import authority_chip, chrome, layout, theme
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.feeds import authority as feed_module

from test_app import make_app
from test_chrome import check
from test_dashboard import monitor, text, view


class FakeBus:
    """What bind would install: collects what the arbiter publishes."""

    def __init__(self):
        self.published, self.bound = [], []

    def bind(self, ros, arbiter):
        self.bound.append(arbiter)
        arbiter.publish = self.published.append


def start(app, clock):
    bus = FakeBus()
    feed = feed_module.start(app, app.ros, bind=bus.bind)
    feed.clock = feed.arbiter.clock = lambda: clock[0]
    feed.arbiter.on_mux({'input': 'auto', 't_ns': int(clock[0] * 1e9)})
    return feed, bus


def test_procedure_start_gives_an_ours_record_and_an_authority_event(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, bus = start(app, clock)
    feed.arbiter.tick()
    assert bus.bound == [feed.arbiter] and feed.record['owner'] == 'none'
    app.running, app.run_id = 'Characterize', 'r1'
    clock[0] += 0.25
    feed.arbiter.tick()
    record = feed.view()['authority_record']
    assert record['owner'] == 'procedure' and 'Characterize' in record['source']
    assert bus.published[-1]['owner'] == 'procedure'      # sent on the bound transport too
    assert app.feeds[-1] is feed
    changes = [f for phase, f in app.ros.lines if phase == 'authority']
    assert changes and changes[-1]['to']['owner'] == 'procedure'


def test_a_stale_record_reads_unknown_and_paused(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    assert feed.view() == {'authority_record': None, 'authority_paused': True}   # nothing yet
    feed.arbiter.tick()
    assert feed.view()['authority_paused'] is False
    clock[0] += 10.0
    assert feed.view() == {'authority_record': None, 'authority_paused': True}


def test_guests_and_review_create_no_arbiter(tmp_path):
    app, _ = make_app(tmp_path)
    bus = FakeBus()
    app.watching = 4242
    assert feed_module.start(app, app.ros, bind=bus.bind) is None
    app.watching, app.review = 0, SimpleNamespace()
    assert feed_module.start(app, app.ros, bind=bus.bind) is None
    app.review, app.args.review = None, '/some/session'
    assert feed_module.start(app, app.ros, bind=bus.bind) is None
    assert bus.bound == []


def test_the_mission_procedures_own_task_reads_as_the_procedure(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    app.running, app.run_id = 'Mission rehearsal', 'r1'
    app._watcher = SimpleNamespace(status={'status': 'RUNNING', 'name': 'gate'})
    feed.arbiter.tick()
    assert feed.record['owner'] == 'external' and feed.record['conflict']   # not ours: an outside task
    running = {'task_gate'}
    app.processes = SimpleNamespace(processes={'task_gate': None}, running=lambda n: n in running)
    clock[0] += 1
    feed.arbiter.tick()
    assert feed.record['owner'] == 'procedure' and not feed.record['conflict']
    assert feed.record['procedure']['task'] == 'gate'


def test_an_unclaimed_publisher_with_a_nonzero_cmd_vel_is_external(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    app.ros.endpoints = lambda topic: [{'node': '/some_agent', 'gid': 'aa', 'self': False}]
    feed.on_cmd(True)
    feed.arbiter.tick()
    assert feed.record['owner'] == 'external' and '/some_agent' in feed.record['source']


# --- the chip word (ui U4) -------------------------------------------------------------------

HERE = Path(__file__).parent
NOW = 1000.0
NS = int(NOW * 1e9)
RC = {'valid': True, 'mode': 'GUIDED', 'rate_hz': 25.1, 'latency_ms': 46}


def record(owner, **fields):
    return {'owner': owner, 'since_ns': NS - 40 * 10 ** 9, **fields}


STATES = {
    'idle': {'owner': 'none'},
    'keys': record('keyboard'),
    'rc': record('rc', rc=RC),
    'hold': record('hold', holds=[{'by': 'keys', 'cause': 'manual', 'since_ns': NS - 40 * 10 ** 9}]),
    'agent': record('agent', since_ns=NS - 240 * 10 ** 9,
                    claims=[{'owner': 'agent', 'who': 'tune_mppi', 'state': 'active'}]),
    'procedure': record('procedure', procedure={'name': 'characterize', 'progress': '7/14'}),
    'goal': record('nav', nav={'goals': [{'status': 'executing', 'by': 'nav', 'distance_m': 12.3}]}),
    'task': record('task', task={'name': 'pair_test', 'status': 'RUNNING'}),
    'unclaimed': record('external', source='? goal (unclaimed)'),
}
WORDS = {'idle': None, 'keys': ('human', 'KEYS'), 'rc': ('human', 'RC 46 ms'),
         'hold': ('hold', 'HOLD keys 40 s'), 'agent': ('agent', 'AGENT tune_mppi 4m'),
         'procedure': ('ours', 'CHARACTERIZE 7/14'), 'goal': ('nav', 'GOAL 12 m'),
         'task': ('ours', 'TASK pair_test'), 'unclaimed': ('unclaimed', '? UNCLAIMED goal')}


@pytest.mark.parametrize('name', STATES)
def test_chip_word_and_hue_per_owner(name):
    assert authority_chip.chip(STATES[name], NOW) == WORDS[name]


def test_unknown_and_foreign_drivers_are_unclaimed():
    assert authority_chip.chip({'owner': 'unknown'}, NOW) == ('unclaimed', '? authority stale')
    assert authority_chip.chip({'owner': 'external', 'source': '? /velocity_smoother'}, NOW) == \
        ('unclaimed', '? /velocity_smoother')


def dump(width=120):
    profile = load_profile('boat')
    snapshot = monitor().snapshot(profile.required_tf)
    out = []
    for name in STATES:
        page = view(profile, host='boat', armed=True, authority=WORDS[name])
        out.append(f'== {name}\n' + text(layout.header(snapshot, page, width), width, 3).splitlines()[0].rstrip())
    return '\n'.join(out) + '\n'


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_header_chip_golden():
    check('authority/chips_120.txt', dump())


def test_header_chip_golden_ascii_brackets_the_word():
    env = {k: v for k, v in os.environ.items() if k != 'TERM'}
    code = (f'import sys; sys.path.insert(0, {str(HERE)!r}); import test_authority_feed as t; '
            'sys.stdout.write(t.dump())')
    done = subprocess.run([sys.executable, '-c', code], env={**env, 'MHS_ASCII': '1'},
                          capture_output=True, text=True, check=True, cwd=HERE.parent)
    done.stdout.encode('ascii')
    assert '[AGENT tune_mppi 4m]' in done.stdout and '[KEYS]' in done.stdout
    check('authority/chips_120_ascii.txt', done.stdout)


# --- grammar invariants 4 and 5 (ui.md section 1) -----------------------------------------------

def solid(item):
    style = str(item.style)
    return 'reverse' in style or ' on ' in style or theme.ASCII and item.plain.startswith('[')


@pytest.mark.parametrize('name', STATES)
def test_invariant_4_header_chips_are_arm_review_or_authority(name):
    page = view(load_profile('boat'), armed=True, authority=WORDS[name])
    chips = [c.plain.strip(' []') for c in chrome.chips(page) if solid(c)]
    assert set(chips) <= {'ARMED', 'REVIEW', (WORDS[name] or ('', ''))[1]}
    assert chrome.chips(dict(page, review=True))[0].plain.strip(' []') == 'REVIEW'


def test_invariant_5_authority_hues_never_colour_status_glyphs():
    tokens = {kind: token for kind, token in theme.CHIPS.items() if kind != 'armed'}
    assert all(token.startswith('auth.') for token in tokens.values())
    assert theme.auth_style('unclaimed') == theme.BAD                 # the one allowed overlap
    assert theme.CHIPS['armed'] == 'fail'                              # and ARMED


def test_motion_subtitle_carries_the_authority_word():
    from mhseals_testing.ui.panels.motion import motion_panel
    page = view(load_profile('boat'), focus='motion', authority=('agent', 'AGENT tune_mppi 4m'))
    assert 'AGENT tune_mppi 4m' in text(motion_panel(monitor().snapshot(), page, 20), 70, 20)


def test_arbiter_is_one_attribute_and_the_view_follows_its_record(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    assert app.arbiter is feed.arbiter
    feed.arbiter.tick()
    assert app.view()['authority'] is None                     # fresh and idle
    app.running, app.run_id = 'Characterize', 'r1'
    clock[0] += 0.25
    feed.arbiter.tick()
    assert app.view()['authority'] == ('ours', 'CHARACTERIZE')


def test_a_stale_or_missing_record_with_an_arbiter_reads_unclaimed_stale(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    app.manual = True                                          # the local guess must not win
    assert app.view()['authority'] == ('unclaimed', '? authority stale')   # no record yet
    feed.arbiter.tick()
    app.manual = False
    clock[0] += 10.0                                           # paused: the record went stale
    assert feed.view()['authority_paused']
    assert app.view()['authority'] == ('unclaimed', '? authority stale')


def test_without_an_arbiter_the_local_state_is_the_guess(tmp_path):
    app, _ = make_app(tmp_path)
    assert app.view()['authority'] is None
    app.manual = True
    assert app.view()['authority'] == ('human', 'KEYS')
    app.manual, app.running = False, 'Drift'
    assert app.view()['authority'] == ('ours', 'DRIFT')


# --- Control rows --------------------------------------------------------------------------------

def control(name, setting='allow'):
    rows = status_rows({**monitor().snapshot(), 'control': authority_chip.control(STATES[name], authority_chip.chip(STATES[name], NOW), NOW, setting)},
                       load_profile('boat').sensors)
    return {r['name']: r for r in rows if r['group'] == 'Control'}


def test_control_rows_in_order_and_before_components():
    profile = load_profile('boat')
    rows = status_rows({**monitor().snapshot(profile.required_tf),
                        'control': authority_chip.control(STATES['rc'], authority_chip.chip(STATES['rc'], NOW), NOW, 'allow')}, profile.sensors)
    names = [r['name'] for r in rows]
    assert names.index('Agents') < names.index('odometry')
    assert [r['name'] for r in rows if r['group'] == 'Control'] == ['Driving', 'RC', 'Hold', 'Agents']


def test_control_values():
    rows = control('rc')
    assert rows['Driving']['value'] == 'RC 46 ms'
    assert rows['RC']['value'] == f'GUIDED{theme.SEPARATOR}25 Hz ~46ms' and rows['RC']['state'] == 'OK'
    assert rows['Hold']['value'] == 'off'
    assert rows['Agents']['value'] == theme.pick('‹ allow ›', '< allow >')
    held = control('hold')
    assert held['Hold']['value'] == 'HOLD manual 40 s' and held['Hold']['style'] == theme.auth_style('hold')
    assert control('idle')['Driving']['value'] == 'idle' and control('idle')['Driving']['style'] is None
    assert control('idle')['RC']['value'] == 'no receiver'


def test_slow_rc_is_flagged():
    slow = dict(STATES['rc'], rc=dict(RC, rate_hz=2.0, latency_ms=560))
    rows = status_rows({**monitor().snapshot(), 'control': authority_chip.control(slow, authority_chip.chip(slow, NOW), NOW, 'allow')},
                       load_profile('boat').sensors)
    rc = next(r for r in rows if r['name'] == 'RC')
    assert rc['state'] == 'WARN' and not rc['plain'] and '2 Hz' in rc['value']


def test_no_control_rows_without_a_live_source():
    assert not [r for r in status_rows(monitor().snapshot(), load_profile('boat').sensors)
                if r['group'] == 'Control']


# --- the app: the feed replaces the local guess; Agents flips -------------------------------------

def test_invariant_5_authority_hue_on_the_driving_row_only():
    rows = control('agent')
    assert rows['Driving']['style'] == theme.auth_style('agent') and rows['Driving']['plain']   # no status glyph
    assert rows['Hold']['plain']


def test_driving_row_shows_stale_in_red_when_the_feed_is_paused(tmp_path):
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    snapshot = app.snapshot()
    driving = next(r for r in status_rows(snapshot, app.profile.sensors) if r['name'] == 'Driving')
    assert driving['value'] == '? authority stale' and driving['style'] == theme.BAD
    feed.arbiter.tick()
    snapshot = app.snapshot()
    assert snapshot['control']['driving'] is None             # fresh and idle
    clock[0] += 10.0
    assert app.snapshot()['control']['driving'] == ('unclaimed', '? authority stale')


def test_driving_row_falls_back_to_the_local_guess_without_an_arbiter(tmp_path):
    app, _ = make_app(tmp_path)
    app.manual = True
    assert app.snapshot()['control']['driving'] == ('human', 'KEYS')


def test_agents_row_flips_with_left_and_right(tmp_path):
    app, _ = make_app(tmp_path)
    rows = status_rows(app.snapshot(), app.profile.sensors)
    app.cursors['status'] = next(i for i, r in enumerate(rows) if r['name'] == 'Agents')
    assert app.settings.agents == 'allow'
    app.dispatch('right')
    assert app.settings.agents == 'block'
    assert status_rows(app.snapshot(), app.profile.sensors)[app.cursors['status']]['value'] == theme.pick('‹ block ›', '< block >')
    app.dispatch('left')
    assert app.settings.agents == 'allow'
    app.arbiter = SimpleNamespace(allow_agents=True)   # the one Arbiter attribute
    app.dispatch('left')
    assert app.arbiter.allow_agents is False
    app.dispatch('left')
    assert app.arbiter.allow_agents is True


def test_sources_report_when_a_mux_is_expected(tmp_path):
    app, _ = make_app(tmp_path)
    feed, _ = start(app, [1000.0])
    app.arming.armed = False
    got = feed.sources()
    assert (got['mux_expected'], got['muxed']) == (False, True)       # muxed, disarmed: idle
    app.arming.armed = True
    got = feed.sources()
    assert (got['mux_expected'], got['muxed']) == (True, True)        # muxed, armed: silent mux is unknown
    app.profile = load_profile('crane_direct')
    got = feed.sources()
    assert got['muxed'] is False                                      # unmuxed: derives as auto


def test_an_agent_claim_shows_the_banner_in_the_live_header(tmp_path):
    """rv2-29: App.view carries the banner text, so the 120-column header draws the key row."""
    app, _ = make_app(tmp_path)
    clock = [1000.0]
    feed, _ = start(app, clock)
    assert app.view()['agent'] is None
    claim, _ = feed.arbiter.table.claim('agent', 'tune_mppi', time.time(), reason='MPPI sweep', pid=5120)
    feed.arbiter.tick()
    parts = app.view()['agent'].split(theme.SEPARATOR)
    assert parts[:2] == [theme.pick('◆', '*') + ' AGENT DRIVING', 'tune_mppi'] and 'pid 5120' in parts
    page = text(layout.header(app.snapshot(), app.view(), 120), 120, 3)
    assert 'AGENT DRIVING' in page and 'E revoke' in page and 'Space stop' in page
    feed.arbiter.on_mux({'input': 'rc', 't_ns': int(clock[0] * 1e9)})   # RC override suspends it
    feed.arbiter.tick()
    assert 'AGENT PAUSED' in app.view()['agent']
    feed.arbiter.table.revoke(claim['id'], 1001.0)
    feed.arbiter.tick()
    assert app.view()['agent'] is None


@pytest.mark.parametrize('owner, shown', [('keyboard', False), ('hold', False), ('rc', True)])
def test_banner_names_rc_override_only_when_rc_owns_the_boat(owner, shown):
    """A valid receiver alone is not an override: keys or a hold taking over in GUIDED is not RC."""
    claim = {'owner': 'agent', 'who': 'tune_mppi', 'state': 'suspended'}
    text = authority_chip.banner(record(owner, rc=RC, claims=[claim]), NOW)
    assert text.startswith(theme.pick('◆', '*') + ' AGENT PAUSED')
    assert ('RC override' in text) is shown


@pytest.mark.parametrize('action, cause, by', [
    ('revoke', 'revoke', 'tui'), ('hold', 'keys', 'operator')])
def test_authority_hold_reaches_the_real_plant_with_attribution(tmp_path, action, cause, by):
    """Revoke and direct feed holds reach the mux and retain manual neutral."""
    import json
    pytest.importorskip('geometry_msgs')
    from mhseals_testing.ros.plant import Plant, NEUTRAL_REPEATS
    app, _ = make_app(tmp_path)
    made, sent = [], []

    def publisher(kind, topic, depth):
        made.append(topic)
        return SimpleNamespace(publish=lambda message: sent.append((topic, message)))

    app.plant = Plant(SimpleNamespace(node=SimpleNamespace(create_publisher=publisher)), app.profile)
    assert made == ['/command_mux/request']
    feed, _ = start(app, [1000.0])
    if action == 'revoke':
        claim = feed.arbiter.handle({'op': 'claim', 'request': 'r1', 'who': 'tune'})['claim']
        feed.arbiter.revoke(claim['id'])
    else:
        feed.hold(cause, by)
    requests = [json.loads(message.data) for topic, message in sent if topic == '/command_mux/request']
    assert requests == [{'op': 'hold', 'cause': cause, 'by': by}]
    manual = [message for topic, message in sent if topic == app.profile.manual_topic]
    assert len(manual) >= NEUTRAL_REPEATS
    assert all((m.linear.x, m.linear.y, m.angular.z) == (0.0, 0.0, 0.0) for m in manual)
    assert made == ['/command_mux/request', app.profile.manual_topic]


def test_plant_reuses_upfront_mux_publisher_and_unmuxed_hold_is_silent(tmp_path):
    pytest.importorskip('geometry_msgs')
    from mhseals_testing.ros.plant import Plant
    app, _ = make_app(tmp_path)
    made, sent = [], []

    def publisher(kind, topic, depth):
        made.append(topic)
        return SimpleNamespace(publish=sent.append)

    ros = SimpleNamespace(node=SimpleNamespace(create_publisher=publisher))
    plant = Plant(ros, app.profile)
    assert made == ['/command_mux/request']
    plant.hold('keys')
    plant.hold('revoke', 'operator')
    assert made.count('/command_mux/request') == 1
    assert made.count(app.profile.manual_topic) == 1
    made.clear()
    sent.clear()
    plant = Plant(ros, load_profile('crane_direct'))
    plant.hold('revoke', 'operator')
    assert made == [] and sent == []


def test_a_task_started_outside_the_tui_shows_as_task_from_the_session_start(tmp_path):
    # Sim (rig seed 1): an external run_task read '? /velocity_smoother' because the watcher
    # only existed once a Mission had run, and then read 'TASK task' (status names it 'task').
    app, _ = make_app(tmp_path)
    watcher = SimpleNamespace(status={})
    app.watcher_factory = lambda: watcher
    feed, _ = start(app, [1000.0])
    assert app._watcher is watcher                    # watching before any Mission
    watcher.status = {'task': 'docking', 'status': 'RUNNING'}
    feed.arbiter.tick()
    record = feed.view()['authority_record']
    assert (record['owner'], record['source']) == ('task', 'docking')
    assert authority_chip.chip(record, 1000.0) == ('ours', 'TASK docking')


def test_a_task_the_mission_procedure_started_is_not_a_second_source(tmp_path):
    app, _ = make_app(tmp_path)
    feed, _ = start(app, [1000.0])
    app._watcher = SimpleNamespace(status={'task': 'pair_test', 'status': 'RUNNING'})
    from mhseals_testing.procedures import PROCEDURES
    app.running = next(p.title for p in PROCEDURES if p.key == 't')  # as run_procedure sets it
    app.processes.processes = {'task_pair_test': None}
    app.processes.running = lambda name: name == 'task_pair_test'
    assert feed.sources()['task'] is None
    assert feed.sources()['procedure']['task'] == 'pair_test'
    app._watcher.status = {'task': 'docking', 'status': 'RUNNING'}   # someone else's task
    assert feed.sources()['task']['task'] == 'docking'
    assert 'task' not in feed.sources()['procedure']                 # so it still conflicts
    app._watcher.status = {'task': 'pair_test', 'status': 'RUNNING'}
    app.running = None                                                # no mission: not ours
    assert feed.sources()['task']['task'] == 'pair_test'
