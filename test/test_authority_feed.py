"""The session owner runs the arbiter and keeps its latest record; guests and review run none."""

import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import authority_chip, chrome, layout, theme
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
