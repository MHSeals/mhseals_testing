"""The Authority popup: golden at 120, E revokes the selected claim only, no status glyphs (ui 5.4)."""
import io
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from rich.console import Console

from mhseals_testing.harness.authority import ClaimTable
from mhseals_testing.ui import app, layout, popups, theme
from mhseals_testing.ui.popups import authority

GOLDEN = Path(__file__).parent / 'golden' / 'authority' / 'popup_120.txt'
NOW = 1000.0
NS = int(1e9)


def claim(id, owner, who, state, ago, **extra):
    return {**dict(id=id, owner=owner, who=who, host=None, pid=None, reason='', since_ns=int((NOW - ago) * NS),
                   seen_ns=0, lease_s=None, state=state, explicit=True), **extra}


def record():
    return dict(owner='agent', source='claim c7 tune_mppi', reason='MPPI vx_max sweep 3/8', since_ns=int((NOW - 252) * NS),
                input='auto', holds=[], seq=118,
                rc={'mode': 'GUIDED', 'rate_hz': 9.1, 'interval_p95_s': 0.071},
                claims=[claim('c7', 'agent', 'tune_mppi', 'active', 252, host='tranquility', pid=5120),
                        claim('c6', 'procedure', 'characterize', 'revoked', 900)],
                inputs={'/cmd_vel': 'velocity_smoother', '/cmd_vel/manual': None},
                nav={'goals': [{'id': '9f2c0001', 'status': 'executing', 'by': 'claim c7'}]})


EVENTS = [dict(phase='claim', t_ns=1), dict(phase='authority', t_ns=NS, cause='claim',
                                            **{'from': {'owner': 'none'}, 'to': {'owner': 'agent'}})]


def render(rec=None, cursor=0):
    state = dict(cursor=cursor)
    box = authority.modal(rec or record(), EVENTS, NOW, state)
    panel, width = layout.popup(box, 120, 30)
    console = Console(width=width, file=io.StringIO(), record=True, no_color=True)
    console.print(panel)
    return console.export_text()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_popup_golden(monkeypatch):
    monkeypatch.setenv('TZ', 'UTC')
    import time
    time.tzset()
    actual = render()
    if os.environ.get('UPDATE_GOLDEN'):
        GOLDEN.parent.mkdir(exist_ok=True)
        GOLDEN.write_text(actual)
    assert actual == GOLDEN.read_text(), 'popup_120.txt differs; UPDATE_GOLDEN=1 to regenerate'


def test_hint_sits_on_the_bottom_border():
    assert render().splitlines()[-1].count('E revoke selected  U release hold  Esc close') == 1


def test_the_cursor_marks_one_claim_and_moves():
    rows, ids = authority.rows(record(), EVENTS, NOW, cursor=1)
    assert ids == ['c7', 'c6']
    marked = [label for label, _ in rows if label.startswith(theme.CURSOR)]
    assert marked == [theme.CURSOR + 'c6']


def test_no_record_says_so():
    assert authority.rows(None, [], NOW) == ([('Driving', 'no authority record yet')], [])


def test_status_glyphs_are_not_used_for_owners():
    text = render()
    assert not set(text) & set('✓!✗◌●○')
    assert 'AGENT' in text and 'OURS' in text


def test_owner_words():
    assert [authority.word(o) for o in ('unknown', 'task', 'nav', 'procedure', 'none')] == \
        ['UNKNOWN', 'TASK', 'GOAL', 'OURS', 'idle']

