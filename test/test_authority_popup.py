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
    return dict(t_ns=int(NOW * NS), owner='agent', source='claim c7 tune_mppi', reason='MPPI vx_max sweep 3/8', since_ns=int((NOW - 252) * NS),
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


class FakeApp:
    def __init__(self, keys, table):
        self.arbiter = SimpleNamespace(record=dict(record(), t_ns=int(NOW * NS), claims=table.claims),
                                         clock=lambda: NOW, table=table,
                                         revoke=lambda id: table.revoke(id, NOW))
        self.paths = SimpleNamespace(journal=Path('/nonexistent/journal'))
        self.notice, self.keys, self.scroll = '', list(keys), app.App.scroll

    def popup(self, box, handle, refresh=None):
        for key in self.keys:
            if key == 'space':
                self.operator_stop()
            if key == 'U':
                self.release_hold()
                continue
            if handle(key):
                break

    show = lambda self, *a, **k: None


def table():
    return ClaimTable([claim('c7', 'agent', 'a', 'active', 5), claim('c8', 'agent', 'b', 'active', 1)])


def test_e_revokes_only_the_selected_claim():
    claims = table()
    fake = FakeApp(['down', 'E'], claims)
    popups.OPENERS['authority'](fake)
    assert [(c['id'], c['state']) for c in claims.claims] == [('c7', 'active'), ('c8', 'revoked')]
    assert fake.notice == 'Revoked c8'


def test_e_on_a_revoked_claim_changes_nothing():
    claims = table()
    claims.revoke('c7', NOW)
    fake = FakeApp(['E'], claims)
    popups.OPENERS['authority'](fake)
    assert [c['state'] for c in claims.claims] == ['revoked', 'active'] and 'not active' in fake.notice


def test_space_stops_the_boat():
    fake = FakeApp(['space'], table())
    stops = fake.stops = []
    fake.operator_stop = lambda: stops.append('stopped by operator')
    popups.OPENERS['authority'](fake)
    assert stops == ['stopped by operator']


def test_lowercase_e_does_not_revoke():
    claims = table()
    popups.OPENERS['authority'](FakeApp(['e'], claims))
    assert [c['state'] for c in claims.claims] == ['active', 'active']


def test_lowercase_u_does_not_release_but_u_does():
    fake = FakeApp(['u'], table())
    fake.release_hold = lambda: setattr(fake, 'released', True)
    popups.OPENERS['authority'](fake)
    assert not hasattr(fake, 'released')
    fake.keys = ['U']
    popups.OPENERS['authority'](fake)
    assert fake.released


def test_e_after_a_claim_disappears_between_ticks():
    claims = table()
    fake = FakeApp([], claims)

    def popup(box, handle, refresh=None):
        handle('down')
        claims.claims.pop()          # released by the table before the next key
        fake.arbiter.record = dict(record(), claims=claims.claims)
        refresh()                    # the tick clamps against the old ids, then the rebuild shrinks them
        handle('E')
    fake.popup = popup
    popups.OPENERS['authority'](fake)
    assert fake.notice.startswith('Revoked c7')


def test_journal_is_reread_only_when_the_file_changes(tmp_path, monkeypatch):
    path = tmp_path / 'journal'
    path.write_text('{"phase": "authority"}\n')
    calls = []
    real = authority.journal.read
    monkeypatch.setattr(authority.journal, 'read', lambda p: calls.append(p) or real(p))
    read = authority.journal_reader(path)
    read(), read(), read()
    assert len(calls) == 1
    path.write_text('{"phase": "authority"}\n{"phase": "claim"}\n')
    assert len(read()) == 2 and len(calls) == 2


def test_inputs_read_topic_from_source():
    assert authority.inputs_line(record()).startswith('/cmd_vel ' + theme.pick('←', '<-') + ' velocity_smoother')
