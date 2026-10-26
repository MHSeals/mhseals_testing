"""The remote e-stop in the TUI: the STOP chip, the STOPPED line, the Space anchor, the Hold row and popup."""
import io
from types import SimpleNamespace

import pytest
from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import authority_chip, layout, theme
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import stop
from test_dashboard import monitor, view

NOW = 1000.0
NS = int(NOW * 1e9)


def held(cause, by='rc'):
    return {'owner': 'hold', 'since_ns': NS - 12 * 10 ** 9,
            'holds': [{'cause': cause, 'by': by, 'since_ns': NS - 12 * 10 ** 9}]}


def text(renderable, width=120, height=None):
    console = Console(width=width, height=height, file=io.StringIO(), record=True)
    console.print(renderable)
    return console.export_text()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode glyph')
def test_a_stop_hold_is_a_stop_chip_and_other_holds_stay_slate():
    assert authority_chip.chip(held('rc_stop'), NOW) == ('stop', '⊘ STOP rc 12 s')
    assert authority_chip.chip(held('keys', 'tui'), NOW) == ('stop', '⊘ STOP keys 12 s')
    assert authority_chip.chip(held('costmap_stale', 'costmap_watchdog'), NOW)[0] == 'hold'
    assert theme.CHIPS['stop'] == 'stop' and theme.CHIPS['armed'] == 'armed'


def test_ascii_stop_chip(monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', True)
    kind, word = authority_chip.chip(held('rc_stop'), NOW)
    assert theme.PALETTE.chip(word, theme.CHIPS[kind], ascii=True).plain == '[STOP rc 12 s]'


def page(authority, width=120, height=40):
    return layout.render(monitor().snapshot(), view(load_profile('boat'), authority=authority), width, height)


@pytest.mark.parametrize('width', [120, 80])
def test_row_two_says_stopped_with_the_release_keys_never_cut(width):
    lines = text(page(authority_chip.chip(held('rc_stop'), NOW), width, 40 if width == 120 else 24),
                 width).splitlines()
    assert 'STOP rc 12 s' in lines[0]
    assert 'STOPPED' in lines[1] and lines[1].rstrip().endswith('U release · Space stop')
    assert 'STOPPED' not in text(page(None, width, 40 if width == 120 else 24), width)


def test_space_wears_the_stop_hue_in_the_footer():
    cell = theme.hint('Space', 'stop', style=layout.key_style('Space'))
    stop = f"bold {theme.PALETTE.style('stop')}"
    assert [str(span.style) for span in cell.spans] == [stop, stop]   # key and label, one unit
    assert len(theme.hint('q', 'quit').spans) == 1                    # other labels stay plain
    assert layout.key_style('q') is None and layout.key_style('E') == theme.PALETTE.style('auth.agent')


def control_rows(record):
    control = authority_chip.control(record, authority_chip.chip(record, NOW), NOW, 'allow')
    rows = status_rows({**monitor().snapshot(), 'control': control}, load_profile('boat').sensors)
    return {r['name']: r for r in rows if r['group'] == 'Control'}, control


def test_the_hold_row_says_stopped_and_the_popup_names_cause_and_release():
    rows, control = control_rows(held('rc_stop'))
    assert 'STOPPED rc 12 s' in rows['Hold']['value'] and rows['Hold']['stop']
    assert 'U releases' in rows['Hold']['detail']
    control['rc'] = {'mode': 'HOLD'}
    body = text(stop.body(control, False, [(1.0, 'STOP: rc stop'), (2.0, 'other')]), 64)
    assert 'RC stop channel (falling edge)' in body and '12 s ago' in body and 'disarmed' in body
    assert 'U is refused until GUIDED' in body and '[ U release hold ]' in body and 'Recent' in body
    assert '  rc stop' in body and 'other' not in body
    assert not control_rows(held('costmap_stale', 'w'))[0]['Hold'].get('stop')


def test_enter_closes_the_stop_popup_and_only_u_asks_to_release():
    from mhseals_testing.ui import popups
    calls, keys = [], ['U', 'enter']
    app = SimpleNamespace(modal=None, refresh=lambda: None, tick=lambda: None, armed=False,
                          keys=SimpleNamespace(read=lambda timeout: keys.pop(0)),
                          release_hold=lambda: calls.append('release'), operator_stop=lambda: None,
                          revoke_agent=lambda: None, monitor=SimpleNamespace(events=[]),
                          snapshot=lambda: {'control': {'holds': [{'cause': 'keys', 'age': '3 s'}]}})
    app.popup = lambda modal, handle, refresh=None: popups.popup(app, modal, handle, refresh)
    stop.open_stop(app)
    assert calls == ['release'] and not keys      # U opened the confirm; Enter only closed


def agent_banner(state='suspended'):
    """The agent banner text for a claim Space suspended (``AGENT PAUSED``)."""
    return authority_chip.banner({'owner': 'hold', 'claims': [
        {'id': 'c7', 'owner': 'agent', 'who': 'claude-3f2a', 'state': state, 'since_ns': NS - int(30e9)}]},
        NOW, desk={'act': None})


@pytest.mark.parametrize('width', [160, 120, 80])
def test_space_under_an_agent_claim_shows_stopped_first_and_the_paused_claim(width):
    """Space never hides STOPPED behind the agent banner: the stop owns the row, the claim is named on it."""
    lines = text(layout.render(monitor().snapshot(), view(
        load_profile('boat'), authority=authority_chip.chip(held('keys', 'tui'), NOW), agent=agent_banner()),
        width, 40 if width >= 120 else 24), width).splitlines()
    assert lines[1].lstrip().startswith(f"{theme.pick('⊘', '(/)')} STOPPED")
    assert ('AGENT PAUSED' if width >= 120 else f'{theme.AGENT_GLYPH} claude-3f2a') in lines[1]
    assert f'claude-3f2a{theme.SEPARATOR}c7' in lines[1] and len(lines[1]) <= width
    assert lines[1].rstrip().endswith(layout.chrome.AGENT_STOP_KEYS)


def test_stopped_wins_the_eye_over_the_agent_claim_on_its_row():
    row = layout.chrome.stop_banner(False, 120, agent=agent_banner())
    styles = {row.plain[span.start:span.end].strip(): str(span.style) for span in row.spans}
    stopped = next(v for k, v in styles.items() if k.endswith('STOPPED'))
    claim = next(v for k, v in styles.items() if 'AGENT PAUSED' in k)
    assert 'bold' in stopped and 'bold' not in claim and 'reverse' not in claim
    assert claim == theme.PALETTE.style('auth.agent')        # unreversed agent hue, not the solid banner


def test_without_a_stop_the_agent_banner_keeps_the_row():
    lines = text(layout.render(monitor().snapshot(), view(load_profile('boat'), authority=('agent', 'AGENT c'),
                                                          agent=agent_banner('active')), 120, 40), 120).splitlines()
    assert 'STOPPED' not in lines[1] and 'AGENT IDLE' in lines[1]
