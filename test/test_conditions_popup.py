"""popup.conditions: rows from a fixed summary, the golden, and Enter / Esc on the panel."""
import io
import math
from pathlib import Path

import pytest
from rich.console import Console

from mhseals_testing.ui import layout, theme
from mhseals_testing.ui.popups import conditions as popup
from test_app import make_app, press

# the UIW16 summary shape: floor and sea are scalars in rad/s
SUMMARY = dict(verdict='marginal', reason='ceiling 66% under 70%', ceiling=0.66, drift_trust=0.88,
               neutral_s=31.0, drift=[0.087, 0.087], se=0.02, state='steady',
               floor=math.radians(0.14), sea=math.radians(0.9))
DETAIL = dict(state='steady', pool=2, dispersion=1.12, trend=[0.0002, -0.0001], trend_z=1.4, windows=[
    dict(ago=120, T=60, v_n=0.081, v_e=0.090, psi=44, r=0.02, flag='retired'),
    dict(ago=60, T=60, v_n=0.088, v_e=0.084, psi=47, r=-0.05, flag=''),
    dict(ago=0, T=24, v_n=0.10, v_e=0.10, psi=50, r=1.5, flag='turn')])
GATES = [('conditions.marginal', 'warn', 'Characterize start', ['good', 'unknown'])]
GOLDEN = Path(__file__).parent / 'golden' / 'conditions'
needs_unicode = pytest.mark.skipif(theme.ASCII, reason='goldens are the Unicode glyph set')


def paint(modal, cols=120):
    panel, width = layout.popup(modal, cols, 30)
    out = io.StringIO()
    Console(width=width, file=out, color_system=None).print(panel)
    return out.getvalue()


def text(rows):
    return '\n'.join(f'{a} {b}' for a, b in rows)


def test_rows_cover_the_spec_items():
    t = text(popup.rows(SUMMARY, DETAIL, GATES))
    for want in ('marginal', 'ceiling 66%', 'trust 88%', 'CI 0.020', 'D 1.12', 'z 1.4', 'steady',
                 'yaw 0.14 deg/s', '0.9 deg/s', 'Windage not wired', 'C_d not wired', 'Characterize start',
                 'retired', 'turn', '-120 s'):
        assert want in t


def test_scalar_floor_and_sea_convert_rad_to_deg():
    t = text(popup.rows(dict(verdict='good', floor=0.0027925, sea=0.01745), None, []))
    assert 'yaw 0.16 deg/s' in t and '1.0 deg/s' in t


def test_unmeasured_summary_reads_dashes():
    t = text(popup.rows(dict(verdict='unknown'), None, []))
    assert 'measuring' in t and theme.DASH in t
    assert 'age_s' not in t and 'Last cuts' not in t


def test_conditions_detail_feeds_the_table():
    from test_harness_conditions import Clock, feed
    from mhseals_testing.harness.conditions import Conditions
    clock = Clock()
    cond = Conditions(clock=clock)
    assert cond.detail() is None
    feed(cond, clock, 90, 0.01)
    d = cond.detail()
    assert d['windows'] and {'ago', 'T', 'v_n', 'v_e', 'psi', 'r', 'flag'} <= set(d['windows'][0])
    assert d['dispersion'] is not None and d['state'] == cond.summary()['state']
    assert any('-' in label for label, _ in popup.table(d['windows'])[1:])


def test_shipped_gates_name_characterize():
    assert any(what == 'Characterize start' and cid == 'conditions.poor' for cid, _, what, _ in popup.gates())


@needs_unicode
@pytest.mark.parametrize('cols', [120, 80])
def test_golden(request, cols):
    out = paint(popup.modal(SUMMARY, DETAIL, GATES), cols)
    path = GOLDEN / f'popup_{cols}.txt'
    if request.config.getoption('--update-goldens'):
        GOLDEN.mkdir(exist_ok=True)
        path.write_text(out)
    assert out == path.read_text()


def test_up_down_and_jk_scroll_the_table():
    m = popup.modal(SUMMARY, dict(DETAIL, windows=DETAIL['windows'] * 20), GATES)
    paint(m, 80)
    assert m['clipped']
    keys = iter(['j', 'down', 'down', 'k', 'up', 'x', 'escape'])
    scrolled = []

    class App:
        conditions = type('C', (), {'summary': lambda s: SUMMARY, 'detail': lambda s: DETAIL})()

        def scroll(self, modal, key, keys):
            return layout_scroll(modal, key, keys)

        def popup(self, modal, handle, refresh=None):
            for key in keys:
                if handle(key):
                    break
                scrolled.append(modal.get('scroll', 0))
    from mhseals_testing.ui.popups import scroll as layout_scroll
    popup.open_conditions(App())
    assert scrolled == [1, 2, 3, 2, 1, 1]


def test_enter_opens_and_escape_closes(tmp_path):
    app, _ = make_app(tmp_path)
    seen = []
    with app.attached():
        app.focus = 'conditions'
        app.popup = lambda modal, handle, refresh=None: seen.append((modal['title'], handle('x'), handle('escape')))
        app.dispatch('enter')
    assert seen == [('Conditions', None, True)]
