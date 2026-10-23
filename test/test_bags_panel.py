from rich.console import Console

from mhseals_testing.harness.bags import Bag
from mhseals_testing.ui.panels import bags as panel


def bag(**fields):
    return Bag(id=fields.pop('id', 'boat_1'), path=None, **fields)


def draw(renderable, width=100):
    console = Console(width=width, record=True, color_system=None)
    console.print(renderable)
    return console.export_text()


def test_flags_in_fixed_order():
    full = bag(pinned=True, markers=4, sent=True, unfinalized=True, health_fails=3, sim=True)
    assert panel.flags(full) == 'pin ▾4 ✓ sent unfin. health✗3 sim'
    assert panel.flags(bag()) == ''


def test_lead_glyph_prefers_recording_then_unfinalized_then_empty():
    assert str(panel.lead(bag(recording=True, unfinalized=True, empty=True))) == '◌'
    assert str(panel.lead(bag(unfinalized=True, empty=True))) == '!'
    assert str(panel.lead(bag(empty=True))) == '·'
    assert str(panel.lead(bag())).strip() == ''


def test_title_is_shown_else_automatic():
    assert panel.title_text(bag(title='wind test', empty=True)) == 'wind test'
    assert panel.title_text(bag(empty=True)) == '(empty)'
    assert panel.title_text(bag(outcome='char 5/6')) == 'char 5/6'


def test_gauge_clamps_and_keeps_width():
    assert str(panel.gauge(2.0, 10)) == '━' * 10
    assert str(panel.gauge(-1, 10)) == '─' * 10
    assert str(panel.gauge(0.5, 10)) == '━' * 5 + '─' * 5


def test_render_lists_rows_gauge_and_cursor():
    rows = [bag(id='boat_a', title='wind test', size=1_100_000_000, duration=3600, pinned=True, markers=2),
            bag(id='boat_b', empty=True, size=8_000_000, duration=120)]
    text = draw(panel.render(rows, (21.4e9, 58e9, 1.2e9), cursor=1, budget=20e9, root='bags', rate=2e5))
    assert 'Disk' in text and '21.4/58 GB' in text and 'budget 20 GB' in text and 'rec ≈0.2 MB/s' in text
    assert 'boat_a' in text and '1.1 GB' in text and 'pin ▾2' in text and '(empty)' in text
    cursor_line = next(line for line in text.splitlines() if 'boat_b' in line)
    assert '›' in cursor_line


def test_render_fits_narrow_width_and_empty_list():
    assert all(len(line) <= 60 for line in draw(panel.render([bag(title='x' * 80)], (1, 2, 0)), 60).splitlines())
    assert 'no sessions yet' in draw(panel.render([], (1, 2, 0)))


def test_summary_shows_note_or_nothing_selected():
    text = draw(panel.summary(bag(id='boat_a', title='t', note='calm', outcome='manual', start=1e9)))
    assert 'boat_a' in text and 'note: calm' in text and 'outcome: manual' in text
    assert 'nothing selected' in draw(panel.summary(None))
