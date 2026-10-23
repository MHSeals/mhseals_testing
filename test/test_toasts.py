"""Health toasts: only warn and fail raise one, repeats dedupe, modals suppress them."""
import io

from rich.console import Console
from rich.text import Text

from mhseals_testing.ui import layout, theme
from mhseals_testing.ui.toasts import MAX_SHOWN, Toasts


def kinds(toasts, now, **kw):
    return [t.kind for t in toasts.visible(now, **kw)]


def test_info_and_unknown_never_toast():
    toasts = Toasts()
    for status in ('info', 'unknown', 'ok'):
        toasts.push('a', status, 'x', 0)
    assert toasts.visible(0) == []


def test_dedupe_updates_count_within_30_s():
    toasts = Toasts()
    toasts.push('a', 'warn', 'low', 0)
    toasts.push('a', 'warn', 'low', 10)
    (toast,) = toasts.visible(10)
    assert toast.count == 2
    toasts.push('a', 'warn', 'low', 100)
    assert toasts.visible(100)[0].count == 1


def test_dedupe_survives_expiry_and_render():
    toasts = Toasts()
    toasts.push('a', 'warn', 'low', 0)
    toasts.visible(7)                                  # rendered after the 6 s lifetime
    toasts.push('a', 'warn', 'low', 10)
    assert toasts.visible(10)[0].count == 2


def test_dedupe_counts_a_flapping_check():
    toasts = Toasts()
    toasts.push('a', 'warn', 'low', 0)
    toasts.visible(1)
    toasts.push('a', 'ok', 'low', 5)
    toasts.visible(6)
    toasts.push('a', 'warn', 'low', 12)
    assert toasts.visible(12)[0].count == 2
    toasts.push('a', 'ok', 'low', 15)
    toasts.push('a', 'warn', 'low', 20)
    assert toasts.visible(20)[0].count == 3
    toasts.push('a', 'warn', 'low', 40)                # first raise is over 30 s old: restart
    assert toasts.visible(40)[0].count == 1


def test_lifetimes_and_recovery_only_after_a_toast():
    toasts = Toasts()
    toasts.push('w', 'warn', 'x', 0)
    toasts.push('f', 'fail', 'y', 0)
    assert kinds(toasts, 5.9) == ['warn', 'fail']
    assert kinds(toasts, 6.1) == ['fail']                    # fail stays
    toasts.push('f', 'ok', 'y', 50)
    assert kinds(toasts, 51) == ['ok']
    assert kinds(toasts, 54) == []
    toasts.push('never', 'ok', 'z', 60)                      # never toasted: no recovery toast
    assert toasts.visible(60) == []


def test_at_most_three_and_ack():
    toasts = Toasts()
    for n in range(5):
        toasts.push(f'f{n}', 'fail', 'x', 0)
    assert len(toasts.visible(1)) == MAX_SHOWN and toasts.hidden(1) == 2
    toasts.ack()
    assert toasts.visible(1) == []


def test_suppressed_under_modal_then_collapses():
    toasts = Toasts()
    toasts.push('old', 'warn', 'w', 0)
    toasts.visible(0)                                  # shown before the modal opened
    toasts.push('a', 'warn', 'x', 1)
    toasts.push('b', 'warn', 'y', 1)
    assert toasts.visible(1, suppressed=True) == []
    (summary,) = toasts.visible(2)
    assert summary.kind == 'info' and '2 new' in summary.text


def test_overlay_bottom_right_above_footer():
    toasts = Toasts()
    toasts.push('a', 'fail', 'GPS lost', 0)
    stack, stack_width = layout.toast_stack(toasts.visible(0), 0, 80)
    base = layout.Layout()
    console = Console(width=80, height=20, file=io.StringIO(), record=True)
    console.print(layout.Overlay(base, stack, stack_width, 80, 20, anchor='bottom_right'))
    lines = console.export_text().splitlines()
    hit = [i for i, line in enumerate(lines) if 'GPS lost' in line]
    assert hit and hit[0] < 20 - layout.FOOTER_ROWS
    assert lines[hit[0]].rstrip().endswith('│') or lines[hit[0]].rstrip().endswith('|')


def drawn(toasts, width, height, now=0, more=0):
    stack, stack_width = layout.toast_stack(toasts, more, width, now, height - 5)
    console = Console(width=width, height=height, file=io.StringIO(), record=True)
    console.print(layout.Overlay(Text('\n'.join(['B' * width] * height)), stack, stack_width, width, height,
                                    anchor='bottom_right'))
    return console.export_text().splitlines()


def test_short_terminal_keeps_header_and_footer_clear():
    toasts = Toasts()
    for n in range(3):
        toasts.push(f'f{n}', 'fail', f'check {n} failed', 0)
    lines = drawn(toasts.visible(0), 80, 12)
    assert lines[:2] + lines[-3:] == ['B' * 80] * 5
    assert 'check 2 failed' in '\n'.join(lines)


def test_toast_capped_at_four_rows_with_ellipsis():
    toasts = Toasts()
    toasts.push('a', 'fail', 'word ' * 60, 0)
    lines = [l for l in drawn(toasts.visible(0), 80, 20) if l != 'B' * 80]
    assert len(lines) == 4 and theme.pick('…', '...') in lines[2]


def test_width_follows_text_within_bounds():
    toasts = Toasts()
    toasts.push('a', 'fail', 'x', 0)
    toasts.push('b', 'fail', 'y' * 200, 0)
    assert layout.toast_stack(toasts.visible(0)[:1], 0, 80)[1] == 26
    assert layout.toast_stack(toasts.visible(0)[1:], 0, 80)[1] == 60
    assert layout.toast_stack(toasts.visible(0)[1:], 0, 30)[1] == 28


def test_fade_in_last_second_of_finite_life_only():
    toasts = Toasts()
    toasts.push('w', 'warn', 'slow', 0)
    toasts.push('f', 'fail', 'down', 0)
    warn, fail = toasts.visible(0)
    assert layout.toast_box(warn, 30, 4.0)[0].style != 'dim'
    assert layout.toast_box(warn, 30, 5.5)[0].style == 'dim'
    assert layout.toast_box(fail, 30, 5.5)[0].style != 'dim'


def test_title_uses_kind_hue():
    toasts = Toasts()
    toasts.push('w', 'warn', 'slow', 0)
    panel = layout.toast_box(toasts.visible(0)[0], 30)[0]
    assert str(layout.theme.WARN) in str(panel.title.style)
    assert str(layout.ACCENT) not in str(panel.title.style)


def test_ascii_mode_has_no_unicode_in_toasts(monkeypatch):
    monkeypatch.setattr(layout.theme, 'ASCII', True)
    monkeypatch.setattr(layout.theme, 'SEPARATOR', ' | ')
    toasts = Toasts()
    toasts.push('a', 'fail', 'GPS low', 0)
    toasts.push('a', 'fail', 'GPS low', 1)
    toasts.push('b', 'fail', 'word ' * 60, 0)
    text = '\n'.join(drawn(toasts.visible(1), 80, 20, now=1))
    assert 'x2' in text and '...' in text and text.isascii()
    quiet = Toasts()
    quiet.push('old', 'warn', 'w', 0)
    quiet.visible(0)
    quiet.push('a', 'warn', 'x', 1)
    quiet.push('b', 'warn', 'y', 1)
    quiet.visible(1, suppressed=True)
    (summary,) = quiet.visible(2)
    assert summary.text == 'Health: 2 new | ! details'


def test_ascii_toast_capped_at_four_rows_with_ellipsis(monkeypatch):
    monkeypatch.setattr(layout.theme, 'ASCII', True)
    toasts = Toasts()
    toasts.push('a', 'fail', 'word ' * 60, 0)
    (toast,) = toasts.visible(0)
    for width in (26, 41, 60):
        assert layout.toast_box(toast, width)[1] == 4
    lines = [l for l in drawn(toasts.visible(0), 80, 20) if l != 'B' * 80]
    assert len(lines) == 4 and '...' in lines[2] and '\n'.join(lines).isascii()
