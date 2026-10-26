"""The arm panel's look (variant 1): muted labels, a counted Checks rule, a caution line, the field in arm.word."""
import io

from rich.console import Console

from mhseals_testing.ui import layout, theme, widgets
from mhseals_testing.ui.popups import choice_modal

ROWS = [(widgets.SECTION, 'Will drive'), ('Outputs', 'ODROID PWM pins (real thrusters)'),
        (widgets.SECTION, 'Checks')]
OK = ROWS + [('Interlocks', theme.icon('good').plain + ' clear')]
BLOCKED = ROWS + [('Interlocks', theme.icon('bad').plain + ' FCU heading is unreliable'),
                  ('', theme.icon('bad').plain + ' another instance is live')]


def plain(renderable, width=80):
    console = Console(width=width, file=io.StringIO(), color_system=None)
    with console.capture() as cap:
        console.print(renderable)
    return cap.get()


def typed_modal(typed='AR'):
    return dict(kind='input', title=f'Arm{theme.SEPARATOR}boat', message='Propellers submerged.', text=typed,
                danger=True, word='ARM', rows=OK, width=72, hint='Type ARM', quiet=True,
                word_hue=theme.PALETTE.style('arm.word'))


def test_checks_rule_counts_what_blocks():
    assert 'all clear' in plain(layout.popup_rows(OK, 60, quiet=True)[2])
    assert '2 blocking' in plain(layout.popup_rows(BLOCKED, 60, quiet=True)[2])
    assert 'blocking' not in plain(layout.popup_rows(BLOCKED, 60)[2])       # other popups unchanged


def test_typed_panel_has_a_caution_line_and_the_word_field():
    shown = plain(layout.popup(typed_modal(), 80)[0])
    assert theme.pick('▲', '!') + ' Propellers submerged.' in shown
    assert 'A R _' in shown and 'needs ARM' in shown


def test_the_field_wears_arm_word_and_a_wrong_letter_still_warns():
    red = theme.PALETTE.style('arm.word')
    field = widgets.word_field('AR', 'ARM', red)
    if red.startswith('#'):
        assert red.lower() in colours(field)
        assert theme.WARN.lower() in colours(widgets.word_field('AX', 'ARM', red))
    assert 'needs ARM' in plain(field) and '! needs ARM' in plain(widgets.word_field('AX', 'ARM', red))


def colours(renderable, width=80):
    """The foreground hexes of the laid-out cells. Reads segment styles rather than ANSI: Rich keeps a
    parsed style's codes from the first console that renders it, so a truecolor render here would
    leak truecolor codes into later 16-colour renders of the same style."""
    console = Console(width=width, file=io.StringIO(), color_system=None)
    return {seg.style.color.triplet.hex for line in console.render_lines(renderable) for seg in line
            if seg.style and seg.style.color and seg.style.color.triplet}


def test_cancel_stays_the_focused_default():
    modal = choice_modal(f'Arm{theme.SEPARATOR}sim', ('Cancel', 'Arm'), {'n': 0, 'y': 1}, rows=BLOCKED,
                         width=72, quiet=True)
    assert modal['selected'] == 0 and modal['quiet']
    assert theme.pick('[▸n Cancel ↵]', '[>n Cancel <]') in plain(layout.popup(modal, 80)[0])
