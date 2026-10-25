"""ui/widgets.py: buttons, choice lists, fields, letter slots, rules and the stop chip."""
import io
import os
from pathlib import Path

import pytest
from rich.console import Console, Group

from mhseals_testing.ui import theme, widgets

GOLDEN = Path(__file__).parent / 'golden' / 'widgets'


def plain(renderable, width=60):
    console = Console(width=width, file=io.StringIO(), record=True)
    console.print(renderable)
    return console.export_text()


def test_only_the_focused_button_carries_enter_and_a_fill():
    idle, focused = widgets.button('Release', 'y', kind='danger'), widgets.button('Cancel', 'n', focused=True)
    assert idle.plain == '[ y Release ]' and focused.plain == '[▸n Cancel ↵]'
    assert any('reverse' in str(s.style) or ' on ' in str(s.style) for s in focused.spans)
    assert not any(' on ' in str(s.style) for s in idle.spans)


def test_ascii_buttons_and_chip(monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', True)
    assert widgets.button('Cancel', 'n', focused=True).plain == '[>n Cancel <]'
    assert widgets.stop_chip('rc', '12 s').plain == '[STOP rc 12 s]'
    assert widgets.countdown(5, 10).plain == '[=====     ] 5 s'


def test_a_short_choice_is_one_row_and_a_long_one_stacks():
    row = widgets.choice_list(['Release', 'Cancel'], ['y', 'n'], 1, {0}, 60)
    assert [line.plain for line in row] == ['[ y Release ]   [▸n Cancel ↵]']
    long = widgets.choice_list(['Save mapping and quit', 'Quit', 'Cancel'], ['s', 'q', 'n'], 0, set(), 60)
    assert [line.plain for line in long] == ['▸ s Save mapping and quit ↵', '  q Quit', '  n Cancel']
    narrow = widgets.choice_list(['Release', 'Cancel'], ['y', 'n'], 0, {0}, 20)
    assert len(narrow) == 2 and widgets.button_row([widgets.button('x' * 30)], 20) is None


def test_word_field_marks_a_broken_prefix_and_a_match():
    assert 'needs ARM' in plain(widgets.word_field('AR', 'ARM'))
    assert '! needs ARM' in plain(widgets.word_field('AN', 'ARM'))
    assert 'ARM ✓' in plain(widgets.word_field('ARM', 'ARM'))


SAMPLES = {
    'buttons': lambda: Group(*widgets.choice_list(['Release', 'Cancel'], ['y', 'n'], 1, {0}, 56),
                             *widgets.choice_list(['Save mapping and quit', 'Quit', 'Cancel'],
                                                  ['s', 'q', 'n'], 0, set(), 56)),
    'field': lambda: Group(widgets.field('gate lef', width=56, limit=80),
                           widgets.field('', width=56, placeholder='add a note'),
                           widgets.field('1 2 3 4', width=56, state='bad', note='need 5 numbers, got 4'),
                           widgets.field('127.5', width=19, suffix='°', note='→ 124.9° true')),
    'word': lambda: Group(widgets.word_field('AR', 'ARM'), widgets.word_field('ARM', 'ARM')),
    'rule': lambda: Group(widgets.rule('Drift', 40), widgets.stop_chip('rc', '12 s'), widgets.countdown(14, 20)),
}


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
@pytest.mark.parametrize('name', sorted(SAMPLES))
def test_widgets_match_their_goldens(name):
    golden, actual = GOLDEN / f'{name}.txt', plain(SAMPLES[name]())
    if os.environ.get('UPDATE_GOLDEN'):
        golden.parent.mkdir(exist_ok=True)
        golden.write_text(actual)
    assert actual == golden.read_text(), f'{golden.name} differs; UPDATE_GOLDEN=1 to regenerate'
