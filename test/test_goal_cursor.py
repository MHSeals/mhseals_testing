"""The minimap.goal cursor: moves in screen frame, yaw, subtitle (ASCII-safe)."""

import numpy as np
import pytest

from mhseals_testing.harness.goals import Grid
from mhseals_testing.ui import theme
from mhseals_testing.ui.minimap.goal import GoalCursor


def grid(fill=0):
    return Grid(np.full((100, 100), fill, dtype=np.int8), 0.4, (-20.0, -20.0))


def test_cursor_moves_yaw_and_subtitle():
    c = GoalCursor((0, 0), 0.4)
    c.move('right', fast=True)  # 8 dots of 0.4 m
    assert c.pos == pytest.approx((3.2, 0)) and c.yaw() == 0
    c.move('up', fast=True)
    assert c.yaw() == pytest.approx(45)
    c.rotate(+1)
    assert c.yaw() == pytest.approx(60)
    c.auto_yaw()
    assert c.yaw() == pytest.approx(45)
    assert c.subtitle(grid()).startswith('goal 4.5 m @045° · cost 0 · in map')
    c.rotate(+1)
    assert c.subtitle(grid()).startswith('goal 4.5 m @060°')  # the goal yaw, not the bearing
    assert 'under' in GoalCursor((0, 0), 0.4).subtitle(grid())


def test_heading_up_moves_in_screen_frame():
    c = GoalCursor((0, 0), 1.0)
    c.move('up', up_yaw=0)  # heading east is drawn up: screen-up is map east
    assert c.pos == pytest.approx((1, 0), abs=1e-9)
    c.move('right', up_yaw=0)  # screen-right is then map south
    assert c.pos == pytest.approx((1, -1), abs=1e-9)
    n = GoalCursor((0, 0), 1.0)
    n.move('up', up_yaw=90)  # heading north up equals north-up view
    assert n.pos == pytest.approx((0, 1), abs=1e-9)


def test_subtitle_ascii_mode(monkeypatch):
    monkeypatch.setattr(theme, 'ASCII', True)
    monkeypatch.setattr(theme, 'SEPARATOR', ' | ')
    c = GoalCursor((0, 0), 0.4)
    c.move('right', fast=True)
    text = c.subtitle(grid())
    assert text.isascii() and ' | ' in text and 'deg' in text
