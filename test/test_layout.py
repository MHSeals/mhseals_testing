"""Breakpoints, Status folding (U13) and per-tab focus memory."""

import io

from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout
from mhseals_testing.ui.dashboard import render, status_rows

from test_app import make_app
from test_dashboard import monitor, view


def test_tiers_follow_the_density_table():
    assert [layout.tier(w) for w in (160, 150, 149, 110, 109, 90, 89, 70, 69, 40)] == [
        'wide', 'wide', 'standard', 'standard', 'compact', 'compact', 'narrow', 'narrow',
        'single', 'single']
    assert layout.status_width(120) == 34 and layout.status_width(100) == 28


def test_fold_takes_quiet_groups_only_until_the_rows_fit():
    groups = {'Sensors': ['LIVE'] * 3, 'Frames': ['LIVE'], 'System': ['OK'] * 5}
    assert layout.fold_groups(groups, 100) == set()
    assert layout.fold_groups(groups, 10) == {'System'}
    assert layout.fold_groups(groups, 1) == {'System', 'Frames', 'Sensors'}


def test_fold_never_hides_warn_fail_busy_or_the_cursor_group():
    for state in ('WARN', 'FAIL', 'MISSING', 'RUNNING', 'CHECKING', 'STALE', 'OFF'):
        assert layout.fold_groups({'System': ['OK', state]}, 1) == set()
    assert layout.fold_groups({'System': ['OK'] * 3}, 1, cursor_group='System') == set()


def page(profile, height, **overrides):
    console = Console(width=120, height=height, file=io.StringIO(), force_terminal=False)
    console.print(render(monitor().snapshot(), view(profile, **overrides), 120, height))
    return console.file.getvalue()


def test_status_panel_folds_quiet_groups_but_keeps_the_warning_row():
    profile = load_profile('boat')
    rows = status_rows(monitor().snapshot(), profile.sensors)
    warning = next(i for i, row in enumerate(rows) if row['name'] == 'compute')
    text = page(profile, 17, cursors={'status': warning})   # cursor on the System warning
    assert 'compute' in text and '✗ odom' in text      # warn and fail rows are never folded
    assert 'Frames  ▸' not in text                     # Frames holds a failure
    m = monitor()
    m.transform('odom', 'base_link', authority='g')    # now every frame edge is live
    console = Console(width=120, height=17, file=io.StringIO())
    console.print(render(m.snapshot(), view(profile, cursors={'status': warning}), 120, 17))
    assert 'Frames  ▸' in console.file.getvalue()


def test_only_drive_draws_status():
    profile = load_profile('boat')
    assert '[Drive]' in page(profile, 34)
    other = page(profile, 34, tab='Bags', focus='list')
    assert '[Bags]' in other and 'Status' not in other


def test_focus_and_cursors_are_restored_per_tab(tmp_path):
    app, _ = make_app(tmp_path)
    app.dispatch('\t')
    app.dispatch('down')
    focus, cursor = app.focus, dict(app.cursors)
    app.dispatch(']')
    assert app.tab == 'Correction' and app.focus == 'table'
    app.dispatch('[')
    assert app.tab == 'Drive' and app.focus == focus and app.cursors == cursor
    app.dispatch('[')
    assert app.tab == 'Bags' and app.focus == 'list'


def test_procedure_keys_do_nothing_off_the_drive_tab(tmp_path):
    app, _ = make_app(tmp_path)
    app.dispatch(']')
    app.dispatch('1')
    assert app.running is None and app.tab == 'Correction'


def status_out(profile, m, height):
    console = Console(width=120, height=height, file=io.StringIO())
    console.print(render(m.snapshot(), view(profile), 120, height))
    return console.file.getvalue()


def test_status_panel_folds_nothing_when_every_row_fits():
    profile = load_profile('boat')
    m = monitor()
    m.transform('odom', 'base_link', authority='g')    # every frame edge is live
    # Frames must stay open from height 25 up; it folds at 24 (the fold budget is height - 4).
    for height in (25, 26, 27, 28, 30, 40):
        out = status_out(profile, m, height)
        assert all(f'{group}  ▸' not in out for group in ('System', 'Frames', 'Sensors'))
    assert 'Frames  ▸' in status_out(profile, m, 24)
