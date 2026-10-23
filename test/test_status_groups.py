"""Status groups (ui 6.1), folding order (U13) and the Battery advisory."""

import io

from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout
from mhseals_testing.ui.panels import FOLD_ORDER, GROUPS
from mhseals_testing.ui.panels.status import status_panel, status_rows

from test_chrome import check
from test_dashboard import monitor, view


def groups_of(rows):
    return list(dict.fromkeys(row['group'] for row in rows))


def test_rows_come_in_ui_group_order():
    profile = load_profile('boat')
    m = monitor()
    m.advise('Bench', 'WARN', 'override on')
    rows = status_rows(m.snapshot(profile.required_tf), profile.sensors)
    found = groups_of(rows)
    assert found == [g for g in GROUPS if g in found] == ['Components', 'Localization', 'System', 'Session']
    assert next(r for r in rows if r['name'] == 'Bench')['group'] == 'Session'
    assert next(r for r in rows if r['name'] == 'compute')['group'] == 'System'
    assert {r['group'] for r in rows if r.get('source') == 'tf'} == {'Localization'}


def test_fold_order_is_u13():
    assert FOLD_ORDER == ('Session', 'System', 'Components', 'Localization', 'Control')
    groups = {'Session': ['OK'], 'System': ['OK'] * 3, 'Components': ['LIVE'] * 3}
    assert layout.fold_groups(groups, 9) == {'Session'}
    assert layout.fold_groups(groups, 6) == {'Session', 'System'}


def test_battery_only_with_a_source():
    profile = load_profile('boat')
    m = monitor()
    names = lambda: [r['name'] for r in status_rows(m.snapshot(profile.required_tf), profile.sensors)]
    assert 'Battery' not in names()
    m.advise('Battery', 'OK', '15.8 V')
    rows = status_rows(m.snapshot(profile.required_tf), profile.sensors)
    assert next(r for r in rows if r['name'] == 'Battery')['group'] == 'System'


def dump(width):
    profile = load_profile('boat')
    snapshot = monitor().snapshot(profile.required_tf)
    out = []
    for height in (30, 20, 14):
        console = Console(file=io.StringIO(), width=width, height=height, color_system=None)
        console.print(status_panel(snapshot, view(profile), height))
        out.append(f'== {width}x{height}\n' + console.file.getvalue().rstrip('\n'))
    return '\n'.join(out) + '\n'


def test_status_goldens():
    for width, name in ((layout.status_width(120), 'idle_120'), (layout.status_width(100), 'idle_100')):
        check(f'status/{name}.txt', dump(width))
