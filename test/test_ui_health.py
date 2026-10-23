"""The health summary row and strip counts come from the engine (UIW05, ui 4.2 / 6.1, health 5.2)."""

import io
from types import SimpleNamespace

from rich.console import Console

from mhseals_testing.health.model import CheckSpec, Result
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout
from mhseals_testing.ui.feeds.health import HealthFeed, summary
from mhseals_testing.ui.panels.status import row_label, status_rows

from test_chrome import check
from test_dashboard import monitor, view


def engine(oks=41, warn=(), fail=(), muted=()):
    """A stub engine: ``oks`` passing checks, then warn and fail titles, with a real report."""
    specs = [CheckSpec(id=f'ok{i}', kind='k', params=None, title=f'Ok {i}') for i in range(oks)]
    specs += [CheckSpec(id=f'w{i}', kind='k', params=None, title=t) for i, t in enumerate(warn)]
    specs += [CheckSpec(id=f'f{i}', kind='k', params=None, title=t) for i, t in enumerate(fail)]
    results = {s.id: Result(status='ok' if s.id.startswith('ok') else 'warn' if s.id[0] == 'w' else 'fail')
               for s in specs}
    return SimpleNamespace(specs=specs, muted=set(muted), last=results)


def test_summary_counts_and_worst():
    assert summary(engine()) == dict(ok=41, warn=0, bad=0, worst='')
    got = summary(engine(10, warn=('yaw', 'gps'), fail=('cmd owner',)))
    assert got == dict(ok=10, warn=2, bad=1, worst='cmd owner')
    assert summary(engine(3, warn=('yaw',)))['worst'] == 'yaw'


def test_summary_skips_muted_and_waits_for_a_tick():
    assert summary(engine(3, warn=('yaw',), muted=('w0',)))['warn'] == 0
    stub = engine(3, warn=('yaw',))
    assert summary(stub)['warn'] == 1
    stub.muted.add('w0')                      # muting through engine.muted, as production does
    assert summary(stub)['warn'] == 0
    quiet = engine()
    quiet.last = None
    assert summary(quiet) is None
    assert HealthFeed().view() == {}


def test_feed_publishes_the_view_key():
    feed = HealthFeed()
    feed.tick(SimpleNamespace(engine=engine(5)), {}, 0.0)
    assert feed.view() == {'health': dict(ok=5, warn=0, bad=0, worst='')}


def test_status_row_zero():
    profile = load_profile('boat')
    snapshot = monitor().snapshot(profile.required_tf)
    assert status_rows(snapshot, profile.sensors)[0]['group'] != 'Health'      # no engine: no row
    for counts, name, value, state in ((summary(engine()), 'Health', '41 ok', 'OK'),
                                       (summary(engine(40, warn=('yaw',))), 'Health', '1 warn', 'WARN')):
        row = status_rows({**snapshot, 'health': counts}, profile.sensors)[0]
        assert (row['group'], row['name'], row['value'], row['state']) == ('Health', name, value, state)


def strip(width, counts):
    profile = load_profile('boat')
    snapshot = monitor().snapshot(profile.required_tf)
    console = Console(file=io.StringIO(), width=width, color_system=None)
    console.print(layout.header(snapshot, view(profile, health=counts), width))
    return console.file.getvalue()


def test_strip_goldens():
    for width in (120, 80):
        check(f'health/strip_{width}.txt', strip(width, summary(engine())))


def test_counts_survive_a_narrow_strip():
    out = strip(80, summary(engine(40, warn=('compass innovation',)))).splitlines()
    assert out[-1].rstrip().endswith('! 1 · compass innovation')
    assert '! 1 · compass innovation' in strip(120, summary(engine(40, warn=('compass innovation',))))


def test_health_label_column_is_stable():
    profile = load_profile('boat')
    snapshot = monitor().snapshot(profile.required_tf)
    cols = []
    for counts in (summary(engine()), summary(engine(40, warn=('yaw',)))):
        row = status_rows({**snapshot, 'health': counts}, profile.sensors)[0]
        cols.append(row_label(row, False).plain.index('Health'))
    assert cols[0] == cols[1] == 4
