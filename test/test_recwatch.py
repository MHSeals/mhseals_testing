"""Recording health: facts, the disk auto-off and the strip text."""

from mhseals_testing.harness.recwatch import RecWatch, strip_text, unfinalized
from mhseals_testing.health.config import load, lint
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.model import Fact
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout


class Fake:
    def __init__(self):
        self.now, self.free, self.size = 0.0, 50.0, 0

    def watch(self):
        return RecWatch(lambda: self.now, free=lambda _: self.free, size=lambda _: self.size)


def test_disk_under_threshold_stops_once_and_rearms_after_off():
    fake = Fake()
    watch = fake.watch()
    assert not watch.sample('bag', True).stop_for_disk
    fake.free = 0.5
    fake.now = 1
    assert watch.sample('bag', True).stop_for_disk
    fake.now = 2
    assert not watch.sample('bag', True).stop_for_disk      # one trip, not one per tick
    fake.now = 3
    assert watch.sample('bag', False).mode == 'off'
    fake.now = 4
    assert watch.sample('bag', True).stop_for_disk           # turned on deliberately: guards again


def test_rate_and_minutes_left_follow_growth():
    fake = Fake()
    watch = fake.watch()
    watch.sample('bag', True)
    fake.now, fake.size = 10, 2_000_000
    sample = watch.sample('bag', True)
    assert sample.rate_bps == 200_000 and round(sample.minutes_left) == round(50e9 / 200_000 / 60)
    assert sample.facts(armed=False)['bag.off_armed'] == 0.0


def test_strip_text_ok_off_armed_and_low_disk_has_no_dots():
    fake = Fake()
    watch = fake.watch()
    watch.sample('bag', True)
    fake.now, fake.size = 10, 2_000_000
    state, text = strip_text(watch.sample('bag', True))
    assert (state, text) == ('ok', 'rec lean ≈0.2 MB/s · 69 h')     # 50 GB at 0.2 MB/s
    off = watch.sample('bag', False)
    assert strip_text(off) == ('off', 'rec off') and strip_text(off, armed=True) == ('warn', 'rec off')
    fake.free = 0.4
    assert strip_text(fake.watch().sample('bag', True))[0] == 'fail'
    for sample in (off, watch.sample('bag', True)):
        for armed in (False, True):
            assert '●' not in layout.rec_chip(dict(rec=sample, armed=armed, recording=True)).plain


def test_unfinalized_counts_segments_without_metadata(tmp_path):
    (tmp_path / 'bag_00').mkdir()
    (tmp_path / 'bag_01').mkdir()
    (tmp_path / 'bag_00' / 'metadata.yaml').write_text('x')
    assert unfinalized(tmp_path) == 1


def test_shipped_system_checks_lint_and_judge_facts(nav_share):
    config = load(user=False, profile=load_profile('boat'))
    assert lint(config.specs, unmatched=config.unmatched) == []
    specs = [s for s in config.specs if s.id in ('compute.disk', 'bag.off_armed')]
    now = [0.0]
    engine = Engine(specs, clock=lambda: now[0])
    facts = {'host.disk_free_gb': Fact(0.4), 'bag.off_armed': Fact(1.0)}
    probes = {'host': (0.0, None)}               # compute.disk reads the host probe's sample
    engine.tick(facts, probes)
    now[0] = 5.0                                  # warn must persist past the raise time
    report = engine.tick(facts, probes)
    assert report.results['compute.disk'].status == 'fail'
    assert report.results['bag.off_armed'].status == 'warn'
