import pytest

from mhseals_testing.perception.probe import Detection as D, Probe, histogram, percentile


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def run(probe, clock, frames, dt=0.1, lag=0.05, **kw):
    """Feed frames (lists of detections) every dt seconds with a fixed latency."""
    for dets in frames:
        clock.t += dt
        probe.observe(clock.t - lag, dets, **kw)


@pytest.fixture
def clock():
    return Clock()


def test_percentile_and_histogram():
    assert percentile([], 0.5) is None
    assert percentile([1, 2, 3, 4], 0.5) == 2.5
    assert histogram([0.0, 0.05, 0.95, 1.0], 10) == [2, 0, 0, 0, 0, 0, 0, 0, 0, 2]


def test_no_frames_no_facts(clock):
    assert Probe('z', clock).facts() == {}


def test_rate_counts_empty_frames_and_latency(clock):
    p = Probe('z', clock)
    run(p, clock, [[]] * 50)                 # 10 Hz for 5 s
    f = p.facts()
    assert f['perc.z.hz'].value == pytest.approx(10, rel=0.05)
    assert f['perc.z.latency'].value == pytest.approx(0.05)
    assert f['perc.z.empty_ratio'].value == 1.0
    assert f['perc.z.max_gap'].value == pytest.approx(0.1)


def test_clock_offset_is_removed_from_latency(clock):
    p = Probe('z', clock)
    run(p, clock, [[]] * 5, lag=0.25, offset=0.2)
    assert p.facts()['perc.z.latency'].value == pytest.approx(0.05)


def test_gap_includes_silence_since_last_frame(clock):
    p = Probe('z', clock)
    run(p, clock, [[]] * 20)
    clock.t += 3.0
    f = p.facts()
    assert f['perc.z.max_gap'].value == pytest.approx(3.0)
    assert f['perc.z.hz'].value < 10


def test_classes_rate_presence_and_threshold(clock):
    p = Probe('z', clock, thresholds={'red': 0.5})
    run(p, clock, [[D('red', 0.9), D('red', 0.3)], []] * 25)    # 5 s, half the frames
    f = p.facts()
    c = f['perc.z.classes']
    assert c.value == 1
    red = c.data['red']
    assert red['presence'] == pytest.approx(0.5, abs=0.05)
    assert red['rate'] == pytest.approx(10, rel=0.1)
    assert red['below_threshold'] == 25
    assert red['hist'][9] == 25 and red['hist'][3] == 25
    assert f['perc.z.confidence'].value == pytest.approx(0.6)
    assert f['perc.z.empty_ratio'].value == pytest.approx(0.5)


def test_class_ages_and_drops_out_of_window(clock):
    p = Probe('z', clock)
    run(p, clock, [[D('a', 0.8)]])
    run(p, clock, [[]] * 20)
    assert p.facts()['perc.z.classes'].data['a']['last_age'] == pytest.approx(2.0)
    clock.t += 70
    p.observe(clock.t - 0.05, [])
    assert p.facts()['perc.z.classes'].value == 0


def test_stamp_errors(clock):
    p = Probe('z', clock)
    p.observe(99.0, [])
    p.observe(99.0, [])                      # repeated
    p.observe(98.0, [])                      # backwards
    p.observe(0.0, [])                       # zero
    p.observe(101.0, [], frame_id='')        # no frame id
    f = p.facts()
    assert f['perc.z.stamp_errors'].value == 4
    assert f['perc.z.latency'].value == pytest.approx(1.0)   # only the first frame is good
    assert f['perc.z.latency'].data['p95'] == pytest.approx(1.0)
    clock.t += 61
    assert p.facts()['perc.z.stamp_errors'].value == 0


def test_history_buckets(clock):
    p = Probe('z', clock, history=5)
    run(p, clock, [[D('a', 0.5)]] * 20, dt=0.25)
    h = p.facts()['perc.z.latency'].data['history']
    assert len(h) == 5 and h[-1] == pytest.approx(0.05)
    assert len(p.facts()['perc.z.confidence'].data['history']) == 5


def test_first_frame_rates_are_not_absurd(clock):
    p = Probe('z', clock)
    p.observe(clock.t - 0.05, [D('a', 0.9)])
    for dt in (0.0, 0.02):
        clock.t += dt
        f = p.facts()
        assert f['perc.z.hz'].value <= 1.0
        assert f['perc.z.classes'].data['a']['rate'] <= 1.0


def test_gap_reports_silence_longer_than_the_rate_window(clock):
    p = Probe('z', clock)
    run(p, clock, [[]] * 20)
    clock.t += 15.0
    f = p.facts()
    assert f['perc.z.max_gap'].value == pytest.approx(10.0)
    assert f['perc.z.hz'].value == 0.0
