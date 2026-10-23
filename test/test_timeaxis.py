import random

from mhseals_testing.ui.timeaxis import FOLD, FOLD_MIN, STEPS, Axis, duration, quantise

WIDTH = 48


def fold_count(axis):
    return sum(1 for c in axis.cols() if c and c[0] == FOLD)


def test_young_session_is_right_aligned():
    axis = Axis(0, 100, WIDTH, [(10, 60)])
    cols = axis.cols()
    assert cols[-1][1] == 100
    assert cols[0] is None
    first = next(i for i, c in enumerate(cols) if c)
    assert all(c is not None for c in cols[first:])


def test_long_gap_folds_and_short_gap_does_not():
    busy = [(0, 60), (60 + 47 * 60, 60 + 47 * 60 + 60)]
    folded = Axis(0, 2990, WIDTH, busy)
    assert fold_count(folded) == 1
    assert folded.folds[0][1] - folded.folds[0][0] > 40 * 60
    # four minutes apart: the 30 s padding leaves 3 min, which saves nothing
    near = Axis(0, 2400, WIDTH, [(0, 600), (840, 2400)])
    assert near.scale == 60
    assert fold_count(near) == 0


def test_four_minute_gap_does_not_fold_at_thirty_seconds_per_column():
    # 30 s/col: the padded gap is 180 s, not more than max(180, 60), so it stays
    axis = Axis(0, 1400, WIDTH, [(0, 400), (640, 1400)])
    assert axis.scale == 30
    assert fold_count(axis) == 0


def test_session_longer_than_the_last_step_terminates():
    for exact in (False, True):
        axis = Axis(0, 86400 * 3, WIDTH, [(0, 86400 * 3)], exact=exact)
        assert len(axis.cols()) == WIDTH
        assert axis.col(86400 * 3 - 1) == WIDTH - 1


def test_more_folds_than_columns_drops_the_shortest():
    busy = [(i * 2000, i * 2000 + 10) for i in range(50)]
    for exact in (False, True):
        axis = Axis(0, 100000, 20, busy, exact=exact)
        assert len(axis.cols()) == 20
        assert fold_count(axis) <= 9


def test_degenerate_width_gives_no_columns():
    assert Axis(0, 100, 0, [(0, 100)]).cols() == []
    assert Axis(0, 100, 0, [], window=(0, 100)).cols() == []
    assert len(Axis(0, 100, 1, [(0, 100)]).cols()) == 1


def test_gap_shorter_than_two_columns_never_folds():
    axis = Axis(0, 20000, WIDTH, [(0, 100), (4000, 4100), (19000, 20000)])
    assert all(g[1] - g[0] > 2 * axis.scale for g in axis.folds)


def test_kept_folds_always_save_columns_over_varied_sessions():
    rng = random.Random(7)
    for _ in range(600):
        t0 = 1.7e9 + rng.uniform(0, 1e5)
        end = t0 + rng.uniform(300, 40000)
        busy = []
        for _ in range(rng.randint(1, 8)):
            a = rng.uniform(t0, end)
            busy.append((a, min(end, a + rng.uniform(1, 900))))
        width = rng.choice([20, 46, 48, 78, 118])
        axis = Axis(t0, end, width, busy, exact=rng.random() < 0.5)
        assert all(g[1] - g[0] > max(FOLD_MIN, 2 * axis.scale) for g in axis.folds)
        assert len(axis.cols()) == width


def test_no_hole_inside_content_and_all_busy_time_is_covered():
    busy = [(0, 120), (1800, 1900), (5000, 5300)]
    axis = Axis(0, 5400, WIDTH, busy)
    cols = axis.cols()
    first = next(i for i, c in enumerate(cols) if c)
    assert None not in cols[first:]
    for a, b in busy:
        assert axis.col(a) is not None
        assert axis.col(b - 1) is not None


def test_columns_are_contiguous_and_quantised():
    axis = Axis(0, 5400, WIDTH, [(0, 120), (5000, 5300)])
    assert axis.scale in STEPS
    cols = [c for c in axis.cols() if c]
    for left, right in zip(cols, cols[1:]):
        if FOLD not in (left[0], right[0]):
            assert abs(right[0] - left[1]) < 1e-6


def test_scale_is_stable_over_a_minute_of_now():
    busy = [(0, 400)]
    scales = {Axis(0, now, WIDTH, busy).scale for now in range(1500, 1560, 5)}
    assert len(scales) == 1


def test_leading_idle_is_trimmed():
    axis = Axis(0, 4000, WIDTH, [(3000, 3900)])
    assert axis.start == 3000 - 30
    assert min(c[0] for c in axis.cols() if c) == axis.start


def test_window_mode_is_linear_without_folds():
    axis = Axis(0, 7200, WIDTH, [], window=(7200 - 600, 7200))
    cols = axis.cols()
    assert fold_count(axis) == 0
    assert cols[0][0] == 6600 and cols[-1][1] == 7200
    assert abs(axis.scale - 600 / WIDTH) < 1e-9


def test_exact_mode_fills_the_width():
    axis = Axis(0, 1000, WIDTH, [(0, 1000)], exact=True)
    assert axis.cols()[0] is not None and axis.cols()[0][0] == 0
    assert axis.scale not in STEPS


def test_labels_do_not_touch_and_end_label_is_placed():
    axis = Axis(0, 3600, WIDTH, [(0, 3600)])
    labels = axis.labels(sel=(1200, 1500))
    spans = sorted((c, c + len(t)) for c, t, _ in labels)
    assert all(b < c for (_, b), (c, _) in zip(spans, spans[1:]))
    assert (WIDTH - 3, 'now', 'end') in labels
    assert labels[0][2] == 'sel'


def test_fold_label_gives_the_gap_length():
    axis = Axis(0, 3300, WIDTH, [(0, 60), (2900, 3300)])
    assert axis.folds
    assert any(role == 'fold' and text == duration(axis.folds[0][1] - axis.folds[0][0])
               for _, text, role in axis.labels())


def test_quantise_rounds_up():
    assert quantise(6) == 7.5
    assert quantise(7.5) == 7.5


def test_mid_session_window_is_anchored_at_its_right_edge():
    axis = Axis(0, 7200, WIDTH, [], window=(3000, 3600))
    cols = axis.cols()
    assert None not in cols
    assert cols[0][0] == 3000 and cols[-1][1] == 3600
    assert axis.col(3000) == 0 and axis.col(3599) == WIDTH - 1
    assert axis.col(3300) == WIDTH // 2
    assert axis.col(3700) is None


def test_short_exact_axis_has_no_empty_columns():
    axis = Axis(0, 100, 118, [(0, 100)], exact=True)
    assert None not in axis.cols()
    assert axis.cols()[0][0] == 0 and axis.cols()[-1][1] == 100
