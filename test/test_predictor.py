import json
import math

from pytest import approx

from mhseals_testing.analysis.predictor import Predictor, steady_speed
from mhseals_testing.analysis.thrust import speed_curve
from mhseals_testing.analysis.tracking import load_settings, record_session, tracking_score

CURVE = dict(d1=0.5, d2=1.5)


def test_steady_speed_inverts_the_fitted_curve():
    fitted = speed_curve([(steady_speed(CURVE, e), e) for e in (0.3, 0.6, 0.9)])
    assert fitted['d1'] == approx(0.5) and fitted['d2'] == approx(1.5)
    assert steady_speed(CURVE, -0.5) == approx(-steady_speed(CURVE, 0.5))
    assert steady_speed(None, 0.5) == 0.0 and steady_speed(dict(d1=2.0, d2=0.0), 1.0) == approx(0.5)


def test_lag_reaches_95_percent_at_t95_and_signs_use_their_own_curve():
    p = Predictor({'surge+': CURVE, 'surge-': dict(d1=1.0, d2=3.0)}, {'surge': 6.0})
    for _ in range(600):
        out = p.step((0.5, 0, 0), 0.01)
    assert out[0] == approx(0.95 * steady_speed(CURVE, 0.5), rel=0.02)
    for _ in range(5000):
        out = p.step((-0.5, 0, 0), 0.01)
    assert out[0] == approx(steady_speed(dict(d1=1.0, d2=3.0), -0.5), rel=1e-3)


def test_drift_rotates_into_the_body_frame_and_skips_yaw():
    p = Predictor({}, {})
    out = p.step((0, 0, 0), 0.1, drift=(0.0, 1.0), yaw=math.pi / 2)   # north current, facing north
    assert out == approx((1.0, 0.0, 0.0), abs=1e-9)


def test_a_boat_that_follows_the_model_scores_high_and_a_wrong_curve_low():
    s = load_settings()
    run = lambda curve: [Predictor({'surge+': curve}, {'surge': 2.0}).step((0.6, 0, 0), 0.1)[0] for _ in range(100)]
    truth = run(CURVE)
    assert tracking_score(truth, truth, [0.6] * 100, 0.02, s) > 0.99
    assert tracking_score(truth, run(dict(d1=2.0, d2=6.0)), [0.6] * 100, 0.02, s) < 0.6


def test_record_session_writes_staleness_and_one_bad_run_never_flags(tmp_path):
    s, path = load_settings(), tmp_path / 'calibration' / 'current.json'
    verdict = None
    for mean in (0.9, 0.91, 0.89, 0.9, 0.9, 0.3):
        verdict = record_session(path, 'surge', dict(mean=mean, n=30, trust=0.9, version=4), s)
    stored = json.loads(path.read_text())['provenance']['tracking']['surge']
    assert len(stored['sessions']) == 6 and stored['staleness'] == verdict and not verdict['flag']


def test_steady_speed_with_a_tiny_positive_d2():
    assert steady_speed(dict(d1=0.5, d2=1e-16), 0.5) == approx(0.5)
    assert steady_speed(dict(d1=0.5, d2=1e-17), 0.5) == approx(0.5)
    assert steady_speed(dict(d1=0.5, d2=0.0), 0.5) == approx(0.5)


def test_bad_samples_leave_the_predictor_state_alone():
    p = Predictor({'surge+': CURVE, 'surge-': CURVE}, {'surge': 1.0})
    before = p.step((0.5, 0, 0), 0.1)[0]
    for effort, dt in ((float('nan'), 0.1), (0.5, float('nan')), (0.5, 0.0), (0.5, -1.0)):
        assert p.step((effort, 0, 0), dt)[0] == approx(before)
    assert p.step((0.5, 0, 0), 0.1)[0] > before


def test_record_session_stores_a_nan_mean_as_null_and_leaves_valid_json(tmp_path):
    path = tmp_path / 'current.json'
    record_session(path, 'surge', dict(mean=float('nan'), n=3, trust=0.9, version=4), load_settings())
    assert json.loads(path.read_text())['provenance']['tracking']['surge']['sessions'][0]['mean'] is None
