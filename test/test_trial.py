import math

from mhseals_testing.analysis.trial import summarize_trial


def samples(gap_at=None, frame_change_at=None):
    rows = []
    for tick in range(60):
        if tick == gap_at:
            continue
        rows.append((tick / 10, dict(vx=.1 if tick < 20 else .3, vy=0., wz=0., x=tick / 100, y=0.,
                                     frame='odom',
                                     child='other' if tick == frame_change_at else 'base_link')))
    return rows


def test_reports_drift_adjusted_sign_not_a_calibration_pass():
    result = summarize_trial(samples(), 'surge', 1, 0, 2, 5.9)
    assert result['verdict'] == 'SIGN MATCH'
    assert math.isclose(result['response'], .2)
    assert math.isclose(result['displacement']['dx'], .38)  # first driven sample is after the start
    assert 'not a calibration pass' in result['detail']
    assert summarize_trial(samples(), 'surge', -1, 0, 2, 5.9)['verdict'] == 'SIGN MISMATCH'
    assert summarize_trial(samples(), 'sway', 1, 0, 2, 5.9)['verdict'] == 'LOW RESPONSE'


def test_missing_gappy_or_mixed_frame_data_is_inconclusive():
    assert summarize_trial(samples(), 'surge', 1, 20, 21, 22)['verdict'] == 'INCONCLUSIVE'
    sparse = [row for row in samples() if row[0] < 2 or row[0] > 5]
    assert summarize_trial(sparse, 'surge', 1, 0, 2, 5.9, stale_after=2)['verdict'] == 'INCONCLUSIVE'
    result = summarize_trial(samples(frame_change_at=30), 'surge', 1, 0, 2, 5.9)
    assert result['verdict'] == 'INCONCLUSIVE' and 'frame' in result['detail']


def test_window_mean():
    from mhseals_testing.analysis.trial import window_mean
    rows = [(t, dict(vx=float(t), vy=None, wz=1.0)) for t in range(5)]
    assert window_mean(rows, 1, 3) == dict(vx=2.0, vy=None, wz=1.0)
    assert window_mean(rows, 10, 11) == dict(vx=None, vy=None, wz=None)
