import pytest

from mhseals_testing.analysis import verdict as v

T = v.load_thresholds()


def test_shipped_thresholds_load():
    assert T == {'good': {'ceiling': 0.90, 'drift_trust': 0.80}, 'poor': {'ceiling': 0.60, 'drift_trust': 0.50}}


@pytest.mark.parametrize('ceiling, trust, grade', [
    (0.95, 0.90, 'good'),
    (0.90, 0.80, 'good'),           # the good threshold itself is good
    (0.75, 0.90, 'marginal'),
    (0.95, 0.70, 'marginal'),
    (0.59, 0.90, 'poor'),
    (0.95, 0.49, 'poor'),
    (0.50, 0.70, 'poor'),           # poor wins over marginal
    (None, 0.90, 'unknown'),
    (0.95, None, 'unknown'),
])
def test_grades(ceiling, trust, grade):
    assert v.verdict(ceiling, trust, T)[0] == grade


def test_reason_names_the_failing_measure_only():
    assert v.verdict(0.50, 0.70, T)[1] == 'ceiling 50% < 60%'
    assert v.verdict(0.95, 0.90, T)[1] == ''


def test_bad_yaml_names_the_file(tmp_path):
    f = tmp_path / 'c.yaml'
    f.write_text('verdict: {good: {ceiling: 0.5, drift_trust: 0.5}, poor: {ceiling: 0.9, drift_trust: 0.1}}\n')
    with pytest.raises(ValueError, match='c.yaml'):
        v.load_thresholds(f)
    f.write_text('window_s: 8\n')
    with pytest.raises(ValueError, match='c.yaml'):
        v.load_thresholds(f)


def test_conditions_settings_still_load_with_the_verdict_group():
    from mhseals_testing.analysis import conditions
    assert conditions.load_settings().window_s == 8
