"""The threshold kind: bands, magnitude, expected values and how the detail reads."""

import pytest

from mhseals_testing.health.kinds.threshold import Params, evaluate
from mhseals_testing.health.model import Fact


def run(fact, **params):
    return evaluate(Params(fact='x', **params), {'x': Fact(*fact) if isinstance(fact, tuple) else Fact(fact)})


@pytest.mark.parametrize('value,below,status', [(3.0, True, 'fail'), (3.4, True, 'warn'), (4.0, True, 'ok'),
                                                (9, False, 'fail'), (6, False, 'warn'), (1, False, 'ok')])
def test_bands_judge_in_the_configured_direction(value, below, status):
    bounds = {'warn': 3.5, 'fail': 3.2} if below else {'warn': 5, 'fail': 8}
    assert run(value, below=below, **bounds).status == status


def test_abs_judges_the_magnitude_but_the_text_keeps_the_sign():
    result = run((-0.8, 'GPS'), abs=True, below=False, warn=.5, fail=60, unit='s', label_first=True)
    assert result.status == 'warn' and result.detail == 'GPS -0.80s'
    assert run(-70, abs=True, below=False, warn=.5, fail=60).status == 'fail'


def test_offsets_of_1000_or_more_print_without_decimals():
    result = run((432123.4, 'GPS'), abs=True, below=False, warn=.5, fail=60, unit='s', label_first=True)
    assert result.detail == 'GPS +432123s'


def test_label_follows_the_value_by_default_and_quiet_drops_it():
    assert run((3.98, '15.9V 4S'), digits=1, unit='V').detail == '4.0V 15.9V 4S'
    assert run((3.98, '15.9V 4S'), quiet=True).detail == ''


def test_expect_accepts_a_value_or_a_list():
    assert run('MANUAL', expect=['MANUAL', 'HOLD']).status == 'ok'
    assert run('AUTO', expect=['MANUAL', 'HOLD']).status == 'fail'
    assert run(True, expect=True).status == 'ok'


def test_a_missing_or_non_numeric_fact_is_unknown():
    assert evaluate(Params(fact='x', warn=1), {}).status == 'unknown'
    assert run(True, warn=1).status == 'unknown'
