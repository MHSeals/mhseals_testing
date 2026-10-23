import struct

from mhseals_testing.health.kinds import topic_rate
from mhseals_testing.health.model import Fact
from mhseals_testing.health.registry import KINDS, build_params, discover
from mhseals_testing.validation import validate_raw


def run(hz, **data):
    discover()
    params = build_params('topic_rate', dict(topic='/gps/fix', hz=5))
    return KINDS['topic_rate'].evaluate(params, {'/gps/fix': Fact(hz, data=data)})


def test_nominal_rate_is_ok_and_deviations_are_marked():
    assert run(5.0, max_gap=.25, stamp_age=.1).status == 'ok'
    assert (run(3.0).status, run(3.0).mark) == ('warn', '↓')
    assert (run(2.0).status, run(2.0).mark) == ('fail', '↓')
    assert (run(9.0).status, run(9.0).mark) == ('warn', '↑')
    assert (run(5.0, max_gap=1.0).status, run(5.0, max_gap=1.0).mark) == ('warn', '~')
    assert run(0).mark == '×'


def test_stamp_age_limit_derives_from_the_rate():
    assert topic_rate.classify(5, 5, stamp_age=1.4)[0] == 'ok'    # under max(0.6, 1.5)
    assert topic_rate.classify(5, 5, stamp_age=1.6)[0] == 'warn'
    assert topic_rate.classify(20, 20, stamp_age=1.4)[0] == 'ok'  # floor of 1.5 s


def test_missing_fact_is_unknown():
    discover()
    params = build_params('topic_rate', dict(topic='/t', hz=5))
    assert KINDS['topic_rate'].evaluate(params, {}).status == 'unknown'


def test_raw_cdr_header_stamp_is_read_without_deserializing():
    data = b'\x00\x01\x00\x00' + struct.pack('<iI', 100, 500_000_000) + b'\x00' * 8
    assert abs(validate_raw(data, 100.6).stamp_age - .1) < 1e-6
    assert validate_raw(data, 110).issue
    big = b'\x00\x00\x00\x00' + struct.pack('>iI', 100, 0) + b'\x00' * 8
    assert validate_raw(big, 100.1).issue == ''
    assert validate_raw(b'\x00\x01', 1).issue == 'truncated message'
