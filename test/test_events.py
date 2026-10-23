"""Event lines name their instance so concurrent suites on one domain stay separable."""

import json
import os

from mhseals_testing import events


def test_payload_carries_pid_and_node():
    event = events.payload('session_start', 1.5, dict(profile='x'))
    assert event['pid'] == os.getpid() and event['node'] == 'mhseals_testing'
    assert event['phase'] == 'session_start' and event['profile'] == 'x'


def test_details_cannot_spoof_the_instance():
    assert events.payload('p', 0.0, dict(pid=1), pid=7)['pid'] == 7


def test_line_is_json():
    assert json.loads(events.line(events.payload('p', 0.0, {})))['phase'] == 'p'


def test_fold_ignores_other_instances_unless_guest():
    lines = [dict(phase='a', pid=1), dict(phase='b', pid=2),
             dict(phase='c', pid=2, guest=True), dict(phase='d')]
    assert [e['phase'] for e in events.ours(lines, 1)] == ['a', 'c']


def test_legacy_bag_without_pids_is_kept_whole():
    lines = [dict(phase='a'), dict(phase='b')]
    assert events.ours(lines, 5) == lines


def test_same_pid_on_another_host_is_not_ours():
    lines = [dict(phase='a', pid=1, host='boat'), dict(phase='b', pid=1, host='tranq'),
             dict(phase='c', pid=1)]
    assert [e['phase'] for e in events.ours(lines, 1, 'boat')] == ['a', 'c']
    assert events.payload('p', 0.0, {})['host']


def test_legacy_lines_join_a_single_tagged_instance_only():
    one = [dict(phase='old'), dict(phase='a', pid=1, host='boat')]
    assert [e['phase'] for e in events.ours(one, 1, 'boat')] == ['old', 'a']
    two = one + [dict(phase='b', pid=1, host='tranq')]
    assert [e['phase'] for e in events.ours(two, 1, 'boat')] == ['a']


def test_unique_keeps_distinct_events_sharing_a_sim_ros_time():
    a = events.payload('marker', 0.0, dict(note='one'))
    b = events.payload('marker', 0.0, dict(note='two'))
    assert [e['note'] for e in events.unique([b, a, a, b])] == ['one', 'two']


def test_unique_orders_by_wall_time_across_a_clock_rewind():
    before = events.payload('marker', 50.0, dict(note='before reset'))
    after = events.payload('marker', 1.0, dict(note='after reset'))
    assert [e['note'] for e in events.unique([after, before])] == ['before reset', 'after reset']


def test_unique_collapses_legacy_lines_without_seq():
    old = dict(phase='p', ros_time=1.0, pid=1, host='h')
    assert events.unique([old, dict(old), dict(old, ros_time=2.0)]) == [old, dict(old, ros_time=2.0)]
