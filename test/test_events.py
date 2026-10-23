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
