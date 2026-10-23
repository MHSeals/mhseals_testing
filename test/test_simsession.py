"""Simulator sessions: extra bag topics, session clock, course truth copy and /clock rewinds (bags.md B14)."""
from types import SimpleNamespace

import yaml

from mhseals_testing.harness import simsession
from mhseals_testing.ros.processes import bag_command


def sim_profile(**changes):
    values = dict(name='crane_full_stack', use_sim_time=True, ground_truth='/crane/odom', course_truth=None)
    return SimpleNamespace(**{**values, **changes})


def test_crane_profile_extras_are_not_dropped_by_the_bag_excludes(tmp_path):
    import re
    command = bag_command(tmp_path / 'b')
    assert '--topics' not in command  # --all-topics would override it and record almost nothing
    regex = command[command.index('--exclude-regex') + 1]
    extras = simsession.extra_topics('crane_full_stack')
    assert '/crane/odom' in extras and '/clock' in extras
    assert not [t for t in extras if re.match(regex, t)]
    assert simsession.extra_topics('boat') == []


def test_sim_session_records_clock_domain_and_copies_the_truth(tmp_path):
    truth = tmp_path / 'truth.json'
    truth.write_text('{"bays": []}')
    record = simsession.start_session(sim_profile(course_truth=str(truth)), tmp_path / 's.session.yaml',
                                      tmp_path / 's.course_truth.json')
    assert record['clock']['domain'] == 'sim' and record['profile']['ground_truth'] == '/crane/odom'
    assert (tmp_path / 's.course_truth.json').read_text() == '{"bays": []}'
    assert yaml.safe_load((tmp_path / 's.session.yaml').read_text())['course_truth'] == 's.course_truth.json'


def test_missing_truth_is_recorded_not_fatal_and_the_boat_has_a_wall_clock(tmp_path):
    sim = simsession.start_session(sim_profile(course_truth=str(tmp_path / 'none.json')),
                                   tmp_path / 'a.session.yaml', tmp_path / 'a.json')
    assert sim['course_truth'] is None and not (tmp_path / 'a.json').exists()
    boat = simsession.start_session(sim_profile(name='boat', use_sim_time=False, ground_truth=None),
                                    tmp_path / 'b.session.yaml', tmp_path / 'b.json')
    assert boat['clock']['domain'] == 'wall' and 'course_truth' not in boat


def test_clock_rewind_reports_once_per_reset_and_ignores_jitter():
    watch = simsession.ClockWatch()
    readings = [10.0, 10.1, 10.05, 11.0, 2.0, 2.1, 2.2]
    assert [watch.feed(t) for t in readings] == [False, False, False, False, True, False, False]
