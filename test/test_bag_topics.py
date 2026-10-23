"""The lean recorder profile must keep the odometry that nav.map_rotation reads."""
from pathlib import Path

import yaml

PROFILE = Path(__file__).resolve().parents[1] / 'config' / 'bags' / 'topics.yaml'


def lean_topics():
    return yaml.safe_load(PROFILE.read_text())['lean']['topics']


def test_lean_profile_records_map_rotation_inputs():
    topics = lean_topics()
    # Field bags to date have neither: rotation needs /odom/gps vs /gps/fix, home needs /odom/global.
    for needed in ('/odom/gps', '/odom/global', '/gps/fix'):
        assert needed in topics


def test_lean_profile_is_unique_absolute_names():
    topics = lean_topics()
    assert len(topics) == len(set(topics))
    assert all(topic.startswith('/') for topic in topics)


def test_lean_profile_keeps_the_rtk_status_topic():
    # Bags up to boat_20261023_035709 carry it as diagnostic_msgs/DiagnosticArray; readers must go by the
    # recorded type, so the topic name stays stable instead of being renamed when ts041 changes the payload.
    assert '/mhseals_testing/rtk/status' in lean_topics()


def test_lean_profile_regex_is_an_include_and_no_listed_topic_is_excluded():
    import re
    lean = yaml.safe_load(PROFILE.read_text())['lean']
    # `-e` includes; an exclude key would drop listed topics such as /mhseals_testing/minimap/costmap.
    assert 'exclude_regex' not in lean
    assert 'regex' in lean
    excludes = [v for k, v in lean.items() if k.startswith('exclude')]
    for pattern in excludes:
        assert not [t for t in lean['topics'] if re.search(pattern, t)]


def test_crane_profile_extra_carries_the_truth_odometry():
    extra = yaml.safe_load(PROFILE.read_text())['extra']['crane_full_stack']['topics']
    assert '/crane/odom' in extra and '/clock' in extra


def test_lean_regex_matches_events_rtk_status_and_pwm():
    import re
    # Jazzy `--topics X -e R` records X plus R's matches; the events topic is only in R, never in the list.
    pattern = yaml.safe_load(PROFILE.read_text())['lean']['regex']
    for topic in ('/mhseals_testing/events', '/mhseals_testing/rtk/status', '/thrusters/pwm'):
        assert re.match(pattern, topic)
