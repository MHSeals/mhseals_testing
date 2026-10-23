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
