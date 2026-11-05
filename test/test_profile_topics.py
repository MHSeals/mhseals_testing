"""The profile's topic map (config/profile.yaml topics:, a profile's topics: overrides a key). A wrong
default is silent, so every shipped default is pinned to the literal it replaced, and the feeds are
checked to subscribe what the profile names."""

import types

import pytest
import yaml

from mhseals_testing import profiles

pytestmark = pytest.mark.smoke

# the literals in ros/node.py and ros/minimap_feeds.py before the map (11-04), byte for byte
OLD = {
    'mavlink_source': '/uas1/mavlink_source',
    'drift': '/calibrated_velocity_controller/drift',
    'gps_fix': '/gps/fix',
    'gps_raw': '/mavros/gpsstatus/gps1/raw',
    'odom_gps': '/odom/gps',
    'odom_global': '/odom/global',
    'odom_local': '/odom/local',
    'local_costmap': '/local_costmap/costmap',
    'global_costmap': '/global_costmap/costmap',
    'plan': '/plan',
    'local_plan': '/local_plan',
    'tracks': '/perception/tracks',
    'objects': '/objects/map',
    'perception_status': '/perception/status',
    'fence_status': '/fence/status',
    'fence_marker': '/fence/marker',
}


def test_shipped_defaults_equal_the_old_literals():
    assert profiles.default_topics() == OLD


@pytest.mark.parametrize('name', profiles.available_profiles())
def test_every_shipped_profile_carries_the_defaults(name):
    assert profiles.load_profile(name).topics == OLD


def test_every_default_has_a_schema_entry():
    schema = yaml.safe_load((profiles.PROFILE_DIR.parent / 'schema' / 'profile.yaml').read_text())
    assert schema.pop('profile') == {'closed': True}
    assert {key.removeprefix('profile.topics.') for key in schema} == set(OLD)


def test_a_profile_overrides_one_key_and_refuses_an_unknown_or_relative_one():
    data = yaml.safe_load((profiles.PROFILE_DIR / 'boat.yaml').read_text())
    profile = profiles.parse_profile({**data, 'topics': {'mavlink_source': '/uas2/mavlink_source'}})
    assert profile.topics == {**OLD, 'mavlink_source': '/uas2/mavlink_source'}
    assert profiles.default_topics() == OLD          # the override never leaks into the defaults
    with pytest.raises(ValueError, match='topics.bogus'):
        profiles.parse_profile({**data, 'topics': {'bogus': '/x'}})
    with pytest.raises(ValueError, match='absolute'):
        profiles.parse_profile({**data, 'topics': {'drift': 'drift'}})


def test_a_test_double_without_topics_reads_the_defaults():
    assert profiles.topics_of(types.SimpleNamespace()) == OLD


def test_the_module_constants_are_the_defaults():
    pytest.importorskip('rclpy')
    from mhseals_testing.ros import minimap_feeds as feeds, node
    assert (node.MAVLINK_TOPIC, node.DRIFT_TOPIC) == (OLD['mavlink_source'], OLD['drift'])
    assert (feeds.LOCAL, feeds.GLOBAL, feeds.TRACKS, feeds.STATUS, feeds.OBJECTS, feeds.FENCE_STATUS,
            feeds.FENCE_MARKER) == tuple(OLD[k] for k in ('local_costmap', 'global_costmap', 'tracks',
                                                          'perception_status', 'objects', 'fence_status',
                                                          'fence_marker'))


class Node:
    def __init__(self):
        self.live = []

    def create_subscription(self, kind, topic, callback, qos):
        self.live.append(topic)
        return topic

    def destroy_subscription(self, sub):
        self.live.remove(sub)

    def create_publisher(self, kind, topic, qos):
        return types.SimpleNamespace(publish=lambda m: None)

    def create_timer(self, period, callback):
        return callback


def test_map_feeds_subscribe_the_profiles_names():
    pytest.importorskip('rclpy')
    from mhseals_testing.ros import minimap_feeds as feeds
    renamed = {key: '/other' + value for key, value in OLD.items()}
    node = Node()
    map_feeds = feeds.MapFeeds(node, topics=renamed)
    objects = renamed['tracks'] if feeds.perception_msgs() else renamed['objects']
    wanted = {renamed[k] for k in ('gps_fix', 'odom_gps', 'odom_global', 'plan', 'local_plan',
                                   'perception_status')} | {objects}
    assert wanted <= set(node.live) and not set(OLD.values()) & set(node.live)
    map_feeds.want_layers(True, False, 20)
    assert renamed['global_costmap'] in node.live
    map_feeds.want_layers(True, False, 2)
    assert renamed['local_costmap'] in node.live and renamed['global_costmap'] not in node.live
