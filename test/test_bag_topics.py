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


def test_lean_profile_records_the_authority_set():
    for needed in ('/cmd_vel', '/cmd_vel/manual', '/cmd_vel/applied', '/command_mux/state',
                   '/command_mux/request', '/mhseals_testing/authority'):
        assert needed in lean_topics()


def test_lean_profile_records_v2_perception_but_nothing_local_to_the_camera_host():
    topics = lean_topics()
    for needed in ('/perception/tracks', '/perception/status', '/objects/map',
                   '/front_camera/preview/compressed'):
        assert needed in topics
    # Raw detections and observations stay on the camera host; the sim keeps its /detections.
    local = ('/detections', '/perception/observations')
    assert not [t for t in topics if 'zed_node' in t or t in local]
    extra = yaml.safe_load(PROFILE.read_text())['extra']['crane_full_stack']['topics']
    assert '/detections' in extra


DROPPED = ('/front/zed_node/imu/data', '/front/zed_node/obj_det/objects',
           '/front/zed_node/left/camera_info', '/front_camera/camera_info',
           '/perception/observations', '/perception/track_grid', '/perception/detection_markers',
           '/tracked_objects', '/perception/costmap', '/front_camera/depth/image/compressedDepth',
           '/cam/camera_info')
KEPT = ('/front/zed_node/status/health', '/perception/tracks', '/perception/status',
        '/detections', '/objects/map', '/mhseals_testing/minimap/costmap', '/zed_node_like',
        '/front_camera/preview/compressed')
HEAVY = ('theora_image_transport/msg/Packet', 'ffmpeg_image_transport_msgs/msg/FFMPEGPacket',
         'point_cloud_interfaces/msg/CompressedPointCloud2', 'zed_msgs/msg/DepthInfoStamped')


def test_full_profile_drops_the_camera_hosts_topics_and_heavy_types():
    import re
    config = yaml.safe_load((PROFILE.parents[1] / 'bags.yaml').read_text())
    regex = re.compile(config['exclude_regex'])
    assert [t for t in DROPPED if not regex.fullmatch(t)] == []
    assert [t for t in KEPT if regex.fullmatch(t)] == []
    assert set(HEAVY) <= set(config['exclude_types'])
