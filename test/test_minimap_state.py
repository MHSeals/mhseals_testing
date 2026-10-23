"""The pure minimap state: crumbs, xte against the initial plan, epochs, floor, tf_static."""
import pytest

from mhseals_testing.ui.minimap.state import MapState, path_distance

LINE = [(0, 0), (10, 0), (20, 0)]


def test_xte_on_a_known_path():
    assert path_distance((5, 2), LINE) == 2
    assert path_distance((25, 0), LINE) == 5
    assert path_distance((3, 4), [(0, 0)]) == 5
    s = MapState()
    assert s.xte((5, 2)) is None
    s.ingest('plan', points=LINE)
    assert s.xte((12, -1.5)) == 1.5


def test_replans_keep_the_initial_reference_until_the_goal_end_moves():
    s = MapState()
    s.ingest('plan', points=LINE)
    s.ingest('plan', points=[(6, 3), (20, 0.1)])  # replan from the boat, same goal
    assert s.initial_plan == LINE
    assert s.plan[0] == (6, 3)
    s.ingest('plan', points=[(6, 3), (20, 5)])    # end moved 4.9 m: a new goal
    assert s.initial_plan == [(6, 3), (20, 5)]


def test_crumbs_by_spacing_or_age_carry_gps_speed_and_xte():
    s = MapState()
    s.ingest('plan', points=LINE)
    s.ingest('gps_raw', fix_type=6, h_acc_m=0.02, vel_cms=65535)
    s.ingest('pose_map', t=0.0, x=0.0, y=1.0)
    s.ingest('pose_map', t=0.1, x=0.1, y=1.0)   # too close and too soon
    s.ingest('pose_map', t=0.2, x=0.3, y=1.0)
    s.ingest('pose_map', t=1.5, x=0.3, y=1.0)   # stationary but a second passed
    assert [c['t'] for c in s.crumbs] == [0.0, 0.2, 1.5]
    assert s.crumbs[0]['fix'] == 6 and s.crumbs[0]['speed_mps'] is None
    assert s.crumbs[1]['xte_m'] == 1.0
    s.ingest('gps_raw', fix_type=3, h_acc_m=3.0, vel_cms=40)
    s.ingest('pose_map', t=3.0, x=2.0, y=0.0)
    assert s.crumbs[-1]['speed_mps'] == 0.4


def test_datum_jump_starts_an_epoch_and_non_finite_poses_are_ignored():
    s = MapState()
    s.ingest('pose_map', t=0, x=0, y=0)
    s.ingest('pose_map', t=1, x=float('nan'), y=0)
    s.ingest('pose_map', t=2, x=50, y=0)
    assert [c['epoch'] for c in s.crumbs] == [0, 1]
    s.restart_trail()
    assert not s.crumbs and s.epoch == 1


def test_detection_floor_tf_static_and_latest_kinds():
    s = MapState(floor=0.5)
    s.ingest('detections', items=[{'score': 0.4}, {'score': 0.5}, {'score': 0.9}])
    assert [d['score'] for d in s.detections] == [0.5, 0.9]
    s.ingest('tf_static', transforms={('base_link', 'lidar'): (0.1, 0, 0.5, 0)})
    s.ingest('tf_static', transforms={('base_link', 'zed'): (0.3, 0, 0.4, 0)})
    assert len(s.tf_static) == 2
    s.ingest('costmap', data='a')
    s.ingest('costmap', data='b')
    assert s.latest['costmap'] == 'b'
    with pytest.raises(ValueError):
        s.ingest('bogus')
