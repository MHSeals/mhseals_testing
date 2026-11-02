"""The RoboBoat fast-sim adapter, its cluster backends and bag replay (mhseals_testing.fastsim_adapter)."""

import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from mhseals_testing.fastsim_adapter import bag, cluster, config, replay

pytestmark = pytest.mark.smoke   # tools/smoke


def test_shipped_config_becomes_core_keys_with_tune_merged():
    doc = yaml.safe_load(config.CONFIG.read_text())
    os.environ.setdefault('FASTSIM_TRUTHS', '/truths')
    d = config.domain(doc, {'params': {'speed': 1.0}, 'family': 'f2'})
    assert d['adapter']['ref'] == config.ADAPTER
    assert d['adapter']['options']['params'] == {'speed': 1.0}
    assert d['adapter']['options']['camera_frame'] == 'front_camera_link'
    assert d['scenarios']['fields']['family'] == 'f2' and '$' not in d['scenarios']['dir']
    assert d['calibration'] == doc['tune']['calibration']
    assert (config.CONFIG.parent / d['calibration']).is_file()
    assert 'tune' not in d and 'core' not in d


def test_mission_overrides_move_the_run_budget_and_refuse_unknown_keys():
    pytest.importorskip('mhseals_nav')
    from mhseals_testing.fastsim_adapter.adapter import mission_params
    base = mission_params('mission')
    slot = base['timeslot']
    wider = mission_params('mission', overrides={'timeslot': {'setup_reserve_s': slot['setup_reserve_s'] - 60}})
    assert wider['timeout'] == base['timeout'] + 60
    assert mission_params('mission', overrides={'timeout': 300.0})['timeout'] == 300.0
    with pytest.raises(Exception):
        mission_params('mission', overrides={'no_such_key': 1})


def test_replay_config_names_every_layer_input():
    cfg = yaml.safe_load(replay.CONFIG.read_text())
    assert set(cfg['tune']['layers']) == {'tracker', 'mission'}
    assert {'observations', 'tracks', 'events'} <= set(cfg['topics'])


def _tf(child, parent, x, y, yaw, t=0.0):
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    sec, ns = int(t), int(round((t - int(t)) * 1e9))
    return NS(child_frame_id=child, header=NS(frame_id=parent, stamp=NS(sec=sec, nanosec=ns)),
              transform=NS(translation=NS(x=x, y=y, z=0.0), rotation=NS(x=q[0], y=q[1], z=q[2], w=q[3])))


def test_tf_composes_a_chain_and_interpolates():
    tf = bag.Tf()
    tf.add(NS(transforms=[_tf('odom', 'map', 10.0, 0.0, math.pi / 2)]), static=True)
    tf.add(NS(transforms=[_tf('base_link', 'odom', 0.0, 0.0, 0.0, t=1.0)]), static=False)
    tf.add(NS(transforms=[_tf('base_link', 'odom', 2.0, 0.0, 0.0, t=3.0)]), static=False)
    R, p = tf.pose_at(2.0, 'map', 'base_link')
    assert np.allclose(p, [10.0, 1.0, 0.0])          # odom x=1 is map +y after the 90 deg turn
    assert math.isclose(math.atan2(R[1, 0], R[0, 0]), math.pi / 2)
    with pytest.raises(LookupError):
        tf.pose_at(9.0, 'map', 'base_link')


def test_match_pairs_same_label_nearest_within_radius():
    T = lambda label, x, y: NS(label=label, x=x, y=y)  # noqa: E731
    pairs, extra, missed = replay.match([T('red', 0, 0), T('green', 5, 0), T('red', 9, 9)],
                                        [T('red', 0.5, 0), T('green', 5, 3)], 1.0)
    assert [(a.x, b.x) for a, b, _ in pairs] == [(0, 0.5)] and (extra, missed) == (2, 1)


def test_segments_follow_the_tasks_the_boat_started():
    events = [{'task': 'nav_channel', 'event': 'task_started', 'time': 5.0},
              {'task': 'nav_channel', 'event': 'gate_cleared', 'time': 9.0},
              {'task': 'return_home', 'event': 'task_started', 'time': 20.0}]
    assert replay.segments(events, {}, 0.0) == [('nav_channel', 5.0), ('return_home', 20.0)]
    assert replay.segments(events, {'task': 'mission'}, 0.0)[0] == ('mission', 5.0)
    assert replay.segments([], {}, 3.0) == [('mission', 3.0)]


def test_cluster_backend_reads_metrics_and_lists_artifacts(tmp_path):
    run = tmp_path / 'runs' / 'v' / 's3'
    (run / 'analysis').mkdir(parents=True)
    (run / 'metrics.json').write_text(json.dumps({'points': 175, 'succeeded': 0.0, 'flag': True}))
    (run / 'run.json').write_text('{}')
    (run / 'track.jsonl').write_text('')
    (run / 'analysis' / 'score.1.json').write_text('{}')
    b = cluster.FastsimBackend('fastsim_roboboat', {})
    b.out, b.variant, b.shas, b.ctx = tmp_path / 'runs', 'v', {'mhseals_nav': 'abc'}, NS(spec={'arm': 'v'})
    assert b.run(3) == run and b.metrics(run) == {'points': 175.0, 'succeeded': 0.0}
    names = {(n, r) for _, n, r in b.artifacts(run)}
    assert {('metrics.json', 'summary'), ('analysis/score.1.json', 'summary'), ('track.jsonl', 'raw'),
            ('code.json', 'summary')} <= names


def test_hypothesis_repo_comes_from_the_job_and_the_rest_need_urls(tmp_path):
    ctx = NS(work_dir=tmp_path / 'jobs' / 'j1', repos={'mhseals_nav': tmp_path / 'nav'},
             resolved={'mhseals_nav': 'f' * 40}, spec={})
    with pytest.raises(RuntimeError, match='no url for mhseals_testing'):
        cluster.FastsimBackend('x', {}).code(ctx)


def test_one_seed_end_to_end(tmp_path):
    """A real fast run on the shipped test course (needs py_trees and a crane checkout)."""
    pytest.importorskip('py_trees')
    truths = Path(os.environ.get('FASTSIM_TRUTHS', '/nonexistent'))
    if not os.environ.get('CRANE_DIR') or not (truths / 'roboboat_2026-s1.json').is_file():
        pytest.skip('needs CRANE_DIR and FASTSIM_TRUTHS holding roboboat_2026-s1.json')
    from mhseals_testing.fastsim_adapter import batch
    scored, results = batch.run(seeds=[1], out=tmp_path, workers=1, allow_dirty=True)
    (run, m), = scored
    assert results[0].outcome in ('SUCCEEDED', 'TIMEOUT', 'FAILED')
    assert 'points' in m and (Path(run) / 'session.json').is_file()
