"""The RoboBoat domain adapter for crane's fast-sim core (crane fastsim/MIGRATION.md).

Every decision is the boat's: mhseals_nav's MissionHost ticks on the core's navigator, with the
limits, tracker hold-over and camera mount read from mhseals_nav's own files. A run folder is a
harness session (``finalize``), so the referee (``harness.score_analyze``) scores it unchanged:

  session.json       mhseals.session/1, kind and source ``kinematic``, the course identity
  course_truth.json  the generator's truth, byte for byte (written by the core)
  runs/<run_id>.json the run record; runs/<run_id>/ the track, task events and statuses
  events.jsonl       the journal on the sim clock: mission_start, horn edges, mission_stop
  fastsim.json       mhseals.fastsim/1 (the old readers' names) beside the core's run.json

The classes follow the core's DomainAdapter and Controller contracts (fastsim/controller.py) by
duck typing, so this module imports without crane on the path; the core loads it by reference.
"""

from datetime import datetime, timedelta, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path

from mhseals_testing import data, events as journal_events
from mhseals_testing.harness import journal, runs
from mhseals_testing.harness.horn import HornJournal, program
from mhseals_testing.strict_yaml import load_strict

PROCEDURE = dict(procedure='Mission rehearsal', key='t')  # the harness's mission procedure tags
RUN_START_NS = 0  # the journal's clock at run start is sim time: runs stay byte-identical
EPOCH = datetime.fromtimestamp(0, timezone.utc)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, default=str).encode()).hexdigest()


def crane_tools(crane):
    """crane's Tools/RoboBoat/course_truth module (horn schedules, identity checks)."""
    path = Path(os.path.expandvars(str(crane))).expanduser() / 'Tools' / 'RoboBoat' / 'course_truth.py'
    spec = importlib.util.spec_from_file_location('course_truth', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def camera_mount(frame):
    """(x, y, yaw) of ``frame`` in base_link from the robot description, and its (path, sha256);
    (None, None) for a boat tree without robot_geometry."""
    try:
        from mhseals_nav.robot_geometry import RobotGeometry, description_path
    except ImportError:
        return None, None
    R, t = RobotGeometry.load().pose(frame)
    path = description_path()
    return ((float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0])),
            (path, hashlib.sha256(path.read_bytes()).hexdigest()))


def boat_limits(overlay=None, camera_frame=None):
    """What the launched stack allows (startup_projection.effective_limits), the tracker's
    hold-over (config/tracker.yaml) and the camera: {limits, perception, tracker, sources}."""
    import mhseals_nav
    from mhseals_nav.startup_projection import effective_limits
    from fastsim.plant import Limits
    root = Path(mhseals_nav.__file__).resolve().parents[1]
    e = effective_limits(limits_file=overlay or '')
    checkers = e['goal_checkers']
    tracker_file = root / 'config' / 'tracker.yaml'
    spec = next(iter(load_strict(tracker_file.read_text()).values()))['ros__parameters']
    sources = {**e['sources'], str(tracker_file): hashlib.sha256(tracker_file.read_bytes()).hexdigest()}
    mount, description = camera_mount(camera_frame) if camera_frame else (None, None)
    if description:
        sources[str(description[0])] = description[1]

    def name(path):
        path = Path(path).resolve()
        return str(path.relative_to(root)) if path.is_relative_to(root) else path.name
    sources = {name(k): v for k, v in sorted(sources.items())}
    limits = Limits(
        vmax=tuple(e['velocity']['max']), vmin=tuple(e['velocity']['min']),
        accel=tuple(e['accel']['max']), decel=tuple(e['accel']['decel']),
        goal_tol=tuple(checkers['general_goal_checker'].values()),
        precise_tol=tuple(checkers['precise_goal_checker'].values()),
        rotate_above=float(e['rotation_shim']['angular_dist_threshold']),
        rotate_wz=float(e['rotation_shim']['rotate_to_heading_angular_vel']),
        progress_radius=float(e['progress']['radius']), progress_s=float(e['progress']['time']),
        pass_radius=float(e['passed_goal_radius']),
        footprint=tuple(tuple(p) for p in e['footprint']),
        extra={'controller': dict(e.get('controller', {})), 'sources': sources})
    perception = {'name': 'cone', 'hfov_deg': float(e['camera']['hfov_deg']),
                  'range_m': float(e['camera']['range_m']), 'occlusion': True,
                  'mount': list(mount) if mount else None}
    tracker = {'class_groups': {g: {'match': list(v['match']), 'prior': v['prior']}
                                for g, v in spec['class_groups'].items()},
               **{k: spec[k] for k in ('timeout', 'tau', 'miss_in_fov_s')}}
    return {'limits': limits, 'perception': perception, 'tracker': tracker, 'sources': sources}


def mission_params(task, mission=None, profile=None, overrides=None):
    """mhseals_nav's params for ``task`` with ``overrides`` merged key by key (a key the task lacks
    is an error); the run budget follows an overridden timeslot, or an overridden timeout wins."""
    from mhseals_nav.tasks.params import load_params, overlay, timeslot
    params = load_params(task, mission, profile)
    if not overrides:
        return params
    merged = overlay(params, overrides, 'overrides')
    if 'timeslot' in merged:
        merged.pop('timeslot' if 'timeout' in overrides else 'timeout', None)
    return timeslot(merged)


def keep(session, task, track, events, statuses, run_id):
    """What the referee judges, as procedures.mission.keep lays it out (that package needs the
    hardware stack): the session's track.jsonl, task_events.jsonl and task_status.jsonl and the
    run's copies under runs/<run_id>/; returns their paths relative to the session."""
    lines = lambda rows: ''.join(json.dumps(row) + '\n' for row in rows)  # noqa: E731
    texts = {'track.jsonl': lines(dict(t=t, x=x, y=y, yaw=yaw) for t, x, y, yaw in track),
             'task_events.jsonl': lines(dict(e, task=task) for e in events),
             'task_status.jsonl': lines(dict(s, task=task) for s in statuses)}
    for name, text in texts.items():
        data.atomic_write(Path(session) / name, text)
        data.atomic_write(Path(session) / 'runs' / run_id / name, text)
    return {name.removesuffix('.jsonl'): str(Path('runs', run_id, name)) for name in texts}


def _vx_max(controller):
    """The controller's own forward limit as launched (FollowPath.vx_max), as the boat asks it."""
    vx = (controller or {}).get('vx_max')
    if not isinstance(vx, float) or not math.isfinite(vx) or vx <= 0:
        raise ValueError(f'FollowPath.vx_max: {vx!r}')
    return vx


class MissionController:
    """mhseals_nav's MissionHost on the core's navigator, its events and statuses kept."""

    def __init__(self, ctx, adapter):
        import py_trees
        from mhseals_nav.tasks.host import MissionHost
        from mhseals_nav.tasks.tracks import Track
        self.Track = Track
        nav = ctx.navigator
        nav.controller_vx_max = lambda: _vx_max(ctx.limits.extra['controller'])
        self.events, self.statuses = [], []

        def received(rows):
            return lambda payload: rows.append({**payload, 'received': ctx.clock['now']})
        py_trees.blackboard.Blackboard.clear()  # one run at a time per process, as on the boat
        self.host = MissionHost(adapter.options['task'], adapter.params, nav, received(self.events),
                                lambda: ctx.clock['now'], status_sink=received(self.statuses))

    def tick(self, obs):
        tracks = [self.Track(t.label, t.x, t.y, t.confidence, None, t.static, False, t.id)
                  for t in obs.tracks]
        return self.host.tick(obs.pose, tracks, obs.fov)

    def close(self):
        pass


class Journal:
    """The session journal as the harness writes it, on the sim clock (deterministic)."""

    def __init__(self, path, tags):
        self.path, self.tags, self.t, self.seq = path, tags, 0.0, 0

    def emit(self, phase, **details):
        event = journal_events.payload(phase, self.t, {**self.tags, **details,
                                                       't_ns': RUN_START_NS + round(self.t * 1e9)},
                                       pid=0, node='fastsim')
        event.update(time=(EPOCH + timedelta(seconds=self.t)).isoformat(), host='fastsim', seq=self.seq)
        self.seq += 1
        journal.append(self.path, event)


class RoboBoatAdapter:
    """Options: ``task``, ``mission`` (a task YAML, default the boat's own), ``profile``,
    ``params`` (mission overrides merged key by key: what a sweep varies), ``limits_overlay``
    (a nav2 limits overlay, as the launch takes it), ``camera_frame``, ``unknown_static``
    ({within, after}: the tracker's UNKNOWN -> STATIC flip) and ``crane`` (a crane checkout)."""

    def __init__(self, base=None, task='mission', mission=None, profile=None, params=None,
                 limits_overlay=None, camera_frame='front_camera_link', unknown_static=None,
                 crane='$CRANE_DIR'):
        promote = unknown_static or {'within': 8.0, 'after': 2.0}
        self.base = base
        self.options = dict(task=task, mission=mission, profile=profile, params=params or {},
                            limits_overlay=limits_overlay, camera_frame=camera_frame,
                            unknown_static={'within_m': promote['within'], 'after_s': promote['after']},
                            crane=crane)
        self.params = mission_params(task, mission, profile, params)
        self.boat = boat_limits(limits_overlay, camera_frame)
        self.crane = crane_tools(crane)
        self.current = None

    def limits(self):
        return self.boat['limits']

    def overrides(self):
        return {'perception': self.boat['perception'],
                'tracker': {**self.boat['tracker'], 'promote': self.options['unknown_static']}}

    def identity(self):
        import mhseals_nav
        from fastsim.identity import git_state
        o = self.options
        return {'mhseals_nav': git_state(mhseals_nav.__file__), 'params_sha256': digest(self.params),
                'params': o['params'], 'limits_sources': self.boat['sources'], 'task': o['task'],
                'mission': o['mission'], 'profile': self.params.get('profile'),
                'limits_overlay': o['limits_overlay']}

    def time_limit(self, ctx):
        return float(self.params['timeout'])

    def condition(self, ctx):
        return {'task': self.options['task'], 'profile': self.params.get('profile')}

    def controller(self, ctx):
        self.current = MissionController(ctx, self)
        return self.current

    def finalize(self, run_dir, result):
        """The harness session layout beside the core's files (module docstring)."""
        out, task, ctrl = Path(run_dir), self.options['task'], self.current
        end = result['sim_s']
        run_id = runs.next_run_id(out / 'runs', PROCEDURE['procedure'])
        (out / 'runs' / run_id).mkdir(parents=True)
        files = keep(out, task, result['track'], ctrl.events, ctrl.statuses, run_id=run_id)
        log = Journal(out / 'events.jsonl', dict(PROCEDURE, run_id=run_id))  # nobody intervenes
        log.emit('mission_start', task=task)
        truth = out / 'course_truth.json'
        sounding, _ = program(truth)
        if sounding:
            scenario = result['context'].scenario
            edge, prog = HornJournal(log.emit, truth), scenario.program(sounding)
            blasts = self.crane.horn_blasts(prog)
            for t, on in sorted((t, on) for pair in blasts for t, on in zip(pair, (True, False))):
                if t <= end:
                    log.t = t
                    edge(json.dumps({'on': on, 'freq_hz': prog['freq_hz']}))
        log.t = round(end, 9)
        log.emit('mission_stop', outcome='complete', by='procedure')  # the procedure's own end
        outcome = 'RUNNING' if result['outcome'] == 'TIMEOUT' else result['outcome']
        provenance = result['provenance']
        record = dict(outcome='complete', cause=None, started=True, files=files,
                      time_limit_s=result['time_limit_s'], profile=self.params.get('profile'),
                      run_start_t=0.0, run_start_t_ns=RUN_START_NS, procedure=PROCEDURE['procedure'],
                      procedure_key='mission', run_id=run_id,
                      results=[dict(procedure='mission', task=task, outcome=outcome)])
        runs.write_run(out / 'runs', record, now=EPOCH, provenance=provenance)
        raw = result['context'].scenario.raw
        session = {'schema': 'mhseals.session/1',
                   'id': f"fastsim_{raw.get('name')}_{provenance.get('variant', 'base')}",
                   'kind': 'kinematic', 'source': 'kinematic', 'clock': {'domain': 'sim'},
                   'course_truth': 'course_truth.json', 'fidelity': result['fidelity'],
                   'course': {'name': raw.get('name'), 'identity': (raw.get('generator') or {}).get('identity'),
                              'file_sha256': result['identity']['scenario_sha256']},
                   'git': provenance.get('git'), 'config': result['adapter']}
        (out / 'session.json').write_text(json.dumps(session, indent=2) + '\n')
        flags = ['camera_at_origin' if f == 'sensor_at_origin' else f for f in result['flags']]
        summary = {k: v for k, v in result.items() if k not in ('track', 'events', 'context', 'error')}
        summary.update(schema='mhseals.fastsim/1', outcome=outcome, flags=flags,
                       logic_exception=result['error'])
        (out / 'fastsim.json').write_text(json.dumps(summary, indent=2, default=str) + '\n')

    def metrics(self, run_dir):
        return referee(run_dir)


def referee(run_dir):
    """The referee's per-run numbers for a session (scored once; the report is kept in it)."""
    from mhseals_testing.harness import score_analyze
    reports = sorted((Path(run_dir) / 'analysis').glob('score.*.json'))
    record = json.loads(reports[-1].read_text()) if reports else score_analyze.analyze(run_dir, now=EPOCH)[1]
    return score_analyze.metrics(record)


def referee_numbers(run_dir):
    """The referee's numeric results, flat (total, penalties, run), for twin diffs (fastsim.pairs)."""
    reports = sorted((Path(run_dir) / 'analysis').glob('score.*.json'))
    if not reports:
        return {}
    report, flat = json.loads(reports[-1].read_text()), {}
    for part in ('total', 'penalties', 'run'):
        for key, value in (report.get(part) or {}).items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                flat[f'{part}.{key}'] = value
            elif isinstance(value, list):
                flat[f'{part}.{key}'] = len(value)
    return flat


def describe_session(session):
    """{pair_key, truth_sha256, seed, condition, source, provenance} of a Unity session, the keys a
    fast run pairs on (fastsim.pairs)."""
    session = Path(session)
    meta = json.loads((session / 'session.json').read_text())
    identity = json.loads((session / 'course_truth.json').read_text())['generator']['identity']
    record = sorted((session / 'runs').glob('*.json'))
    run = json.loads(record[-1].read_text()) if record else {}
    condition = {'task': ((run.get('results') or [{}])[0]).get('task', 'mission'),
                 'profile': run.get('profile'), 'time_limit_s': float(run.get('time_limit_s') or 0.0)}
    return {'pair_key': identity['pair_key'], 'truth_sha256': identity['truth_sha256'],
            'seed': identity.get('seed'), 'condition': condition, 'source': meta.get('source', meta.get('kind')),
            'provenance': {'git': meta.get('git'), 'config': meta.get('config')}}
