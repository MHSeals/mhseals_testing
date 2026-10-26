"""One run record for every tier, loaded by a per-source adapter that declares capabilities.

The double loop (fast tier <-> Unity <-> real boat) shares one scorer. Each source has an
adapter here that reads a session directory into the same record and says what the source can
provide (``caps``). The rules (``score.judge_record``) read only the record and ``caps``; the
source name is a label and nothing in scoring branches on it. A new tier costs one adapter.

Record::

    {source, session, meta, truth, track [(t, x, y, yaw)], window (t0, t1, end reason),
     time_limit_s | None, window_source, authority [{t, by, cause}], authority_problem,
     changes [{t_sim, id, value}], state_log_detail [str], task_events, statuses,
     contacts_reported [{t, object, by}] | None, caps}

``time_limit_s`` comes from session.json only. Without it the run is scored to the track end
and every earned row is provisional (the owner sets the official duration; none is guessed).

Capabilities (``caps``)::

    truth         exact | surveyed      sims place the course; a real course is surveyed
    truth_sigma_m float | None          survey 1-sigma (surveyed only)
    contacts      geometric | physics | judge
                  geometric: track vs truth geometry, no physics (the track can pass through
                  an object); physics: track vs truth geometry on a physically simulated
                  track; judge: the judges' contact sheet scores, geometry is diagnostic only
    state_log     missing | complete | partial | contradictory   (states.jsonl)
                  complete: present (even empty), every line parses, times in order, no gap
                  record, and any end record ({id: log_end, t_sim}) reaches the window end;
                  partial: a torn/unparsable line, a {id: log_gap} record, an end record
                  before the window end, or session.json states_log.complete false;
                  contradictory: one state given two values at one time, or time running back
    horn          recorded | partial | contradictory | planned | unknown
                  recorded: a complete log decides (no horn record = never sounded); partial:
                  a recorded horn is provisional, no horn is unscorable; contradictory:
                  unscorable; planned: no log, the course program's trigger stands in
                  (provisional at most); unknown: no log and the source cannot vouch for
                  silence, so the harbor row is unscorable
    claims        bool                  the boat's published task events/status were kept
    reports       bool                  decoded RoboCommand reports exist (none yet)
"""

import json
from pathlib import Path

from mhseals_testing import data

CAPS = ('truth', 'truth_sigma_m', 'contacts', 'state_log', 'horn', 'claims', 'reports')
MARKERS = ('log_gap', 'log_end')


class Refused(Exception):
    """The session cannot be scored; the message says why."""


def run_window(track, meta, time_limit_s=None):
    """(t0, t1, end reason) from the harness: run start, then the time limit or track end.
    No limit in session.json (or passed in): the track end, never a guessed duration."""
    t0 = meta.get('run_start_t', track[0][0])
    limit = meta.get('time_limit_s') or time_limit_s
    if limit and t0 + limit < track[-1][0]:
        return t0, t0 + limit, 'time_limit'
    return t0, track[-1][0], 'track_end'


def _authority(events, t_start):
    """Owner changes in track time, aligned on the harness's mission_start = run start."""
    started = next((e for e in events if e.get('phase') == 'mission_start'), None)
    if started is None or t_start is None or 'ros_time' not in started:
        return [], 'authority events not aligned to track time (no mission_start)'
    offset = t_start - started['ros_time']
    out = []
    for e in events:
        if e.get('phase') == 'authority':
            out.append(dict(t=e['ros_time'] + offset, by=(e.get('to') or {}).get('owner'),
                            cause=e.get('cause')))
        elif e.get('phase') == 'disarmed':
            out.append(dict(t=e['ros_time'] + offset, by='external', cause='disarmed'))
    return out, None


def state_log(path, t_end, meta):
    """(changes, status, detail) of a states.jsonl: missing | complete | partial |
    contradictory (module docstring)."""
    if not path.exists():
        return [], 'missing', []
    partial, contra, changes, seen, last = [], [], [], {}, -float('inf')
    for n, line in enumerate(path.read_text(errors='replace').splitlines(), 1):
        if not line.strip():
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            partial.append(f'line {n} torn or unparsable')
            continue
        t = rec.get('t_sim') if isinstance(rec, dict) else None
        if not isinstance(t, (int, float)):
            partial.append(f'line {n} has no numeric t_sim')
            continue
        if rec.get('id') == 'log_gap':
            partial.append(f'gap recorded at {t}')
            continue
        if rec.get('id') == 'log_end':
            if t < t_end:
                partial.append(f'log ends at {t}, before the window end {round(t_end, 3)}')
            continue
        if t < last:
            contra.append(f'line {n} goes back in time ({t} < {last})')
        last = max(last, t)
        if seen.setdefault((t, rec.get('id')), rec.get('value')) != rec.get('value'):
            contra.append(f'{rec.get("id")} has two values at {t}')
        changes.append(rec)
    if (meta.get('states_log') or {}).get('complete') is False:
        partial.append('session.json states_log.complete is false')
    status = 'contradictory' if contra else 'partial' if partial else 'complete'
    return changes, status, contra + partial


def _files(session, time_limit_s):
    """What every source writes the same way: truth, track, window, authority, logs."""
    truth = data.read_json(session / 'course_truth.json')
    if truth is None:
        raise Refused(f'{session.name}: no course_truth.json')
    track = [(r['t'], r['x'], r['y'], r.get('yaw') or 0.0)
             for r in data.read_jsonl(session / 'track.jsonl') if {'t', 'x', 'y'} <= set(r)]
    if not track:
        raise Refused(f'{session.name}: no track.jsonl samples')
    meta = data.read_json(session / 'session.json', {}) or {}
    window = run_window(track, meta, time_limit_s)
    authority, problem = _authority(list(data.read_jsonl(session / 'events.jsonl')), window[0])
    changes, log, detail = state_log(session / 'states.jsonl', window[1], meta)
    claims = (session / 'task_events.jsonl').exists() or (session / 'task_status.jsonl').exists()
    return dict(session=session, meta=meta, truth=truth, track=track, window=window,
                time_limit_s=meta.get('time_limit_s') or time_limit_s,
                state_log_detail=detail,
                window_source=('session.json run_start_t' if 'run_start_t' in meta
                               else 'harness track start') + ' + ' +
                ('session.json time_limit_s' if meta.get('time_limit_s')
                 else 'track end (no time_limit_s: rows provisional)'),
                authority=authority, authority_problem=problem, changes=changes,
                task_events=list(data.read_jsonl(session / 'task_events.jsonl')),
                statuses=list(data.read_jsonl(session / 'task_status.jsonl')),
                contacts_reported=None,
                caps=dict(truth='exact', truth_sigma_m=None, state_log=log,
                          horn=dict(missing='planned', complete='recorded').get(log, log),
                          claims=claims,
                          reports=False))


def _sim(contacts):
    def load(session, time_limit_s):
        record = _files(session, time_limit_s)
        record['caps']['contacts'] = contacts
        return record
    return load


def _real(session, time_limit_s):
    """A real run: surveyed course truth (same format, ``survey.sigma_m``), the RTK track as
    track.jsonl, judges' contacts in judge_contacts.jsonl ({t, object, by}; absent = not
    recorded, so strikes are unscorable), and no physics. Silence of the horn is not known
    without a state log."""
    record = _files(session, time_limit_s)
    sigma = (record['truth'].get('survey') or {}).get('sigma_m') \
        or record['meta'].get('truth_sigma_m')
    if not sigma:
        raise Refused(f'{session.name}: real run without survey sigma_m in course_truth.json '
                      'or truth_sigma_m in session.json')
    sheet = session / 'judge_contacts.jsonl'
    record['contacts_reported'] = list(data.read_jsonl(sheet)) if sheet.exists() else None
    record['caps'].update(truth='surveyed', truth_sigma_m=float(sigma), contacts='judge',
                          horn='unknown' if record['caps']['state_log'] == 'missing'
                          else record['caps']['horn'])
    return record


ADAPTERS = {'kinematic': _sim('geometric'), 'unity': _sim('physics'), 'real': _real}


def source_of(meta):
    """session.json ``source``, else ``kinematic`` for kind kinematic, else ``unity``."""
    return meta.get('source') or ('kinematic' if meta.get('kind') == 'kinematic' else 'unity')


def load(session, time_limit_s=None):
    """The run record of a session directory, through its source's adapter."""
    session = Path(session)
    if not session.is_dir():
        raise Refused(f'{session}: no such session')
    meta = data.read_json(session / 'session.json', {}) or {}
    source = source_of(meta)
    if source not in ADAPTERS:
        raise Refused(f'{session.name}: no adapter for source {source!r} '
                      f'(known: {", ".join(ADAPTERS)})')
    record = ADAPTERS[source](session, time_limit_s)
    record['source'] = source
    assert set(record['caps']) == set(CAPS), 'an adapter must declare every capability'
    return record
