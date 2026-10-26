"""One run record for every tier, loaded by a per-source adapter that declares capabilities.

The double loop (fast tier <-> Unity <-> real boat) shares one scorer. Each source has an
adapter here that reads a session directory into the same record and says what the source can
provide (``caps``). The rules (``score.judge_record``) read only the record and ``caps``; the
source name is a label and nothing in scoring branches on it. A new tier costs one adapter.

Record::

    {source, session, meta, truth, track [(t, x, y, yaw)], window (t0, t1, end reason),
     window_source, authority [{t, by, cause}], authority_problem, changes [{t_sim, id, value}],
     task_events, statuses, contacts_reported [{t, object, by}] | None, caps}

Capabilities (``caps``)::

    truth         exact | surveyed      sims place the course; a real course is surveyed
    truth_sigma_m float | None          survey 1-sigma (surveyed only)
    contacts      geometric | physics | judge
                  geometric: track vs truth geometry, no physics (the track can pass through
                  an object); physics: track vs truth geometry on a physically simulated
                  track; judge: the judges' contact sheet scores, geometry is diagnostic only
    state_log     bool                  a recorded course-state log (states.jsonl) exists
    horn          recorded | planned | unknown
                  recorded: the state log decides; planned: no log, the course program's
                  trigger stands in (provisional at most); unknown: no log and the source
                  cannot vouch for silence, so the harbor row is unscorable
    claims        bool                  the boat's published task events/status were kept
    reports       bool                  decoded RoboCommand reports exist (none yet)
"""

from pathlib import Path

from mhseals_testing import data

CAPS = ('truth', 'truth_sigma_m', 'contacts', 'state_log', 'horn', 'claims', 'reports')
TIME_LIMIT_S = 300.0     # mirrors score.PARAMS['time_limit_s']; the adapter is passed it


class Refused(Exception):
    """The session cannot be scored; the message says why."""


def run_window(track, meta, time_limit_s=TIME_LIMIT_S):
    """(t0, t1, end reason) from the harness: run start, then the time limit or track end."""
    t0 = meta.get('run_start_t', track[0][0])
    limit = meta.get('time_limit_s') or time_limit_s
    if t0 + limit < track[-1][0]:
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
    changes = list(data.read_jsonl(session / 'states.jsonl'))
    claims = (session / 'task_events.jsonl').exists() or (session / 'task_status.jsonl').exists()
    return dict(session=session, meta=meta, truth=truth, track=track, window=window,
                window_source=('session.json run_start_t' if 'run_start_t' in meta
                               else 'harness track start') + ' + ' +
                ('session.json time_limit_s' if meta.get('time_limit_s')
                 else 'PARAMS time_limit_s'),
                authority=authority, authority_problem=problem, changes=changes,
                task_events=list(data.read_jsonl(session / 'task_events.jsonl')),
                statuses=list(data.read_jsonl(session / 'task_status.jsonl')),
                contacts_reported=None,
                caps=dict(truth='exact', truth_sigma_m=None, state_log=bool(changes),
                          horn='recorded' if changes else 'planned', claims=claims,
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
                          horn='recorded' if record['changes'] else 'unknown')
    return record


ADAPTERS = {'kinematic': _sim('geometric'), 'unity': _sim('physics'), 'real': _real}


def source_of(meta):
    """session.json ``source``, else ``kinematic`` for kind kinematic, else ``unity``."""
    return meta.get('source') or ('kinematic' if meta.get('kind') == 'kinematic' else 'unity')


def load(session, time_limit_s=TIME_LIMIT_S):
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
