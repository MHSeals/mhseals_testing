"""One run record for every tier, loaded by a per-source adapter that declares capabilities.

The double loop (fast tier <-> Unity <-> real boat) shares one scorer. Each source has an
adapter here that reads a session directory into the same record and says what the source can
provide (``caps``). The rules (``score.judge_record``) read only the record and ``caps``; the
source name is a label and nothing in scoring branches on it. A new tier costs one adapter.

Record::

    {source, session, run_id | None, meta, truth, track [(t, x, y, yaw)],
     window (t0, t1, end reason), time_limit_s | None, window_source,
     authority [{t, by, cause}], authority_known, authority_problem, horn_note,
     mission_stop {outcome, by, cause} | None, mission_stop_why,
     changes [{t_sim, id, value}], state_log_detail [str], task_events, statuses,
     contacts_reported [{t, object, by}] | None, caps}

A harness run (the run record ``<session>/runs/<run_id>.json`` that links its own files in
``result.files`` {track, task_events, task_status}) is read from that record:
``result.run_start_t`` (track clock) and ``result.time_limit_s`` (a key there decides, even
null). Its slice of the session journal ``events.jsonl`` (its ``mission_start`` ..
``mission_stop``, matched by run id) gives the holds and takeovers (``phase:
authority``/``disarmed``) and the horn (``phase: horn``, one per on/off edge), which then
replaces any horn in states.jsonl. ``mission_stop`` is the completeness marker. Journal times
are wall ``t_ns`` (the track clock's rate), aligned on ``result.run_start_t_ns`` when recorded,
else on ``mission_start`` = run start, which may come before the launch: then a journaled
horn's T6.zone is provisional (``horn_note``). An older session (no such record) is read from
session.json and the session-level files. Without a time limit the run is scored to the track
end and every earned row is provisional: the handbook sets no run duration (state/referee.md),
so none is guessed.

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
                  (a harness run: the journal slice's status, by the same rules, with a
                  missing mission_stop or a torn line inside as partial)
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
HORN = 'harbor_alert.horn'


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


def _seconds(event, anchor):
    """An event's journal time in seconds: wall ``t_ns`` (the track clock's rate) when both
    carry it, else ``ros_time`` (sim time, 0.0 before the first /clock)."""
    if 't_ns' in event and 't_ns' in anchor:
        return event['t_ns'] / 1e9
    return event.get('ros_time')


def journal(path, run_id=None):
    """The session journal's slice for one run: ``{events, started, ended, torn}``, or None
    without a journal or a ``mission_start``. A run id picks its own ``mission_start`` ..
    ``mission_stop``; without one (older sessions, whose session files are the latest run)
    the last ``mission_start`` and the next ``mission_stop``. ``ended`` is the stop event
    (None: the journal never recorded the run's end), ``torn`` the unparsable lines inside."""
    if not path.exists():
        return None
    lines = []
    for line in path.read_text(errors='replace').splitlines():
        if line.strip():
            try:
                event = json.loads(line)
            except ValueError:
                event = None
            lines.append(event if isinstance(event, dict) else None)

    def mine(event, phase):
        return event and event.get('phase') == phase and (
            run_id is None or event.get('run_id') == run_id)
    starts = [n for n, e in enumerate(lines) if mine(e, 'mission_start')]
    if not starts:
        return None
    first = starts[0] if run_id else starts[-1]
    last = next((n for n in range(first + 1, len(lines)) if mine(lines[n], 'mission_stop')),
                None)
    part = lines[first:len(lines) if last is None else last + 1]
    return dict(events=[e for e in part if e], started=lines[first],
                ended=None if last is None else lines[last], torn=part.count(None))


def _authority(events, offset, started):
    """Owner changes (holds, takeovers, disarms) in track time."""
    out = []
    for e in events:
        t = _seconds(e, started)
        if t is None:
            continue
        if e.get('phase') == 'authority':
            out.append(dict(t=t + offset, by=(e.get('to') or {}).get('owner'),
                            cause=e.get('cause')))
        elif e.get('phase') == 'disarmed':
            out.append(dict(t=t + offset, by='external', cause='disarmed'))
    return out


def horn_log(part, offset):
    """(changes, status, detail) of the journaled horn (``phase: horn``, one per on/off edge)
    in one run's slice. complete: the run's ``mission_stop`` is journaled and nothing inside is
    torn; partial: no ``mission_stop`` (cut short) or a torn line; contradictory: two edges at
    one time with different values, or time running back. An ``on`` edge's value is the
    pattern (``one``/``two``), else ``on:<Hz>`` (unknown pattern, no zone commanded)."""
    changes, partial, contra, seen, last = [], [], [], {}, -float('inf')
    for e in part['events']:
        if e.get('phase') != 'horn':
            continue
        t = _seconds(e, part['started'])
        if not isinstance(t, (int, float)):
            partial.append('a horn event without a time')
            continue
        t += offset
        value = (e.get('pattern') or f"on:{e.get('freq_hz')}") if e.get('on') else 'off'
        if t < last:
            contra.append(f'horn edge at {round(t, 3)} goes back in time')
        last = max(last, t)
        if seen.setdefault(t, value) != value:
            contra.append(f'the horn is {seen[t]} and {value} at {round(t, 3)}')
        changes.append(dict(t_sim=t, id=HORN, value=value))
    if part['ended'] is None:
        partial.append('the journal has no mission_stop for this run (cut short)')
    if part['torn']:
        partial.append(f"{part['torn']} torn journal line(s) inside the run")
    status = 'contradictory' if contra else 'partial' if partial else 'complete'
    return changes, status, contra + partial


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


def _timing(result, meta, time_limit_s):
    """(run_start_t, time_limit_s, source text): the run record's ``result`` first (a key
    present there decides, even null), else session.json (older sessions), else the caller's
    rules value (None: the handbook sets no run duration)."""
    where = 'run record' if result else 'session.json'
    start = result.get('run_start_t', meta.get('run_start_t'))
    limit = result['time_limit_s'] if 'time_limit_s' in result else meta.get('time_limit_s')
    text = (f'{where} run_start_t' if start is not None else 'harness track start') + ' + ' + \
        (f'{where} time_limit_s' if limit else 'rules time_limit_s' if time_limit_s
         else 'track end (no time_limit_s: rows provisional)')
    return start, limit or time_limit_s, text


def _files(session, run, time_limit_s):
    """What every source writes the same way: truth, track, window, authority, logs. ``run``
    is the harness's run record (``runs/<run_id>.json``) or None for an older session."""
    truth = data.read_json(session / 'course_truth.json')
    if truth is None:
        raise Refused(f'{session.name}: no course_truth.json')
    result = (run or {}).get('result') or {}
    files = result.get('files') or {}

    def at(key):    # a record with outcome fields links exactly what the run wrote: no fallback
        if key in files or not has_outcome(run):     # to the session's latest-run files
            return session / files.get(key, f'{key}.jsonl')
        return session / 'runs' / str(run.get('run_id')) / f'{key}.unlinked'
    track = [(r['t'], r['x'], r['y'], r.get('yaw') or 0.0)
             for r in data.read_jsonl(at('track')) if {'t', 'x', 'y'} <= set(r)]
    if not track:
        raise Refused(f'{session.name}: no track samples in {at("track").name}')
    meta = data.read_json(session / 'session.json', {}) or {}
    start, limit, window_source = _timing(result, meta, time_limit_s)
    window = run_window(track, {} if start is None else dict(run_start_t=start), limit)
    warnings, approximate, offset = [], False, None
    part = journal(session / 'events.jsonl', (run or {}).get('run_id'))
    if part and 'run_start_t_ns' in result and 't_ns' in part['started']:
        offset = window[0] - result['run_start_t_ns'] / 1e9
    elif part and _seconds(part['started'], part['started']) is not None:
        offset = window[0] - _seconds(part['started'], part['started'])
        approximate = bool(files)
        if approximate:
            warnings.append('journal aligned on mission_start = run start; the launch may come '
                            'later (no run_start_t_ns in the run record)')
    if offset is None:
        warnings.append('authority events not aligned to track time (no mission_start)')
    authority = [] if offset is None else _authority(part['events'], offset, part['started'])
    changes, log, detail = state_log(session / 'states.jsonl', window[1], meta)
    horn = dict(missing='planned', complete='recorded').get(log, log)
    horn_note = None
    if files and offset is not None:      # a harness run journals the horn: it decides
        heard, status, problems = horn_log(part, offset)
        changes = [c for c in changes if c.get('id') != HORN] + heard
        horn = dict(complete='recorded').get(status, status)
        detail = detail + [f'journal: {p}' for p in problems]
        if approximate and any(c['value'] != 'off' for c in heard):
            horn_note = 'the horn time is aligned on mission_start, not the launch'
    claims = at('task_events').exists() or at('task_status').exists()
    stop = (part or {}).get('ended') or record_end(run)
    return dict(session=session, run_id=(run or {}).get('run_id'), meta=meta, truth=truth,
                authority_known=offset is not None,
                mission_stop=None if not stop else {k: stop.get(k)
                                                    for k in ('outcome', 'by', 'cause')},
                mission_stop_why=None if stop else 'no journal mission_start' if not part
                else 'the journal has no mission_stop for this run',
                track=track, window=window, time_limit_s=limit, state_log_detail=detail,
                window_source=window_source, authority=authority,
                authority_problem='; '.join(warnings) or None, horn_note=horn_note,
                changes=changes,
                task_events=list(data.read_jsonl(at('task_events'))),
                statuses=list(data.read_jsonl(at('task_status'))),
                contacts_reported=None,
                caps=dict(truth='exact', truth_sigma_m=None, state_log=log, horn=horn,
                          claims=claims, reports=False))


def _sim(contacts):
    def load(session, run, time_limit_s):
        record = _files(session, run, time_limit_s)
        record['caps']['contacts'] = contacts
        return record
    return load


def _real(session, run, time_limit_s):
    """A real run: surveyed course truth (same format, ``survey.sigma_m``), the RTK track as
    track.jsonl, judges' contacts in judge_contacts.jsonl ({t, object, by}; absent = not
    recorded, so strikes are unscorable), and no physics. Silence of the horn is not known
    without a state log."""
    record = _files(session, run, time_limit_s)
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


def run_of(path):
    """(session directory, run record or None) of a session directory or a run record
    ``<session>/runs/<run_id>.json``. A session directory means its latest judged run (one
    whose record links its files); a session without one is an older session, read from its
    session-level files."""
    path = Path(path)
    if path.suffix == '.json' and path.parent.name == 'runs':
        run = data.read_json(path)
        if not has_files(run):
            result = (run or {}).get('result') or {}
            if result.get('started') is False:
                raise Refused(f'{path.name}: not started ({result.get("outcome")}): '
                              f'{result.get("cause")}')
            raise Refused(f'{path.name}: the run record links no track (the run was not judged)')
        return path.parent.parent, run
    if not path.is_dir():
        raise Refused(f'{path}: no such session')
    return path, next(iter(judged_runs(path)[-1:]), None)


def judged_runs(session):
    """The run records under ``<session>/runs`` that link their own files, in run order."""
    out = []
    for target in sorted(Path(session).glob('runs/*.json')):
        run = data.read_json(target)
        if has_files(run):
            out.append(run)
    return out


def all_runs(session):
    """Every run record under ``<session>/runs`` in run order, with or without files (v7: an
    aborted, cancelled, errored or refused run is still a run of the batch). Records of other
    procedures (``procedure`` set and not ``mission``) are not mission runs and are skipped."""
    out = []
    for target in sorted(Path(session).glob('runs/*.json')):
        run = data.read_json(target)
        if isinstance(run, dict) and (has_files(run) or run.get('procedure') in (None, 'mission')):
            out.append(run)
    return out


def has_outcome(run):
    """The record carries the writer's outcome fields (``result.outcome``, testing dc042f84)."""
    return 'outcome' in ((run or {}).get('result') or {})


def has_files(run):
    """The run can be scored from its own files. Older records: they link files (only judged
    runs did). Records with outcome fields (``files`` always present, ``{}`` before launch,
    partial on abort or error): the run started and links a track."""
    result = (run or {}).get('result') or {}
    if not has_outcome(run):
        return bool(result.get('files'))
    return result.get('started') is not False and bool((result.get('files') or {}).get('track'))


def record_end(run):
    """The record's own run end in ``mission_stop``'s shape ({outcome, by, cause}), or None
    for an older record or one that never started. ``complete`` stays ``complete``; aborted,
    cancelled and error keep their names and cause (the journal's stop, when present, wins)."""
    result = (run or {}).get('result') or {}
    if not has_outcome(run) or result.get('started') is False:
        return None
    return dict(outcome=result['outcome'], by=result.get('by') or 'harness',
                cause=result.get('cause'))


def harness_end(session, run):
    """(started, mission_stop or None) of a run record from its journal slice: started when the
    journal has its ``mission_start`` or the record a run start."""
    part = journal(Path(session) / 'events.jsonl', run.get('run_id'))
    result = run.get('result') or {}
    started = result['started'] if isinstance(result.get('started'), bool) else (
        part is not None or any(k in result for k in ('run_start_t', 'run_start_t_ns')))
    stop = (part or {}).get('ended')
    if stop:
        return started, {k: stop.get(k) for k in ('outcome', 'by', 'cause')}
    return started, record_end(run)


def load(path, time_limit_s=None):
    """The run record of a session directory or a run record file (``run_of``), through the
    session's source adapter."""
    session, run = run_of(path)
    meta = data.read_json(session / 'session.json', {}) or {}
    source = source_of(meta)
    if source not in ADAPTERS:
        raise Refused(f'{session.name}: no adapter for source {source!r} '
                      f'(known: {", ".join(ADAPTERS)})')
    record = ADAPTERS[source](session, run, time_limit_s)
    record['source'] = source
    assert set(record['caps']) == set(CAPS), 'an adapter must declare every capability'
    return record
