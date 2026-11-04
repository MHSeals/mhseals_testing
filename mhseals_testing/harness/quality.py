"""``boat_agent quality [SESSION]``: a quick data-quality summary of a session's bags. Read-only.

For the session profile's command topic, odometry and sensors (plus any ``--topics``) it gives each topic's
count, rate, largest gap and the gaps longer than its tolerance (3 periods of the sensor's nominal ``hz``, else
GAP_S), flagging a rate below LOW_RATE of nominal. ``coverage`` answers "was the boat measured while it was
commanded": the share of the time the command topic was live (messages no more than CMD_GAP_S apart) that odometry
was live too (no more than its tolerance apart). Only those topics are read (bagread.read_raw), never decoded.
"""

from pathlib import Path

from mhseals_testing import data
from mhseals_testing.harness import bagread

GAP_S = 1.0           # gap tolerance of a topic with no nominal rate
CMD_GAP_S = 0.5       # command messages closer than this are one commanded stretch
LOW_RATE = 0.8        # a rate below this share of the nominal hz is flagged
MAX_GAPS = 5          # the longest gaps listed per topic


def plan(session, extra=()):
    """``{topic: {'role', 'hz'}}`` from the session's profile (command, odometry, sensors) and ``extra``."""
    from mhseals_testing.profiles import load_profile
    header = data.read_json(Path(session) / 'session.json') or {}
    wanted, problem = {}, ''
    try:
        profile = load_profile(header['profile'])
    except Exception as error:      # noqa: BLE001 - an unknown or missing profile: say so, read the extra topics
        profile, problem = None, f"profile {header.get('profile')!r}: {error}"
    if profile is not None:
        wanted[profile.command_topic] = {'role': 'command', 'hz': None}
        wanted[profile.odometry] = {'role': 'odometry', 'hz': None}
        for sensor in profile.sensors:
            wanted.setdefault(sensor.topic, {'role': f'sensor:{sensor.name}', 'hz': sensor.hz})
            if sensor.topic == profile.odometry and sensor.hz:
                wanted[sensor.topic]['hz'] = sensor.hz
    for topic in extra:
        wanted.setdefault(topic, {'role': 'extra', 'hz': None})
    return wanted, problem


def tolerance(hz):
    return 3.0 / hz if hz else GAP_S


def topic_stats(stamps, hz, start_ns, end_ns):
    """Count, rate, largest gaps (window edges included) and flags of one topic's sorted stamps."""
    edges = [start_ns, *stamps, end_ns] if start_ns is not None else stamps
    gaps = sorted(((b - a) / 1e9, a) for a, b in zip(edges, edges[1:]))[::-1]
    span = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0.0
    rate = (len(stamps) - 1) / span if span > 0 else 0.0
    limit = tolerance(hz)
    long = [g for g in gaps if g[0] > limit]
    flags = [*(['missing'] if not stamps else []), *(['low_rate'] if hz and stamps and rate < LOW_RATE * hz else []),
             *(['gaps'] if long else [])]
    return {'count': len(stamps), 'rate_hz': round(rate, 2), 'nominal_hz': hz,
            'max_gap_s': round(gaps[0][0], 3) if gaps else None, 'gap_tolerance_s': round(limit, 3),
            'gaps_over': len(long), 'longest': [{'s': round(s, 3), 'at_ns': at} for s, at in long[:MAX_GAPS]],
            'flags': flags}


def live(stamps, gap_s):
    """The stretches ``[(a, b)]`` in which consecutive stamps are at most ``gap_s`` apart."""
    out = []
    for a, b in zip(stamps, stamps[1:]):
        if (b - a) / 1e9 <= gap_s:
            if out and out[-1][1] == a:
                out[-1] = (out[-1][0], b)
            else:
                out.append((a, b))
    return out


def overlap(first, second):
    """Total nanoseconds both interval lists cover (each sorted, disjoint)."""
    total, j = 0, 0
    for a, b in first:
        while j < len(second) and second[j][1] <= a:
            j += 1
        k = j
        while k < len(second) and second[k][0] < b:
            total += max(0, min(b, second[k][1]) - max(a, second[k][0]))
            k += 1
    return total


def summarize(session, extra=(), last_s=None, reader=None):
    """The quality document of one session (or bag) directory; ``reader`` is bagread.read_raw's seam."""
    reader = reader or bagread.read_raw
    session = Path(session)
    wanted, problem = plan(session, extra)
    stamps = {topic: [] for topic in wanted}
    segments = []
    for topic, _type, _data, t_ns in reader(session, topics=list(wanted),
                                             on_segment=lambda p, e: segments.append({'dir': p.name, 'error': e})):
        stamps[topic].append(t_ns)
    every = sorted(t for s in stamps.values() for t in s)
    start_ns, end_ns = (every[0], every[-1]) if every else (None, None)
    if last_s is not None and every:
        start_ns = max(start_ns, end_ns - int(last_s * 1e9))
        stamps = {k: [t for t in v if t >= start_ns] for k, v in stamps.items()}
    for v in stamps.values():
        v.sort()
    topics = {t: {'role': w['role'], **topic_stats(stamps[t], w['hz'], start_ns, end_ns)} for t, w in wanted.items()}
    doc = {'ok': True, 'session': str(session), 'window': {'start_ns': start_ns, 'end_ns': end_ns,
           'duration_s': round((end_ns - start_ns) / 1e9, 3) if every else 0.0}, 'segments': segments,
           'topics': topics}
    command = next((t for t, w in wanted.items() if w['role'] == 'command'), None)
    odometry = next((t for t, w in wanted.items() if w['role'] == 'odometry'), None)
    if command and odometry:
        commanded = live(stamps[command], CMD_GAP_S)
        measured = live(stamps[odometry], tolerance(wanted[odometry]['hz']))
        span = sum(b - a for a, b in commanded)
        doc['coverage'] = {'command': command, 'odometry': odometry, 'commanded_s': round(span / 1e9, 3),
                           'measured_share': round(overlap(commanded, measured) / span, 3) if span else None}
    bad = sorted(t for t, s in topics.items() if s['flags'])
    doc['verdict'] = 'clean' if not bad and not problem and every else 'check'
    doc['flagged'] = bad
    if problem:
        doc['warning'] = problem
    return doc
