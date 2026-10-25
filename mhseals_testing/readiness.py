"""Decide whether thrusters may move.

Only sensor and TF health block motion: without trustworthy odometry and
frames, a trial cannot be scored and Nav2 cannot drive safely. Power, links
and compute are advisories: always shown, never blocking, because the
operator (not the suite) owns the e-stop and the boat.

Arming itself is refused only while a node the health checks gating ``arm`` require is off the
graph: the boat's safety watchdogs (config/health/preflight.yaml), so a boat never arms without
them silently.
"""


def arm_nodes(specs):
    """FQNs the enabled checks gating ``arm`` require, in file order."""
    return tuple(dict.fromkeys(node for spec in specs if spec.enabled and 'arm' in spec.gates
                               for node in getattr(spec.params, 'require', None) or ()))


def arm_refusal(required, nodes):
    """Why arming is refused with ``nodes`` (FQNs) on the graph, or ''."""
    missing = [node for node in required if node not in set(nodes)]
    if not missing:
        return ''
    return (f"safety watchdog{'s' if len(missing) > 1 else ''} not running: {', '.join(missing)}"
            ' (they start with odometry on the boat)')


BEAT_STALE_S = 2.0    # s without a watchdog's ~/status beat (0.5 s apart) before it counts as dead


def liveness_refusal(required, beats, now, off=(), stale_s=BEAT_STALE_S, runs=None):
    """Why arming is refused by the watchdogs' own word, or '': faster than the graph, which keeps
    a crashed node until its DDS lease ends.

    ``beats`` maps a watchdog FQN to when its ``~/status`` was last received (``now``'s clock);
    one never heard (a build without the beat) is left to the graph and ``off``, the parts holding
    the watchdogs that their computer reports off.

    A beat counts as dead only while another required watchdog's beat is fresh and has been
    arriving without a gap over ``stale_s`` (``runs``: when each one's unbroken run began) for over
    ``stale_s`` past the silent one's last: the link was up all that time, so that one is gone.
    When every beat stops together (a radio dropout) nothing here refuses, nor in the first
    ``stale_s`` after the link returns, while the others' first beats are still on their way; the
    boat's own link watchdog holds, and the graph with its grace and the device report remain.
    """
    runs = runs or {}
    silent = []
    for node in required:
        if node not in beats:
            continue
        if any(now - beats[other] <= stale_s
               and now - max(runs.get(other, beats[other]), beats[node]) > stale_s
               for other in required if other != node and other in beats):
            silent.append(f'{node} {now - beats[node]:.1f} s')
    reasons = []
    if silent:
        reasons.append(f"safety watchdog{'s' if len(silent) > 1 else ''} silent: {', '.join(silent)} "
                       '(crashed, or the link to the boat is down)')
    if off:
        reasons.append(f"safety watchdogs reported off: {', '.join(off)}")
    return '; '.join(reasons)


def motion_blockers(snapshot, profile, bench_override=False):
    """Reasons motion is refused, empty when ready.

    ``bench_override`` is the operator's explicit acknowledgement of a bench
    test with no sensors running; it skips both checks, since TF cannot exist
    without odometry. Procedures that score motion still refuse to run then.
    """
    blockers = []
    if not bench_override:
        for sensor in profile.sensors:
            if not sensor.required:
                continue
            row = snapshot['sensors'].get(sensor.name)
            if row is None:
                blockers.append(f'{sensor.name}: no messages on {sensor.topic}')
            elif row['status'] != 'LIVE':
                blockers.append(f"{sensor.name}: {row['status']} {row['issue']}".rstrip())
        for chain in snapshot['tf']:
            if chain['status'] != 'LIVE':
                blockers.append(f"TF {chain['source']} -> {chain['target']}: {chain['detail']}")
    return blockers


# Advisories fed by subscriptions rather than polled: nothing refreshes them
# when the publisher dies, so their age decides whether they still count.
SUBSCRIBED = ('flight controller', 'battery')
STALE_AFTER = 3.0


def advisories(snapshot, order=('FAIL', 'WARN', 'N/A', 'OK')):
    """Advisory rows, worst first: (name, level, detail). Never consulted for interlocks.

    A subscription-fed row older than STALE_AFTER seconds is shown N/A, not its last verdict.
    """
    rows = []
    for name, row in snapshot['advisories'].items():
        if name in SUBSCRIBED and row.get('age', 0) > STALE_AFTER:
            rows.append((name, 'N/A', f"stale ({row['age']:.0f}s)"))
        else:
            rows.append((name, row['level'], row['detail']))
    return sorted(rows, key=lambda row: (order.index(row[1]), row[0]))
