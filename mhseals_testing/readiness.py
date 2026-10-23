"""Decide whether thrusters may move.

Only sensor and TF health block motion: without trustworthy odometry and
frames, a trial cannot be scored and Nav2 cannot drive safely. Power, links
and compute are advisories: always shown, never blocking, because the
operator (not the suite) owns the e-stop and the boat.
"""


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
