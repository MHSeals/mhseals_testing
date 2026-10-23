"""Session report: one JSON file next to the session bag.

Per-run records (status, exclusion) live in harness/runs.py; this file only
summarises the session and its git provenance.

It records what was tested (profile, hardware config), what happened (events,
procedure results with their time windows) and where the raw data is (the
bag), so any result can be re-analysed from the bag later.
"""

from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path
import tempfile

from mhseals_testing.harness import journal


def build_report(profile, snapshot, bag=None, hardware_config=None, now=None, journal_events=None,
                 provenance=None, conditions=None):
    """``journal_events`` (the session journal) replaces the monitor's capped, monotonic event list."""
    if journal_events:
        logged = journal_events
    else:
        logged = [dict(time=time, message=message) for time, message in snapshot['events']]
    return dict(
        time=(now or datetime.now().astimezone()).isoformat(),
        profile=asdict(profile),
        hardware_config=hardware_config,
        provenance=provenance,  # per-repo git hash, dirty flag and diff sha from the git probe
        conditions=conditions,
        bag=str(bag) if bag else None,
        results=snapshot['results'],
        events=logged,
        markers=journal.markers(journal_events or []),
        telemetry={key: snapshot[key] for key in ('sensors', 'tf', 'advisories')})


def write_report(report, path):
    """Write atomically, so a crash never leaves half a report."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.report-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as output:
            # allow_nan=False: a NaN that slipped past validation fails loudly here.
            json.dump(report, output, indent=2, allow_nan=False)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o644)  # mkstemp creates 0600; reports are shared
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return path
