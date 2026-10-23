"""The /mhseals_testing/events line format, free of ROS so folds and tests can share it.

Several suites can publish on one domain, so every line names its instance
(host, pid and node). A fold keeps its own lines and any line tagged ``guest``;
a bag whose lines carry no pid at all predates tagging and is one instance.
"""

from datetime import datetime
import itertools
import json
import os
import socket
import time

NODE = 'mhseals_testing'

# Per-process emission counter: ros_time is 0.0 until the first sim /clock and repeats within a tick
_seq = itertools.count()


def payload(phase, ros_time, details, pid=None, node=NODE):
    """The event dict; ``details`` may not override the instance identity.

    ``t_ns`` (wall-clock ns at emit, the one time axis of the bag and the journal) and ``rec``
    (whether the recorder was on) default here; a caller that stamped a keypress passes its own ``t_ns``.
    """
    return dict({'t_ns': time.time_ns(), 'rec': True, **details},
                time=datetime.now().astimezone().isoformat(), ros_time=ros_time,
                phase=phase, pid=os.getpid() if pid is None else pid, node=node,
                host=socket.gethostname(), seq=next(_seq))


def line(event):
    return json.dumps(event, sort_keys=True, default=str)


def ours(events, pid, host=None):
    """Events from instance ``pid`` (on ``host`` when given), plus tagged guest lines.

    Pids are per host and recycle, so a line that names a host must match it.
    Lines without a pid are pre-tagging legacy lines: they are kept while the session
    holds at most one tagged instance, since they can only be that one's.
    """
    instances = {(e.get('host'), e['pid']) for e in events if 'pid' in e}
    legacy = len(instances) <= 1
    return [e for e in events
            if ('pid' not in e and legacy)
            or (e.get('pid') == pid and (host is None or e.get('host', host) == host))
            or e.get('guest')]


def unique(events):
    """One copy of each event, in wall-time order.

    Transient-local replays the retained events into every new segment's recorder and into
    other instances' recorders, stamped at receipt. Readers therefore merge segments through
    this and never place events by the bag message stamp. ``ros_time`` is not an identity or
    an order: in sim it is 0.0 before the first /clock, repeats within a tick and rewinds on
    reset, so it is for display only. Lines without ``seq`` (older bags) are keyed whole.
    """
    seen = {}
    for e in events:
        key = (e.get('host'), e.get('pid'), e['seq']) if 'seq' in e else line(e)
        seen.setdefault(key, e)
    return sorted(seen.values(), key=lambda e: (e.get('time') or '', e.get('seq') or 0))
