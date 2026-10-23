"""The /mhseals_testing/events line format, free of ROS so folds and tests can share it.

Several suites can publish on one domain, so every line names its instance
(host, pid and node). A fold keeps its own lines and any line tagged ``guest``;
a bag whose lines carry no pid at all predates tagging and is one instance.
"""

from datetime import datetime
import json
import os
import socket

NODE = 'mhseals_testing'


def payload(phase, ros_time, details, pid=None, node=NODE):
    """The event dict; ``details`` may not override the instance identity."""
    return dict(details, time=datetime.now().astimezone().isoformat(), ros_time=ros_time,
                phase=phase, pid=os.getpid() if pid is None else pid, node=node,
                host=socket.gethostname())


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
    """One copy of each event, in ros_time order.

    Transient-local replays the retained events into every new segment's recorder and into
    other instances' recorders, stamped at receipt. Readers therefore merge segments through
    this and place events by the payload ``ros_time``, never by the bag message stamp.
    """
    seen = {}
    for e in events:
        seen.setdefault((e.get('host'), e.get('pid'), e.get('ros_time'), e.get('phase')), e)
    return sorted(seen.values(), key=lambda e: e.get('ros_time') or 0.0)
