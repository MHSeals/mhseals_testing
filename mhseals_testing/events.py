"""The /mhseals_testing/events line format, free of ROS so folds and tests can share it.

Several suites can publish on one domain, so every line names its instance
(pid and node). A fold keeps its own lines and any line tagged ``guest``.
"""

from datetime import datetime
import json
import os

NODE = 'mhseals_testing'


def payload(phase, ros_time, details, pid=None, node=NODE):
    """The event dict; ``details`` may not override the instance identity."""
    return dict(details, time=datetime.now().astimezone().isoformat(), ros_time=ros_time,
                phase=phase, pid=os.getpid() if pid is None else pid, node=node)


def line(event):
    return json.dumps(event, sort_keys=True, default=str)


def ours(events, pid):
    """Events from instance ``pid``, plus any explicitly tagged guest lines."""
    return [e for e in events if e.get('pid') == pid or e.get('guest')]
