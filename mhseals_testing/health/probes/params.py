"""params probe: reads watched parameters from nodes, including the FCU through MAVROS.

READ ONLY. This probe calls ``client.get_parameters`` and nothing else: never a
set, never ``param/pull`` (which would overwrite the FCU's values with a stale
cache). ``client`` is ros/params' thin wrapper; tests pass a fake.
"""

import time

from mhseals_testing.health.model import Fact

PERIOD = {'ros': 60.0, 'fcu': 300.0}      # seconds between reads; the FCU is slow and rarely changes


def fact_name(node, name):
    """The fact a parameter is published under, read by kinds/params.py."""
    return f'param.{node}.{name}'


def collect(client, watch, clock=time.monotonic):
    """``{fact name: Fact}`` for ``watch`` = ``{node: [parameter names]}``.

    Names the node does not report (MAVROS has not loaded them yet) get no fact,
    so the check reads ``unknown``. A node that cannot be reached raises.
    """
    now, facts = clock(), {}
    for node, names in watch.items():
        values = client.get_parameters(node, list(names))
        for name in names:
            if name in values and values[name] is not None:
                facts[fact_name(node, name)] = Fact(values[name], label=node, time=now)
    return facts
