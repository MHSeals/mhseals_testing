"""The authority UI's field knobs: config/agent.yaml ``tune.authority`` and ``lease.motion_s``, read once.

ui/authority_chip and ui/takeover read them here; the code defaults are agent/config.TUNE (and MOTION_S for the
lease), which apply when agent.yaml cannot be found or read. ROS-free.
"""

from functools import cache

from mhseals_testing.agent import config as agent_config

MOTION_S = 10.0    # lease.motion_s's shipped value: the chip's motion lease total when an act carries none


@cache
def tune():
    """``{working_s, stop_wait_s, view_wait_s, motion_s}`` in seconds."""
    try:
        data = agent_config.load()
    except (OSError, ValueError, ImportError, LookupError):    # no source tree or install, or a bad file
        return {**agent_config.TUNE['authority'], 'motion_s': MOTION_S}
    return {**data['tune']['authority'], 'motion_s': float(data['lease']['motion_s'])}


def get(key):
    return tune()[key]
