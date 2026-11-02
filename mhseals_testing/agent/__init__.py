"""Field agents: the verbs, their channel to the TUI, and what they may change.

    cli       boat_trial's agent verbs (status, run, stop, arm, ...) and the boat_agent entry
    channel   the file channel between an agent and the TUI desk (requests, replies, notes)
    config    config/agent.yaml: what agents may tune, arm in sim, and the watch over it
    grants    standing operator grants and their verdicts
    fcu       FCU parameter writes an agent may make, with the change store
    relay     the dev host's hop to a boat's checkout (unreachable answers from the cache)
    ros_stop  stop's backup path: a mux hold over ROS when no TUI answers
"""

from importlib import import_module

# name -> submodule; resolved on first use, so importing the facade loads nothing (some submodules need rclpy)
_EXPORTS = {
    'Agent': 'cli',
    'needs_why': 'cli',
    'who_of': 'cli',
    'request': 'channel',
    'reply': 'channel',
    'redact': 'channel',
    'Tail': 'channel',
    'Watch': 'config',
    'settable': 'config',
    'AgentFcu': 'fcu',
    'Store': 'fcu',
    'granted': 'grants',
    'verdict': 'grants',
    'unreachable': 'relay',
    'hold': 'ros_stop',
}
__all__ = sorted(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    value = getattr(import_module(f'{__name__}.{_EXPORTS[name]}'), name)
    globals()[name] = value
    return value
