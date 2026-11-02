"""Water conditions: drift, waves and stillness, from samples to the panel's evidence.

    estimator  the ROS-free estimator at neutral (world drift, wave floor, stationarity) and its settings
    segments   admitted sample segments rated by the estimator
    evidence   admission evidence from independent observers
    feed       the source seam the TUI polls each tick
"""

from importlib import import_module

# name -> submodule; resolved on first use, so importing the facade loads nothing (some submodules need rclpy)
_EXPORTS = {
    'load_settings': 'estimator',
    'Conditions': 'segments',
    'MAX_HZ': 'segments',
    'STILL_FRESH_S': 'segments',
    'stillness': 'segments',
    'EvidenceGate': 'evidence',
    'EvidenceState': 'evidence',
    'CommandObservation': 'evidence',
    'ConditionsFeed': 'feed',
}
__all__ = sorted(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    value = getattr(import_module(f'{__name__}.{_EXPORTS[name]}'), name)
    globals()[name] = value
    return value
