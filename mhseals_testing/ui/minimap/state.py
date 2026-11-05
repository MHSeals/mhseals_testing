"""Moved to ``mhseals_testing.model.map_state``; this path stays for one release (it is that module, so
attributes and monkeypatches are shared)."""
import sys

from mhseals_testing.model import map_state as _moved

sys.modules[__name__] = _moved
