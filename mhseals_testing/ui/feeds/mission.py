"""Moved to ``mhseals_testing.model.mission_status``; this path stays for one release (it is that module, so
attributes and monkeypatches are shared)."""
import sys

from mhseals_testing.model import mission_status as _moved

sys.modules[__name__] = _moved
