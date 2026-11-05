"""Moved to ``mhseals_testing.ui.rtk.pane``; this path stays for one release (it is that module, so attributes
and monkeypatches are shared)."""
import sys

from mhseals_testing.ui.rtk import pane as _moved

sys.modules[__name__] = _moved
