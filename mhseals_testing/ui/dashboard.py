"""Compatibility shim: the dashboard now lives in ``ui.layout`` and ``ui.panels.*``.

Everything the app and the tests import from here is re-exported unchanged; new code imports
from the modules directly. Removed with the Motion rewrite (ts052).
"""

from mhseals_hardware.thruster_mixer import POSITIONS  # noqa: F401  (app reads dashboard.POSITIONS)
from mhseals_testing.ui import theme  # noqa: F401  (dashboard.theme)
from mhseals_testing.ui.layout import *  # noqa: F401,F403
from mhseals_testing.ui.layout import (  # noqa: F401
    FOLD, MIN_SIZE, PINNED_HINTS, SIDEBAR, TOP_ROW, WIDE, Overlay, Window, _height,
    fit_hints, fit_strip, footer, header, health_strip, popup, render, too_small)
from mhseals_testing.ui.panels import PANELS, TITLES, window  # noqa: F401
from mhseals_testing.ui.panels.activity import activity_panel, result_detail, result_label  # noqa: F401
from mhseals_testing.ui.panels.motion import (  # noqa: F401
    ARROWS, DRIFT_AGREE, DRIFT_DEADBAND, bearing, drift_arrow, drift_line, drift_now, drift_vector,
    expected_drift, motion_panel, sparkline)
from mhseals_testing.ui.panels.procedures import procedures_panel  # noqa: F401
from mhseals_testing.ui.panels.status import (  # noqa: F401
    MARKS, localization_rows, short, status_panel, status_rows)
from mhseals_testing.ui.panels.thrusters import (  # noqa: F401
    NEUTRAL, READBACK_SHORT, effort_bar, thruster_rows, thrusters_panel)
from mhseals_testing.ui.theme import ACCENT, MUTED, SEPARATOR  # noqa: F401
