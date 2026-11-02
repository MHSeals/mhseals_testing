"""boat_test: the interactive test suite, one page after cx.

Starts passive: it watches sensors, TF and advisories and records the whole
session to one bag. Five panels take focus (Tab): Status, Motion, Thrusters,
Procedures, Activity. Motion needs an explicit arm (a, confirmed in a popup)
and passes the interlocks in readiness.py on every tick; any error or Space
stops at neutral and disarms. Prompts are popups on the same screen.

The facade: App (core.py) coordinates mixins, one per area (loop, view, actions/);
startup.py parses arguments and runs a session; help.py holds the key tables.
"""

from mhseals_testing.ui.app.core import (  # noqa: F401
    App, main, session_main, review_main, parse_args, build_parser, stack_overlay, settle_discovery,
    nav2_recorder, build_nav2_live, restart_as, SessionPaths, HELP, GLOBAL_HINTS, PANEL_HINTS, MANUAL_HINTS,
    CORRECTION_HINTS, help_rows, help_width, UPDOWN, LEFTRIGHT, WATCHDOG_GRACE, HOLD_CONFIRM_S, IDLE_S,
    Detach, decoupled, STOPPING_ERRORS)
