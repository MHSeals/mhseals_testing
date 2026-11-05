"""The App's mixin roster: one class per area (loop, view, actions/), in App's method resolution order."""

from mhseals_testing.ui.app.actions.annotate import Annotate
from mhseals_testing.ui.app.actions.arm import Arm
from mhseals_testing.ui.app.actions.bags import Bags
from mhseals_testing.ui.app.actions.holds import Holds
from mhseals_testing.ui.app.actions.labels import Labels
from mhseals_testing.ui.app.actions.manual import Manual
from mhseals_testing.ui.app.actions.operator import Operator
from mhseals_testing.ui.app.actions.popups import Popups
from mhseals_testing.ui.app.actions.procedures import Procedures
from mhseals_testing.ui.app.actions.quit import Quit
from mhseals_testing.ui.app.actions.settings import Settings
from mhseals_testing.ui.app.loop import Loop
from mhseals_testing.ui.app.view import View

ROSTER = (Loop, View, Popups, Procedures, Manual, Arm, Holds, Settings, Annotate, Labels, Bags, Operator, Quit)
