"""ui.dashboard is a shim over ui.layout and ui.panels.*: same objects, no copies."""

from mhseals_testing.ui import dashboard, layout
from mhseals_testing.ui.panels import activity, motion, procedures, status, thrusters


def test_shim_reexports_the_split_modules():
    for module, names in ((layout, ('render', 'popup', 'footer', 'header', 'fit_hints', 'MIN_SIZE')),
                          (status, ('status_panel', 'status_rows', 'short')),
                          (motion, ('motion_panel', 'drift_line')),
                          (thrusters, ('thrusters_panel', 'thruster_rows', 'effort_bar')),
                          (procedures, ('procedures_panel',)),
                          (activity, ('activity_panel', 'result_label', 'result_detail'))):
        for name in names:
            assert getattr(dashboard, name) is getattr(module, name), name
