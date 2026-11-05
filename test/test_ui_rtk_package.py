"""The RTK UI is one package, ui/rtk/ (audit finding 10): base controls, row, icon, pane. The old module paths
are the moved modules for one release, so imports and monkeypatches on them still reach the code."""
from pathlib import Path

import mhseals_testing.ui as ui


def test_old_paths_are_the_moved_modules():
    from mhseals_testing.ui import rtk_icon, rtk_pane, rtk_row
    from mhseals_testing.ui.rtk import icon, pane, row
    assert (rtk_icon, rtk_pane, rtk_row) == (icon, pane, row)
    from mhseals_testing.ui.rtk_row import RtkView
    assert RtkView is row.RtkView


def test_no_rtk_ui_code_outside_the_package():
    """ui/rtk_*.py are shims only (a few lines); new RTK UI code goes in ui/rtk/."""
    for path in Path(ui.__file__).parent.glob('rtk_*.py'):
        assert 'sys.modules[__name__] = _moved' in path.read_text() and len(path.read_text().splitlines()) < 10, path
