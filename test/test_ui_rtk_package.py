"""The RTK UI is one package, ui/rtk/ (audit finding 10): base controls, row, icon, pane. Moved modules keep
no shim at their old path: importers use the real module (atlas #37)."""
from pathlib import Path

import mhseals_testing
import mhseals_testing.ui as ui


def test_no_rtk_ui_code_outside_the_package():
    assert not list(Path(ui.__file__).parent.glob('rtk_*.py'))


def test_no_module_aliases_itself_to_another():
    root = Path(mhseals_testing.__file__).parent
    shims = [p for p in root.rglob('*.py') if 'sys.modules[__name__]' in p.read_text()]
    assert shims == []
