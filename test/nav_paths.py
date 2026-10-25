"""Where the mhseals_nav tree is, for the tests that compare against it.

A plain module (not conftest) so tests can import it: test/sim has its own conftest.
"""

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def find_nav(env=None, parent=None):
    """Locate mhseals_nav: $MHSEALS_NAV, a sibling checkout (plain or integ-), then the ament share dir."""
    env = os.environ if env is None else env
    parent = parent or ROOT.parent
    candidates = [Path(env['MHSEALS_NAV'])] if env.get('MHSEALS_NAV') else []
    candidates += [parent / 'mhseals_nav', parent / 'integ-mhseals_nav']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('mhseals_nav')))
    except Exception:
        pass
    return next((c.resolve() for c in candidates if c.is_dir()), None)   # absolute: tests chdir
