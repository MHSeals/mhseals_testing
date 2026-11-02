"""Import path for the crane judge core (crane_ml ``judge/``) and fastsim.

``judge_pack/roboboat`` is a namespace directory (no ``__init__.py``): the boat install
(find_packages) and the import-all smoke test skip it, because it needs a crane checkout
(CRANE_ROOT, default ~/crane_ml) that the boat does not have.
"""

import os
from pathlib import Path
import sys


def core():
    """Import path for the crane judge core (and fastsim, for the replayer)."""
    try:
        import judge  # noqa: F401
        return True
    except ImportError:
        pass
    for root in (os.environ.get('CRANE_ROOT'), '~/crane_ml'):
        if root and (Path(root).expanduser() / 'judge' / 'record.py').exists():
            sys.path.insert(0, str(Path(root).expanduser()))
            return True
    return False
