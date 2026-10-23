"""Where the harness keeps its outputs: one root, never relative to the current directory."""

import os
from pathlib import Path

HOME_WORKSPACE = Path.home() / 'astro_dock'


def workspace():
    """The colcon workspace: the parent of ``src`` (source tree) or ``install`` (installed)."""
    for parent in Path(__file__).resolve().parents:
        if parent.name in ('src', 'install'):
            return parent.parent
    return HOME_WORKSPACE


def data_root():
    """``$MHSEALS_DATA``, else ``<workspace>/bags`` (gitignored, shared with tranquility)."""
    override = os.environ.get('MHSEALS_DATA')
    return Path(override).expanduser() if override else workspace() / 'bags'
