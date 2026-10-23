"""The lasting Nav2 limits overlay, ``~/.config/mhseals/nav2.yaml``.

Measured limits used to live only in a per-session ``<bag>.nav2.yaml`` that was applied by
hand and lost on restart. The overlay now lives in one user file that the Navigation
component passes to navigation.launch.py as ``limits_file:=`` (which merges it with the
package defaults via ``merge_limits``). Whoever changes limits writes the file first
(``save``) and only then sets live parameters, so a restart comes back with the same values.
See config_history.md section 2.3. No ROS here.
"""

import os
from pathlib import Path

import yaml

from mhseals_testing import data


def path():
    """``$MHSEALS_NAV2`` or ``~/.config/mhseals/nav2.yaml`` (next to hardware.yaml)."""
    override = os.environ.get('MHSEALS_NAV2')
    if override:
        return Path(override).expanduser()
    return Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'mhseals/nav2.yaml'


def load(target=None):
    """The overlay mapping; a missing, empty or non-mapping file is no overlay."""
    try:
        loaded = yaml.safe_load(Path(target or path()).read_text())
    except FileNotFoundError:
        return {}
    return loaded if isinstance(loaded, dict) else {}


def save(overlay, target=None):
    """Atomically replace the overlay with ``overlay`` (a mapping); returns the path written."""
    if not isinstance(overlay, dict):
        raise TypeError('the Nav2 overlay must be a mapping')
    target = Path(target or path())
    data.atomic_write(target, yaml.safe_dump(overlay, sort_keys=False))
    return target


def launch_argument(target=None):
    """The ``limits_file:=`` launch argument for the Navigation component."""
    return f'limits_file:={target or path()}'
