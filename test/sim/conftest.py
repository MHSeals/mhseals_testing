"""Sim tests (marker ``sim``): skipped unless $SIMRIG names the rig.

``SIMRIG_SESSION=<dir>`` reuses a recorded hold_still session; otherwise the rig records one
(needs the player, a display and the roboboat_dev container, see simrig.md).
"""
import os
from pathlib import Path
import uuid

import pytest

from simrig import rig_dir, run_hold_still


def pytest_collection_modifyitems(items):
    for item in items:
        if 'sim' in item.keywords or item.path.parent.name == 'sim':
            item.add_marker(pytest.mark.sim)


@pytest.fixture(scope='session')
def rig():
    path = rig_dir()
    if path is None:
        pytest.skip('sim rig not found: set SIMRIG to the rig directory')
    return path


@pytest.fixture(scope='session')
def session(rig, tmp_path_factory):
    """A hold_still session directory (bag, course_truth.json, session.yaml)."""
    given = os.environ.get('SIMRIG_SESSION')
    if given:
        return Path(given)
    domain = 230 + uuid.uuid4().int % 20
    return run_hold_still(rig, tmp_path_factory.mktemp('hold_still') / 's', domain=domain)
