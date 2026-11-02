"""The view-model seam (ui/app/model.py): a draw consumes one Frame, and every key a live App puts in
its snapshot (the boat's state) or its view (the station's) is declared on Snapshot or ViewState
(the test App's feeds included; a live feed's own keys ride along undeclared)."""

import pytest

from mhseals_testing.ui.app.model import Frame, Snapshot, ViewState
from test_app import make_app


def declared(kind):
    return set(kind.__annotations__)


@pytest.mark.smoke
@pytest.mark.parametrize('tab', ['Drive', 'Correction', 'Map', 'Bags', 'Fusion'])
def test_live_keys_are_declared(tmp_path, tab):
    app, _ = make_app(tmp_path)
    app.tabs.current = tab
    frame = app.frame()
    assert isinstance(frame, Frame)
    assert not set(frame.snapshot) - declared(Snapshot), " ".join(sorted(set(frame.snapshot) - declared(Snapshot)))
    assert not set(frame.view) - declared(ViewState), " ".join(sorted(set(frame.view) - declared(ViewState)))


def test_render_draws_the_frame(tmp_path):
    app, _ = make_app(tmp_path)
    seen = []
    app.frame = lambda: seen.append(1) or Frame(app.snapshot(), app.view(), 120, 40)
    assert app.render() is not None and seen == [1]
