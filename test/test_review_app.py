import json

import pytest

from test_app import make_app, procedure
from test_review import events, review
from mhseals_testing.ui.app import parse_args
from mhseals_testing.ui.review import find_session, load_session


def test_review_chip_replaces_arm_state_and_the_page_fits():
    app, _ = make_app_review()
    for width, height in ((120, 40), (80, 30), (48, 20)):
        text = _text(app, width, height)
        assert 'REVIEW' in text.splitlines()[0] and 'Armed' not in text.splitlines()[0]
        assert 'Disarmed' not in text
        assert len(text.splitlines()) <= height


def _text(app, width, height):
    from rich.console import Console
    console = Console(width=width, height=height, record=True, force_terminal=False)
    app.console.size = (width, height)          # the header is laid out for the app's own size
    console.print(app.render())
    return console.export_text()


def make_app_review(tmp_path=None):
    import tempfile
    from pathlib import Path
    tmp = Path(tempfile.mkdtemp()) if tmp_path is None else tmp_path
    app, boat = make_app(tmp)
    app.review = review()
    return app, boat


def test_arming_and_procedures_are_refused_in_review(tmp_path):
    app, boat = make_app_review(tmp_path)
    with app.attached():
        for key in ('a', 'f', 'm'):
            app.dispatch(key)
        assert not app.armed and app.arming.calls == [] and boat.sent == []
        assert 'off' in app.notice
        app.arm_panel()
        app.run_procedure(procedure('f'))
        assert not app.armed and app.arming.calls == [] and boat.sent == []
        assert app.review is not None


def test_review_keys_seek_jump_and_leave(tmp_path):
    app, _ = make_app_review(tmp_path)
    for key in ('right', '}', ',', '.', '<', '>', 'p'):
        app.dispatch(key)
    assert app.review.playing and app.review.playhead == 140
    app.dispatch('escape')
    assert app.review is None and app.notice == 'Left review'


def test_review_flag_opens_a_session_and_quits_on_escape(tmp_path):
    session = tmp_path / 'boat_1'
    session.mkdir()
    (session / 'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events()) + '\n')
    assert find_session('boat_1', tmp_path) == session == find_session('', tmp_path)
    assert load_session(session).length == 100
    app, _ = make_app(tmp_path, ['--review', 'boat_1'])
    assert app.args.review == 'boat_1'
    app.review = load_session(session)
    app.dispatch('escape')
    assert app.quit
    assert parse_args(['--output', str(tmp_path / 's')])[0].review is None
    with pytest.raises(ValueError):
        find_session('nope', tmp_path)
