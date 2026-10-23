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


def test_space_in_review_stops_outputs_and_pauses(tmp_path):
    app, _ = make_app_review(tmp_path)
    app.review.playing = True
    app.arming.armed = True
    app.dispatch('space')
    assert not app.review.playing and 'disarm' in app.arming.calls


def test_armed_fault_still_stops_in_review_over_a_live_session(tmp_path):
    app, _ = make_app_review(tmp_path)
    app.arming.armed = True
    app.arming.fault = lambda: 'driver died'
    app.tick()
    assert 'disarm' in app.arming.calls
    app.arming.armed = True
    app.arming.calls.clear()
    app.args.review = ''                       # --review has no live outputs to guard
    app.tick()
    assert app.arming.calls == []


def test_panel_edit_keys_do_not_reach_the_thrusters_in_review(tmp_path):
    app, _ = make_app_review(tmp_path)
    app.focus = 'thrusters'
    before = (app.draft, app.grabbed)
    for key in ('r', '4', 'enter', 'u', 'down', 'enter', 'down'):
        app.dispatch(key)
    assert (app.draft, app.grabbed) == before and app.modal is None
    app.focus = 'activity'
    app.dispatch('v')
    assert app.activity == 'log'


def test_q_confirms_in_review_unless_it_is_the_review_flag(tmp_path):
    app, _ = make_app_review(tmp_path)
    called = []
    app.confirm_quit = lambda: called.append(1)
    app.dispatch('q')
    assert called and not app.quit
    app.args.review = ''
    app.dispatch('q')
    assert app.quit


def test_scrubber_is_pure_ascii_in_ascii_mode():
    import os, subprocess, sys
    code = ("import sys; sys.path.insert(0, sys.argv[1])\n"
            "from test_review import review\n"
            "from mhseals_testing.ui import scrubber\n"
            "r = review(); r.go(140, 0)\n"
            "for w in (120, 48):\n"
            "    for t in scrubber.rows(r, w, 'note'): sys.stdout.write(t.plain)\n")
    env = {k: v for k, v in os.environ.items() if k not in ('MHS_ASCII', 'TERM')}
    out = subprocess.run([sys.executable, '-c', code, os.path.dirname(__file__)],
                         env={**env, 'MHS_ASCII': '1'}, capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert out.stdout.isascii() and out.stdout
