import json
import os
from pathlib import Path

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
            "from mhseals_testing.ui.panels import scrubber\n"
            "r = review(); r.go(140, 0)\n"
            "for w in (120, 48):\n"
            "    for t in scrubber.review_footer(r, w, 'note'): sys.stdout.write(t.plain)\n")
    env = {k: v for k, v in os.environ.items() if k not in ('MHS_ASCII', 'TERM')}
    out = subprocess.run([sys.executable, '-c', code, os.path.dirname(__file__)],
                         env={**env, 'MHS_ASCII': '1'}, capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert out.stdout.isascii() and out.stdout


GOLDEN = Path(__file__).parent / 'golden' / 'timeline'


@pytest.mark.parametrize('width', [120, 80])
def test_review_footer_golden(width, tmp_path, monkeypatch):
    monkeypatch.setattr('mhseals_testing.ui.theme.ASCII', False)
    app, _ = make_app_review(tmp_path)
    app.review.go(140, 0)
    lines = _text(app, width, 40).splitlines()
    text = '\n'.join(line.rstrip() for line in lines[-6:]) + '\n'
    path = GOLDEN / f'review_footer_{width}.txt'
    if os.environ.get('UPDATE_GOLDEN'):
        path.write_text(text)
    assert path.read_text() == text


def test_z_cycles_the_activity_loupe_and_changes_its_label(tmp_path):
    app, _ = make_app_review(tmp_path)
    app.focus = 'activity'
    seen = []
    for _ in range(4):
        seen.append(next(line for line in _text(app, 120, 40).splitlines() if 'loupe' in line and 'review' in line))
        app.dispatch('z')
    assert ['12m' in seen[0], '30m' in seen[1], '2m' in seen[2], seen[3] == seen[0]] == [True] * 4


def test_ui_scrubber_module_is_gone():
    root = Path(__file__).parent.parent / 'mhseals_testing'
    assert not (root / 'ui' / 'scrubber.py').exists()
    for path in list(root.rglob('*.py')) + list(Path(__file__).parent.glob('*.py')):
        text = path.read_text()
        assert 'from mhseals_testing.ui import scrubber' not in text.replace('ui.panels', '') or path.name == Path(__file__).name
        assert 'ui.scrubber' not in text or path.name == Path(__file__).name


def test_loupe_hint_shows_only_with_activity_focused(tmp_path):
    app, _ = make_app_review(tmp_path)
    app.focus = 'status'
    assert ('z', 'Loupe') not in app.view()['hints']
    app.focus = 'activity'
    assert ('z', 'Loupe') in app.view()['hints']


def test_find_session_skips_data_folders_and_resolves_ids(tmp_path):
    import os
    line = json.dumps(events()[0]) + '\n'
    for rel, age in (('sessions/20261023_old', 100), ('sessions/20261023_new', 200), ('boat_legacy', 50)):
        folder = tmp_path / rel
        folder.mkdir(parents=True)
        (folder / 'events.jsonl').write_text(line)
        os.utime(folder / 'events.jsonl', (age, age))
    for other in ('config-history', 'calibration', 'logs'):
        (tmp_path / other).mkdir()                      # newest by mtime, but not sessions
    assert find_session('', tmp_path) == tmp_path / 'sessions' / '20261023_new'
    assert find_session('20261023_old', tmp_path) == tmp_path / 'sessions' / '20261023_old'
    assert find_session('boat_legacy', tmp_path) == tmp_path / 'boat_legacy'
    with pytest.raises(ValueError):
        find_session('sessions', tmp_path)
