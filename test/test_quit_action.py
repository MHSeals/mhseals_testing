"""Quitting (ui/app/actions/quit.py) armed or holding: neutral, then disarm, and never a release (atlas #158)."""

from test_app import make_app, release_app


def quit_and_close(app):
    app.hold_confirm = lambda *args, **kwargs: True     # armed: the held q (config/input.yaml quit_armed)
    app.choose = lambda title, options, *args, **kwargs: 'Quit'
    with app.attached():
        app.dispatch('q')
    assert app.quit
    app.close()


def test_quit_while_armed_sends_neutral_then_disarms(tmp_path):
    app, boat = make_app(tmp_path)
    app.arming.armed = True
    quit_and_close(app)
    assert boat.sent and set(boat.sent) == {'neutral'}
    assert not app.armed and app.arming.calls[-1] == 'disarm'
    assert app.notice == 'Recording off'                 # the last close step: the bag flushed


def test_quit_while_holding_keeps_the_hold(tmp_path):
    app, boat, calls = release_app(tmp_path)
    app.arming.armed = True
    boat.hold('keys')
    quit_and_close(app)
    assert calls == [] and boat.holds == ['keys']       # the hold outlives the session: only U releases it
    assert set(boat.sent) == {'neutral'} and not app.armed
    assert app.notice == 'Recording off'                 # the last close step: the bag flushed
