"""U's release (ui/app/actions/holds.py): one path with boat_agent release, armed or disarmed (atlas #98 #158)."""

from mhseals_hardware import hold_store
from mhseals_testing.runner import STALE_MUX_S
from mhseals_testing.ui.feeds import authority as authority_feed

from test_app import press, release_app


def confirm_release(app):
    with app.attached():
        press(app, 'up', 'enter')       # Cancel is the default: up to Release
        app.dispatch('U')


def test_hold_then_release_while_armed_sends_one_mux_release(tmp_path):
    app, boat, calls = release_app(tmp_path)
    app.arming.armed = True
    boat.hold('keys')                   # the mux holds; the live mux state shows it
    app.arbiter.on_mux({'input': 'hold', 'holds': [{'cause': 'keys', 'by': 'tui'}], 'rc': {'mode': 'GUIDED'}})
    confirm_release(app)
    assert calls == ['release'] and boat.holds == ['keys'] and boat.sent == []
    assert app.notice == 'Release requested' and app.armed
    app.clock.now += STALE_MUX_S + 1    # armed, a stale mux refuses: no saved file stands in for it
    app.dispatch('U')
    assert app.notice == 'Release refused: mux state is stale or unknown' and calls == ['release']


def test_release_while_disarmed_clears_the_stopped_muxs_saved_holds(tmp_path):
    """Postmortem H1: the mux runs only while armed, so U disarmed releases its saved holds for the next arm."""
    app, boat, calls = release_app(tmp_path)
    path = authority_feed.hold_file(app)
    hold_store.save(path, {'keys': {'cause': 'keys', 'by': 'tui', 'since': 5.0, 'token': None}})
    app.clock.now += STALE_MUX_S + 1    # disarmed: the mux stopped, its state went stale
    confirm_release(app)
    assert hold_store.load(path) == {} and calls == [] and boat.sent == []
    assert app.notice == 'Release requested' and 'hold_release' in app.ros.events
    app.dispatch('U')                   # nothing left to release
    assert app.notice == 'Release refused: the stopped mux saved no holds'
    hold_store.save(path, {'keys': {'cause': 'keys', 'by': 'tui', 'since': 6.0, 'token': None}})
    app.arming.remote = object()        # a remote boat's mux saves there: a local file is not its record
    app.dispatch('U')
    assert app.notice == 'Release refused: mux state is stale or unknown' and len(hold_store.load(path)) == 1
