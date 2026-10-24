"""Malformed recorded event payloads are contained at the ReviewSource boundary."""
from functools import partial

from mhseals_testing.ui.source import ReviewSource
import pytest
from test_bags_open_review import FakeReader, S, selected_app


class Msg:
    def __init__(self, data):
        self.data = data


PAYLOADS = {
    'list': lambda: Msg('[1]'), 'text': lambda: Msg('"x"'),
    'int-phase': lambda: Msg('{"phase": 5}'), 'nodata-list': lambda: ['x'],
    'missing-phase': lambda: Msg('{}'), 'badjson': lambda: Msg('{')}


def install(monkeypatch, bad_at, payload):
    def opener(segment):
        stamps = (10, 15) if segment.name == '00' else (25, 30, 40)
        topic = '/mhseals_testing/events'
        rows = [(topic, payload() if t == bad_at else {'phase': 'marker'}, t * S)
                for t in stamps]
        return FakeReader(rows), {'/mhseals_testing/events': 'inert'}
    monkeypatch.setattr('mhseals_testing.ui.app.ReviewSource', partial(
        ReviewSource, opener=opener, decode=lambda kind, message: message))


def owners(app):
    return (app.source, app.monitor, app.ros, app.processes, app.recorder, app.paths)


@pytest.mark.parametrize('name', sorted(PAYLOADS))
def test_tick_payload(tmp_path, monkeypatch, name):
    app, boat, _ = selected_app(tmp_path, monkeypatch)
    install(monkeypatch, 30, PAYLOADS[name])
    live = owners(app)
    app.dispatch('enter')
    assert app.review is not None
    app.review.playhead, app.review.due = 35.0, None
    try:
        app.tick()
    except Exception as error:
        pytest.fail(f'tick escaped {type(error).__name__}: {error}')
    assert owners(app) == live and not app.closed and boat.sent == []
    assert app.notice


@pytest.mark.parametrize('name', sorted(PAYLOADS))
def test_entry_payload(tmp_path, monkeypatch, name):
    app, boat, _ = selected_app(tmp_path, monkeypatch)
    install(monkeypatch, 10, PAYLOADS[name])
    live = owners(app)
    try:
        app.dispatch('enter')
    except Exception as error:
        import mhseals_testing.ui.app as m
        caught = isinstance(error, m.STOPPING_ERRORS)
        pytest.fail(f'enter escaped {type(error).__name__} (run catches={caught}): {error}')
    assert app.review is None and app.notice
    assert owners(app) == live and not app.closed and boat.sent == []
