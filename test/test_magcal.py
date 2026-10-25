"""Magcal maths, the 42006 ACK rule (through FcuParams) and the confirm-before-write procedure; no ROS needed."""

import math
from types import SimpleNamespace

from mhseals_testing.analysis.magcal import four_heading_fit, magcal_errors, true_heading
from mhseals_testing.procedures.magcal import calibrate, verify
from mhseals_testing.ros.fcu import FIXED_MAG_CAL_YAW, FcuParams


def synthetic(headings, offset, north=470.0, east=0.0):
    out = []
    for psi in headings:
        c, s = math.cos(math.radians(psi)), math.sin(math.radians(psi))
        out.append((north * c + east * s + offset[0], -north * s + east * c + offset[1]))
    return out


def test_fit_recovers_offset_correction():
    headings = [10, 100, 190, 280]
    dx, dy = four_heading_fit(headings, synthetic(headings, (40, -25), east=60))
    assert math.isclose(dx, -40, abs_tol=1e-6) and math.isclose(dy, 25, abs_tol=1e-6)


def test_fit_refuses_clustered_headings():
    assert four_heading_fit([10, 15, 20], synthetic([10, 15, 20], (5, 5))) is None


def test_errors_wrap_and_declination():
    assert true_heading(355, 10) == 5
    errors, passed = magcal_errors([5, 99, 190, 281], [355, 90, 180, 270], declination=10)
    assert passed and all(abs(e) <= 1 for e in errors)
    assert not magcal_errors([5, 99, 190, 290], [355, 90, 180, 270], declination=10)[1]
    assert verify([0], [359])['errors'][0] == 1 and verify([0], [359])['passed']


class Link:
    """FcuParams' link with only what 42006 needs: armed state and command acks."""

    def __init__(self, ack, armed=False):
        self.ack, self.is_armed, self.sent = ack, armed, None

    def armed(self):
        return self.is_armed

    def command(self, command, params):
        self.sent = (command, params)
        return self.ack


def test_ack_rule():
    link = Link(True)
    assert FcuParams(link, meta={}).fixed_mag_cal(123.0, confirmed=True)
    command, params = link.sent
    assert command == FIXED_MAG_CAL_YAW and params[:2] == [123.0, 1.0] and params[4:] == [0, 0, 0]
    assert not FcuParams(Link(False), meta={}).fixed_mag_cal(123.0, confirmed=True)  # not ACCEPTED
    assert not FcuParams(Link(True, armed=True), meta={}).fixed_mag_cal(123.0, confirmed=True)
    assert not FcuParams(Link(True), meta={}).fixed_mag_cal(123.0)  # never unconfirmed


def typed_word(text, captured=None):
    """Drive the real typed popup callback without a terminal or live App."""
    from mhseals_testing.ui.popups import type_word

    def popup(modal, handle):
        if captured is not None:
            captured.append(dict(modal))
        if text is None:
            return handle('escape')
        for key in text:
            handle(key)
        return handle('enter')

    return lambda *args, **kwargs: type_word(SimpleNamespace(popup=popup), *args, **kwargs)


def test_calibrate_writes_only_after_typed_confirmation():
    calls = []
    send = lambda yaw, mask: calls.append((yaw, mask)) or True
    for confirm in (None, lambda text: True, typed_word(None), typed_word(''),
                    typed_word('yes'), typed_word('boat')):
        result = calibrate({'heading': 90, 'declination': 5}, confirm, send)
        assert result['status'] == 'declined' and result['reason']
        assert calls == []
    assert calibrate({'heading': 90, 'declination': 5}, typed_word('FCU'), send) == dict(
        status='sent', yaw=95.0)
    assert calls == [(95.0, 1)]
    result = calibrate({'heading': 90}, typed_word('FCU'), lambda y, m: False)
    assert result['status'] == 'failed' and result['reason']
    assert calibrate({}, typed_word('FCU'), send)['status'] == 'invalid'


def test_headless_yes_cannot_write_through_registry():
    from mhseals_testing.procedures.magcal import run
    sent, recorded = [], []
    ctx = SimpleNamespace(
        services=dict(magcal={'heading': 90, 'declination': 5},
                      fcu_send=lambda yaw, mask: sent.append((yaw, mask)) or True),
        choose=lambda *a, **k: 'yes', record=recorded.append)
    result = run(ctx)
    assert result['status'] == 'unavailable' and 'typed FCU' in result['reason']
    assert sent == [] and recorded[0]['verdict'] == 'UNAVAILABLE'
    ctx.services['type_word'] = typed_word('FCU')
    assert run(ctx)['status'] == 'sent' and sent == [(95.0, 1)]
    assert recorded[-1]['verdict'] == 'SENT'
    ctx.services['type_word'] = typed_word(None)
    assert run(ctx)['status'] == 'declined' and len(sent) == 1
    assert recorded[-1]['reason']


def test_registry_entry_without_fcu_or_settings_does_not_raise():
    from mhseals_testing.procedures import PROCEDURES
    entry = next(p for p in PROCEDURES if p.key == 'g')
    recorded = []
    ctx = SimpleNamespace(services={}, choose=lambda *a, **k: 1 / 0, record=recorded.append)
    result = entry.run(ctx)
    assert result['status'] == 'unavailable' and 'Magcal popup' in result['reason']
    assert recorded[0]['verdict'] == 'UNAVAILABLE'


class Keyed:
    """An app whose popups take keys from a list; records each modal's note after every key."""

    def __init__(self, *keys):
        self.keys, self.notes = list(keys), []

    def popup(self, modal, handle):
        while True:
            result = handle(self.keys.pop(0))
            self.notes.append((modal.get('state'), modal.get('note')))
            if result is not None:
                return result


def test_heading_popup_validates_and_enter_waits_for_a_valid_bearing():
    import pytest
    from mhseals_hardware.menu import Cancelled
    from mhseals_testing.ui.popups.magcal import ask_heading
    app = Keyed('enter', '4', '0', '0', 'enter', 'backspace', 'backspace', 'backspace',
                '1', '2', '7', '.', '5', 'enter')
    assert ask_heading(app, declination=-2.6) == dict(heading=127.5, declination=-2.6)
    assert app.notes[3] == ('bad', '0 to 360 only') and app.notes[4] == app.notes[3]   # Enter ignored
    assert app.notes[-2][0] == 'ok' and '124.9° true' in app.notes[-2][1]
    with pytest.raises(Cancelled):
        ask_heading(Keyed('9', 'escape'))
    with pytest.raises(Cancelled):
        ask_heading(Keyed('x', 'enter', 'escape'))


def test_the_heading_popup_is_asked_only_with_a_link_and_a_typed_confirm():
    from mhseals_testing.procedures.magcal import run
    asked, recorded = [], []
    ctx = SimpleNamespace(services=dict(magcal=lambda: asked.append(1) or {'heading': 90},
                                        type_word=typed_word('FCU')), record=recorded.append)
    assert run(ctx)['status'] == 'unavailable' and asked == []
    ctx.services['fcu_send'] = lambda yaw, mask: True
    assert run(ctx)['status'] == 'sent' and asked == [1]
