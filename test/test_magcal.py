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


def test_calibrate_writes_only_after_confirmation():
    calls = []
    send = lambda yaw, mask: calls.append((yaw, mask)) or True
    assert calibrate({'heading': 90, 'declination': 5}, lambda t: False, send)['status'] == 'declined'
    assert calls == []
    assert calibrate({'heading': 90, 'declination': 5}, lambda t: True, send) == dict(status='sent', yaw=95.0)
    assert calls == [(95.0, 1)]
    assert calibrate({'heading': 90}, lambda t: True, lambda y, m: False)['status'] == 'failed'
    assert calibrate({}, lambda t: True, send)['status'] == 'invalid'


def test_headless_set_heading_through_the_registry():
    from mhseals_testing.procedures import PROCEDURES
    entry = next(p for p in PROCEDURES if p.key == 'g')
    sent, recorded, asked = [], [], []
    ctx = SimpleNamespace(
        services=dict(magcal={'heading': 90, 'declination': 5},
                      fcu_send=lambda yaw, mask: sent.append((yaw, mask)) or True),
        choose=lambda *a, **k: asked.append(k) or 'yes', record=recorded.append)
    assert entry.run(ctx)['status'] == 'sent' and sent == [(95.0, 1)]
    assert recorded[0]['verdict'] == 'SENT' and asked[0]['danger'] is True
    ctx.choose = lambda *a, **k: 'no'
    assert entry.run(ctx)['status'] == 'declined' and len(sent) == 1


def test_registry_entry_without_fcu_or_settings_does_not_raise():
    from mhseals_testing.procedures import PROCEDURES
    entry = next(p for p in PROCEDURES if p.key == 'g')
    recorded = []
    ctx = SimpleNamespace(services={}, choose=lambda *a, **k: 1 / 0, record=recorded.append)
    result = entry.run(ctx)
    assert result['status'] == 'unavailable' and 'Magcal popup' in result['reason']
    assert recorded[0]['verdict'] == 'UNAVAILABLE'
