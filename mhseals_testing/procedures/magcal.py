"""Compass calibration from a handheld bearing (gps spec 4.3). Only write: 42006, after confirmation."""

from mhseals_testing.analysis.magcal import magcal_errors, true_heading


def calibrate(settings, confirm, send):
    """Send the fixed-yaw calibration for settings['heading'] (handheld bearing, deg).

    settings: heading, declination (default 0), mask (default 1). confirm(text) -> bool
    is the operator's yes; send(yaw, mask) -> bool is ros.fcu.FcuParams.fixed_mag_cal
    (confirmed) over a MavrosLink. Returns {'status': 'sent'|'declined'|'failed'|'invalid', 'yaw': ...}. Nothing
    is sent without confirmation, and no MAG_CAL_REPORT is awaited.
    """
    try:
        bearing = float(settings['heading'])
    except (KeyError, TypeError, ValueError):
        return dict(status='invalid', reason='heading is required (handheld bearing in degrees)')
    yaw = true_heading(bearing, float(settings.get('declination', 0.0)))
    if not confirm(f'Write compass calibration for true heading {yaw:.1f} deg (42006)?'):
        return dict(status='declined', yaw=yaw)
    ok = send(yaw, int(settings.get('mask', 1)))
    return dict(status='sent' if ok else 'failed', yaw=yaw)


def run(ctx):
    """Registry entry. Headless: ctx.services['magcal'] = settings dict (heading, ...).

    ``ctx.services['fcu_send'](yaw, mask)`` is FcuParams.fixed_mag_cal over a spun MavrosLink.
    The operator's yes is a danger ctx.choose popup. Without the settings or the FCU
    link (the Magcal popup and link wiring, ts014/ts034/ts045) it records 'unavailable'.
    """
    settings, send = ctx.services.get('magcal'), ctx.services.get('fcu_send')
    if not settings or send is None:
        result = dict(status='unavailable',
                      reason='needs the Magcal popup / FCU link (ts014/ts034/ts045)')
    else:
        result = calibrate(
            settings,
            lambda text: ctx.choose('Compass calibration', ('yes', 'no'), {'y': 0, 'n': 1},
                                    message=text, danger=True) == 'yes',
            send)
    ctx.record(dict(procedure='compass calibration', verdict=result['status'].upper(), **result))
    return result


def verify(ekf_yaws, bearings, declination=0.0):
    """Per-heading errors and pass/fail for the verify headings."""
    errors, passed = magcal_errors(ekf_yaws, bearings, declination)
    return dict(errors=errors, passed=passed)
