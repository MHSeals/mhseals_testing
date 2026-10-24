"""Compass calibration from a handheld bearing (gps spec 4.3). Only write: 42006, after confirmation."""

from mhseals_testing.analysis.magcal import magcal_errors, true_heading


CONFIRM_WORD = 'FCU'


def calibrate(settings, confirm, send):
    """Send the fixed-yaw calibration for settings['heading'] (handheld bearing, deg).

    settings: heading, declination (default 0, warned when absent), mask (default 1).
    confirm is the typed popup, ``type_word(title, message, word) -> bool`` (ui.popups.type_word
    bound to an app); only True proceeds, so a yes/no callable or None declines. send(yaw, mask)
    -> bool is ros.fcu.FcuParams.fixed_mag_cal (confirmed) over a MavrosLink. Returns
    {'status': 'sent'|'declined'|'failed'|'invalid', 'yaw': ..., 'reason': ...}. Nothing is sent
    without the typed word, and no MAG_CAL_REPORT is awaited.
    """
    try:
        bearing = float(settings['heading'])
    except (KeyError, TypeError, ValueError):
        return dict(status='invalid', reason='heading is required (handheld bearing in degrees)')
    yaw = true_heading(bearing, float(settings.get('declination', 0.0)))
    text = f'Write compass calibration for true heading {yaw:.1f} deg (42006)?'
    if 'declination' not in settings:
        text += ' Warning: no declination given, assuming 0.'
    try:
        confirmed = (confirm is not None
                     and confirm('Compass calibration', text, CONFIRM_WORD) is True)
    except TypeError:
        confirmed = False
    if not confirmed:
        return {'status': 'declined', 'yaw': yaw,
                'reason': f'typed {CONFIRM_WORD} confirmation not given'}
    if send(yaw, int(settings.get('mask', 1))):
        return dict(status='sent', yaw=yaw)
    return dict(status='failed', yaw=yaw, reason='FCU did not accept the write (no ACK, armed or disconnected)')


def run(ctx):
    """Registry entry. Headless: ctx.services['magcal'] = settings dict (heading, ...).

    ``ctx.services['fcu_send'](yaw, mask)`` is FcuParams.fixed_mag_cal over a spun MavrosLink and
    ``ctx.services['type_word']`` the typed confirmation. Without the settings, the FCU link or the
    typed callback (a headless yes/no choice is not enough) it records 'unavailable'.
    """
    settings, send = ctx.services.get('magcal'), ctx.services.get('fcu_send')
    typed = ctx.services.get('type_word')
    if not settings or send is None or typed is None:
        result = {'status': 'unavailable',
                  'reason': ('needs the Magcal popup, an FCU link and a typed '
                             f'{CONFIRM_WORD} confirmation')}
    else:
        result = calibrate(settings, typed, send)
    ctx.record(dict(procedure='compass calibration', verdict=result['status'].upper(), **result))
    return result


def verify(ekf_yaws, bearings, declination=0.0):
    """Per-heading errors and pass/fail for the verify headings."""
    errors, passed = magcal_errors(ekf_yaws, bearings, declination)
    return dict(errors=errors, passed=passed)
