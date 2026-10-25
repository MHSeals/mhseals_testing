"""The compass calibration heading: the handheld bearing the bow points at, checked per key.

``procedures.magcal`` takes ``heading`` (a handheld magnetic bearing, degrees) and ``declination``
(east positive) and writes ``true_heading(heading, declination)``. This popup only collects the
bearing; the FCU write keeps its own typed confirmation in the procedure.
"""

from mhseals_hardware.menu import Cancelled
from mhseals_testing.analysis.magcal import true_heading
from mhseals_testing.analysis.magnetic import SITE
from mhseals_testing.ui import theme


def parse_heading(text):
    """The bearing in [0, 360] for ``text``; ValueError says what is wrong."""
    try:
        value = float(text)
    except ValueError:
        raise ValueError('a number of degrees') from None
    if not 0.0 <= value <= 360.0:
        raise ValueError('0 to 360 only')
    return value


def note(text, declination):
    """(state, line) under the field: the true heading when valid, else what is wrong."""
    if not text.strip():
        return 'idle', 'handheld bearing in degrees, 0 to 360'
    try:
        bearing = parse_heading(text)
    except ValueError as error:
        return 'bad', str(error)
    return 'ok', (f'{theme.pick("→", "->")} {true_heading(bearing, declination):.1f}° true '
                  f'(decl {declination:+.1f}°)')


def ask_heading(app, declination=SITE['declination_deg']):
    """Settings for ``procedures.magcal``: heading and declination; Esc raises Cancelled.

    Enter on an empty or invalid value is ignored and the popup stays open.
    """
    modal = dict(kind='input', title='Compass calibration', text='', danger=False, rows=None,
                 width=64, message='Handheld bearing the bow points at', suffix='°',
                 hint=f'Enter use{theme.SEPARATOR}Esc cancel')

    def update():
        modal['state'], modal['note'] = note(modal['text'], declination)

    def handle(key):
        from mhseals_testing.ui.popups import edit
        if key == 'enter':
            return (dict(heading=parse_heading(modal['text']), declination=declination)
                    if modal['state'] == 'ok' else None)
        if key == 'escape':
            raise Cancelled('cancelled by operator')
        modal['text'] = edit(modal['text'], key)[:8]
        update()
    update()
    return app.popup(modal, handle)
