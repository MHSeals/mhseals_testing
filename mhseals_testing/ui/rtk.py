"""Base controls for the Odometry popup (``r``, popup.odometry rtk_base; gps.md 7.5).

Survey (minimum time and accuracy), known position and settings. Each one is saved with
``rtk.save_config`` (atomic) and restarts the base once through ``harness.reload.rtk_restart``.
``app.rtk_link`` is the running base: an object with ``base`` (the session) and ``reload``
(the harness Reload); without one there is no base and nothing is written.
"""
from copy import deepcopy
from math import isfinite

from mhseals_hardware.menu import Cancelled
from mhseals_testing import rtk
from mhseals_testing.harness import reload as reload_
from mhseals_testing.ui import rtk_pane, theme

OPTIONS = ['Start survey', 'Known position', 'Settings']
SHORTCUTS = {'s': 0, 'k': 1, 'e': 2}


def pane_rows(events, now, width=40, ascii=False):
    """Odometry popup rows for the RTK pane: a header, then one row per line."""
    return [('RTK', '')] + [('', line) for line in rtk_pane.rtk_pane(events, now, width, ascii)]


def parse_numbers(text, count):
    """``count`` floats from a line; ValueError says what is wrong."""
    words = (text or '').replace(',', ' ').split()
    if len(words) != count:
        raise ValueError(f'need {count} numbers')
    try:
        numbers = [float(word) for word in words]
    except ValueError:
        raise ValueError('numbers only') from None
    if not all(isfinite(number) for number in numbers):
        raise ValueError('numbers only')
    return numbers


def _asked(app, title, message, text):
    answer = app.ask_text(title, message, text)
    if answer is None:
        raise Cancelled('cancelled by operator')
    return answer


def _confirm(app, title, message):
    if app.choose(title, ['Start', 'Cancel'], {'y': 0}, message=message, default=1, risky={0}) != 'Start':
        raise Cancelled('cancelled by operator')


def _choose(app, config):
    """(new config, positions for the restart, drop the current position) for the chosen control; Cancelled backs out."""
    answer = app.choose('RTK base', OPTIONS, SHORTCUTS, default=0, rows=[
        ('survey', f"{config['survey_seconds']} s, {config['survey_accuracy']} m"),
        ('positions', ', '.join(config['positions']) or 'none saved'),
        ('radio', f"{config['radio'] or 'off'} {config['radio_baud']} baud")])
    new = deepcopy(config)
    if answer == 'Start survey':
        text = _asked(app, 'Survey', 'minimum seconds, accuracy m',
                      f"{config['survey_seconds']} {config['survey_accuracy']}")
        new['survey_seconds'], new['survey_accuracy'] = (lambda s, a: (int(s), a))(*parse_numbers(text, 2))
        rtk.validate(new)
        _confirm(app, 'Start survey', f"{new['survey_seconds']} s to {new['survey_accuracy']} m; "
                                      'the base re-surveys')
        return new, {}, True
    if answer == 'Known position':
        if not config['positions']:
            raise ValueError('no saved positions in rtk.yaml')
        name = app.choose('Known position', list(config['positions']))
        _confirm(app, 'Known position', f'fix the base at {name}')
        return new, {name: config['positions'][name]}, True
    text = _asked(app, 'Settings', 'radio port (- for none), baud',
                  f"{config['radio'] or '-'} {config['radio_baud']}")
    port, _, baud = text.rpartition(' ')
    new['radio'], new['radio_baud'] = ('' if port.strip() in ('', '-') else port.strip()), int(parse_numbers(baud, 1)[0])
    return new, new['positions'], False


def open_controls(app, restart=reload_.rtk_restart):
    """Run the controls; returns True when the base was restarted. Notices go to app.notice."""
    link = getattr(app, 'rtk_link', None)
    if link is None:
        app.notice = 'no base'
        return False
    try:
        config = rtk.load_config()
        new, positions, reset = _choose(app, config)
        rtk.validate(new)
        rtk.save_config(new)
    except Cancelled:
        return False
    except (ValueError, OSError) as error:
        app.notice = f'RTK base: {error}'
        return False
    link.reload.apply('rtk', config, new)
    held = link.reload.pending.setdefault('rtk', {}).setdefault(reload_.CHILD, [])
    if not held:
        held.append('restart')               # an explicit survey of unchanged settings still restarts
    if reset:
        link.base.position = None            # else rtk_restart resumes the position in use
    done = restart(link.reload, link.base, new, positions, app.emit)
    what = 'survey' if reset and not positions else 'known position' if reset else 'settings'
    app.notice = f'RTK base restarted{theme.SEPARATOR}{what}' if done else 'RTK base not restarted'
    return done
