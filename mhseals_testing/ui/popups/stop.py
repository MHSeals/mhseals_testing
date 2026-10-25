"""The Stop popup: what stopped the boat, what was sent and what releases it (Status > Hold, Enter).

Rows come from ``snapshot['control']`` (the hold and the RC mode) and the monitor's ``STOP:``
events. Enter closes; ``U`` opens the ``Release hold?`` confirm, so nothing here releases on its own.
"""

from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing.ui import authority_chip, theme, widgets

CAUSES = {'rc_stop': 'RC stop channel (falling edge)', 'keys': 'Space (operator stop)',
          'fault': 'arming fault'}
RECENT = 3


def body(control, armed, events, width=60):
    holds = (control or {}).get('holds') or [{}]
    hold, rc = holds[0], (control or {}).get('rc') or {}
    mode = rc.get('mode') or theme.DASH
    grid = theme.grid({'header': '', 'style': f'bold {theme.ACCENT}'},
                      {'header': '', 'no_wrap': False, 'overflow': 'fold', 'ratio': 1})
    grid.show_header = False
    ago = f"{theme.SEPARATOR}{hold['age']} ago" if hold.get('age') else ''
    grid.add_row('Cause', CAUSES.get(hold.get('cause'), str(hold.get('cause') or theme.DASH)) + ago)
    grid.add_row('Neutral', 'sent')
    grid.add_row('Outputs', theme.labelled('bad', 'armed') if armed else theme.labelled('ok', 'disarmed'))
    grid.add_row('RC switch', f'{mode}  (U is refused until GUIDED)' if mode == 'HOLD' else mode)
    buttons = Table.grid(padding=(0, 2))
    buttons.add_column(style=f'bold {theme.ACCENT}', min_width=9)
    buttons.add_column()
    buttons.add_row('Release', widgets.button_row([widgets.button('release hold', 'U'),
                                                  widgets.button('close', 'Esc', focused=True)], width)
                    or Text('U release · Esc close'))
    parts = [grid, buttons]
    stops = [message[len('STOP: '):] for _, message in events if str(message).startswith('STOP: ')]
    if stops:
        parts += [widgets.rule('Recent', width)] + [Text(f'  {s}', style=theme.MUTED) for s in stops[-RECENT:]]
    return Group(*parts)


def modal(app):
    control = app.snapshot().get('control') or {}
    hold = ((control.get('holds') or [{}])[0])
    return dict(kind='message', title=f"Stop{theme.SEPARATOR}{authority_chip.stop_cause(hold) or 'hold'}",
                body=body(control, app.armed, list(getattr(app.monitor, 'events', ()))),
                width=64, border=theme.PALETTE.style('stop'), release=True,
                hint=f'U release{theme.SEPARATOR}Esc close')


def open_stop(app):
    """Show the popup, refreshed while open; Enter, Esc or q closes. U is routed by ``popups.popup``."""
    m = modal(app)
    app.popup(m, lambda key: True if key in ('escape', 'q', 'enter') else None,
              lambda: m.update(modal(app)))
