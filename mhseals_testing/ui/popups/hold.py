"""Hold-to-confirm (D-20261101-28): a popup that confirms once Enter, the action's own key or a mouse button
is held until its bar fills (ui/input.py times it per terminal, config/input.yaml says which actions use it).

A tap, a release, a gap in the key-repeat stream or any other key empties the bar; Esc (or n) declines; Space
stops the boat at once and declines, as in every popup; E revokes the agent. After a keyboard confirm the held
key's repeats are swallowed until it is let go, so a held Enter never reaches the page (where it runs a row).
"""

from rich.console import Group
from rich.text import Text

from mhseals_testing.ui import input as key_input
from mhseals_testing.ui import theme

STOP_KEYS = ('space', 'press:space')
DECLINE = ('escape', 'n')
READ_S = 0.05            # s per read: the bar's frame
NAMES = {'enter': 'Enter', 'mouse-left': 'click'}


def fill(share, width=24):
    """``▕██████▌     ▏`` (``[=====     ]`` in ASCII) for a 0..1 share."""
    share = max(0.0, min(1.0, share))
    if theme.ASCII:
        filled = round(share * width)
        return '[' + '=' * filled + ' ' * (width - filled) + ']'
    eighths = round(share * width * 8)
    bar = '█' * (eighths // 8) + ('▏▎▍▌▋▊▉'[eighths % 8 - 1] if eighths % 8 else '')
    return '▕' + bar.ljust(width) + '▏'


def label(key):
    """How the popup names the keys that confirm: ``Enter or a`` (and a click with the mouse on)."""
    return ' or '.join(NAMES.get(name, name) for name in dict.fromkeys(('enter', key)) if name)


def body(rows, width, quiet, share, verb, key):
    from mhseals_testing.ui import layout       # layout draws every popup; imported here, after it loaded
    parts = layout.popup_rows(rows, width - 4, quiet) if rows else []
    line = Text(f'Hold {label(key)} to {verb}  ', style=f'bold {theme.BAD}')
    line.append(fill(share), style=theme.BAD if share else theme.MUTED)
    return Group(*parts, Text(''), line)


def confirm(app, title, verb, rows=None, message=None, key=None, width=64, quiet=False):
    """True once a hold completes; False when declined, stopped or another popup's key ended it."""
    keys, timing = app.keys, key_input.load()
    names = list(dict.fromkeys(name for name in ('enter', key, 'mouse-left') if name))
    holds = [key_input.hold(name, keys, timing) for name in names]
    base = [(hold.first_gap_s, hold.gap_s) for hold in holds]
    hint = theme.SEPARATOR.join((f'Hold {label(key)}', 'Esc cancel', 'Space stop'))
    modal = dict(kind='message', title=title, message=message, width=width, danger=True, quiet=quiet,
                 hint=hint, word_hue=None)
    previous, app.modal = app.modal, modal
    try:
        with key_input.mouse(keys):
            done = None
            while done is None:
                now = app.clock()
                modal['body'] = body(rows, width, quiet, max(h.progress(now) for h in holds), verb, key)
                app.refresh()
                app.tick()
                # The hold heard nothing while drawing: its repeat gaps stretch by that time, or a slow frame
                # (images, a link) would empty the bar on every repeat.
                busy = max(0.0, app.clock() - now)
                for hold, (first, gap) in zip(holds, base):
                    hold.first_gap_s, hold.gap_s = first + busy, gap + busy
                done = step(app, holds, keys.read(timeout=READ_S), key)
    finally:
        app.modal = previous
    if done:       # a held key repeats on after the confirm; a mouse button sends nothing more
        swallow(app, keys, [name for name, hold in zip(names, holds)
                            if hold.started is not None and not name.startswith('mouse-')], timing)
    app.refresh()
    if done:
        event = getattr(getattr(app, 'monitor', None), 'event', None)
        if event is not None:
            event(f'hold confirmed: {title}')
    return done


def step(app, holds, pressed, key):
    """One read's effect: True confirmed, False declined, None still waiting."""
    now = app.clock()
    if pressed in STOP_KEYS:
        app.operator_stop()
        return False
    if pressed in DECLINE and pressed != key:
        return False
    if pressed == 'E' and key != 'E':
        app.revoke_agent()
    fed = [hold.feed(pressed, now) if pressed is not None else hold.tick(now) for hold in holds]
    return True if any(fed) else None


def swallow(app, keys, names, timing):
    """Read until the held key has been let go (no repeat of it for the repeat delay); Space still stops."""
    if not names:
        return
    quiet = timing[key_input.terminal(keys).kind]['first_gap']
    last = app.clock()
    app.say(f"Confirmed{theme.SEPARATOR}let go of {', '.join(NAMES.get(name, name) for name in names)}")
    while app.clock() - last < quiet:
        pressed = keys.read(timeout=READ_S)
        if pressed in STOP_KEYS:
            app.operator_stop()
            return
        if pressed is not None and pressed.rpartition(':')[2] in names:
            last = app.clock()
        app.tick()
