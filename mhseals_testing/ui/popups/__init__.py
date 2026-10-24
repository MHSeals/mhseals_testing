"""Popups over the live dashboard: functions over an ``app`` that has ``modal``, ``keys``,
``refresh``, ``tick``, ``popup`` and ``scroll``. ``App`` keeps one-line methods that call these."""

from mhseals_hardware.menu import Cancelled
from mhseals_testing.runner import fold
from mhseals_testing.ui import theme
from mhseals_testing.ui.popups import conditions, config, history, power

STOP_KEYS = ('space', 'press:space')
UPDOWN = theme.pick('↑↓', 'Up/Down')


def popup(app, modal, handle, refresh=None):
    """Show ``modal`` and feed it keys until ``handle(key)`` returns a value.

    Background work keeps ticking while the popup waits, so test pulses keep
    flowing and the dashboard behind it stays live. ``refresh``, if given, is
    called after each tick to rebuild the body.
    """
    previous = app.modal
    app.modal = modal
    try:
        while True:
            app.refresh()
            key = app.keys.read(timeout=0.1)
            app.tick()
            if refresh:
                refresh()
            if key is None:
                continue
            if key in STOP_KEYS and modal.get('kind') != 'input':
                app.operator_stop()  # Space stops in every modal but popup.text
            if modal.get('kind') != 'input':
                # Authority owns E itself so it can revoke the selected claim.
                if key == 'E' and not str(modal.get('title', '')).startswith('Authority'):
                    app.revoke_agent()
                    continue
                if key == 'U':
                    app.release_hold()
                    continue
            result = handle(key)
            if result is not None:
                return result
    finally:
        app.modal = previous
        app.refresh()


def scroll(modal, key, keys):
    """Move a clipped popup's body one line for a key in ``keys`` (up, down); True if it moved."""
    if key in keys:
        modal['scroll'] = max(0, modal.get('scroll', 0) + (1 if key == keys[1] else -1))
        return True
    return False


def show(app, title, message=None, rows=None, width=60):
    modal = dict(kind='message', title=title, message=message, rows=rows, width=width,
                 hint='Any key closes')

    def handle(key):
        return not (modal['clipped'] and app.scroll(modal, fold(key), ('k', 'j')))
    app.popup(modal, handle)


def prompt(app, message, title='Ready'):
    """Wait for Enter; Escape cancels the procedure."""
    def handle(key):
        key = fold(key)
        if key == 'enter':
            return ''
        if key in ('escape', 'q'):
            raise Cancelled('cancelled by operator')
    return app.popup(dict(kind='message', title=title, message=message,
                           hint=f'Enter continue{theme.SEPARATOR}Esc cancel'), handle)


def choose(app, title, options, shortcuts=None, message=None, danger=False, rows=None,
           selected=0, width=64):
    modal = dict(kind='choice', title=title, message=message, options=list(options),
                 selected=selected, danger=danger, rows=rows, width=width,
                 hint=theme.SEPARATOR.join((f'{UPDOWN} select', 'Enter choose', 'Esc cancel')),
                 keys=[None] * len(options),
                 scroll_hint='[ ]')
    for key, index in (shortcuts or {}).items():
        modal['keys'][index] = key

    def handle(key):
        key = fold(key)
        if app.scroll(modal, key, ('[', ']')):
            return None
        if key in ('up', 'k'):
            modal['selected'] = (modal['selected'] - 1) % len(options)
        elif key in ('down', 'j'):
            modal['selected'] = (modal['selected'] + 1) % len(options)
        elif key == 'enter':
            return options[modal['selected']]
        elif shortcuts and key in shortcuts:
            return options[shortcuts[key]]
        elif key in ('escape', 'q'):
            raise Cancelled('cancelled by operator')
    return app.popup(modal, handle)


def ask_text(app, title, message, text=''):
    """Edit one line of text in a popup; None when cancelled."""
    modal = dict(kind='input', title=title, message=message, text=text, danger=False,
                 rows=None, width=64, hint=f'Enter apply{theme.SEPARATOR}Esc cancel')

    def handle(key):
        if key == 'enter':
            return modal['text']
        if key == 'escape':
            return False
        if key in ('backspace', 'ctrl-h', '\x7f'):
            modal['text'] = modal['text'][:-1]
        elif len(key) == 1 and key.isprintable():
            modal['text'] += key
    return app.popup(modal, handle) or None


def type_word(app, title, message, word, rows=None, width=64):
    """A typed confirmation for real outputs: only the exact word proceeds."""
    modal = dict(kind='input', title=title, message=message, text='', danger=True,
                 rows=rows, width=width, hint=f'Type {word.upper()} and Enter{theme.SEPARATOR}Esc cancel')

    def handle(key):
        if key == 'enter':
            return modal['text'].strip().lower() == word.lower()
        if key == 'escape':
            return False
        if key in ('backspace', 'ctrl-h', '\x7f'):
            modal['text'] = modal['text'][:-1]
        elif len(key) == 1 and key.isprintable():
            modal['text'] += key.upper()
    return app.popup(modal, handle)


# Keymap action id -> opener(app). App.open routes through it, so a popup is added here rather
# than by another branch in App.dispatch. Ids not listed (health ...) wait for their popups.
OPENERS = {
    'history': lambda app: history.open_history(app),
    'odometry': lambda app: app.odometry(),
    'conditions': lambda app: conditions.open_conditions(app),
    'config': lambda app: config.open_config(app),
    'power': lambda app: power.open_power(app),
    'perception': lambda app: app.perception_popup(),
}
