"""Popups over the live dashboard: functions over an ``app`` that has ``modal``, ``keys``,
``refresh``, ``tick``, ``popup`` and ``scroll``. ``App`` keeps one-line methods that call these."""

from mhseals_hardware.menu import Cancelled
from mhseals_testing.runner import fold
from mhseals_testing.ui import theme
from mhseals_testing.ui.popups import authority, conditions, config, history, inspect, power

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
            if key in STOP_KEYS and modal.get('kind') != 'input':
                app.operator_stop()  # Space stops in every modal but popup.text
            app.tick()
            if refresh:
                refresh()
            if key is None:
                continue
            if modal.get('kind') != 'input':
                # Authority owns E itself so it can revoke the selected claim.
                if key == 'E' and not str(modal.get('title', '')).startswith('Authority'):
                    app.revoke_agent()
                    continue
                if key == 'U':
                    if str(modal.get('title', '')).startswith('Authority'):
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


SAFE_WORDS = ('cancel', 'close', 'keep', 'no', 'disarm')   # never a danger button
CHOOSE_KEYS = dict(up=['up', 'k'], down=['down', 'j'], left=['left', 'h'], right=['right', 'l', 'tab'],
                   select=['enter'], decline=['n'], cancel=['escape', 'q'])   # when the keymap is unreadable


def safe(option):
    return str(option).split(' ', 1)[0].strip('[]').lower() in SAFE_WORDS


def choose_keys():
    """key -> action of ``popup.choose`` from the keymap YAML (the user overlay applied)."""
    from mhseals_testing.ui import keymap
    try:
        context = keymap.load()[0].doc.get('popup.choose') or CHOOSE_KEYS
    except Exception:              # a popup must open even when the keymap file is broken
        context = CHOOSE_KEYS
    return keymap.key_map(context)


def choice_modal(title, options, shortcuts=None, message=None, danger=False, rows=None, default=0,
                 width=64, risky=None):
    """The ``choice`` modal dict; ``default`` is the focused option, the one Enter picks.

    ``risky`` (default: with ``danger``, every option that is not Cancel/Close/Keep/No/Disarm)
    is drawn as danger buttons. A danger default is a ValueError, so Enter alone never acts.
    """
    options = list(options)
    if risky is None:
        risky = {i for i, o in enumerate(options) if not safe(o)} if danger else set()
    if default in risky:
        raise ValueError(f'{title}: the default must be a safe option, not {options[default]!r}')
    keys = [None] * len(options)
    for key, index in (shortcuts or {}).items():
        keys[index] = key
    return dict(kind='choice', title=title, message=message, options=options, selected=default,
                danger=danger, risky=set(risky), rows=rows, width=width, keys=keys,
                hint=theme.SEPARATOR.join((f'{UPDOWN} select', 'Enter choose', 'Esc cancel')),
                scroll_hint='[ ]')


def choose(app, title, options, shortcuts=None, message=None, danger=False, rows=None,
           default=0, width=64, risky=None):
    modal = choice_modal(title, options, shortcuts, message, danger, rows, default, width, risky)
    options, keys = modal['options'], choose_keys()
    decline = next((i for i, o in enumerate(options) if safe(o) and o.lower() in ('cancel', 'close')), None)

    def handle(key):
        key = fold(key)
        if app.scroll(modal, key, ('[', ']')):
            return None
        if shortcuts and key in shortcuts:          # a call's own shortcuts win
            return options[shortcuts[key]]
        action = keys.get(key)
        if action in ('up', 'left'):
            modal['selected'] = (modal['selected'] - 1) % len(options)
        elif action in ('down', 'right'):
            modal['selected'] = (modal['selected'] + 1) % len(options)
        elif action == 'select':
            return options[modal['selected']]
        elif action == 'decline' and decline is not None:
            return options[decline]
        elif action == 'cancel':
            raise Cancelled('cancelled by operator')
    return app.popup(modal, handle)


def edit(text, key, upper=False):
    """``text`` after one key in a one-line field: backspace deletes, Space and printables type."""
    if key in ('backspace', 'ctrl-h', '\x7f'):
        return text[:-1]
    if key == 'space':
        key = ' '
    if len(key) == 1 and key.isprintable():
        return text + (key.upper() if upper else key)
    return text


def ask_text(app, title, message, text='', check=None):
    """Edit one line of text in a popup; None when cancelled.

    ``check(text)`` runs per key; its ValueError is shown under the field and Enter is ignored
    while it fails, so the popup stays open on an invalid value.
    """
    modal = dict(kind='input', title=title, message=message, text=text, danger=False,
                 rows=None, width=64, hint=f'Enter apply{theme.SEPARATOR}Esc cancel')

    def validate():
        if check is not None:
            try:
                check(modal['text'])
                modal['state'], modal['note'] = 'ok', ''
            except ValueError as error:
                modal['state'], modal['note'] = 'bad', str(error)

    def handle(key):
        if key == 'enter':
            return None if modal.get('state') == 'bad' else modal['text']
        if key == 'escape':
            return False
        modal['text'] = edit(modal['text'], key)
        validate()
    validate()
    return app.popup(modal, handle) or None


def type_word(app, title, message, word, rows=None, width=64):
    """A typed confirmation for real outputs: only the exact word proceeds.

    Enter does nothing until the word matches, so a typo is corrected in place; only Esc
    declines. Typing stops at the word's length (one slot per letter).
    """
    word = word.upper()
    modal = dict(kind='input', title=title, message=message, text='', danger=True, word=word,
                 rows=rows, width=width, hint=f'Type {word} and Enter{theme.SEPARATOR}Esc cancel')

    def handle(key):
        if key == 'enter':
            return True if modal['text'] == word else None
        if key == 'escape':
            return False
        modal['text'] = edit(modal['text'], key, upper=True)[:len(word)]
    return app.popup(modal, handle)


# Keymap action id -> opener(app). App.open routes through it, so a popup is added here rather
# than by another branch in App.dispatch. Ids not listed (health ...) wait for their popups.
OPENERS = {
    'authority': lambda app: authority.open(app),
    'history': lambda app: history.open_history(app),
    'odometry': lambda app: app.odometry(),
    'conditions': lambda app: conditions.open_conditions(app),
    'config': lambda app: config.open_config(app),
    'power': lambda app: power.open_power(app),
    'perception': lambda app: app.perception_popup(),
    'inspect': lambda app: inspect.open_inspect(app),
}
