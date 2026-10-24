"""The Config status row and popup.config (ui 6.1, authority 7.2).

The row reads ``Config  ✓ 9 files · keymap 14:05``: how many surfaces the watcher has, and the
newest change the config history knows (subsystem and time). A rejected file replaces it with
``! hardware.yaml rejected``. The popup lists one row per surface: ``✓`` loaded, ``↻`` file
changed and not yet applied (Enter applies it), ``⚠`` error. ``r`` (restart the TUI's code) is
shown disabled until the reload rework (TS092) lands.
"""

from datetime import datetime

from mhseals_testing.harness.watch import stat_key
from mhseals_testing.ui import theme

GLYPH = dict(ok=theme.pick('✓', 'ok'), pending=theme.pick('↻', '~'), bad=theme.pick('⚠', '!'))
HINT = theme.SEPARATOR.join(('Enter apply', 'r restart code (not yet)', 'Esc close'))


def newest_change(history):
    """'keymap 14:05' of the newest change per subsystem (the latest of them), or ''."""
    key = stat_key(history.log)
    if key is not None and getattr(history, '_newest', (0,))[0] == key:   # the log has not been written since
        return history._newest[1]
    latest = {}
    for change in history.changes():   # one pass: the last change of each subsystem
        latest[change['subsystem']] = change
    last = max(latest.values(), key=lambda c: c['time'], default=None)
    text = f"{last['subsystem']} {datetime.fromisoformat(last['time']):%H:%M}" if last else ''
    history._newest = (key, text)
    return text


def summary(watcher, history):
    """(level, text) for the Config row."""
    rows = watcher.rows()
    bad = [name for name, ok, _, _ in rows if not ok]
    if bad:
        return 'WARN', f"! {watcher.watches[bad[0]].path.name} rejected"
    text = f"{GLYPH['ok']} {len(rows)} file" + 's' * (len(rows) != 1)
    changed = newest_change(history)
    return 'OK', text + (f'{theme.SEPARATOR}{changed}' if changed else '')


def pending(watched):
    """True when the file on disk differs from what was last applied or rejected."""
    return watched.error is None and stat_key(watched.path) != watched.key


def rows(watcher, selected):
    """(label, text) per surface, the selected one marked with a cursor."""
    out = []
    for index, watched in enumerate(watcher.watches.values()):
        if watched.error:
            text = f"{GLYPH['bad']} {watched.error}"
        elif pending(watched):
            text = f"{GLYPH['pending']} pending (file changed)"
        else:
            at = f' {datetime.fromtimestamp(watched.loaded_at):%H:%M}' if watched.loaded_at else ''
            text = f"{GLYPH['ok']} {watched.path.name}{at}"
        out.append((f"{'›' if index == selected else ' '} {watched.name}", text))
    return out


def open_config(app):
    """popup.config: up/down move, Enter applies a pending row, Esc closes."""
    state = dict(selected=0)
    names = lambda: [*app.watcher.watches, 'Catalog']
    def menu_rows():
        count = len(app.watcher.watches)
        return rows(app.watcher, state['selected']) + [(
            ('› ' if state['selected'] == count else '  ') + 'Catalog', 'Read-only package inventory')]
    modal = dict(kind='message', title='Config', rows=menu_rows(), width=76, hint=HINT)

    def handle(key):
        key = key.lower() if len(key) == 1 else key
        count = len(names())
        if key in ('up', 'k') and count:
            state['selected'] = (state['selected'] - 1) % count
        elif key in ('down', 'j') and count:
            state['selected'] = (state['selected'] + 1) % count
        elif key == 'enter' and count:
            if state['selected'] == len(app.watcher.watches):
                from mhseals_testing.ui.popups.catalog import open_catalog
                open_catalog(app)
            else:
                watched = app.watcher.watches[names()[state['selected']]]
                if pending(watched):
                    app.announce(app.watcher.reload(watched.name))
        elif key == 'escape':
            return True
        modal['rows'] = menu_rows()
    app.popup(modal, handle, refresh=lambda: modal.update(rows=menu_rows()))
