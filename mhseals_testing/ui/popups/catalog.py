"""Scrollable read-only parameter inventory from each package's installed config/schema.yaml."""
import json

from mhseals_testing import schema
from mhseals_testing.ui import keymap


def details(app, label, record):
    modal = dict(kind='message', title=label, width=76,
                 rows=[('', line) for line in json.dumps(record, indent=2).splitlines()],
                 hint='Up/Down scroll · Esc back')

    def handle(key):
        action = keymap.dispatch('popup.catalog_detail', key)
        if action == 'close':
            return True
        if action in ('up', 'down'):
            modal['scroll'] = max(0, modal.get('scroll', 0) + (1 if action == 'down' else -1))
    app.popup(modal, handle)


def summary(entry):
    span = f" [{entry['range'][0]}, {entry['range'][1]}]" if 'range' in entry else ''
    return f"{entry['type']} {entry['units']}{span} · {entry['tier']}"


def open_catalog(app, view=None):
    """Every package and key remains reachable in narrow terminals; ``view`` is schema.rows()."""
    found, missing = schema.rows() if view is None else view
    items = [(package, dict(status='not available', reason=why))
             for package, why in missing.items()]
    items += [(f'{package}: {key}', entry) for package, key, entry in found]
    items = items or [('No schemas', dict(status='no package installs a config/schema.yaml'))]
    state = dict(selected=0)
    modal = dict(kind='message', title='Catalog (read only)', width=76,
                 hint='Up/Down select · Enter details · Esc back')

    def refresh():
        label, record = items[state['selected']]
        modal['rows'] = [(label, record['status'] if 'status' in record else summary(record))]
        modal['message'] = f"{state['selected'] + 1}/{len(items)} · {record.get('doc', '')}"
        modal['scroll'] = 0

    def handle(key):
        action = keymap.dispatch('popup.catalog', key)
        if action == 'up':
            state['selected'] = (state['selected'] - 1) % len(items)
        elif action == 'down':
            state['selected'] = (state['selected'] + 1) % len(items)
        elif action == 'detail':
            details(app, *items[state['selected']])
        elif action == 'close':
            return True
        refresh()
    refresh()
    app.popup(modal, handle)
