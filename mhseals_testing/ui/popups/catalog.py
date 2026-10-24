"""Scrollable read-only inventory using the same view as boat_trial config catalog."""
import json

from mhseals_testing.catalog_inspect import installed


def details(app, label, record):
    modal = dict(kind='message', title=label, width=76,
                 rows=[('', line) for line in json.dumps(record, indent=2).splitlines()],
                 hint='Up/Down scroll · Esc back')

    def handle(key):
        if key in ('escape', 'q'):
            return True
        if key in ('up', 'k', '[', 'down', 'j', ']'):
            modal['scroll'] = max(0, modal.get('scroll', 0)
                                  + (1 if key in ('down', 'j', ']') else -1))
    app.popup(modal, handle)


def open_catalog(app, view=None):
    """All packages, families and leaves remain reachable in narrow terminals."""
    view = installed() if view is None else view
    items = [(f"Coverage: {item['owner']}", item) for item in view['packages']]
    items += [(row['key'], row) for row in view['rows']]
    items.insert(0, ('Selection', view['selection']))
    state = dict(selected=0)
    modal = dict(kind='message', title='Catalog (read only)', width=76,
                 message=view['selection']['label'], hint='Up/Down select · Enter details · Esc back')

    def refresh():
        label, record = items[state['selected']]
        modal['rows'] = [(label, record.get('status', repr(record.get('default', ''))))]
        modal['message'] = f"{state['selected'] + 1}/{len(items)} · {view['selection']['label']}"
        modal['scroll'] = 0

    def handle(key):
        if key in ('up', 'k'):
            state['selected'] = (state['selected'] - 1) % len(items)
        elif key in ('down', 'j'):
            state['selected'] = (state['selected'] + 1) % len(items)
        elif key == 'enter':
            details(app, *items[state['selected']])
        elif key in ('escape', 'q'):
            return True
        refresh()
    refresh()
    app.popup(modal, handle)
