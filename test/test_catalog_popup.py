"""Real Config handlers keep read-only inspection separate from watcher actions."""
from types import SimpleNamespace

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import catalog
from test_app import make_app


def test_real_config_row_catalog_and_narrow_navigation(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path)
    app.watch_config()
    names = [row['name'] for row in status_rows(app.snapshot(), load_profile('boat').sensors)]
    app.cursors['status'] = names.index('Config')
    def forbidden(*a, **kw):
        raise AssertionError('read-only catalog called writer/adapter')
    monkeypatch.setattr(app.watcher, 'reload', forbidden)
    monkeypatch.setattr(app.history, 'changes', forbidden)
    rows = [dict(key=f'{family}.leaf', default=3, blockers=['runtime-authority-not-provided'])
            for family in ('hardware', 'nav2', 'mavros', 'lidar')]
    view = dict(rows=rows, packages=[dict(owner='mhseals_nav', status='available')],
                selection=dict(status='unknown', label='selection unknown; package inventory only'))
    monkeypatch.setattr(catalog, 'installed', lambda: view)
    seen = []
    def popup(modal, handle, refresh=None):
        seen.append(modal['title'])
        if modal['title'] == 'Config':
            assert any(label.strip() == 'Catalog' for label, _ in modal['rows'])
            for _ in range(len(app.watcher.watches)):
                handle('down')
            handle('enter')
        elif modal['title'] == 'Catalog (read only)':
            from rich.console import Console
            from mhseals_testing.ui import layout
            import io
            for _ in range(2 + len(rows)):
                output = io.StringIO()
                Console(file=output, width=36, height=12).print(layout.popup(modal, 36, 12)[0])
                assert output.getvalue()
                handle('enter')
                handle('down')
        else:
            handle('down')
            assert modal['scroll'] == 1
            assert handle('escape') is True
        assert handle('escape') is True
    app.popup = popup
    app.on_status('enter')
    assert seen[:2] == ['Config', 'Catalog (read only)']
    assert all(row['key'] in seen for row in rows)


def test_catalog_uses_existing_popup_emergency_passthrough():
    from mhseals_testing.ui import popups
    calls = []
    keys = iter(['space', 'E', 'b', 'enter', 'space', 'E', 'escape', 'escape'])
    app = SimpleNamespace(modal=None, refresh=lambda: None, tick=lambda: None,
                          keys=SimpleNamespace(read=lambda **kw: next(keys)),
                          operator_stop=lambda: calls.append('stop'),
                          revoke_agent=lambda: calls.append('revoke'))
    app.popup = lambda modal, handle: popups.popup(app, modal, handle)
    catalog.open_catalog(app, dict(rows=[], packages=[], selection={
        'status': 'unknown', 'label': 'selection unknown; package inventory only'}))
    assert calls == ['stop', 'revoke', 'stop', 'revoke']
    assert app.modal is None
