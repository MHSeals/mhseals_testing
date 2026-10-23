"""The Config status row and popup.config (ui 6.1, authority 7.2)."""

import io

from rich.console import Console

from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import layout
from mhseals_testing.ui.panels.status import status_panel, status_rows
from mhseals_testing.ui.popups import config

from test_app import make_app
from test_chrome import check
from test_dashboard import view


def config_lines(app, width=34):
    profile = load_profile('boat')
    console = Console(file=io.StringIO(), width=width, height=40, color_system=None)
    console.print(status_panel(app.monitor.snapshot(profile.required_tf), view(profile), 40))
    return ''.join(line + '\n' for line in console.file.getvalue().splitlines() if 'Config' in line or 'C…' in line)


def test_config_row_clean_and_rejected_goldens(tmp_path):
    app, _ = make_app(tmp_path)
    app.history.changes = lambda sub=None: [dict(subsystem='keymap', time='2026-10-23T14:05:00+00:00')]
    app.history.head = lambda sub: app.history.changes()[0]
    app.watch_config()
    clean = config_lines(app) + '==\n' + config_lines(app, 28)
    (tmp_path / 'hardware.yaml').write_text('channel_map: 5\n')
    app.announce(app.watcher.reload('hardware'))
    app.watch_config()
    check('status/config_row.txt', clean + '--\n' + config_lines(app) + '==\n' + config_lines(app, 28))


def test_enter_on_the_config_row_opens_the_popup_and_golden(tmp_path):
    app, _ = make_app(tmp_path)
    app.watch_config()
    profile = load_profile('boat')
    names = [r['name'] for r in status_rows(app.snapshot(), profile.sensors)]
    app.cursors['status'] = names.index('Config')
    modals = []
    app.popup = lambda modal, handle, refresh=None: modals.append((modal, handle))
    app.on_status('enter')
    modal, handle = modals[0]
    assert modal['title'] == 'Config'
    console = Console(file=io.StringIO(), width=120, color_system=None)
    app.watcher.watches['hardware'].loaded_at = None   # the clock time is not golden
    modal['rows'] = config.rows(app.watcher, 0)
    console.print(layout.popup(modal, 120, 30)[0])
    check('status/config_popup_120.txt', console.file.getvalue())
    (tmp_path / 'hardware.yaml').write_text('channel_map: [4, 3, 2, 1]\n')
    handle('down')
    assert 'pending' in modal['rows'][0][1]
    handle('enter')   # applies the pending row
    assert app.args.hardware_config['channel_map'] == [4, 3, 2, 1]
    handle('r')       # restart_code is shown disabled: nothing happens
    assert handle('escape') is True
    assert 'not yet' in config.HINT


def test_popup_rows_follow_the_files_without_a_keypress(tmp_path):
    app, _ = make_app(tmp_path)
    app.watch_config()
    seen = {}

    def popup(modal, handle, refresh=None):
        seen['before'] = modal['rows'][0][1]
        (tmp_path / 'hardware.yaml').write_text('channel_map: [4, 3, 2, 1]\n')
        refresh()
        seen['after'] = modal['rows'][0][1]
    app.popup = popup
    config.open_config(app)
    assert 'pending' not in seen['before'] and 'pending' in seen['after']
