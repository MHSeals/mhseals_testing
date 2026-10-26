"""The Session group's Graphics row: theme.yaml ``graphics`` set from the TUI (ui/pixels.py tiers)."""

import pytest

from mhseals_testing.ui import palette, pixels, theme
from mhseals_testing.ui.panels.status import graphics_rows, status_rows

from test_app import make_app


@pytest.fixture
def overlay(tmp_path, monkeypatch):
    path = tmp_path / 'config' / 'theme.yaml'
    monkeypatch.setattr(palette, 'OVERLAY', path)
    monkeypatch.delenv('NO_COLOR', raising=False)
    monkeypatch.delenv('MHS_ASCII', raising=False)
    monkeypatch.delenv('TMUX', raising=False)
    shipped = theme.PALETTE
    pixels.forget()
    yield path
    theme.use_palette(shipped)
    pixels.use('text')
    pixels.forget()


def chip(word):
    return theme.pick(f'‹ {word} ›', f'< {word} >')


def graphics_row(app):
    rows = status_rows(app.snapshot(), app.profile.sensors)
    app.cursors['status'] = next(i for i, r in enumerate(rows) if r['name'] == 'Graphics')
    app.focus = 'status'
    return rows[app.cursors['status']]


def test_the_row_reads_the_setting_and_names_the_tier_only_when_it_differs():
    assert graphics_rows({}, ()) == []                       # only the attached App feeds it
    row = graphics_rows({'graphics': ('off', 'text')}, ())[0]
    assert row['value'] == chip('off') and not row['detail'].startswith('drawing')
    row = graphics_rows({'graphics': ('auto', 'sixel')}, ())[0]
    assert row['value'] == chip('auto') and row['detail'].startswith('drawing sixel' + theme.SEPARATOR)
    assert not graphics_rows({'graphics': ('sixel', 'sixel')}, ())[0]['detail'].startswith('drawing')
    row = graphics_rows({'graphics': ('kitty', 'text')}, ())[0]   # forced, but NO_COLOR or ASCII: text
    assert row['detail'].startswith('drawing text') and row['group'] == 'Session' and row['state'] == 'OK'


def test_the_conditions_title_names_an_image_tier_only(overlay):
    import test_conditions_drive as tcd
    pixels.use('text')
    assert 'sixel' not in tcd.page(120, 40)
    pixels.use('sixel')
    assert f'4 min{theme.SEPARATOR}sixel' in tcd.page(120, 40)


def test_arrows_step_off_auto_kitty_sixel_save_the_overlay_and_switch_the_tier(tmp_path, overlay):
    app, _ = make_app(tmp_path)
    with app.attached():
        assert theme.PALETTE.graphics == 'off' and pixels.TIER == 'text'
        assert graphics_row(app)['value'] == chip('off')
        app.dispatch('left')                                  # already first: stays, writes nothing
        assert not overlay.exists()
        app.dispatch('right')                                 # auto: no terminal to ask here, so text
        assert overlay.read_text() == 'graphics: auto\n' and pixels.SETTING == 'auto' and pixels.TIER == 'text'
        app.dispatch('right')
        app.dispatch('right')
        assert overlay.read_text() == 'graphics: sixel\n' and pixels.SETTING == 'sixel'
        assert pixels.TIER == 'text'                          # forced, but still no tty here
        assert graphics_row(app)['value'] == chip('sixel') and 'sixel' in app.notice
        app.dispatch('right')                                 # no wrap past sixel
        assert theme.PALETTE.graphics == 'sixel'
        app.dispatch('left')
        app.dispatch('left')
        app.dispatch('left')
        assert overlay.read_text() == 'graphics: off\n' and pixels.TIER == 'text'
    assert pixels.TIER == 'text'                              # detached: the stream is unwrapped
    assert not isinstance(app.console.file, pixels._Stream)
