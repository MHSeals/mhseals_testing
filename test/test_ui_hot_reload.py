"""The TUI hot-reloads its own YAML in the existing tick: keymap, theme, minimap, settings, health rules."""

import yaml

from mhseals_testing.analysis import classes
from mhseals_testing.health import config as health_config
from mhseals_testing.ui import keymap, palette, theme

from test_app import make_app


def start(tmp_path, monkeypatch):
    for module, name, file in ((keymap, 'USER_FILE', 'keymap.yaml'), (palette, 'OVERLAY', 'theme.yaml'),
                               (health_config, 'USER_FILE', 'health.yaml')):
        monkeypatch.setattr(module, name, tmp_path / file)
    app, _ = make_app(tmp_path)
    seen = []
    app.ros.emit = lambda phase, **details: seen.append((phase, details))
    return app, seen


def settle(app):
    app.watch_config()      # a changed file is read once its stat has been the same on two polls
    app.watch_config()


def test_keymap_edit_rebinds_the_minimap_keys_and_a_bad_one_keeps_the_last(tmp_path, monkeypatch):
    app, seen = start(tmp_path, monkeypatch)
    assert app.map_keys['c'] == 'layer_costmap'
    (tmp_path / 'keymap.yaml').write_text('view.minimap:\n  layer_costmap: ["1"]\n')
    settle(app)
    assert app.map_keys['1'] == 'layer_costmap' and 'c' not in app.map_keys
    assert seen[-1][0] == 'config_reloaded' and seen[-1][1]['file'] == 'keymap' and seen[-1][1]['sha']
    (tmp_path / 'keymap.yaml').write_text('view.minimap:\n  layer_costmap: ["1", nope\n')
    settle(app)
    assert app.map_keys['1'] == 'layer_costmap' and app.watcher.watches['keymap'].error
    assert 'keymap' in app.notice


def test_theme_overlay_changes_the_live_styles(tmp_path, monkeypatch):
    app, _ = start(tmp_path, monkeypatch)
    before = theme.ACCENT
    tokens = yaml.safe_load(palette.default_path().read_text())['tokens']
    other = 'bright_red' if before != 'bright_red' else 'bright_blue'
    (tmp_path / 'theme.yaml').write_text(yaml.safe_dump({'tokens': {'accent': dict(tokens['accent'], ansi=other)}}))
    palette_before = theme.PALETTE
    settle(app)
    assert theme.PALETTE is not palette_before
    assert theme.PALETTE.style('accent', tier='16') == other
    assert theme.AUTHORITY_HUES['task'] == theme.ACCENT
    (tmp_path / 'theme.yaml').write_text('theme: purple\n')
    kept = theme.PALETTE
    settle(app)
    assert theme.PALETTE is kept and app.watcher.watches['theme'].error
    (tmp_path / 'theme.yaml').unlink()
    settle(app)
    assert theme.ACCENT == before


def test_minimap_classes_reload(tmp_path, monkeypatch):
    app, _ = start(tmp_path, monkeypatch)
    path = tmp_path / 'minimap.yaml'
    path.write_text(classes._default_path().read_text())
    app.watcher.watches['minimap'].path = path
    app.watcher.reload('minimap')
    table = app.minimap.table
    path.write_text(path.read_text() + '\n')
    settle(app)
    assert app.minimap.table is not table
    path.write_text('classes: [\n')
    settle(app)
    assert app.minimap.table is not None and app.watcher.watches['minimap'].error


def test_settings_edit_applies_and_our_own_save_is_not_announced(tmp_path, monkeypatch):
    app, seen = start(tmp_path, monkeypatch)
    path = app.watcher.watches['settings'].path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('amplitude: 0.5\n')
    settle(app)
    assert app.settings.amplitude == 0.5
    path.write_text('amplitude: 7\n')
    settle(app)
    assert app.settings.amplitude == 0.5 and app.watcher.watches['settings'].error
    seen.clear()
    from mhseals_testing.harness import settings
    settings.save(app.settings)
    app.watcher.reload('settings')
    settle(app)
    assert not seen


def test_health_rules_apply_on_the_next_evaluation_but_not_while_armed(tmp_path, monkeypatch):
    app, _ = start(tmp_path, monkeypatch)
    def severity():
        return next(s for s in app.health.specs if s.id == 'conditions.poor').severity

    assert severity() == 'fail'
    old_engine = app.engine
    app.arming.armed = True
    (tmp_path / 'health.yaml').write_text('override:\n  conditions.poor: {severity: warn}\n')
    settle(app)
    assert severity() == 'fail' and app.engine is old_engine      # held while armed
    app.arming.armed = False
    app.watch_config()
    assert severity() == 'warn' and app.engine is not old_engine
    (tmp_path / 'health.yaml').write_text('override: [\n')
    settle(app)
    assert severity() == 'warn' and app.watcher.watches['health'].error


def test_settings_and_health_edits_wait_for_a_running_procedure(tmp_path, monkeypatch):
    app, _ = start(tmp_path, monkeypatch)
    def severity():
        return next(s for s in app.health.specs if s.id == 'conditions.poor').severity

    before = app.settings.amplitude
    note = app.config_note
    app.running = 'Controller'
    path = app.watcher.watches['settings'].path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('amplitude: 0.9\n')
    (tmp_path / 'health.yaml').write_text('override:\n  conditions.poor: {severity: warn}\n')
    (tmp_path / 'theme.yaml').write_text('not: [valid\n')
    settle(app)
    assert app.settings.amplitude == before and severity() == 'fail'
    assert app.config_note == note      # only the hardware file labels the thrusters panel
    app.running = None
    app.watch_config()
    assert app.settings.amplitude == 0.9 and severity() == 'warn'


def test_no_ros_client_is_touched(tmp_path, monkeypatch):
    app, seen = start(tmp_path, monkeypatch)
    (tmp_path / 'keymap.yaml').write_text('view.minimap:\n  layer_costmap: ["1"]\n')
    settle(app)
    assert {phase for phase, _ in seen} == {'config_reloaded'}
