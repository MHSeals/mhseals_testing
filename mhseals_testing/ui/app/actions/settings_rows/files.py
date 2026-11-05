"""Watched files: config, keymap, theme, minimap, settings and health reloads, held while armed or running."""

from mhseals_testing.analysis import classes as classes_module
from mhseals_testing.harness import settings
from mhseals_testing.harness.watch import Watched
from mhseals_testing.health import config as health_config
from mhseals_testing.history import adapters
from mhseals_testing.ui import keymap as keymap_module, palette, theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.popups import config as config_popup
from mhseals_testing.ui.app.help import map_key_maps


HELD = 'applies once disarmed and idle'   # a health or settings edit read while armed or running


class Files:
    """Watched files, mixed into Settings."""

    def watch_config(self):
        """Poll every watched file once; announce what changed (harness/watch.py)."""
        for change in self.watcher.poll():
            self.announce(change)
        self.adopt_health()
        self.monitor.advise('Config', *config_popup.summary(self.watcher, self.history))

    def announce(self, change, observe=True):
        """Show a reload's result and emit its event; returns the Change for callers that check ``ok``.

        ``observe`` logs a hand edit to the history; a revert passes False, it records itself."""
        self._announce(change, observe)
        return change

    def _announce(self, change, observe=True):
        watched = self.watcher.watches.get(change.name)
        if observe and change.ok and watched is not None and watched.subsystem:
            # A hand edit reaches the history now; our own save was recorded
            # already, so this finds no difference.
            adapters.observe_file(self.history, watched.subsystem, watched.path, self.warn_history)
        if not change.ok:
            if change.name == 'hardware':
                self.config_note = 'invalid edit'
            if change.report:
                self.say(f'Config rejected, keeping the last good one: {change.error}')
            return
        if change.name == 'hardware':
            self.config_note = watched.path.name
        self.say(f'Config read from {watched.path.name}{theme.SEPARATOR}{watched.warning}' if watched.warning
                 else f'Config reloaded from {watched.path.name}')
        self.ros.emit('config_reloaded', file=change.name, sha=change.sha, result='ok',
                      path=str(watched.path))

    def watch_own_files(self):
        """Hot-reload the TUI's own YAML in the same poll; nothing here touches ROS."""
        def load_keymap(_path):
            keymap, warning = keymap_module.load(user=keymap_module.USER_FILE)
            if warning:
                raise ValueError(warning)
            return keymap

        def load_theme(_path):
            new, warning = palette.load_theme()
            if warning:
                raise ValueError(warning)
            return new

        def apply_keymap(keymap):
            self.map_keys, self.map_tab_keys = map_key_maps(keymap.doc)
            self.fusion_keys = keymap_module.key_map(keymap.doc.get('tab.fusion'))

        def apply_theme(new):
            theme.use_palette(new)
            if self.keys is not None and new.graphics != pixels.SETTING:   # Graphics row or a hand edit
                pixels.restart(self.keys, self.console, self.write_terminal, new.graphics)
                self.repaint()
            self.minimap.redraw.mark()

        def apply_minimap(table):
            self.minimap.table = table
            self.minimap.redraw.mark()

        def apply_held(_value):     # read while armed or running: say it waits, not that it reloaded
            self.adopt_health()
            return dict(status='unknown', detail=HELD) if self.armed or self.running else None

        def apply_settings(new):    # procedures read settings each step: held while one runs
            self.pending_settings = new
            return apply_held(new)

        for watched in (
                Watched('keymap', keymap_module.USER_FILE, load_keymap, apply_keymap,
                        subsystem='keymap'),
                Watched('theme', palette.OVERLAY, load_theme, apply_theme),
                Watched('minimap', classes_module.FILE, classes_module.load_classes,
                        apply_minimap),
                Watched('settings', settings.settings_path(),
                        lambda path: settings.load(path, strict=True), apply_settings),
                Watched('health', health_config.USER_FILE, lambda _path: self.health.reload(),
                        apply_held, subsystem='health')):
            self.watcher.add(watched)

    def adopt_health(self):
        """Health rules and settings are read while armed or running: edits wait for neither."""
        if self.armed or self.running:
            return
        for watched in self.watcher.watches.values():
            if watched.warning == HELD:
                watched.warning = None
        self.health.adopt_pending()
        new, self.pending_settings = self.pending_settings, None
        if new is not None:     # a key the file dropped goes back to its default
            for spec in settings.SPECS:
                if not spec.reset:
                    setattr(self.settings, spec.key, getattr(new, spec.key))
