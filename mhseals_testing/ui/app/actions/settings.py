"""Config, hardware and health: watching files, applying edits and the Status panel."""

import time

import yaml

from mhseals_hardware.configuration import config_sha, load_config, save_config, update_config
from mhseals_hardware.menu import Cancelled
from mhseals_testing import calibration, data
from mhseals_testing.analysis import classes as classes_module
from mhseals_testing.harness import journal, settings
from mhseals_testing.nav2 import edit as nav2_edit
from mhseals_testing.harness.watch import Watched
from mhseals_testing.health import legacy as health, config as health_config
from mhseals_testing.history import adapters, revert
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ros.arming import EFFORT_TOPIC, command_input, session_hardware
from mhseals_testing.ui import keymap as keymap_module, palette, parts, popups, rtk, rtk_icon, theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.panels import correction
from mhseals_testing.ui.feeds import agent as agent_feed
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import config as config_popup, course, odometry, stop as stop_popup

from mhseals_testing.ui.app.help import map_key_maps


ADVISORY_PERIOD = 1.0


HELD = 'applies once disarmed and idle'   # a health or settings edit read while armed or running


class Settings:
    """Config, hardware and health, mixed into App."""

    def refresh_advisories(self):
        """Sample the non-blocking checks at most once per ADVISORY_PERIOD."""
        if self.clock() - self.last_advisory < ADVISORY_PERIOD:
            return
        self.last_advisory = self.clock()
        seen, live = self.foreign_instances()
        self.feed_map()
        self.monitor.advise('instances', 'FAIL' if live else 'WARN' if seen else 'OK',
                            ', '.join(seen) if seen else 'no other instance')
        self.sync_guests()
        snapshot = self.snapshot()
        for name, (level, detail) in self.health.tick(snapshot).items():
            self.monitor.advise(name, level, detail)
        if self.health.error:
            self.monitor.advise('health config', 'FAIL', self.health.error)
        if self.profile.use_sim_time and self.ros.now() == 0:
            # Without /clock every stamp looks stale; say why instead.
            self.monitor.advise('clock', 'FAIL', 'use_sim_time but no /clock received')
        network = self.network.poll() if self.network else None
        if network:
            self.monitor.advise('network', *network)
        if getattr(self.network, 'clocks', None):       # pre.clock_sync judges clock.spread
            self.health.facts.publish('clock_sync', self.network.clock_facts)
        self.monitor.advise('recording', 'OK' if self.processes.running('bag') else 'FAIL',
                            str(self.paths.bag))
        self.watch_recording()
        exited = [name for name in self.processes.exited() if name != 'bag']
        if exited:
            self.monitor.advise('processes', 'WARN', 'exited: ' + ', '.join(exited))
        if self.profile.outputs != 'none':
            topic = EFFORT_TOPIC if self.profile.velocity_controller else command_input(self.profile)
            self.monitor.advise('PWM driver', *health.pwm_driver(
                self.ros.nodes(), self.ros.subscribers(topic), topic,
                self.armed, self.processes.running('hardware')))
        if self.profile.outputs == 'odroid':
            config = self.args.hardware_config
            level, detail, self.pwm_readback = health.pwm_readback(
                config['pwm_chips'], config['pwm_channels'], self.armed, config['frequency'])
            self.monitor.advise('PWM readback', level, detail)

    def feed_map(self):
        """The recorded costmap follows the Recorder mode and the session owner (map.md 4)."""
        feeds = getattr(self.ros, 'map', None)
        if feeds is not None:
            nav = any(n.rsplit('/', 1)[-1] == 'planner_server' for n in self.ros.nodes())
            feeds.want_record(not self.watching, self.recorder.mode != 'off', nav)

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
                self.notice = f'Config rejected, keeping the last good one: {change.error}'
            return
        if change.name == 'hardware':
            self.config_note = watched.path.name
        self.notice = (f'Config read from {watched.path.name}{theme.SEPARATOR}{watched.warning}' if watched.warning
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
                self.last_drawn = None
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
                Watched('minimap', classes_module._default_path(), classes_module.load_classes,
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

    def load_hardware(self, _path):
        return load_config(self.args.config)

    def apply_hardware(self, config):
        """Take ``config`` and report what the hardware did with it, not just that it was saved.

        Arming a remote thrusters host, the boat's hardware.yaml is the only config: a local edit or
        reload is refused and never replaces it (it is read from the boat at session start and each arm).
        """
        remote = getattr(self.arming, 'remote', None) if self.arming is not None else None
        if remote is not None:
            return dict(status='refused', error=f"the boat's hardware.yaml is the config here: edit it on "
                                                f"{remote.host.name}; it is read again at each arm")
        driven = self.armed and self.profile.outputs != 'none'
        if driven:
            # Complete snapshot staging before replacing the last good in-memory state.
            save_config(session_hardware(self.profile, config, self.paths.hardware), self.paths.hardware)
        previous = self.args.hardware_config or {}
        self.args.hardware_config = config
        if config.get('manual_amplitude') != previous.get('manual_amplitude'):
            self.manual_speed = None   # a new configured default wins over the session's last speed
        if self.arming is not None:
            self.arming.hardware_config = config  # arming saves this copy
        if not driven:
            return dict(status='saved')
        sha = config_sha(load_config(self.paths.hardware))
        end = time.monotonic() + 4.0   # the driver polls the file at 1 Hz
        while True:
            status = getattr(self.ros, 'driver_status', None) or {}
            if status.get('requested_config_sha') == sha and any(
                    name.endswith('/thruster_pwm_node') for name in self.ros.nodes()):
                break
            if time.monotonic() >= end:
                return dict(status='unknown', detail='driver did not confirm the edit')
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                time.sleep(0.1)
            if key in ('space', 'escape', 'press:space', 'press:escape'):
                self.stop('stopped by operator')
                return dict(status='saved')
        error = status.get('error') or ''
        if error or status.get('pending_restart_keys'):
            return dict(status='refused', error=error or 'restart required')
        # The driver reports these accepted, but the controller keeps its plant
        # and the mux waits for idle/hold.
        if config.get('controller') != previous.get('controller'):
            return dict(status='next_session', detail='controller: restart-only, not applied')
        if config.get('command_mux') != previous.get('command_mux'):
            return dict(status='unknown', detail='mux applies at idle/hold; not confirmed')
        return dict(status='applied')

    def warn_history(self, message):
        self.notice = message

    def observe_config(self):
        """At start: log config files edited since the last session as external changes."""
        adapters.observe_all(self.history, {'hardware': self.config_file}, self.warn_history)

    def save_hardware(self, **changes):
        """Write ``changes`` into the operator's hardware YAML; the watcher applies it.

        Returns False, keeping the draft, when the file on disk is broken, or when a remote boat's
        hardware.yaml is the config (refused before any write).
        """
        remote = getattr(self.arming, 'remote', None) if self.arming is not None else None
        if remote is not None:
            self.notice = f"Not saved: the boat's hardware.yaml is the config, edit it on {remote.host.name}"
            return False
        adapters.observe_file(self.history, 'hardware', self.config_file, self.warn_history)   # a hand edit first
        try:
            update_config(changes, self.args.config)
        except (OSError, ValueError, TypeError, yaml.YAMLError):
            self.notice = f'Fix hardware.yaml first: {self.config_file}'
            return False
        adapters.record_file(self.history, 'hardware', 'tui:hardware', self.config_file, self.warn_history)
        change = self.announce(self.watcher.reload('hardware'))  # skip the settle wait
        if not change.ok:
            return False
        self.draft = self.grabbed = None
        return True

    def on_status(self, key):
        rows = status_rows(self.snapshot(), self.profile.sensors)
        if key in ('left', 'right') and self.agents_row(rows):
            self.toggle_agents()
        elif key in ('left', 'right') and self.row_setting(rows) == 'on_exit' and not self.watching:
            state = self.map_state()
            client = self.fence_client()
            if state is not None and client is not None:
                self.notice = course.toggle_on_exit(self, client, state.fence_status)
        elif key in ('left', 'right') and self.row_setting(rows) == 'agent_motion':
            agent_feed.step_setting(self, self.row_setting(rows), key)
        elif key in ('left', 'right') and self.row_setting(rows) == 'recording':
            self.step_recording(key)
        elif key in ('left', 'right') and self.row_setting(rows) == 'graphics':
            self.step_graphics(key)
        elif key == 'e':
            row = rows[min(self.cursors['status'], len(rows) - 1)] if rows else None
            if row and row['name'] == nav2_edit.NAME and self.nav2_file is not None:
                self.nav2_popup()
        elif key == 'enter':
            if rows:
                row = rows[min(self.cursors['status'], len(rows) - 1)]
                if row.get('source') == 'sensor' and row['name'] == 'odometry':
                    return self.open('odometry')
                if row.get('source') == 'part' and not self.watching:
                    return parts.toggle(self, row['key'])
                if row.get('source') == 'stack' and not self.watching:
                    return parts.toggle_stack(self)
                if row['name'] == 'Driving':
                    return self.open('authority')
                if row['name'] == 'Course area' and not self.watching:
                    self.notice = course.open_course(self)
                    return None
                if row['name'] == 'Hold':
                    return stop_popup.open_stop(self) if row.get('stop') else self.release_hold()
                if row['name'] in ('RTK', 'Source'):
                    return self.open('odometry')
                if row['name'] == 'Config':
                    return self.open('config')
                if row['name'].lower() == 'battery':   # the health advisory is named 'battery'
                    return self.open('power')
                return popups.inspect.open_inspect(self, row)   # the view fits the row (sensor, tf, ...)

    @property
    def facts(self):
        """The health facts by name, among them the perc.<source>.* ones of the perception probe."""
        return self.health.facts.read()[0]

    def power_facts(self):
        """The latest ``power.*`` facts (ros.health feeds them), or None without a voltage source."""
        return getattr(self.ros, 'power_facts', None)

    def row_setting(self, rows):
        row = rows[min(self.cursors['status'], len(rows) - 1)] if rows else None
        return row.get('setting') if row else None

    def agents_row(self, rows):
        return self.row_setting(rows) == 'agents'

    def toggle_agents(self):
        """Flip the Agents setting; a bound arbiter follows (block refuses new agent claims)."""
        self.settings.agents = 'block' if self.settings.agents == 'allow' else 'allow'
        arbiter = getattr(self, 'arbiter', None)
        if arbiter is not None:
            arbiter.allow_agents = self.settings.agents == 'allow'
        self.notice = f'Agents {self.settings.agents}'
        desk = getattr(self, 'desk', None)
        if desk is not None:
            desk.setting_changed()           # block stops an agent run in flight, as E does

    def open(self, action_id):
        """Open the popup registered for ``action_id`` in ``popups.OPENERS``; an unbound id does nothing."""
        opener = popups.OPENERS.get(action_id)
        if opener is None:
            return False
        opener(self)
        return True

    def revert_env(self):
        """The revert blocker probes over this session, and the mux hold used while armed.

        ``ros.mux_hold(token)`` (a context manager) is optional; without it an armed
        hardware revert is refused (revert.blockers).
        """
        hold = getattr(self.ros, 'mux_hold', None)
        return revert.Env(
            probes={'procedure': lambda: self.running is not None, 'pulses': lambda: self.test_pulses is not None},
            armed=lambda: self.armed,
            **({'hold': hold} if hold else {}))

    def odometry(self):
        """Fused sources popup. Parameters and rows are read here, once, so nothing is
        computed while it is closed. ``ros.get_parameters(node)`` is optional (None: unreadable)."""
        if self.review:
            profile = getattr(self.source, 'profile', self.profile)
            rows = [(row['name'], row['state'], row['value'])
                    for row in status_rows(self.snapshot(), profile.sensors)
                    if row.get('source') == 'sensor']
            self.show('Pose (review)', rows=rows, width=76)
            return
        fetch = getattr(self.ros, 'get_parameters', lambda node: None)
        params = {node: fetch(node) for node in odometry.ESTIMATES}
        sources = [(row['name'], row['state'].lower(), row['value'])
                   for row in status_rows(self.snapshot(), self.profile.sensors)
                   if row.get('source') == 'sensor']
        rows = odometry.rows(params, sources)
        events = [e for e in journal.read(self.paths.journal) if e.get('phase') == 'gps_fix']
        if self.rtk.current is not None:     # the correction source: its icon, ring and label
            rows += [('Source' if i == 0 else '', line)
                     for i, line in enumerate(rtk_icon.pane_lines(self.rtk.current, theme.ASCII))]
        if events or self.rtk_link is not None:
            rows += rtk.pane_rows(events, time.time(), 44, theme.ASCII)
        try:   # r opens the RTK base controls next to e (no base says so)
            answer = self.choose('Pose', ['Restart EKFs', 'Base controls', 'Magcal', 'Close'],
                                 {'e': 0, 'r': 1, 'c': 2}, default=3, rows=rows, width=76)
        except Cancelled:
            return
        if answer == 'Restart EKFs':
            self.restart_ekf()
        elif answer == 'Magcal':          # keymap popup.odometry c: the compass calibration (g)
            self.run_procedure(next(p for p in PROCEDURES if p.key == 'g'))
        elif answer == 'Base controls':
            rtk.open_controls(self)

    def restart_ekf(self):
        """Restart the EKF child (popup.odometry e); refused with the reason while the boat is busy.

        The EKF and navsat rows share one keeper child, so either row restarts both (HR9).
        """
        if self.ekf is None:
            self.notice = 'EKF restart unavailable in this session'
            return
        result = self.ekf.restart()
        self.notice = ('EKFs restarted' if result['restarted']
                       else 'EKF restart refused: ' + '; '.join(result['blocked']))

    def correction_state(self):
        """What the Correction tab draws besides the config: the column and the state.json fits."""
        if self.tab != 'Correction':
            return None
        saved = (data.read_json(data.path('state.json'), {}) or {}).get('fits') or {}
        current = data.read_json(data.path('calibration', 'current.json'), {})
        derived = correction.provenance_fits(current, self.conditions.evaluate().get('ceilings'))
        fits = {d: {**derived.get(d, {}), **saved.get(d, {})} for d in {*derived, *saved}}
        shown = calibration.marks(self.args.hardware_config, self.history)
        fits = {**(fits or {}), **{d: {**(fits or {}).get(d, {}), 'mark': m} for d, m in shown.items()}} if shown else fits
        return dict(fits=fits, column=self.column, **self.correction_side())

    def correction_side(self):
        """Readiness (conditions + the characterize gates) and Tracking (feed + calibration file) models."""
        blocking, warnings = (), ()
        if self.engine is not None:
            # gates are keyed by the procedure letter, as availability() does
            procedure = next(p for p in PROCEDURES if p.title == 'Characterize')
            blocking, warnings = self.engine.gate(
                procedure.key, self.args.bench_override, 'armed' in procedure.needs)
        characterization = self.args.hardware_config.get('characterization') or {}
        provenance = (data.read_json(data.path('calibration', 'current.json'), {}) or {}).get('provenance') or {}
        return dict(
            readiness=correction.readiness_data(self.conditions.summary(), blocking, warnings,
                                                characterization.get('wave_floor')),
            tracking=correction.tracking_data(self.tracking.scores, self.tracking.axes,
                                              provenance.get('tracking') or {}, self.tracking.version))

    def correction_focus(self):
        """(title, position) of the footer's focus line: the cursor column, as in ui 5.7."""
        model = correction.tab_view(self.args.hardware_config, self.correction_state())
        return 'Correction', f"{model['columns'][model['cursor']]['key'].replace('-', theme.pick('−', '-'))} column"

    def on_correction(self, key):
        """Correction tab keys (keymap tab.correction); True when the key was one of them."""
        step = {'left': -1, 'h': -1, 'right': 1, 'l': 1}.get(key)
        if step is not None:
            model = correction.tab_view(self.args.hardware_config, self.correction_state())
            self.column = correction.move(model, step)
        elif key in ('r', 'R'):
            model = correction.tab_view(self.args.hardware_config, self.correction_state())
            self.characterize(model['columns'][model['cursor']]['key'] if key == 'r' else '')
        elif key == 'enter':
            model = correction.tab_view(self.args.hardware_config, self.correction_state())
            holds = correction.hold_rows(model)
            self.show(f"{model['columns'][model['cursor']]['key']} holds", rows=holds, width=50)
        elif key == 'A':
            self.apply_pending()
        elif key == 'I':
            self.hardware_changed()
        else:
            return False
        return True

    def apply_pending(self):
        """A: put an interrupted run's backed-up C back. Disarmed, the driver is down and starts from the YAML,
        which still holds it; armed, it goes to the running driver and is read back."""
        staging = self.staging()
        if staging.pending() is None:
            self.notice = 'Nothing pending'
            return
        try:
            staging.restore() if self.armed else staging.recover()
        except Cancelled as error:
            self.notice = f'C not restored: {error}'
            return
        self.notice = 'C restored' + ('' if self.armed else ' to the YAML (the driver starts from it)')

    def hardware_changed(self):
        """I: a typed note, then a confirm, then hardware_epoch + 1 (invalidates every fit); never while moving."""
        if self.moving():
            self.notice = 'Not while moving'
            return
        epoch = self.args.hardware_config.get('hardware_epoch', 0) + 1
        try:
            note = self.ask_text('Hardware changed', 'What changed? (e.g. new thruster on pin 3)')
            if note is None:
                return
            if self.choose(f'Bump hardware epoch to {epoch}?', ('Bump', 'Cancel'), {'y': 0},
                           message='Every C+/C- and speed-curve fit becomes stale and needs a refit.',
                           danger=True, default=1) != 'Bump':
                return
        except Cancelled:
            return
        if self.save_hardware(hardware_epoch=epoch):
            self.notice = f'Hardware changed: epoch {epoch}, calibrations need a refit'
            change = self.history.head('hardware')
            if change:
                self.history.label(change['id'], note=note)

    def on_conditions(self, key):
        if key == 'enter':
            self.open('conditions')

    @property
    def engine(self):
        """The health engine behind the advisory rows (gates, snapshots), or None."""
        return getattr(self.health, 'engine', None)

    def health_tick(self, facts, probes, session=None):
        """Evaluate health and journal its transitions, so trial windows can be judged."""
        report = self.engine.tick(facts, probes, session)
        self.journal.record(report)
        return report

    def health_snapshot(self, label, procedure):
        """Journal and emit the shown health; 'end' is taken before teardown."""
        if self.engine is None or self.engine.last is None:
            return
        entry = self.journal.snapshot(label, self.engine.last, self.engine.clock())
        self.ros.emit('health_snapshot', procedure=procedure.key, **entry)

    def set_setting(self, key, value, who=None):
        """Change one setting as the operator's Settings change does: the value, the saved file (our own save, not
        a hand edit) and a journal line; an agent's (``who``) is a notice and a toast naming it too."""
        old = getattr(self.settings, key)
        setattr(self.settings, key, value)
        settings.save(self.settings)
        self.watcher.reload('settings')
        self.emit('setting', key=key, old=old, value=value, by='agent' if who else 'operator', who=who)
        if who:
            self.notice = f'agent {who} set {key}={value}'
            self.toasts.push(f'agent.setting.{key}', 'warn', self.notice, self.monitor.clock())
