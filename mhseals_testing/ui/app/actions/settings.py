"""Config, hardware and health: watching files, applying edits and the Status panel."""

import time

from mhseals_hardware.menu import Cancelled
from mhseals_testing import calibration, data
from mhseals_testing.harness import journal, settings
from mhseals_testing.history import revert
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ros.arming import command_profile
from mhseals_testing.ui import rtk, rtk_icon, theme
from mhseals_testing.ui.panels import correction
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import odometry

from mhseals_testing.ui.app.actions.settings_rows.files import HELD, Files
from mhseals_testing.ui.app.actions.settings_rows.hardware import Hardware
from mhseals_testing.ui.app.actions.settings_rows.health import ADVISORY_PERIOD, Health
from mhseals_testing.ui.app.actions.settings_rows.status import StatusRows


__all__ = ['ADVISORY_PERIOD', 'HELD', 'Settings']


class Settings(Health, Files, Hardware, StatusRows):
    """Config, hardware and health, mixed into App."""

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

    def switch_command_mode(self, mode):
        """Hot-switch the command path (D-20261101-06): this session takes the profile its start would build for
        ``mode``, so procedures, the view and the next arm (driver alone, or the velocity controller in front)
        follow at once, no restart. Only disarmed, idle and with the last arm's children gone; the error, else ''."""
        base = getattr(self.args, 'base_profile', None)
        if base is None or self.arming is None or self.review is not None or self.watching:
            return 'this session has no command path to switch; restart it with --set command_mode=' + mode
        if self.armed or self.running or self.manual:
            return 'armed or busy: it switches only while disarmed and between runs'
        try:
            profile = command_profile(base, mode)
            self.arming.switch_profile(profile)
        except (RuntimeError, ValueError) as error:
            return str(error)
        self.profile = profile
        if getattr(self.source, 'profile', None) is not None:
            self.source.profile = profile
        self.emit('command_mode', mode=mode, profile=profile.name)
        return ''

    def set_setting(self, key, value, who=None):
        """Change one setting as the operator's Settings change does: the value, the saved file (our own save, not
        a hand edit) and a journal line; an agent's (``who``) is a notice and a toast naming it too."""
        old = getattr(self.settings, key)
        if key == 'command_mode' and value != old:
            error = self.switch_command_mode(value)
            if error:
                self.notice = f'command_mode not switched: {error}'
                return error
        setattr(self.settings, key, value)
        settings.save(self.settings)
        self.watcher.reload('settings')
        self.emit('setting', key=key, old=old, value=value, by='agent' if who else 'operator', who=who)
        if who:
            self.notice = f'agent {who} set {key}={value}'
            self.toasts.push(f'agent.setting.{key}', 'warn', self.notice, self.monitor.clock())
