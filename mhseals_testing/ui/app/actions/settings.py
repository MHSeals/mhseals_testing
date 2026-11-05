"""Config, hardware and health: watching files, applying edits and the Status panel."""

from mhseals_testing import calibration, data
from mhseals_testing.harness import settings
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ros.arming import command_profile
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import correction

from mhseals_testing.ui.app.actions.settings_rows.files import HELD, Files
from mhseals_testing.ui.app.actions.settings_rows.hardware import Hardware
from mhseals_testing.ui.app.actions.settings_rows.health import ADVISORY_PERIOD, Health
from mhseals_testing.ui.app.actions.settings_rows.pose import Pose
from mhseals_testing.ui.app.actions.settings_rows.status import StatusRows


__all__ = ['ADVISORY_PERIOD', 'HELD', 'Settings']


class Settings(Health, Files, Hardware, StatusRows, Pose):
    """Config, hardware and health, mixed into App."""

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
