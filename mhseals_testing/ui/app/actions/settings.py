"""Config, hardware and health: watching files, applying edits and the Status panel.

The row groups live in settings_rows/ (health, files, hardware, status, pose, correction); this module composes
them and keeps the command-mode switch and the one setting writer.
"""

from mhseals_testing.harness import settings
from mhseals_testing.ros.arming import command_profile

from mhseals_testing.ui.app.actions.settings_rows.correction import Correction
from mhseals_testing.ui.app.actions.settings_rows.files import HELD, Files
from mhseals_testing.ui.app.actions.settings_rows.hardware import Hardware
from mhseals_testing.ui.app.actions.settings_rows.health import ADVISORY_PERIOD, Health
from mhseals_testing.ui.app.actions.settings_rows.pose import Pose
from mhseals_testing.ui.app.actions.settings_rows.status import StatusRows


__all__ = ['ADVISORY_PERIOD', 'HELD', 'Settings']


class Settings(Health, Files, Hardware, StatusRows, Pose, Correction):
    """Config, hardware and health, mixed into App."""

    def switch_command_mode(self, mode):
        """Hot-switch the command path (D-20261101-06): this session takes the profile its start would build for
        ``mode``, so procedures, the view and the next arm (driver alone, or the velocity controller in front)
        follow at once, no restart. Only disarmed, idle and with the last arm's children gone; the error, else ''."""
        base = getattr(self.args, 'base_profile', None)
        if base is None or self.arming is None or self.review is not None or self.watching:
            return (f'command_mode stays {self.settings.command_mode}, not {mode}: this session (a review, a viewer '
                    'or one that never arms) has no command path to switch; set it on the TUI that drives the boat, '
                    f'or start this one with --set command_mode={mode}')
        if self.armed or self.running or self.manual:
            return (f'command_mode stays {self.settings.command_mode}, not {mode}: it switches only disarmed and '
                    'between runs; allowed once status .armed is false and no run or manual drive is on')
        try:
            profile = command_profile(base, mode)
            self.arming.switch_profile(profile)
        except (RuntimeError, ValueError) as error:
            return str(error)
        self.rebuild(profile=profile)
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
                self.say(f'command_mode not switched: {error}')
                return error
        setattr(self.settings, key, value)
        settings.save(self.settings)
        self.watcher.reload('settings')
        self.emit('setting', key=key, old=old, value=value, by='agent' if who else 'operator', who=who)
        if who:
            self.say(f'agent {who} set {key}={value}')
            self.toasts.push(f'agent.setting.{key}', 'warn', self.notice, self.monitor.clock())
