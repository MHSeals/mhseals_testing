"""The hardware YAML: loading, applying (and the driver's confirmation), saving, the backed-up C and the epoch."""

import time

import yaml

from mhseals_hardware.configuration import config_sha, load_config, save_config, update_config
from mhseals_testing.model import Cancelled
from mhseals_testing.history import adapters
from mhseals_testing.ros.arming import session_hardware
from mhseals_testing.ui import keymap
from mhseals_testing.ui.popups import STOP_KEYS


class Hardware:
    """The hardware YAML, mixed into Settings."""

    def load_hardware(self, _path):
        return load_config(self.args.config)

    def apply_hardware(self, config):
        """Take ``config`` and report what the hardware did with it, not just that it was saved.

        Arming a remote thrusters host, the boat's hardware.yaml is the only config: a local edit or
        reload is refused and never replaces it (it is read from the boat at session start and each arm).
        """
        remote = self.arming.remote if self.arming is not None else None
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
            self.forget_manual_speed()
        if self.arming is not None:
            self.arming.hardware_config = config  # arming saves this copy
        if not driven:
            return dict(status='saved')
        sha = config_sha(load_config(self.paths.hardware))
        end = time.monotonic() + 4.0   # the driver polls the file at 1 Hz
        while True:
            status = self.ros.driver_status or {}
            if status.get('requested_config_sha') == sha and any(
                    name.endswith('/thruster_pwm_node') for name in self.ros.nodes()):
                break
            if time.monotonic() >= end:
                return dict(status='unknown', detail='driver did not confirm the edit')
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                time.sleep(0.1)
            if key in STOP_KEYS or keymap.matches('popup.pins_wait', 'stop', key, press=True):
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
        self.say(message)

    def observe_config(self):
        """At start: log config files edited since the last session as external changes."""
        adapters.observe_all(self.history, {'hardware': self.config_file}, self.warn_history)

    def save_hardware(self, **changes):
        """Write ``changes`` into the operator's hardware YAML; the watcher applies it.

        Returns False, keeping the draft, when the file on disk is broken, or when a remote boat's
        hardware.yaml is the config (refused before any write).
        """
        remote = self.arming.remote if self.arming is not None else None
        if remote is not None:
            self.say(f"Not saved: the boat's hardware.yaml is the config, edit it on {remote.host.name}")
            return False
        adapters.observe_file(self.history, 'hardware', self.config_file, self.warn_history)   # a hand edit first
        try:
            update_config(changes, self.args.config)
        except (OSError, ValueError, TypeError, yaml.YAMLError):
            self.say(f'Fix hardware.yaml first: {self.config_file}')
            return False
        adapters.record_file(self.history, 'hardware', 'tui:hardware', self.config_file, self.warn_history)
        change = self.announce(self.watcher.reload('hardware'))  # skip the settle wait
        if not change.ok:
            return False
        self.drop_grab(draft=True)
        return True

    def apply_pending(self):
        """A: put an interrupted run's backed-up C back. Disarmed, the driver is down and starts from the YAML,
        which still holds it; armed, it goes to the running driver and is read back."""
        staging = self.staging()
        if staging.pending() is None:
            self.say('Nothing pending')
            return
        try:
            staging.restore() if self.armed else staging.recover()
        except Cancelled as error:
            self.say(f'C not restored: {error}')
            return
        self.say('C restored' + ('' if self.armed else ' to the YAML (the driver starts from it)'))

    def hardware_changed(self):
        """I: a typed note, then a confirm, then hardware_epoch + 1 (invalidates every fit); never while moving."""
        if self.moving():
            self.say('Not while moving')
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
            self.say(f'Hardware changed: epoch {epoch}, calibrations need a refit')
            change = self.history.head('hardware')
            if change:
                self.history.label(change['id'], note=note)
