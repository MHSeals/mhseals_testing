"""Arming: the interlocks, the arm panel, arming and disarming, and stopping."""

import textwrap
import threading
import traceback

from mhseals_testing.model import Cancelled
from mhseals_testing.agent import grants
from mhseals_testing.devices import DeviceError
from mhseals_testing.profiles import drives_pwm_pins, outputs_description
from mhseals_testing.readiness import arm_nodes, arm_refusal, liveness_refusal, motion_blockers
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import input as key_input, keymap
from mhseals_testing.ui import theme, widgets
from mhseals_testing.ui.popups import STOP_KEYS, pins

from mhseals_testing.ui.app.shared import STOPPING_ERRORS, decoupled


MUX_SETTLE_S = 5.0       # s after arming for the mux's first state (it starts last, then 1 Hz)


FOREIGN_GRACE = 30.0     # s: a dead DDS participant lingers until its lease expires; later it counts as live


ARM_WRAP = 52  # arm-panel value column: the 72-wide popup less its label column


class Arm:
    """Arming, mixed into App."""

    def blockers(self):
        reasons = motion_blockers(self.snapshot(), self.profile, self.args.bench_override)
        if self.ros.fault:
            reasons.insert(0, 'ROS fault: ' + self.ros.fault)
        # A file backup is not evidence of the running driver correction, even during a run.
        return reasons + self.staging().blockers()

    def pin_refusal(self):
        """Arming waits for the sysfs readback after a pin change (s or M runs it)."""
        return pins.refusal(self)

    def pins_verified(self, ok):
        """A readback confirmed the saved pins (or there is none to read); False refuses arming until one does."""
        self.state.arm.pins_unverified = not ok

    def arm_blockers(self):
        """What the arm panel shows: the interlocks plus what arming itself would refuse."""
        rival = self.arming.rival() if self.arming and self.profile.outputs != 'none' else ''
        refusals = [reason for reason in (self.thruster_refusal(), self.pin_refusal(), self.watchdog_refusal())
                    if reason]
        return (self.blockers() + ([rival] if rival else []) + refusals
                + self.plant.conflicts() + self.foreign_blockers())

    def watchdog_refusal(self):
        """Why arming is refused for a safety watchdog that is not alive, or ''.

        The nodes come from the enabled health checks gating ``arm`` (the boat profile's
        pre.*_watchdog), read live; a mute does not lift them. Off the simulator an unreadable
        health config refuses too, since it may be what names them.
        """
        if self.stack is not None:
            self.stack.here(fresh=True)     # arming decides on a current process view
        return '; '.join(filter(None, self.watchdog_reasons()))

    def watchdog_reasons(self):
        """(graph, liveness): a required watchdog off the graph, and one whose own ``~/status``
        beat stopped or whose computer reports it off (liveness_refusal: within about 2 s, where
        the graph keeps a crashed node until its DDS lease ends)."""
        if self.health.error and self.profile.use_sim_time is not True:
            return f'health config unreadable, so the safety watchdogs are unchecked: {self.health.error}', ''
        required = arm_nodes(self.health.specs)
        if not required:
            return '', ''
        try:
            nodes = self.ros.nodes()
        except Exception as error:  # noqa: BLE001 - an unreadable graph refuses, never passes
            return f'node graph unreadable, so the safety watchdogs are unchecked: {error}', ''
        self.ros.watch_beats(required)
        names = {node.rsplit('/', 1)[-1] for node in required}
        off = []
        if self.stack is not None:
            off = [f'{item.title} on {item.remote or "this computer"}' for key, item in self.stack.items.items()
                   if names & set(item.nodes) and self.stack.reported(key) is False]
        return arm_refusal(required, nodes), liveness_refusal(required, self.ros.beats(), self.clock(), off,
                                                              runs=self.ros.runs())

    def thruster_refusal(self):
        """Why the real thruster driver may not start on this computer, or ''.

        Only a device-registry host with the ``thrusters`` capability drives real pins; a laptop
        or the camera host running a boat profile must not, except by arming that host remotely
        (ros/arm_link.py, set up at session start). An unreadable registry refuses too.
        """
        if not drives_pwm_pins(self.profile):
            return ''
        try:
            me = self.profile.placed[0]
        except DeviceError as error:
            return f'device registry: {error}'
        if 'thrusters' not in me.capabilities and self.arming.remote is None:
            return f'{me.name} has no thrusters capability in devices.yaml; the driver runs on the boat computer'
        return ''

    def foreign_instances(self):
        """Suite nodes on the graph that no live registry entry explains: (seen, confirmed live).

        A crashed instance leaves its DDS node for tens of seconds, so a newcomer is only an
        advisory for FOREIGN_GRACE; one still there after that is treated as live.
        """
        names = set(self.ros.instances())
        state = self.registry.state() if self.registry else {}
        known = {entry['node'] for entry in [state.get('owner') or {}, *state.get('guests', {}).values()]
                 if 'node' in entry}
        now = self.clock()
        seen = self.state.arm.foreign_seen = {name: self.state.arm.foreign_seen.get(name, now)
                                              for name in names - known}
        return sorted(seen), [name for name, since in seen.items() if now - since >= FOREIGN_GRACE]

    def foreign_blockers(self):
        return [f'another instance {name} is live' for name in self.foreign_instances()[1]]

    def supervision_blockers(self):
        """What must hold while anything drives the boat, the suite or Nav2."""
        if not self.armed:
            return ['outputs are not armed']
        fault = self.arming.fault()
        return ([fault] if fault else []) + self.blockers()

    def motion_blockers(self):
        """When the suite commands, nothing else may publish the command topic."""
        return self.supervision_blockers() + (self.plant.conflicts() if self.armed else [])

    def arm_rows(self):
        """What arming means right now: outputs, driver, config and every interlock."""
        remote = self.arming.remote if self.arming is not None else None
        outputs = outputs_description(self.profile, self.args.hardware_config['pwm_topic']) \
            + (f' on {remote.describe()}' if remote and drives_pwm_pins(self.profile) else '')
        driver = ('running' if self.armed else 'starts on arm') if self.profile.outputs != 'none' \
            else 'none'
        rows = [(widgets.SECTION, 'Will drive'), ('Outputs', outputs),
                ('Commands', f'{self.profile.command_topic} ({self.profile.command_units})'),
                ('Driver', driver), (widgets.SECTION, 'Setup'),
                ('Config', str(self.config_file)),
                ('Decoupling', theme.icon('off').plain + ' identity' if decoupled(self.args.hardware_config) is None
                 else theme.icon('ok').plain + ' fitted (Thrusters c clears)'), (widgets.SECTION, 'Checks')]
        blockers = [] if self.armed else self.arm_blockers()
        if not blockers:
            rows.append(('Interlocks', theme.icon('ok').plain + ' clear'))
        # Long reasons wrap onto continuation rows instead of being cut by the popup.
        for index, reason in enumerate(blockers[:4]):
            for number, line in enumerate(textwrap.wrap(reason, ARM_WRAP)):
                label = 'Interlocks' if not index and not number else ''
                rows.append((label, (theme.icon('bad').plain + ' ' if not number else '  ') + line))
        if self.args.bench_override:
            rows.append(('Override', 'bench: sensor and TF interlocks off'))
        return rows

    def confirm_drive(self, title):
        """The typed word (or, with ``hold: arm``, the hold) that starts the real driver, for arming and the
        pin readback alike."""
        message = 'Propellers submerged, area clear, physical power cutoff within reach.'
        if key_input.by_hold('arm'):
            return self.hold_confirm(title, 'arm', rows=self.arm_rows(), message=message, key='a', width=72,
                                     quiet=True)
        return self.type_word(title, message, 'arm', rows=self.arm_rows(), width=72, quiet=True)

    def arm_panel(self):
        """One popup for both directions; Space still stops without it.

        An unexpected error never ends the TUI: outputs go neutral (disarmed if it got that far) and the
        error shows as a fail toast, its traceback in logs/tui-errors.log. The usual stopping errors
        (a driver that fails to start) still reach the caller, which stops as before."""
        try:
            self._arm_panel()
        except STOPPING_ERRORS:
            raise
        except Exception as error:  # noqa: BLE001 - a bug in arming refuses and stays up, never exits
            self.arm_failed(error)

    def arm_failed(self, error):
        """Neutral outputs after an unexpected arming error, and say so in a toast."""
        text = f'Arming failed: {type(error).__name__}: {error}'
        try:
            self.paths.logs.mkdir(parents=True, exist_ok=True)
            with open(self.paths.logs / 'tui-errors.log', 'a') as log:
                log.write(traceback.format_exc())
        except OSError:
            pass
        try:
            if self.armed:
                self.disarm(text)
            else:
                if self.arming is not None and self.arming.pending is not None:
                    self.arming.cancel()
                if self.plant is not None:
                    self.plant.neutral()
                    self.plant.release()
        finally:
            self.toasts.push('arm.error', 'fail', text, self.monitor.clock())
            self.say(f'{text}; outputs neutral')

    def _arm_panel(self):
        if self.review:
            self.say('Review: arming is off')
            return
        if not self.source.live:
            self.say('Arming is disabled in review')
            return
        if self.watching:
            self.say(f'Watching pid {self.watching}: no arming')
            return
        title = f'Arm{theme.SEPARATOR}{self.profile.name}'
        try:
            if self.armed:
                if self.choose(f'Armed{theme.SEPARATOR}{self.profile.name}',
                               ('Disarm', 'Close'), {'d': 0, 'c': 1}, rows=self.arm_rows(),
                               danger=True, width=72, quiet=True) == 'Disarm':
                    self.disarm()
                return
            if self.arming is None:
                raise RuntimeError('this session cannot arm (no ROS)')
            refusal = self.thruster_refusal() or self.pin_refusal() or self.watchdog_refusal()
            if refusal:
                self.say(f'Arming refused: {refusal}')
                return
            if drives_pwm_pins(self.profile):
                ok = self.confirm_drive(title)
            elif key_input.by_hold('arm'):
                ok = self.hold_confirm(title, 'arm', rows=self.arm_rows(), key='a', width=72, quiet=True)
            else:
                ok = self.choose(title, ('Cancel', 'Arm'), {'n': 0, 'y': 1},
                                 rows=self.arm_rows(), width=72, quiet=True) == 'Arm'
        except Cancelled:
            ok = False
        if not ok:
            self.say('Arming cancelled')
            return
        refusal = self.watchdog_refusal()      # one may have gone while the operator typed
        if refusal:
            self.say(f'Arming refused: {refusal}')
            return
        # Another /cmd_vel source (Nav2) doesn't stop arming: Nav2 procedures need it, and
        # suite-commanded motion refuses to run beside it on every tick (motion_blockers).
        arbiter = self.arbiter if muxed(self.profile) else None
        since = arbiter.clock() if arbiter is not None else None
        self.wait_for_driver()
        if self.plant is not None:
            self.plant.engage()
        self.state.arm.armed_by = None           # the operator's arm; an agent's sets {'who', 'sim'}
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware))
        self.say('Armed at neutral')
        if arbiter is not None:
            self.settle_mux(arbiter, since)

    def agent_arm(self, who, checks, sim=True):
        """An agent's arm: the arm panel's path, its popup replaced by the desk's sim checks (``sim``) or by
        ``boat_agent arm``'s countdown banner (ui/feeds/agent_ops.py).

        Only the desk calls it, after its checks passed. The panel's refusals and
        driver start are the same; ``armed`` is journaled with ``by: agent``, who and the checks passed.
        A hold the mux restores is never released: arming is undone and the hold named. Returns ''
        when armed with the mux on auto, else why not (and disarmed).
        """
        if self.review or not self.source.live or self.watching or self.arming is None:
            return 'this session cannot arm'
        refusal = self.thruster_refusal() or self.pin_refusal() or self.watchdog_refusal()
        if refusal:
            return refusal
        arbiter = self.arbiter if muxed(self.profile) else None
        if arbiter is None:
            return 'no command mux to arm through'
        since = arbiter.clock()
        try:
            self.wait_for_driver()
        except (Cancelled, RuntimeError) as error:
            return str(error) or 'arming cancelled'
        if self.plant is not None:
            self.plant.engage()
        self.state.arm.armed_by = {'who': who, 'sim': sim}   # the ARMED chip reads ◆ who [sim-arm]; disarm clears it
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware), by='agent', who=who, sim_arm=list(checks))
        what = 'sim auto-arm' if sim else 'agent arm'
        self.say(f'Armed for agent {who} ({what})')
        why = ''
        if not self.await_mux(arbiter, since, lambda mux: True):
            why = 'the mux did not report after arming'
        elif self.override_cause():
            with arbiter.lock:
                mux = arbiter.mux or {}
            # the holds as status names them (.authority.record.holds), not the mux state's cause, and who releases each
            why = 'the mux holds ' + grants.holds_text(mux.get('holds'), mux.get('cause') or self.override_cause())
        if why:
            self.disarm(f'{what} undone: {why}', by='agent')
        return why

    def settle_mux(self, arbiter, since):
        """After arming, wait for the mux's first state; a hold it restored is offered for release.

        The mux starts with the holds it saved (a Space before the disarm), so whatever drives
        next would meet a stale or held mux. Only the human releases, through U's own confirm.
        """
        if not self.await_mux(arbiter, since, lambda mux: True):
            self.say('Armed; the mux has not reported yet')
        elif self.override_cause() == 'hold':
            since = arbiter.clock()
            if self.release_hold():
                self.await_mux(arbiter, since, lambda mux: mux.get('input') != 'hold')

    def await_mux(self, arbiter, since, done, timeout=MUX_SETTLE_S):
        """Repaint until the mux publishes a state after ``since`` that ``done(mux)`` accepts.

        False on timeout or Esc (keymap popup.arming cancel); Space stops the boat as everywhere. Each pass waits
        0.1 s."""
        for _ in range(round(timeout / 0.1)):
            with arbiter.lock:
                mux, at = arbiter.mux, arbiter.mux_at
            if mux is not None and at is not None and at > since and done(mux):
                return True
            self.refresh()
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                self.arming.sleep(0.1)
            if key in STOP_KEYS:
                self.operator_stop()
                return False
            if keymap.matches('popup.arming', 'cancel', key, press=True):
                return False
        return False

    def arm(self):
        self.arm_panel()

    def wait_for_driver(self):
        """Start the driver and show it starting; Space or Esc cancels.

        The view keeps redrawing while the driver comes up, so arming reads as
        loading, never as a frozen screen.
        """
        self.state.arm.ever_armed = True
        if self.arming.remote is None:
            self.arming.begin()
        else:                                    # the ssh read ran off the key loop; None refuses
            self.arming.begin(self.read_boat_config())
        started = self.clock()
        while self.arming.poll() == 'starting':  # raises when the driver fails
            self.say(f'Arming: starting the thruster driver '
                     f'{self.clock() - started:.0f} s{theme.SEPARATOR}Space cancels')
            self.refresh()
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                self.arming.sleep(0.1)
            if key in STOP_KEYS or keymap.matches('popup.arming', 'cancel', key, press=True):
                self.arming.cancel()
                raise Cancelled('arming cancelled')

    def read_boat_config(self):
        """A remote arm's ssh read of the boat's hardware.yaml, off the key loop (up to ~16 s on a bad
        link): the view redraws, Space stops, E revokes and Esc cancels meanwhile. None without a remote
        boat; a failed read leaves None and ``begin`` refuses with the reason."""
        remote = self.arming.remote
        if remote is None:
            return None
        box = {}
        thread = threading.Thread(target=lambda: box.update(config=remote.fetch_config()),
                                  name='boat-config', daemon=True)
        thread.start()
        started = self.clock()
        while thread.is_alive():
            self.say(f'Arming: reading the boat config on {remote.host.name} '
                     f'{self.clock() - started:.0f} s{theme.SEPARATOR}Esc cancels')
            self.refresh()
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                thread.join(0.1)
            if key in STOP_KEYS:
                self.operator_stop()
                raise Cancelled('arming cancelled')
            if keymap.matches('popup.arming', 'cancel', key, press=True):
                raise Cancelled('arming cancelled')
            if key == keymap.REVOKE_KEY:
                self.revoke_agent()
        return box.get('config')

    def disarm(self, reason='disarmed', by='operator'):
        self.clear_pulses()
        if self.plant is not None:
            self.plant.neutral()
            self.plant.release()
        self.set_command()
        if self.armed:
            self.arming.disarm()
            self.state.arm.armed_by = None
            self.watcher.watches['hardware'].warning = None
            self.ros.emit('disarmed', reason=reason, by=by)
        self.say(f'{reason}; outputs neutral')

    def operator_stop(self, agent=None):
        """Space requests a sticky manual hold, then always neutralizes and disarms. ``agent`` is the id of an
        agent whose ``stop`` this is (None: the operator's Space); ``stopped_by`` keeps it for the desk's hints."""
        self.set_command()
        stopped_by = self.state.arm.stopped_by = f'agent {agent}' if agent \
            else f'viewer {self.client_name()}' if self.watching else 'operator'
        try:
            if self.plant is not None:
                self.plant.hold('keys', f'view {self.client_name()}') if self.watching else self.plant.hold('keys')
        finally:
            self.stop(f'stopped by {stopped_by}')

    def stop(self, reason, by='operator'):
        """Neutral and disarmed, whatever happened; ``by`` is operator, fault or error.

        A watcher publishes neutral too: Space always works.
        """
        if self.services.procedures.paused:
            self.cancel_run(f'by {by}: {reason}')
        try:
            self.disarm(reason, by)
        finally:
            self.monitor.event('STOP: ' + reason)
