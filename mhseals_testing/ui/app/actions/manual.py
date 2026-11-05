"""Manual control and the Thrusters panel: pulses, the mapping and its pins."""

import time

from mhseals_hardware.manual_state import BINDINGS
from mhseals_hardware.thruster_mixer import IDENTITY, MAX_PWM, MIN_PWM, POSITIONS
from mhseals_hardware.version import code_version
from mhseals_testing.harness import operator_state
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ui import keymap, theme
from mhseals_testing.ui import input as key_input
from mhseals_testing.ui.panels.thrusters import READBACK_TOLERANCE
from mhseals_testing.ui.popups import pins

from mhseals_testing.ui.app.help import MICRO, UPDOWN
from mhseals_testing.ui.app.shared import PULSE_STEP, decoupled


MANUAL_DESK_S = 0.2      # s; the agent desk's tick from inside manual control
READBACK_STALE_S = 1.0   # s; the driver's ~/output (10 Hz) older than this is no readback
READBACK_GRACE_S = 0.5   # s; a new pulse may take this long to show in the readback
NEUTRAL_REPEATS = 3      # copies of the ending neutral, as ros/plant.py sends, so one dropped message can't leave it on


NEUTRAL = 1500
MANUAL_ACTIONS = tuple(BINDINGS)   # keymap.yaml manual: actions ManualState drives by
WIRES = {'wire_1': 1, 'wire_2': 2, 'wire_3': 3, 'wire_4': 4}   # panel.thrusters: the output a row goes to


class Manual:
    """Manual control and the Thrusters panel, mixed into App."""

    def manual_blockers(self):
        """Manual driving needs armed outputs and no other command source, not odometry or TF:
        the operator is watching the boat, and a localization fault must not take it away."""
        now = self.manual_now()
        return now + self.manual_checks() if self.armed else now

    def manual_now(self):
        """The manual interlocks read from memory, cheap enough for every manual publish."""
        if not self.armed:
            return ['outputs are not armed']
        return ['ROS fault: ' + self.ros.fault] if self.ros.fault else []

    def manual_checks(self):
        """The manual interlocks that ask the graph or read files: manual evaluates them beside its key loop."""
        if not self.armed:
            return ['outputs are not armed']   # never a cached clear for an unarmed boat
        fault = self.arming.fault()
        return ([fault] if fault else []) + self.plant.conflicts() + self.staging().blockers()

    def manual_options(self):
        """ManualState's bindings and chord_hold, and the leave keys: keymap.yaml manual:, input.yaml tune.
        Read at each manual start, so an edit applies to the next one."""
        current = keymap.current()
        return dict(bindings={action: current.keys('manual', action) for action in MANUAL_ACTIONS},
                    chord_hold=key_input.load_file()['tune']['chord_hold'],
                    leave=tuple(current.keys('manual', 'leave')))

    def mapping(self):
        """The thruster mapping being shown: the unsaved draft, else the saved config."""
        config = self.draft or self.args.hardware_config
        return list(config['channel_map']), list(config['reversed'])

    def draft_chips(self):
        """The PWM chip path of each physical output: the draft's, else the saved config's."""
        return list({**self.args.hardware_config, **(self.draft or {})}['pwm_chips'])

    def pins_moved(self):
        return self.draft is not None and 'pwm_chips' in self.draft

    def edit_mapping(self, channel_map, reversed_, chips=None):
        saved = self.args.hardware_config
        chips = self.draft_chips() if chips is None else list(chips)
        moved = chips != list(saved['pwm_chips'])   # pwm_chips joins the draft only when it moves
        unchanged = (channel_map == list(saved['channel_map'])
                     and reversed_ == list(saved['reversed']) and not moved)
        self.draft = None if unchanged else dict(channel_map=channel_map, reversed=reversed_,
                                                 **(dict(pwm_chips=chips) if moved else {}))
        self.say('Mapping unchanged' if unchanged else 'Unsaved mapping; s saves, u undoes')

    def on_thrusters(self, key):
        position = self.cursors['thrusters']
        channel_map, reversed_ = self.mapping()
        action = keymap.dispatch(keymap.STACKS['drive/thrusters'], key)
        if action in ('pulse_down', 'pulse_up'):
            self.nudge(position, PULSE_STEP if action == 'pulse_up' else -PULSE_STEP)
        elif action == 'neutral':
            self.clear_pulses('Test pulses neutral')
        elif action == 'activate':
            self.grabbed = None if self.grabbed is not None else position
            self.say('Moving ' + POSITIONS[position].upper() + f'; {UPDOWN} moves it, '
                     'Enter drops' if self.grabbed is not None else 'Dropped')
        elif action in WIRES:
            output = WIRES[action]
            other = channel_map.index(output)
            channel_map[other], channel_map[position] = channel_map[position], output
            self.edit_mapping(channel_map, reversed_)
        elif action == 'reverse':
            reversed_[position] = not reversed_[position]
            self.edit_mapping(channel_map, reversed_)
        elif action == 'save':
            if self.draft is None:   # a pin change still unread: s retries the readback
                self.say(pins.verify(self) if self.pins_unverified else 'Nothing to save')
                return
            moved = self.pins_moved()
            if moved and pins.busy(self):
                self.say(pins.REFUSED)
                return
            if moved:
                pins.mark_pending(self)   # before the write: a failed reload cannot arm
            chips = dict(pwm_chips=self.draft['pwm_chips']) if moved else {}
            if self.save_hardware(channel_map=channel_map, reversed=reversed_, **chips):
                self.say(pins.verify(self) if moved else f'Mapping saved to {self.config_file.name}')
        elif action == 'undo':
            self.drop_grab(draft=True)
            self.say('Mapping changes undone')
        elif action == 'thruster_set':
            self.run_procedure(next(p for p in PROCEDURES if p.key == key))
        elif action == 'clear_decoupling':
            if decoupled(self.args.hardware_config) is None:
                self.say('No decoupling to clear')
            elif self.choose('Clear the fitted decoupling? Thrust goes back to the plain mixer.',
                             ('Keep', 'Clear'), {'n': 0, 'y': 1}) == 'Clear':
                if self.save_hardware(decoupling=list(sum(IDENTITY, ())), decoupling_negative=None):
                    self.say('Decoupling cleared')

    def move_grabbed(self, step):
        """Swap the grabbed thruster's output and reversal with its neighbour's, and follow it."""
        target = self.grabbed + step
        if not 0 <= target < 4:
            return
        channel_map, reversed_ = self.mapping()
        for values in (channel_map, reversed_):
            values[self.grabbed], values[target] = values[target], values[self.grabbed]
        self.grabbed = self.cursors['thrusters'] = target
        self.edit_mapping(channel_map, reversed_)

    def nudge(self, position, step):
        """Raw test pulse on one thruster's output, through the armed driver."""
        if not self.armed or self.profile.outputs == 'none':
            self.say('Arm first (a); test pulses go through the thruster driver')
            return
        conflicts = ([self.pin_refusal()] if self.pin_refusal() else []) + self.plant.conflicts()
        if conflicts:
            self.say(f'Test pulses refused: {conflicts[0]}')
            return
        pulses = list(self.test_pulses or [NEUTRAL] * 4)
        output = self.mapping()[0][position] - 1  # the draft wiring, to check it before saving
        pulses[output] = max(MIN_PWM, min(MAX_PWM, pulses[output] + step))
        self.test_pulses = pulses
        self.pulse_started()
        self.ros.emit('test_pulses', pulses=pulses)
        self.say(f'Testing output {output + 1}: {pulses[output]} {MICRO}s; 0 or Tab ends')

    def set_command(self, command=(0.0, 0.0, 0.0)):
        """The commanded (surge, sway, yaw) the Command glyph shows; neutral by default."""
        self.command = command

    def drop_grab(self, draft=False):
        """Let go of the grabbed thruster; ``draft`` also drops the unsaved mapping."""
        self.grabbed = None
        if draft:
            self.draft = None

    def forget_manual_speed(self):
        """A new configured manual_amplitude wins over the session's last speed."""
        self.manual_speed = None

    def clear_pulses(self, notice=None):
        if self.test_pulses is not None:
            self.test_pulses = None
            if self.publish_pulses is not None:
                for _ in range(NEUTRAL_REPEATS):   # one dropped message must not leave the pulse on
                    self.publish_pulses([NEUTRAL] * 4)
            self.ros.emit('test_pulses', pulses=[NEUTRAL] * 4)
        if notice:
            self.say(notice)

    def end_manual(self, procedure):
        """Back to normal when manual control (m) ends, if manual is still the one m set."""
        if procedure.key == 'm' and operator_state.load()['reason'] == 'manual control (m)':
            self.set_operator_state('normal', reason='manual control ended', notice=False)

    def mux_input(self):
        """The mux's latest input from the arbiter; 'auto' when no mux state has been seen."""
        mux = getattr(getattr(self, 'arbiter', None), 'mux', None)
        return str(mux.get('input')) if isinstance(mux, dict) and mux.get('input') else 'auto'

    def set_pulses(self, pulses):
        """Raw pulses per output for procedures (identification); None ends them."""
        if pulses is None:
            self.clear_pulses()
        else:
            if list(pulses) != self.test_pulses:
                self.pulse_started()
            self.test_pulses = list(pulses)
            if self.publish_pulses is not None:
                self.publish_pulses(self.test_pulses)

    def pulse_started(self):
        """A new test pulse: its grace starts, and what the driver last refused or ended is old news."""
        output = self.driver_output() or {}
        self.pulses_since, self.pulse_verdict = time.monotonic(), None
        self._pulse_baseline = {key: (output.get(key) or {}).get('t_ns') for key in ('refused', 'ended')}

    def driver_output(self):
        """The driver's latest ~/output (what its outputs hold, read back) while fresh, else None."""
        ros = getattr(self, 'ros', None)
        output, at = getattr(ros, 'driver_output', None), getattr(ros, 'driver_output_at', None)
        if output is None or at is None or time.monotonic() - at > READBACK_STALE_S:
            return None
        return output

    def pulse_readback(self):
        """Each tick: the Rb column from the driver's readback, and whether a test pulse reached the outputs.

        A test pulse that the outputs do not hold (after READBACK_GRACE_S), or with no readback at all,
        says so in the notice, with what they hold and who wrote it; so does a pulse the driver refused
        or ended (its reason: e-stop latched, mux not idle, guard stale), and a driver older than this TUI.
        """
        output = self.driver_output()
        held = output.get('pulses') if output else None
        if held and len(held) == 4:
            self.show_driver_readback(held)
        else:
            held = None
        pulses = self.test_pulses
        since = getattr(self, 'pulses_since', 0.0)
        if pulses is None or not self.armed or time.monotonic() - since < READBACK_GRACE_S:
            return
        tested = [index for index, pulse in enumerate(pulses) if pulse != NEUTRAL] or range(4)
        sent = ', '.join(f'{index + 1}: {pulses[index]}' for index in tested)
        news = {key: entry for key in ('refused', 'ended')
                if (entry := (output or {}).get(key))
                and entry.get('t_ns') != getattr(self, '_pulse_baseline', {}).get(key)
                and entry.get('reason') != 'ended by its sender'}
        stale = self.driver_staleness(output)
        if news.get('refused') and (output or {}).get('source') != 'test':
            verdict = f'Test pulse REFUSED by the driver: {news["refused"]["reason"]}; sent {sent} {MICRO}s'
        elif news.get('ended') and (output or {}).get('source') != 'test':
            verdict = f'Test pulse ENDED by the driver: {news["ended"]["reason"]}; sent {sent} {MICRO}s'
        elif not held:
            verdict = (f'Test pulse UNVERIFIED: no readback from the driver ({(output or {}).get("error") or "no ~/output"});'
                       f' sent {sent} {MICRO}s')
        elif all(abs(held[index] - pulses[index]) <= READBACK_TOLERANCE for index in range(4)):
            verdict = (f'Test pulse at the outputs: {sent} {MICRO}s, read back'
                       + ('' if getattr(self, 'running', None) else '; 0 or Tab ends'))
        else:
            verdict = (f'Test pulse NOT at the outputs: sent {sent} {MICRO}s, outputs hold '
                       f'{", ".join(f"{index + 1}: {held[index]}" for index in tested)} ({output.get("source")})')
        if stale and not verdict.startswith('Test pulse at'):
            verdict = f'{verdict}. {stale}'
        if verdict != getattr(self, 'pulse_verdict', None):
            self.pulse_verdict = verdict
            self.say(verdict)
            if not verdict.startswith('Test pulse at'):
                self.ros.emit('test_pulses_unverified', pulses=list(pulses), held=held,
                              source=(output or {}).get('source'),
                              reason=next((entry['reason'] for entry in news.values()), None), driver=stale or None)

    def driver_staleness(self, output):
        """'' when the driver runs this TUI's mhseals_hardware code, else what to do about it."""
        plant = getattr(self, 'plant', None)
        if getattr(plant, 'pulse_fallback', False):
            return ('driver is older than this TUI (it has no test_pulses/held): re-arm to load it, '
                    'and redeploy mhseals_hardware if that does not help')
        theirs = (output or {}).get('version')
        if output is None or not theirs:
            return ('driver is older than this TUI (its ~/output has no version): re-arm to load it'
                    if output is not None else '')
        ours = code_version()
        if theirs.get('digest') != ours['digest']:
            name = lambda v: f"{v.get('git') or '?'}{'+' if v.get('dirty') else ''}/{v.get('digest')}"  # noqa: E731
            return (f'driver runs other mhseals_hardware code (driver {name(theirs)}, this TUI {name(ours)}): '
                    're-arm to load it after a deploy')
        return ''

    def manual_show(self, command, amplitude, chords, done=False):
        """Redraw the page for manual control: Motion focused, manual keys in the footer."""
        self.manual = not done
        self.manual_speed = amplitude    # kept for the next time manual is enabled
        desk = getattr(self, 'desk', None)
        if desk is not None and not done and time.monotonic() - getattr(self, '_manual_desk_at', 0.0) >= MANUAL_DESK_S:
            # The key loop is manual's own: the desk keeps answering (motion refused, dry runs busy, no
            # Nav2 call) and status.json stays current, and an agent goal still live is cancelled.
            self._manual_desk_at = time.monotonic()
            desk.tick()
        self.command, self.chords = tuple(command), bool(chords)
        self.monitor.command(command)
        self.annotate_queued()        # marks typed while driving, journaled after this publish
        if 'motion' in self.panels():
            self.focus = 'motion'
        active = ' '.join(f'{axis} {value:+.2f}' for axis, value in
                          zip(('surge', 'sway', 'yaw'), command) if value) or 'neutral'
        self.say('Manual control ended' if done else
                 f"Manual {amplitude:.0%}{theme.SEPARATOR}{active}{theme.SEPARATOR}"
                 + ('Chords' if chords else 'Timed combos' + (
                     f' (no key releases: {why})' if (why := key_input.chords_unavailable(self.keys)) else '')))
        self.refresh(force=done)
