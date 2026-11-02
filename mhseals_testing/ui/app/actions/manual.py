"""Manual control and the Thrusters panel: pulses, the mapping and its pins."""

import time

from mhseals_hardware.thruster_mixer import IDENTITY, MAX_PWM, MIN_PWM, POSITIONS
from mhseals_testing.harness import operator_state
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ui import theme
from mhseals_testing.ui.popups import pins

from mhseals_testing.ui.app.help import MICRO, UPDOWN
from mhseals_testing.ui.app.shared import PULSE_STEP, decoupled


MANUAL_DESK_S = 0.2      # s; the agent desk's tick from inside manual control


NEUTRAL = 1500


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
        self.notice = 'Mapping unchanged' if unchanged else 'Unsaved mapping; s saves, u undoes'

    def on_thrusters(self, key):
        position = self.cursors['thrusters']
        channel_map, reversed_ = self.mapping()
        if key in ('left', 'right', 'h', 'l'):
            self.nudge(position, PULSE_STEP if key in ('right', 'l') else -PULSE_STEP)
        elif key == '0':
            self.clear_pulses('Test pulses neutral')
        elif key == 'enter':
            self.grabbed = None if self.grabbed is not None else position
            self.notice = ('Moving ' + POSITIONS[position].upper() + f'; {UPDOWN} moves it, '
                           'Enter drops' if self.grabbed is not None else 'Dropped')
        elif key in ('1', '2', '3', '4'):
            other = channel_map.index(int(key))
            channel_map[other], channel_map[position] = channel_map[position], int(key)
            self.edit_mapping(channel_map, reversed_)
        elif key == 'r':
            reversed_[position] = not reversed_[position]
            self.edit_mapping(channel_map, reversed_)
        elif key == 's':
            if self.draft is None:   # a pin change still unread: s retries the readback
                self.notice = pins.verify(self) if self.pins_unverified else 'Nothing to save'
                return
            moved = self.pins_moved()
            if moved and pins.busy(self):
                self.notice = pins.REFUSED
                return
            if moved:
                pins.mark_pending(self)   # before the write: a failed reload cannot arm
            chips = dict(pwm_chips=self.draft['pwm_chips']) if moved else {}
            if self.save_hardware(channel_map=channel_map, reversed=reversed_, **chips):
                self.notice = pins.verify(self) if moved else f'Mapping saved to {self.config_file.name}'
        elif key == 'u':
            self.draft = self.grabbed = None
            self.notice = 'Mapping changes undone'
        elif key == 'i':
            self.run_procedure(next(p for p in PROCEDURES if p.key == 'i'))
        elif key == 'c':
            if decoupled(self.args.hardware_config) is None:
                self.notice = 'No decoupling to clear'
            elif self.choose('Clear the fitted decoupling? Thrust goes back to the plain mixer.',
                             ('Keep', 'Clear'), {'n': 0, 'y': 1}) == 'Clear':
                if self.save_hardware(decoupling=list(sum(IDENTITY, ())), decoupling_negative=None):
                    self.notice = 'Decoupling cleared'

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
            self.notice = 'Arm first (a); test pulses go through the thruster driver'
            return
        conflicts = ([self.pin_refusal()] if self.pin_refusal() else []) + self.plant.conflicts()
        if conflicts:
            self.notice = f'Test pulses refused: {conflicts[0]}'
            return
        pulses = list(self.test_pulses or [NEUTRAL] * 4)
        output = self.mapping()[0][position] - 1  # the draft wiring, to check it before saving
        pulses[output] = max(MIN_PWM, min(MAX_PWM, pulses[output] + step))
        self.test_pulses = pulses
        self.ros.emit('test_pulses', pulses=pulses)
        self.notice = f'Testing output {output + 1}: {pulses[output]} {MICRO}s; 0 or Tab ends'

    def clear_pulses(self, notice=None):
        if self.test_pulses is not None:
            self.test_pulses = None
            if self.publish_pulses is not None:
                self.publish_pulses([NEUTRAL] * 4)
            self.ros.emit('test_pulses', pulses=[NEUTRAL] * 4)
        if notice:
            self.notice = notice

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
            self.test_pulses = list(pulses)
            if self.publish_pulses is not None:
                self.publish_pulses(self.test_pulses)

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
        if 'motion' in self.panels():
            self.focus = 'motion'
        active = ' '.join(f'{axis} {value:+.2f}' for axis, value in
                          zip(('surge', 'sway', 'yaw'), command) if value) or 'neutral'
        self.notice = ('Manual control ended' if done else
                       f"Manual {amplitude:.0%}{theme.SEPARATOR}{active}{theme.SEPARATOR}"
                       + ('Chords' if chords else 'Single key'))
        self.refresh(force=done)
