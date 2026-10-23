"""boat_test: the interactive test suite, one page after cx.

Starts passive: it watches sensors, TF and advisories and records the whole
session to one bag. Five panels take focus (Tab): Status, Motion, Thrusters,
Procedures, Activity. Motion needs an explicit arm (a, confirmed in a popup)
and passes the interlocks in readiness.py on every tick; any error or Space
stops at neutral and disarms. Prompts are popups on the same screen.
"""

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import socket
import subprocess
import time

import yaml
from rich.console import Console
from rich.live import Live

from mhseals_hardware.configuration import config_path, configure_args, load_config, save_config, update_config
from mhseals_hardware.fcu import default_fcu_url
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.menu import Cancelled
from mhseals_hardware.thruster_mixer import IDENTITY, MAX_PWM, MIN_PWM, validate_mixer
from mhseals_testing import health
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.procedures import PROCEDURES, Context, Settings, unavailable
from mhseals_testing.profiles import available_profiles, load_profile
from mhseals_testing.readiness import motion_blockers
from mhseals_testing.report import build_report, write_report
from mhseals_testing.ros.arming import EFFORT_TOPIC
from mhseals_testing.runner import Aborted, Runner, fold
from mhseals_testing.ui import dashboard, theme
from mhseals_testing.ui.dashboard import PANELS, WIDE, status_rows

# Ctrl+h/j/k/l and Ctrl+arrows move focus to the panel in that direction, by layout:
#   wide:   Status | Motion     | Thrusters        narrow:  Status | Thrusters
#                  | Procedures | Activity                         | Procedures
#                                                                  | Activity
DIRECTIONS = {'ctrl-h': 'left', 'ctrl-left': 'left', 'ctrl-l': 'right', 'ctrl-right': 'right',
              'ctrl-k': 'up', 'ctrl-up': 'up', 'ctrl-j': 'down', 'ctrl-down': 'down'}
NEIGHBOURS = {
    True: {'status': {'right': 'motion'},
           'motion': {'left': 'status', 'right': 'thrusters', 'down': 'procedures'},
           'thrusters': {'left': 'motion', 'down': 'activity'},
           'procedures': {'left': 'status', 'right': 'activity', 'up': 'motion'},
           'activity': {'left': 'procedures', 'up': 'thrusters'}},
    False: {'status': {'right': 'thrusters'},
            'thrusters': {'left': 'status', 'down': 'procedures'},
            'procedures': {'left': 'status', 'up': 'thrusters', 'down': 'activity'},
            'activity': {'left': 'status', 'up': 'procedures'}},
}
ADVISORY_PERIOD = 1.0
FRAME = 1 / 30           # s; continuous redraws (motion, manual) stop at 30 per second
PULSE_STEP = 10          # µs per ←/→ in the Thrusters panel
NEUTRAL = 1500


def decoupled(config):
    """The active decoupling matrix, or None when it is the identity."""
    values = tuple(float(value) for value in config.get('decoupling', sum(IDENTITY, ())))
    return None if values == sum(IDENTITY, ()) else values


# Errors that stop a procedure: the plant goes neutral and outputs disarm.
STOPPING_ERRORS = (Aborted, Cancelled, RuntimeError, OSError, ValueError, KeyError)
GLOBAL_HINTS = [('Ctrl hjkl', 'Focus'), ('a', 'Arm panel'), ('Space', 'Stop'), ('?', 'Help'),
                ('q', 'Quit')]
PANEL_HINTS = {
    'status': [('↑↓', 'Select'), ('Enter', 'Details')],
    'motion': [('m', 'Manual'), ('+/-', 'Amplitude')],
    'thrusters': [('←→', 'Pulse'), ('Enter', 'Move'), ('1-4', 'Output'), ('r', 'Reverse'),
                  ('s', 'Save'), ('u', 'Undo'), ('0', 'Neutral'), ('i', 'Identify'),
                  ('c', 'Clear decoupling')],
    'procedures': [('Enter', 'Run'), ('↑↓', 'Select')],
    'activity': [('Enter', 'Details'), ('v', 'Results/log')],
}
MANUAL_HINTS = [('W/S ↑↓', 'Surge'), ('A/D', 'Sway'), ('←→', 'Yaw'), ('+/-', 'Speed'),
                ('Space', 'Stop'), ('x / Esc', 'Leave manual')]
HELP = [
    ('Ctrl h j k l / arrows', 'Focus the panel to the left, below, above, right'),
    ('Tab / Shift Tab', 'Cycle focus through the panels'),
    ('↑↓ / j k', 'Move within the focused panel'),
    ('Procedure keys', 'Run from any panel; focused-panel edit keys take precedence'),
    ('Enter', 'Run, open or show details'),
    ('a', 'Arm panel: outputs, interlocks, arm or disarm'),
    ('Space', 'Stop: neutral and disarm'),
    ('m', 'Manual control'),
    ('+ / -', 'Trial amplitude'),
    ('x', 'Write the session report'),
    ('Thrusters ←→', f'Test pulse ±{PULSE_STEP} µs on the selected thruster (armed)'),
    ('Thrusters 0', 'All test pulses back to neutral'),
    ('Thrusters Enter', 'Pick up the thruster; ↑↓ moves it, Enter drops it'),
    ('Thrusters 1-4 / r', 'Wire it to an output / reverse it (a draft until saved)'),
    ('Thrusters s / u', 'Save the draft to the config (hot-reloaded) / undo it'),
    ('Thrusters i', 'Thruster set: identify outputs by pulsing each one'),
    ('Activity v', 'Switch between results and the event log'),
    ('q', 'Quit: neutral, disarm, flush the bag'),
]


@dataclass(frozen=True)
class SessionPaths:
    bag: Path
    report: Path
    logs: Path
    hardware: Path  # snapshot of the hardware config the driver runs with
    nav2: Path      # Nav2 params with measured limits, written by characterization

    @classmethod
    def from_output(cls, output):
        output = Path(output)
        return cls(output, output.with_suffix('.report.json'),
                   output.parent / 'logs' / output.name,
                   output.with_suffix('.hardware.yaml'), output.with_suffix('.nav2.yaml'))


class App:
    """The page, its key handling and the procedure context.

    ROS, processes, arming, the plant, keys, the live view and the raw-pulse
    publisher are injected so tests drive the whole loop with fakes.
    """

    def __init__(self, args, profile, monitor, ros, processes, paths, console=None,
                 arming=None, plant=None, open_keys=KeyReader, live=Live,
                 clock=time.monotonic, foreground=subprocess.call, nav2=None,
                 task_watcher=None, pulses=None, open_events=None):
        self.args = args
        self.profile = profile
        self.monitor = monitor
        self.ros = ros
        self.processes = processes
        self.paths = paths
        self.console = console or Console()
        self.arming = arming
        self.plant = plant
        self.open_keys = open_keys
        self.live_factory = live
        self.clock = clock
        self.foreground = foreground
        self.publish_pulses = pulses
        # Manual control needs key releases (Kitty protocol) to tell held keys from taps.
        self.open_events = open_events or (lambda: KeyReader(report_events=True))
        self.manual = False
        self.nav2_factory, self._nav2 = nav2, None
        self.watcher_factory, self._watcher = task_watcher, None
        self.settings = Settings(args.amplitude, args.baseline, args.command_duration,
                                 args.settle, args.repeats, args.observe, args.nav_distance,
                                 args.station_hold, args.station_tolerance, args.turn)
        self.focus = 'status'
        self.cursors = {panel: 0 for panel in PANELS}
        self.activity = 'results'
        self.modal = None
        self.running = None
        self.test_pulses = None
        self.draft = None        # unsaved thruster mapping: channel_map and reversed
        self.grabbed = None      # thruster row being moved with ↑↓
        self.notice = 'Passive monitor; a opens the arm panel'
        self.command = (0.0, 0.0, 0.0)
        self.last_advisory = float('-inf')
        self.pwm_readback = [theme.DASH] * 4
        self.keys = self.live = None
        self.last_frame = 0.0
        self.quit = False
        self.closed = False
        self.started = clock()
        self.config_file = config_path(args.config)
        self.config_mtime = self.mtime()
        self.config_note = self.config_file.name if self.config_file.exists() else 'defaults'

    @property
    def armed(self):
        return self.arming is not None and self.arming.armed

    def snapshot(self):
        return self.monitor.snapshot(self.profile.required_tf)

    # --- interlocks ------------------------------------------------------------

    def blockers(self):
        reasons = motion_blockers(self.snapshot(), self.profile, self.args.bench_override)
        if self.ros.fault:
            reasons.insert(0, 'ROS fault: ' + self.ros.fault)
        return reasons

    def supervision_blockers(self):
        """What must hold while anything drives the boat, the suite or Nav2."""
        if not self.armed:
            return ['outputs are not armed']
        fault = self.arming.fault()
        return ([fault] if fault else []) + self.blockers()

    def manual_blockers(self):
        """Manual driving needs armed outputs and no other command source, not odometry or TF:
        the operator is watching the boat, and a localization fault must not take it away."""
        if not self.armed:
            return ['outputs are not armed']
        fault = self.arming.fault()
        reasons = [fault] if fault else []
        if self.ros.fault:
            reasons.append('ROS fault: ' + self.ros.fault)
        return reasons + self.plant.conflicts()

    def motion_blockers(self):
        """When the suite commands, nothing else may publish the command topic."""
        return self.supervision_blockers() + (self.plant.conflicts() if self.armed else [])

    # --- background work, every tick ---------------------------------------------

    def tick(self):
        self.refresh_advisories()
        self.watch_config()
        if self.test_pulses is not None and self.publish_pulses is not None:
            self.publish_pulses(self.test_pulses)
        if self.armed and self.arming.fault():
            self.stop(self.arming.fault())

    def refresh_advisories(self):
        """Sample the non-blocking checks at most once per ADVISORY_PERIOD."""
        if self.clock() - self.last_advisory < ADVISORY_PERIOD:
            return
        self.last_advisory = self.clock()
        snapshot = self.snapshot()
        if self.profile.use_sim_time and self.ros.now() == 0:
            # Without /clock every stamp looks stale; say why instead.
            self.monitor.advise('clock', 'FAIL', 'use_sim_time but no /clock received')
        else:
            self.monitor.advise('clock', *health.clock(snapshot))
        self.monitor.advise('compute', *health.read_compute(self.paths.bag.parent))
        self.monitor.advise('recording', 'OK' if self.processes.running('bag') else 'FAIL',
                            str(self.paths.bag))
        exited = [name for name in self.processes.exited() if name != 'bag']
        if exited:
            self.monitor.advise('processes', 'WARN', 'exited: ' + ', '.join(exited))
        if self.profile.outputs != 'none':
            topic = EFFORT_TOPIC if self.profile.velocity_controller else self.profile.command_topic
            self.monitor.advise('PWM driver', *health.pwm_driver(
                self.ros.nodes(), self.ros.subscribers(topic), topic,
                self.armed, self.processes.running('hardware')))
        if self.profile.outputs == 'odroid':
            config = self.args.hardware_config
            level, detail, self.pwm_readback = health.pwm_readback(
                config['pwm_chips'], config['pwm_channels'], self.armed, config['frequency'])
            self.monitor.advise('PWM readback', level, detail)

    # --- hardware config: edits from the panel or any editor, hot-reloaded -------

    def mtime(self):
        try:
            return self.config_file.stat().st_mtime_ns
        except OSError:
            return None

    def watch_config(self):
        mtime = self.mtime()
        if mtime == self.config_mtime:
            return
        self.config_mtime = mtime
        try:
            self.reload_config()
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:  # bad syntax or types
            self.notice = f'Config rejected, keeping the last good one: {error}'
            self.config_note = 'invalid edit'
            return
        self.notice = f'Config reloaded from {self.config_file.name}'

    def reload_config(self):
        self.args.hardware_config = load_config(self.args.config)
        self.config_note = self.config_file.name
        if self.arming is not None:
            self.arming.hardware_config = self.args.hardware_config  # arming saves this copy
        if self.armed and self.profile.outputs != 'none':
            # The driver hot-reloads its snapshot; keep it the session's config.
            save_config(dict(self.args.hardware_config, output_backend=self.profile.outputs),
                        self.paths.hardware)
        self.ros.emit('config_reloaded', path=str(self.config_file),
                      channel_map=list(self.args.hardware_config['channel_map']),
                      reversed=list(self.args.hardware_config['reversed']))

    def save_hardware(self, **changes):
        """Write ``changes`` into the operator's hardware YAML; the watcher applies it."""
        update_config(changes, self.args.config)
        self.draft = self.grabbed = None
        self.config_mtime = None  # reload on the next tick even within one mtime tick
        self.watch_config()

    # --- rendering ---------------------------------------------------------------

    def panels(self):
        return [p for p in PANELS if p != 'motion' or self.console.size.width >= WIDE]

    def rows(self, panel):
        if panel == 'status':
            return len(status_rows(self.snapshot(), self.profile.sensors))
        if panel == 'thrusters':
            return 4
        if panel == 'procedures':
            return len(PROCEDURES)
        if panel == 'activity':
            snapshot = self.snapshot()
            return len(snapshot['events' if self.activity == 'log' else 'results'])
        return 0

    def view(self):
        config = self.args.hardware_config
        blockers = self.motion_blockers() if self.armed else self.blockers()
        notice = self.notice if not blockers or self.running else (
            f'{self.notice}{dashboard.SEPARATOR}blocked: {blockers[0]}')
        count = self.rows(self.focus)
        hints = MANUAL_HINTS if self.manual else PANEL_HINTS[self.focus] + GLOBAL_HINTS
        return dict(
            focus=self.focus, cursors=self.cursors, modal=self.modal, running=self.running,
            activity=self.activity, now=self.monitor.clock(), notice=notice, hints=hints,
            position=f'{min(self.cursors[self.focus], count - 1) + 1}/{count}' if count else '',
            armed=self.armed, arm_label=('armed' + dashboard.SEPARATOR + self.profile.outputs)
            if self.armed else 'disarmed', command=self.command,
            profile=self.profile.name, units=self.profile.command_units,
            outputs=self.profile.outputs, specs=self.profile.sensors,
            host=socket.gethostname(), domain=os.environ.get('ROS_DOMAIN_ID', '0'),
            recording=self.processes.running('bag'), bench_override=self.args.bench_override,
            hardware=config, matrix=validate_mixer(config['thruster_matrix']),
            test_pulses=self.test_pulses, pwm_readback=self.pwm_readback,
            draft=self.draft, grabbed=self.grabbed,
            config_note=self.config_note,
            procedures=[dict(key=p.key, title=p.title, description=p.description,
                             reason=self.availability(p)) for p in PROCEDURES])

    def render(self):
        return dashboard.render(self.snapshot(), self.view(),
                                self.console.size.width, self.console.size.height)

    def refresh(self, force=True):
        """Draw now. Continuous callers pass force=False to cap redraws at 1/FRAME."""
        if self.live is None:
            return
        now = time.monotonic()
        if not force and now - self.last_frame < FRAME:
            return
        self.last_frame = now
        self.live.update(self.render(), refresh=True)

    def export(self):
        report = build_report(self.profile, self.snapshot(), self.paths.bag,
                              self.args.hardware_config)
        path = write_report(report, self.paths.report)
        self.notice = f'Report saved: {path}'
        return path

    # --- the loop ----------------------------------------------------------------

    @contextmanager
    def attached(self):
        """One key reader and one full-screen live view for the whole session."""
        # No refresh thread: the loop draws right after each key, and nothing else writes
        # to the terminal between frames, so mode switches such as Kitty's reach it intact.
        with self.open_keys() as keys, self.live_factory(
                self.render(), console=self.console, screen=True, auto_refresh=False) as live:
            self.keys, self.live = keys, live
            try:
                yield self
            finally:
                self.keys = self.live = None

    def run(self):
        with self.attached():
            try:
                while not self.quit:
                    try:
                        key = self.keys.read(timeout=0.1)
                        if key is not None:
                            try:
                                self.dispatch(key)
                            except STOPPING_ERRORS as error:
                                self.stop(str(error) or type(error).__name__)
                        self.tick()
                        self.refresh(force=key is not None)
                    except KeyboardInterrupt:
                        # Ctrl+C quits like q, minus the question: anything moving has
                        # already stopped at neutral (the runner neutralizes on any exit).
                        self.quit = True
                        self.modal = None
                        self.notice = 'Interrupted'
            finally:
                self.close()  # on q and on Ctrl+C alike, while the page is still up

    # --- quitting ------------------------------------------------------------------

    def confirm_quit(self):
        """Say what quitting does, and offer to keep an unsaved mapping."""
        rows = [('Outputs', 'neutral, then disarm' if self.armed else 'already disarmed'),
                ('Recording', f'flush {self.paths.bag}'),
                ('Report', str(self.paths.report)),
                ('Results', str(len(self.snapshot()['results'])))]
        options = ['Quit', 'Cancel']
        if self.draft is not None:
            rows.append(('Mapping', 'unsaved changes'))
            options = ['Save mapping and quit', 'Quit without saving', 'Cancel']
        try:
            choice = self.choose('Quit', options, {'y': 0, 'n': len(options) - 1}, rows=rows,
                                 width=72)
        except Cancelled:
            return
        if choice == 'Cancel':
            return
        if choice == 'Save mapping and quit':
            channel_map, reversed_ = self.mapping()
            self.save_hardware(channel_map=channel_map, reversed=reversed_)
        self.quit = True

    def close(self):
        """Shut down in order, showing each step: nothing may drive, then flush, then report.

        Safe to call more than once; main() calls it too in case the page never opened.
        """
        if self.closed:
            return
        self.closed = True
        steps = [('Outputs', self.close_outputs), ('Children', self.close_children),
                 ('Report', self.close_report), ('Recording', self.close_recording)]
        rows = [[name, '◌ waiting'] for name, _ in steps]
        self.modal = dict(kind='message', title='Closing session', rows=rows, width=64,
                          hint='Ctrl+C again skips the rest')
        for row, (_, step) in zip(rows, steps):
            row[1] = theme.icon('busy').plain + ' working'
            self.refresh()
            try:
                row[1] = theme.DOT + ' ' + step()
            except Exception as error:  # keep going: later steps still matter
                row[1] = f"{theme.icon('bad').plain} {error}"
        self.refresh()
        self.modal = None

    def close_outputs(self):
        was_armed = self.armed
        self.stop('session ended')
        return 'neutral and disarmed' if was_armed else 'were disarmed'

    def close_children(self):
        names = [name for name in list(self.processes.processes)
                 if name != 'bag'] if hasattr(self.processes, 'processes') else []
        for name in names:
            self.processes.stop(name)
        return f"stopped {', '.join(names)}" if names else 'none running'

    def close_report(self):
        self.ros.emit('session_stop')
        return self.export().name

    def close_recording(self):
        if not self.processes.running('bag'):
            return 'was not running'
        self.processes.stop('bag', timeout=20)  # SIGINT lets ros2 bag finish the file
        return f'flushed {self.paths.bag.name}'

    def summary(self):
        """What the session left behind, printed once the page is gone."""
        results = self.snapshot()['results']
        passed = sum(1 for r in results if dashboard.theme.kind(r.get('verdict', '')) == 'good')
        minutes = (self.clock() - self.started) / 60
        return [('Duration', f'{minutes:.1f} min'), ('Results', f'{passed}/{len(results)} good'),
                ('Bag', str(self.paths.bag)), ('Report', str(self.paths.report))]

    def focus_on(self, panel):
        if self.focus == 'thrusters' and panel != 'thrusters':
            self.clear_pulses()  # test pulses only while their panel has focus
        self.focus = panel

    def move(self, step):
        count = self.rows(self.focus)
        if count:
            self.cursors[self.focus] = max(0, min(count - 1, self.cursors[self.focus] + step))

    def dispatch(self, key):
        """Apply one key on the page."""
        panels = self.panels()
        if key in ('\t', 'backtab'):
            index = panels.index(self.focus) if self.focus in panels else 0
            self.focus_on(panels[(index + (-1 if key == 'backtab' else 1)) % len(panels)])
        elif key in DIRECTIONS:
            wide = 'motion' in panels
            neighbour = NEIGHBOURS[wide].get(self.focus, {}).get(DIRECTIONS[key])
            if neighbour:
                self.focus_on(neighbour)
        elif key in ('up', 'down', 'j', 'k'):
            step = -1 if key in ('up', 'k') else 1
            if self.focus == 'thrusters' and self.grabbed is not None:
                self.move_grabbed(step)
            else:
                self.move(step)
        elif key == 'space':
            self.stop('stopped by operator')
        elif key == 'q':
            self.confirm_quit()
        elif key == 'escape':  # backs out; never quits
            if self.grabbed is not None:
                self.grabbed = None
                self.notice = 'Dropped'
        elif key == '?':
            self.show('Help', rows=HELP, width=76)
        elif key == 'a':
            self.arm_panel()
        elif key == 'x':
            self.export()
        elif key in ('+', '=', '-', '_'):
            step = 0.05 if key in ('+', '=') else -0.05
            self.settings.amplitude = round(min(1.0, max(0.05, self.settings.amplitude + step)), 2)
            self.notice = f'Amplitude {self.settings.amplitude:g} {self.profile.command_units}'
        elif (key in {p.key for p in PROCEDURES}
              and not (self.focus == 'thrusters' and key in ('1', '2', '3', 'c', 's'))
              and not (self.focus == 'activity' and key == 'v')):
            self.on_procedures(key)
        else:
            getattr(self, f'on_{self.focus}')(key)

    # --- panel keys --------------------------------------------------------------

    def on_status(self, key):
        if key == 'enter':
            rows = status_rows(self.snapshot(), self.profile.sensors)
            if rows:
                row = rows[min(self.cursors['status'], len(rows) - 1)]
                self.show(row['name'], rows=[('Group', row['group']),
                                             ('State', row['state'].lower()),
                                             ('Detail', row['detail'])])

    def on_motion(self, key):
        pass

    def on_procedures(self, key):
        if key == 'enter':
            self.run_procedure(PROCEDURES[self.cursors['procedures']])
            return
        procedure = next((p for p in PROCEDURES if p.key == key), None)
        if procedure is not None:
            self.cursors['procedures'] = PROCEDURES.index(procedure)
            self.run_procedure(procedure)

    def on_activity(self, key):
        if key == 'v':
            self.activity = 'log' if self.activity == 'results' else 'results'
            self.cursors['activity'] = 0
        elif key == 'enter' and self.activity == 'results':
            results = list(reversed(self.snapshot()['results']))
            if results:
                result = results[min(self.cursors['activity'], len(results) - 1)]
                self.show(dashboard.result_label(result), width=76, rows=[
                    (name, value) for name, value in result.items()
                    if not isinstance(value, dict)])

    def mapping(self):
        """The thruster mapping being shown: the unsaved draft, else the saved config."""
        config = self.draft or self.args.hardware_config
        return list(config['channel_map']), list(config['reversed'])

    def edit_mapping(self, channel_map, reversed_):
        saved = self.args.hardware_config
        unchanged = (channel_map == list(saved['channel_map'])
                     and reversed_ == list(saved['reversed']))
        self.draft = None if unchanged else dict(channel_map=channel_map, reversed=reversed_)
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
            self.notice = ('Moving ' + dashboard.POSITIONS[position].upper() + '; ↑↓ moves it, '
                           'Enter drops' if self.grabbed is not None else 'Dropped')
        elif key in ('1', '2', '3', '4'):
            other = channel_map.index(int(key))
            channel_map[other], channel_map[position] = channel_map[position], int(key)
            self.edit_mapping(channel_map, reversed_)
        elif key == 'r':
            reversed_[position] = not reversed_[position]
            self.edit_mapping(channel_map, reversed_)
        elif key == 's':
            if self.draft is None:
                self.notice = 'Nothing to save'
                return
            self.save_hardware(channel_map=channel_map, reversed=reversed_)
            self.notice = f'Mapping saved to {self.config_file.name}'
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
                self.save_hardware(decoupling=list(sum(IDENTITY, ())))
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
        conflicts = self.plant.conflicts()
        if conflicts:
            self.notice = f'Test pulses refused: {conflicts[0]}'
            return
        pulses = list(self.test_pulses or [NEUTRAL] * 4)
        output = self.mapping()[0][position] - 1  # the draft wiring, to check it before saving
        pulses[output] = max(MIN_PWM, min(MAX_PWM, pulses[output] + step))
        self.test_pulses = pulses
        self.ros.emit('test_pulses', pulses=pulses)
        self.notice = f'Testing output {output + 1}: {pulses[output]} {theme.pick('µs', 'us')}; 0 or Tab ends'

    def clear_pulses(self, notice=None):
        if self.test_pulses is not None:
            self.test_pulses = None
            if self.publish_pulses is not None:
                self.publish_pulses([NEUTRAL] * 4)
            self.ros.emit('test_pulses', pulses=[NEUTRAL] * 4)
        if notice:
            self.notice = notice

    # --- popups --------------------------------------------------------------------

    def popup(self, modal, handle):
        """Show ``modal`` and feed it keys until ``handle(key)`` returns a value.

        Background work keeps ticking while the popup waits, so test pulses keep
        flowing and the dashboard behind it stays live.
        """
        self.modal = modal
        try:
            while True:
                self.refresh()
                key = self.keys.read(timeout=0.1)
                self.tick()
                if key is None:
                    continue
                result = handle(key)
                if result is not None:
                    return result
        finally:
            self.modal = None
            self.refresh()

    def show(self, title, message=None, rows=None, width=60):
        def handle(key):
            return True
        self.popup(dict(kind='message', title=title, message=message, rows=rows, width=width,
                        hint='Any key closes'), handle)

    def prompt(self, message, title='Ready'):
        """Wait for Enter; Escape cancels the procedure."""
        def handle(key):
            if key == 'enter':
                return ''
            if key in ('escape', 'q'):
                raise Cancelled('cancelled by operator')
        return self.popup(dict(kind='message', title=title, message=message,
                               hint=f'Enter continue{theme.SEPARATOR}Esc cancel'), handle)

    def choose(self, title, options, shortcuts=None, message=None, danger=False, rows=None,
               selected=0, width=64):
        modal = dict(kind='choice', title=title, message=message, options=list(options),
                     selected=selected, danger=danger, rows=rows, width=width,
                     hint='↑↓ select · Enter choose · Esc cancel', keys=[None] * len(options))
        for key, index in (shortcuts or {}).items():
            modal['keys'][index] = key

        def handle(key):
            key = fold(key)
            if key in ('up', 'k'):
                modal['selected'] = (modal['selected'] - 1) % len(options)
            elif key in ('down', 'j'):
                modal['selected'] = (modal['selected'] + 1) % len(options)
            elif key == 'enter':
                return options[modal['selected']]
            elif shortcuts and key in shortcuts:
                return options[shortcuts[key]]
            elif key in ('escape', 'q'):
                raise Cancelled('cancelled by operator')
        return self.popup(modal, handle)

    def type_word(self, title, message, word, rows=None, width=64):
        """A typed confirmation for real outputs: only the exact word proceeds."""
        modal = dict(kind='input', title=title, message=message, text='', danger=True,
                     rows=rows, width=width, hint=f'Type {word.upper()} and Enter{theme.SEPARATOR}Esc cancel')

        def handle(key):
            if key == 'enter':
                return modal['text'].strip().lower() == word.lower()
            if key == 'escape':
                return False
            if key in ('backspace', 'ctrl-h', '\x7f'):
                modal['text'] = modal['text'][:-1]
            elif len(key) == 1 and key.isprintable():
                modal['text'] += key.upper()
        return self.popup(modal, handle)

    # --- arming --------------------------------------------------------------------

    def arm_rows(self):
        """What arming means right now: outputs, driver, config and every interlock."""
        outputs = {'odroid': 'ODROID PWM pins (real thrusters)',
                   'topic': f'simulator over ROS ({self.args.hardware_config["pwm_topic"]})',
                   'none': 'commands straight to the plant'}[self.profile.outputs]
        driver = ('running' if self.armed else 'starts on arm') if self.profile.outputs != 'none' \
            else 'none'
        rows = [('Outputs', outputs), ('Driver', driver),
                ('Commands', f'{self.profile.command_topic} ({self.profile.command_units})'),
                ('Config', str(self.config_file)),
                ('Decoupling', theme.icon('off').plain + ' identity' if decoupled(self.args.hardware_config) is None
                 else theme.DOT + ' fitted (Thrusters c clears)')]
        blockers = self.blockers()
        rows.append(('Interlocks', theme.DOT + ' clear' if not blockers else f"{theme.icon('bad').plain} {blockers[0]}"))
        rows += [('', f"{theme.icon('bad').plain} {reason}") for reason in blockers[1:4]]
        if self.args.bench_override:
            rows.append(('Override', 'bench: sensor and TF interlocks off'))
        return rows

    def arm_panel(self):
        """One popup for both directions; Space still stops without it."""
        title = f'Arm{dashboard.SEPARATOR}{self.profile.name}'
        try:
            if self.armed:
                if self.choose(f'Armed{dashboard.SEPARATOR}{self.profile.name}',
                               ('Disarm', 'Close'), {'d': 0, 'c': 1}, rows=self.arm_rows(),
                               danger=True, width=72) == 'Disarm':
                    self.disarm()
                return
            if self.arming is None:
                raise RuntimeError('this session cannot arm (no ROS)')
            if self.profile.outputs == 'odroid':
                ok = self.type_word(title, 'Propellers submerged, area clear, physical power '
                                    'cutoff within reach.', 'arm', rows=self.arm_rows(), width=72)
            else:
                ok = self.choose(title, ('Cancel', 'Arm'), {'n': 0, 'y': 1},
                                 rows=self.arm_rows(), width=72) == 'Arm'
        except Cancelled:
            ok = False
        if not ok:
            self.notice = 'Arming cancelled'
            return
        # Another /cmd_vel source (Nav2) doesn't stop arming: Nav2 procedures need it, and
        # suite-commanded motion refuses to run beside it on every tick (motion_blockers).
        self.wait_for_driver()
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware))
        self.notice = 'Armed at neutral'

    def arm(self):
        self.arm_panel()

    def wait_for_driver(self):
        """Start the driver and show it starting; Space or Esc cancels.

        The view keeps redrawing while the driver comes up, so arming reads as
        loading, never as a frozen screen.
        """
        self.arming.begin()
        started = self.clock()
        while self.arming.poll() == 'starting':  # raises when the driver fails
            self.notice = (f'Arming: starting the thruster driver '
                           f'{self.clock() - started:.0f} s{theme.SEPARATOR}Space cancels')
            self.refresh()
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                self.arming.sleep(0.1)
            if key in ('space', 'escape', 'press:space', 'press:escape'):
                self.arming.cancel()
                raise Cancelled('arming cancelled')

    def disarm(self, reason='disarmed'):
        self.clear_pulses()
        if self.plant is not None:
            self.plant.neutral()
        self.command = (0.0, 0.0, 0.0)
        if self.armed:
            self.arming.disarm()
            self.ros.emit('disarmed', reason=reason)
        self.notice = f'{reason}; outputs neutral'

    def stop(self, reason):
        """Neutral and disarmed, whatever happened."""
        try:
            self.disarm(reason)
        finally:
            self.monitor.event('STOP: ' + reason)

    # --- procedures ---------------------------------------------------------------

    def availability(self, procedure):
        reason = unavailable(procedure, self.profile, self.armed, self.args.bench_override)
        # Procedures that measure motion can't be picked without odometry and TF.
        if not reason and 'sensors' in procedure.needs and not self.args.bench_override:
            blockers = motion_blockers(self.snapshot(), self.profile)
            if blockers:
                reason = blockers[0]
        return reason

    def run_procedure(self, procedure):
        reason = self.availability(procedure)
        if reason and 'armed' in procedure.needs and not self.armed:
            self.arm_panel()  # offer to arm, then carry on with the procedure
            reason = self.availability(procedure)
        if reason:
            self.notice = f'{procedure.title}: {reason}'
            return
        self.clear_pulses()
        self.running = procedure.title
        results = len(self.snapshot()['results'])
        self.notice = f'Running {procedure.title}'
        try:
            procedure.run(self.procedure_context())
        except Cancelled:
            self.notice = f'{procedure.title} cancelled'
            self.disarm(f'{procedure.title} cancelled')
            return
        finally:
            self.running = None
        if len(self.snapshot()['results']) > results:  # show what it found
            self.focus, self.activity = 'activity', 'results'
            self.cursors['activity'] = 0
            self.notice = f'{procedure.title} complete'

    def procedure_context(self):
        return Context(
            profile=self.profile, monitor=self.monitor, settings=self.settings,
            motion=self.motion, prompt=self.prompt, choose=self.choose, record=self.record,
            supervise=self.supervise, observe=self.observe, stale_after=self.args.stale_after,
            services=dict(
                nav2=self.nav2, task_watcher=self.task_watcher, start=self.processes.start,
                running=self.processes.running, stop=self.processes.stop,
                nav2_params=nav2_params(), nav2_overlay=self.paths.nav2,
                hardware_config=self.args.hardware_config, user_config=self.args.config,
                disarm=self.disarm, pulses=self.set_pulses, save_hardware=self.save_hardware,
                manual=dict(plant=self.plant, blockers=self.manual_blockers,
                            open_events=self.open_events, show=self.manual_show,
                            clock=self.monitor.clock)))

    def set_pulses(self, pulses):
        """Raw pulses per output for procedures (identification); None ends them."""
        if pulses is None:
            self.clear_pulses()
        else:
            self.test_pulses = list(pulses)
            if self.publish_pulses is not None:
                self.publish_pulses(self.test_pulses)

    def motion(self, name, **details):
        """A Runner that commands the plant, inside the live page."""
        return self.session(name, self.motion_blockers, details)

    def supervise(self, name, **details):
        """A Runner for watching Nav2 drive; the suite does not command meanwhile."""
        return self.session(name, self.supervision_blockers, details)

    def observe(self, name, **details):
        """A passive measurement window; no arming or readiness needed."""
        return self.session(name, lambda: [], details)

    @contextmanager
    def session(self, name, blockers, details):
        def show(command, remaining, label):
            self.command = command
            self.notice = f'{label or name}{dashboard.SEPARATOR}{remaining:.1f}s{theme.SEPARATOR}Space aborts'
            if self.test_pulses is not None and self.publish_pulses is not None:
                self.publish_pulses(self.test_pulses)
            self.refresh(force=False)

        runner = Runner(self.plant, blockers, self.keys, show=show,
                        emit=self.ros.emit, clock=self.monitor.clock)
        try:
            with runner.session(name, **details):
                yield runner
        finally:
            self.command = (0.0, 0.0, 0.0)

    def task_watcher(self):
        if self._watcher is None:
            if self.watcher_factory is None:
                raise RuntimeError('no task watcher in this session')
            self._watcher = self.watcher_factory()
        return self._watcher

    def nav2(self):
        if self._nav2 is None:
            if self.nav2_factory is None:
                raise RuntimeError('no Nav2 client in this session')
            self._nav2 = self.nav2_factory()
        return self._nav2

    def record(self, result):
        self.monitor.record_result(result)
        self.ros.emit('result', **result)
        self.export()

    def manual_show(self, command, amplitude, event_mode, done=False):
        """Redraw the page for manual control: Motion focused, manual keys in the footer."""
        self.manual = not done
        self.command = tuple(command)
        if 'motion' in self.panels():
            self.focus = 'motion'
        active = ' '.join(f'{axis} {value:+.2f}' for axis, value in
                          zip(('surge', 'sway', 'yaw'), command) if value) or 'neutral'
        self.notice = ('Manual control ended' if done else
                       f"Manual {amplitude:.0%}{dashboard.SEPARATOR}{active}{dashboard.SEPARATOR}"
                       + ('held keys' if event_mode else 'tap-repeat mode'))
        self.refresh(force=done)


def nav2_params():
    """mhseals_nav's checked-in Nav2 parameters, or None outside a ROS install."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_nav')) / 'config' / 'nav2_params.yaml'
    except (ImportError, KeyError):
        return None
    return path if path.exists() else None


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0], allow_abbrev=False)
    parser.add_argument('--profile', default='boat',
                        help=f'plant profile name ({", ".join(available_profiles())}) or YAML path')
    parser.add_argument('--velocity-control', action='store_true',
                        help='command m/s and rad/s through calibrated_velocity_controller')
    parser.add_argument('--start-stack', action='store_true',
                        help="launch the profile's measurement stack; default attaches to running topics")
    parser.add_argument('--fcu-url', default='',
                        help='MAVROS FCU URL for --start-stack (default: detect the flight controller)')
    parser.add_argument('--start-sensors', action='store_true',
                        help='with --start-stack, also start local sensor drivers')
    parser.add_argument('--bench-override', action='store_true',
                        help='allow motion with no sensors or TF (bench test, scoring disabled)')
    parser.add_argument('--stale-after', type=float, default=2.0)
    parser.add_argument('--amplitude', type=float, default=0.25,
                        help='trial size in the profile units (effort fraction or m/s, rad/s)')
    parser.add_argument('--baseline', type=float, default=5.0, help='neutral seconds before a trial')
    parser.add_argument('--command-duration', type=float, default=3.0)
    parser.add_argument('--settle', type=float, default=5.0, help='neutral seconds after a trial')
    parser.add_argument('--repeats', type=int, default=1, help='trials per direction (1-5)')
    parser.add_argument('--observe', type=float, default=10.0,
                        help='seconds of passive measurement for perception checks')
    parser.add_argument('--nav-distance', type=float, default=4.0,
                        help='side of the waypoint square, m')
    parser.add_argument('--station-hold', type=float, default=60.0, help='station keeping, s')
    parser.add_argument('--turn', type=float, default=20.0,
                        help='characterization: seconds of steady turning, so the fit can '
                             'tell current and wind from the boat itself')
    parser.add_argument('--station-tolerance', type=float, default=1.5,
                        help='station keeping radius, m')
    parser.add_argument('--output', type=Path, default=None,
                        help='session bag path (default bags/<profile>_<time>)')
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args = configure_args(parser, argv)  # adds --config and the merged hardware_config
    profile = load_profile(args.profile)
    if args.velocity_control:
        profile = profile.with_velocity_control()
    if args.start_stack and profile.bringup is None:
        parser.error(f'profile {profile.name} has no bring-up; start its stack separately')
    for name in ('stale_after', 'baseline', 'command_duration', 'settle', 'observe'):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= 60:
            parser.error(f"--{name.replace('_', '-')} must be within (0, 60] seconds")
    if not 0 < args.amplitude <= 1:
        parser.error('--amplitude must be within (0, 1]')
    if not 1 <= args.nav_distance <= 50 or not 10 <= args.station_hold <= 600 \
            or not 0.2 <= args.station_tolerance <= 10:
        parser.error('--nav-distance (1-50 m), --station-hold (10-600 s) and '
                     '--station-tolerance (0.2-10 m) are out of range')
    if not 1 <= args.repeats <= 5:
        parser.error('--repeats must be within 1..5')
    args.output = args.output or Path('bags') / (
        f"{profile.name.replace('+', '_')}_{datetime.now():%Y%m%d_%H%M%S}")
    return args, profile


def bringup_command(profile, args):
    arguments = dict(profile.bringup)
    if arguments.get('fcu') != 'false':
        # The flight controller's stable /dev/serial/by-id name when one is plugged in.
        arguments['fcu_url'] = args.fcu_url or default_fcu_url()
    if args.start_sensors:
        arguments['start_sensors'] = 'true'
    return ['ros2', 'launch', 'mhseals_testing', 'measurement.launch.py',
            *(f'{key}:={value}' for key, value in arguments.items())]


def main(argv=None):
    args, profile = parse_args(argv)
    # ROS imports stay here so the App (and its tests) never need rclpy.
    import rclpy

    from mhseals_testing.ros.arming import Arming
    from mhseals_testing.ros.health import subscribe_fcu
    from mhseals_testing.ros.mission import TaskWatcher
    from mhseals_testing.ros.nav2 import Nav2
    from mhseals_testing.ros.node import RosIO
    from mhseals_testing.ros.plant import Plant
    from mhseals_testing.ros.processes import ProcessManager, bag_command, unique_path

    from rclpy.signals import SignalHandlerOptions

    paths = SessionPaths.from_output(unique_path(args.output))
    console = Console()
    # rclpy's own SIGINT handler would shut ROS down before the graceful close could
    # send neutral and the final events; without it Ctrl+C is a KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    monitor = BoatMonitor(args.stale_after, history_sensors=tuple(
        sensor.name for sensor in profile.sensors if sensor.kind == 'odometry'))
    ros = RosIO(profile, monitor, args.stale_after)
    processes = ProcessManager(paths.logs)
    plant = Plant(ros, profile)
    arming = Arming(profile, ros, processes, args.hardware_config, paths.hardware)
    app = None
    try:
        if profile.bringup and profile.bringup.get('fcu') != 'false':
            subscribe_fcu(ros, monitor)
        paths.bag.parent.mkdir(parents=True, exist_ok=True)
        processes.start('bag', bag_command(paths.bag))
        if args.start_stack:
            time.sleep(2.0)  # let discovery find an already-running stack
            if ros.publishers(profile.odometry):
                raise RuntimeError(f'{profile.odometry} already has publishers; '
                                   'attach without --start-stack')
            processes.start('measurement', bringup_command(profile, args))
        app = App(args, profile, monitor, ros, processes, paths, console, arming, plant,
                  nav2=lambda: Nav2(ros, profile.map_frame or profile.odom_frame,
                                    profile.base_frame),
                  task_watcher=lambda: TaskWatcher(ros), pulses=plant.test_pulses)
        ros.emit('session_start', profile=profile.name, bag=str(paths.bag))
        app.run()
    except KeyboardInterrupt:  # before the page opened, or a second Ctrl+C while closing
        console.print('\n[yellow]Interrupted; stopping at neutral.[/]')
    finally:
        try:
            if app is not None:
                app.close()  # already done by run() unless the page never opened
            else:
                plant.neutral()
                arming.disarm()
        finally:
            plant.close()
            processes.stop_all()
            ros.close()
            rclpy.try_shutdown()
    if app is not None:
        table = dashboard.theme.grid({'header': '', 'style': f'bold {dashboard.ACCENT}'},
                                     {'header': '', 'ratio': 1}, padding=(0, 2))
        table.show_header = False
        for label, value in app.summary():
            table.add_row(label, value)
        console.print(dashboard.theme.block(table, 'Session closed', expand=False))


if __name__ == '__main__':
    main()
