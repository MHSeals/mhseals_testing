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
import shlex
import socket
import subprocess
import time

from rich.console import Console
from rich.live import Live

from mhseals_hardware.configuration import config_path, configure_args, load_config, save_config
from mhseals_hardware.fcu import default_fcu_url
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.menu import Cancelled
from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM, validate_mixer
from mhseals_testing import health
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.procedures import PROCEDURES, Context, Settings, unavailable
from mhseals_testing.profiles import available_profiles, load_profile
from mhseals_testing.readiness import motion_blockers
from mhseals_testing.report import build_report, write_report
from mhseals_testing.runner import Aborted, Runner
from mhseals_testing.ui import dashboard
from mhseals_testing.ui.dashboard import PANELS, WIDE, status_rows

ADVISORY_PERIOD = 1.0
PULSE_STEP = 10          # µs per ←/→ in the Thrusters panel
NEUTRAL = 1500
# Errors that stop a procedure: the plant goes neutral and outputs disarm.
STOPPING_ERRORS = (Aborted, Cancelled, RuntimeError, OSError, ValueError, KeyError)
GLOBAL_HINTS = [('Tab', 'Focus'), ('a', 'Arm'), ('Space', 'Stop'), ('?', 'Help'), ('q', 'Quit')]
PANEL_HINTS = {
    'status': [('↑↓', 'Select'), ('Enter', 'Details')],
    'motion': [('m', 'Manual'), ('+/-', 'Amplitude')],
    'thrusters': [('←→', 'Pulse'), ('0', 'Neutral'), ('r', 'Reverse'), ('1-4', 'Output'),
                  ('i', 'Identify'), ('e', 'Edit config')],
    'procedures': [('Enter', 'Run'), ('↑↓', 'Select')],
    'activity': [('Enter', 'Details'), ('v', 'Results/log')],
}
HELP = [
    ('Tab / Shift Tab', 'Move focus between panels'),
    ('↑↓ / j k', 'Move within the focused panel'),
    ('Enter', 'Run, open or show details'),
    ('a', 'Arm or disarm the thruster outputs'),
    ('Space', 'Stop: neutral and disarm'),
    ('m', 'Manual control'),
    ('+ / -', 'Trial amplitude'),
    ('x', 'Write the session report'),
    ('Thrusters ←→', f'Test pulse ±{PULSE_STEP} µs on the selected thruster (armed)'),
    ('Thrusters 0', 'All test pulses back to neutral'),
    ('Thrusters r / 1-4', 'Reverse it / wire it to an output (saved, hot-reloaded)'),
    ('Thrusters i / e', 'Identify outputs by pulses / edit the config in $EDITOR'),
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
                 task_watcher=None, pulses=None):
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
        self.nav2_factory, self._nav2 = nav2, None
        self.watcher_factory, self._watcher = task_watcher, None
        self.settings = Settings(args.amplitude, args.baseline, args.command_duration,
                                 args.settle, args.repeats, args.observe, args.nav_distance,
                                 args.station_hold, args.station_tolerance)
        self.focus = 'status'
        self.cursors = {panel: 0 for panel in PANELS}
        self.activity = 'results'
        self.modal = None
        self.running = None
        self.test_pulses = None
        self.notice = 'Passive monitor; a arms the outputs'
        self.command = (0.0, 0.0, 0.0)
        self.last_advisory = float('-inf')
        self.pwm_readback = ['—'] * 4
        self.keys = self.live = None
        self.quit = False
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
        if self.profile.outputs == 'odroid':
            config = self.args.hardware_config
            level, self.pwm_readback = health.pwm_readback(config['pwm_chips'],
                                                           config['pwm_channels'])
            self.monitor.advise('PWM readback', level, ' | '.join(self.pwm_readback))

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
        except (OSError, ValueError, KeyError) as error:
            self.notice = f'Config rejected, keeping the last good one: {error}'
            self.config_note = 'invalid edit'
            return
        self.notice = f'Config reloaded from {self.config_file.name}'

    def reload_config(self):
        self.args.hardware_config = load_config(self.args.config)
        self.config_note = self.config_file.name
        if self.armed and self.profile.outputs != 'none':
            # The driver hot-reloads its snapshot; keep it the session's config.
            save_config(dict(self.args.hardware_config, output_backend=self.profile.outputs),
                        self.paths.hardware)
        self.ros.emit('config_reloaded', path=str(self.config_file),
                      channel_map=list(self.args.hardware_config['channel_map']),
                      reversed=list(self.args.hardware_config['reversed']))

    def save_hardware(self, **changes):
        """Write ``changes`` into the operator's hardware YAML; the watcher applies it."""
        config = dict(load_config(self.args.config), **changes)
        save_config(config, self.args.config)
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
        hints = PANEL_HINTS[self.focus] + [hint for hint in GLOBAL_HINTS
                                           if not (hint[0] == 'a' and self.focus == 'thrusters')]
        hints = [('a', 'Disarm') if hint == ('a', 'Arm') and self.armed else hint
                 for hint in hints]
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
            config_note=self.config_note,
            procedures=[dict(key=p.key, title=p.title, description=p.description,
                             reason=self.availability(p)) for p in PROCEDURES])

    def render(self):
        return dashboard.render(self.snapshot(), self.view(),
                                self.console.size.width, self.console.size.height)

    def refresh(self):
        if self.live is not None:
            self.live.update(self.render())

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
        with self.open_keys() as keys, self.live_factory(
                self.render(), console=self.console, screen=True, refresh_per_second=10) as live:
            self.keys, self.live = keys, live
            try:
                yield self
            finally:
                self.keys = self.live = None

    @contextmanager
    def suspended(self):
        """Hand the terminal to another full-screen tool (manual control, $EDITOR)."""
        live, keys = self.live, self.keys
        live.stop()
        keys.__exit__(None, None, None)
        try:
            yield
        finally:
            keys.__enter__()
            live.start(refresh=True)

    def run(self):
        with self.attached():
            while not self.quit:
                key = self.keys.read(timeout=0.1)
                if key is not None:
                    try:
                        self.dispatch(key)
                    except STOPPING_ERRORS as error:
                        self.stop(str(error) or type(error).__name__)
                self.tick()
                self.refresh()

    def move(self, step):
        count = self.rows(self.focus)
        if count:
            self.cursors[self.focus] = max(0, min(count - 1, self.cursors[self.focus] + step))

    def dispatch(self, key):
        """Apply one key on the page."""
        panels = self.panels()
        if key in ('\t', 'backtab'):
            if self.focus == 'thrusters':
                self.clear_pulses()
            index = panels.index(self.focus) if self.focus in panels else 0
            self.focus = panels[(index + (-1 if key == 'backtab' else 1)) % len(panels)]
        elif key in ('up', 'k'):
            self.move(-1)
        elif key in ('down', 'j'):
            self.move(1)
        elif key == 'space':
            self.stop('stopped by operator')
        elif key in ('q', 'escape'):
            self.quit = True
        elif key == '?':
            self.show('Help', rows=HELP, width=76)
        elif key == 'a':
            if self.armed:
                self.disarm()
            else:
                self.arm()
        elif key == 'x':
            self.export()
        elif key in ('+', '=', '-', '_'):
            step = 0.05 if key in ('+', '=') else -0.05
            self.settings.amplitude = round(min(1.0, max(0.05, self.settings.amplitude + step)), 2)
            self.notice = f'Amplitude {self.settings.amplitude:g} {self.profile.command_units}'
        elif key == 'm':
            self.run_procedure(next(p for p in PROCEDURES if p.key == 'm'))
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

    def on_thrusters(self, key):
        position = self.cursors['thrusters']
        config = self.args.hardware_config
        if key in ('left', 'right', 'h', 'l'):
            self.nudge(position, PULSE_STEP if key in ('right', 'l') else -PULSE_STEP)
        elif key == '0':
            self.clear_pulses('Test pulses neutral')
        elif key == 'r':
            flips = list(config['reversed'])
            flips[position] = not flips[position]
            self.save_hardware(reversed=flips)
        elif key in ('1', '2', '3', '4'):
            mapping = list(config['channel_map'])
            output, other = int(key), mapping.index(int(key))
            mapping[other], mapping[position] = mapping[position], output  # swap
            self.save_hardware(channel_map=mapping)
        elif key == 'i':
            self.run_procedure(next(p for p in PROCEDURES if p.key == 'i'))
        elif key == 'e':
            self.edit_config()

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
        output = self.args.hardware_config['channel_map'][position] - 1
        pulses[output] = max(MIN_PWM, min(MAX_PWM, pulses[output] + step))
        self.test_pulses = pulses
        self.ros.emit('test_pulses', pulses=pulses)
        self.notice = f'Testing output {output + 1}: {pulses[output]} µs; 0 or Tab ends'

    def clear_pulses(self, notice=None):
        if self.test_pulses is not None:
            self.test_pulses = None
            if self.publish_pulses is not None:
                self.publish_pulses([NEUTRAL] * 4)
            self.ros.emit('test_pulses', pulses=[NEUTRAL] * 4)
        if notice:
            self.notice = notice

    def edit_config(self):
        if not self.config_file.exists():
            save_config(load_config(self.args.config), self.args.config)
        editor = os.environ.get('VISUAL') or os.environ.get('EDITOR') or 'vi'
        with self.suspended():
            self.foreground([*shlex.split(editor), str(self.config_file)])
        self.watch_config()

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
                               hint='Enter continue · Esc cancel'), handle)

    def choose(self, title, options, shortcuts=None, message=None, danger=False):
        modal = dict(kind='choice', title=title, message=message, options=list(options),
                     selected=0, danger=danger, hint='↑↓ select · Enter choose · Esc cancel',
                     keys=[None] * len(options))
        for key, index in (shortcuts or {}).items():
            modal['keys'][index] = key

        def handle(key):
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

    def type_word(self, title, message, word):
        """A typed confirmation for real outputs: only the exact word proceeds."""
        modal = dict(kind='input', title=title, message=message, text='', danger=True,
                     hint=f'Type {word.upper()} and Enter · Esc cancel')

        def handle(key):
            if key == 'enter':
                return modal['text'].strip().lower() == word.lower()
            if key == 'escape':
                return False
            if key == 'backspace' or key in ('\x7f', '\x08'):
                modal['text'] = modal['text'][:-1]
            elif len(key) == 1 and key.isprintable():
                modal['text'] += key.upper()
        return self.popup(modal, handle)

    # --- arming --------------------------------------------------------------------

    def arm(self):
        if self.arming is None:
            raise RuntimeError('this session cannot arm (no ROS)')
        # Another /cmd_vel source (Nav2) doesn't stop arming: Nav2 procedures need it, and
        # suite-commanded motion refuses to run beside it on every tick (motion_blockers).
        override = ('\nBench override: sensor and TF interlocks are off.'
                    if self.args.bench_override else '')
        title = f'Arm{dashboard.SEPARATOR}{self.profile.name}'
        try:
            if self.profile.outputs == 'odroid':
                ok = self.type_word(title, 'Real thruster outputs. Propellers submerged, area '
                                    'clear, physical power cutoff within reach.' + override, 'arm')
            else:
                what = ('Simulator thruster outputs over ROS.' if self.profile.outputs == 'topic'
                        else 'Commands straight to the plant.')
                ok = self.choose(title, ('Cancel', 'Arm'), {'n': 0, 'y': 1},
                                 message=what + override) == 'Arm'
        except Cancelled:
            ok = False
        if not ok:
            self.notice = 'Arming cancelled'
            return
        self.arming.arm()
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware))
        self.notice = 'Armed at neutral'

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
        return unavailable(procedure, self.profile, self.armed, self.args.bench_override)

    def run_procedure(self, procedure):
        reason = self.availability(procedure)
        if reason:
            self.notice = f'{procedure.title}: {reason}'
            return
        self.clear_pulses()
        self.running = procedure.title
        self.notice = f'Running {procedure.title}'
        try:
            procedure.run(self.procedure_context())
        except Cancelled:
            self.notice = f'{procedure.title} cancelled'
            self.disarm(f'{procedure.title} cancelled')
            return
        finally:
            self.running = None
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
                manual=dict(plant=self.plant, blockers=self.motion_blockers,
                            console=self.console, status=self.manual_status,
                            suspended=self.suspended)))

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
            self.notice = f'{label or name}{dashboard.SEPARATOR}{remaining:.1f}s · Space aborts'
            if self.test_pulses is not None and self.publish_pulses is not None:
                self.publish_pulses(self.test_pulses)
            self.refresh()

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

    def manual_status(self, label, amplitude, event_mode):
        self.notice = (f'Manual {amplitude:.0%} {label}{dashboard.SEPARATOR}'
                       + ('multi-key' if event_mode else 'single-key fallback'))
        self.focus = 'motion' if 'motion' in self.panels() else self.focus
        return self.render()


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

    paths = SessionPaths.from_output(unique_path(args.output))
    console = Console()
    rclpy.init()
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
    except KeyboardInterrupt:
        console.print('\n[yellow]Interrupted; stopping at neutral.[/]')
    finally:
        try:
            if app is not None:
                app.stop('session ended')
                ros.emit('session_stop')
                app.export()  # before the bag stops, so the report names a complete bag
            else:
                plant.neutral()
                arming.disarm()
        finally:
            plant.close()
            processes.stop_all()
            ros.close()
            rclpy.try_shutdown()
    console.print(f'Session bag: {paths.bag}\nReport: {paths.report}')


if __name__ == '__main__':
    main()
