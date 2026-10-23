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
import errno
import os
from pathlib import Path
import signal
import socket
import subprocess
import textwrap
import time

import yaml
from rich.console import Console
from rich.live import Live

from mhseals_hardware.configuration import config_path, configure_args, load_config, save_config, update_config
from mhseals_hardware.fcu import default_fcu_url
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.menu import Cancelled
from mhseals_hardware.thruster_mixer import IDENTITY, MAX_PWM, MIN_PWM, validate_mixer
from mhseals_testing import nav2_limits
from mhseals_testing.data import data_root
from mhseals_testing.harness import host, journal, runs, settings
from mhseals_testing.harness.apply import Staging
from mhseals_testing.harness.components import Stack, components
from mhseals_testing.harness.conditions import Conditions
from mhseals_testing.harness.nav2_edit import Nav2File
from mhseals_testing.harness.registry import Registry
from mhseals_testing.harness.watch import Watched, Watcher
from mhseals_testing.health import legacy as health
from mhseals_testing.health.advisories import Health
from mhseals_testing.health.probes import Facts
from mhseals_testing.health.journal import Journal
from mhseals_testing.history import History, adapters
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.procedures import PROCEDURES, unavailable
from mhseals_testing.profiles import available_profiles, load_profile
from mhseals_testing.readiness import motion_blockers
from mhseals_testing.report import build_report, write_report
from mhseals_testing.ros.arming import EFFORT_TOPIC, command_input
from mhseals_testing.runner import Aborted, Runner, fold
from mhseals_testing.ui import dashboard, popups, theme
from mhseals_testing.ui.takeover import decide
from mhseals_testing.ui.dashboard import PANELS, WIDE, status_rows
from mhseals_testing.ui.layout import TabMemory
from mhseals_testing.ui.popups import odometry, pins
from mhseals_testing.ui.source import LiveSource

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
FOREIGN_GRACE = 30.0     # s: a dead DDS participant lingers until its lease expires; later it counts as live
FRAME = 1 / 30           # s; continuous redraws (motion, manual) stop at 30 per second
PULSE_STEP = 10          # µs per ←/→ in the Thrusters panel
NEUTRAL = 1500


class Detach(KeyboardInterrupt):
    """SIGUSR1: another instance is taking over. Unwinds like Ctrl+C, then hands the children on."""


def decoupled(config):
    """The active decoupling matrix, or None when both C+ and C- are the identity."""
    identity = sum(IDENTITY, ())
    values = tuple(float(value) for value in config.get('decoupling', identity))
    negative = config.get('decoupling_negative')
    if negative is not None and tuple(float(value) for value in negative) != identity:
        return values
    return None if values == identity else values


# Errors that stop a procedure: the plant goes neutral and outputs disarm.
STOPPING_ERRORS = (Aborted, Cancelled, RuntimeError, OSError, ValueError, KeyError)
UPDOWN, LEFTRIGHT = theme.pick('↑↓', 'Up/Down'), theme.pick('←→', 'Left/Right')
MICRO, PLUSMINUS = theme.pick('µ', 'u'), theme.pick('±', '+/-')
GLOBAL_HINTS = [('Ctrl hjkl', 'Focus'), ('a', 'Arm panel'), ('Space', 'Stop'), ('?', 'Help'),
                ('q', 'Quit')]
PANEL_HINTS = {
    'status': [(UPDOWN, 'Select'), ('Enter', 'Details')],
    'motion': [('m', 'Manual'), ('+/-', 'Amplitude')],
    'thrusters': [(LEFTRIGHT, 'Pulse'), ('Enter', 'Move'), ('1-4', 'Output'), ('r', 'Reverse'),
                  ('s', 'Save'), ('u', 'Undo'), ('0', 'Neutral'), ('i', 'Identify'),
                  ('c', 'Clear decoupling'), ('p', 'Pins')],
    'procedures': [('Enter', 'Run'), (UPDOWN, 'Select')],
    'activity': [('Enter', 'Details'), ('v', 'Results/log')],
}
MANUAL_HINTS = [(f'W/S {UPDOWN}', 'Surge'), ('A/D', 'Sway'), (LEFTRIGHT, 'Yaw'), ('+/-', 'Speed'),
                ('Space', 'Stop'), ('x / Esc', 'Leave manual')]
HELP = [
    ('Ctrl h j k l / arrows', 'Focus the panel to the left, below, above, right'),
    ('Tab / Shift Tab', 'Cycle focus through the panels'),
    (f'{UPDOWN} / j k', 'Move within the focused panel'),
    ('Procedure keys', 'Run from any panel; focused-panel edit keys take precedence'),
    ('Enter', 'Run, open or show details'),
    ('o (Status)', 'Odometry: what the EKFs fuse, and the sources'),
    ('a', 'Arm panel: outputs, interlocks, arm or disarm'),
    ('Space', 'Stop: neutral and disarm'),
    ('m', 'Manual control'),
    ('+ / -', 'Trial amplitude'),
    ('x', 'Write the session report'),
    ('b / B', 'Mark the moment (journal) / add a note to the last marker'),
    (f'Thrusters {LEFTRIGHT}',
     f'Test pulse {PLUSMINUS}{PULSE_STEP} {MICRO}s on the selected thruster (armed)'),
    ('Thrusters 0', 'All test pulses back to neutral'),
    ('Thrusters Enter', f'Pick up the thruster; {UPDOWN} moves it, Enter drops it'),
    ('Thrusters 1-4 / r', 'Wire it to an output / reverse it (a draft until saved)'),
    ('Thrusters s / u', 'Save the draft to the config (hot-reloaded) / undo it'),
    ('Thrusters p', 'Edit the PWM pins and MOSFET line; applied live, refused while armed'),
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
    journal: Path   # append-only events: markers, phases, results
    runs: Path      # one runs/<run_id>.json per procedure run

    @classmethod
    def from_output(cls, output):
        output = Path(output)
        return cls(output, output.with_suffix('.report.json'),
                   output.parent / 'logs' / output.name,
                   output.with_suffix('.hardware.yaml'), output.with_suffix('.nav2.yaml'),
                   output.with_suffix('.events.jsonl'), output.with_suffix('.runs'))


ARM_WRAP = 52  # arm-panel value column: the 72-wide popup less its label column


class App:
    """The page, its key handling and the procedure context.

    ROS, processes, arming, the plant, keys, the live view and the raw-pulse
    publisher are injected so tests drive the whole loop with fakes.
    """

    def __init__(self, args, profile, monitor, ros, processes, paths, console=None,
                 arming=None, plant=None, open_keys=KeyReader, live=Live,
                 clock=time.monotonic, foreground=subprocess.call, nav2=None,
                 task_watcher=None, pulses=None, open_events=None, registry=None, watching=0,
                 facts=None, source=None, nav2_file=None):
        self.args = args
        self.registry, self.watching = registry, watching   # watching: pid of the owner we only watch
        self.detaching = False
        self.foreign_seen, self.guests_seen = {}, set()
        self.profile = profile
        self.monitor = monitor
        self.ros = ros
        self.source = source or LiveSource(monitor, profile)
        self.processes = processes
        self.paths = paths
        self.console = console or Console()
        self.arming = arming
        self.plant = plant
        self.conditions = Conditions()      # fed by the ingest path (ts054); unfed it reads unknown
        self.open_keys = open_keys
        self.live_factory = live
        self.clock = clock
        self.foreground = foreground
        self.publish_pulses = pulses
        self.journal = Journal()
        # Manual control needs key releases (Kitty protocol) to tell held keys from taps.
        self.open_events = open_events or (lambda: KeyReader(report_events=True))
        self.manual = False
        self.nav2_factory, self._nav2 = nav2, None
        self.nav2_file = nav2_file  # harness.nav2_edit.Nav2File, or None
        self.watcher_factory, self._watcher = task_watcher, None
        self.run_id = None  # the procedure run in progress; its results carry this id
        self.settings = args.settings
        self.tabs = TabMemory()
        self.focus = 'status'
        self.cursors = {panel: 0 for panel in PANELS}
        self.activity = 'results'
        self.modal = None
        self.running = None
        self._wall_offset = None
        self.tags = {}           # procedure, key and run_id of the run in progress
        self.test_pulses = None
        self.draft = None        # unsaved thruster mapping: channel_map and reversed
        self.grabbed = None      # thruster row being moved with ↑↓
        self.notice = theme.OVERLAY_WARNING or 'Passive monitor; a opens the arm panel'
        self.command = (0.0, 0.0, 0.0)
        self.last_advisory = float('-inf')
        self.health = Health(profile, paths.bag.parent, facts, clock)
        self.pwm_readback = [theme.DASH] * 4
        self.keys = self.live = None
        self.last_frame = 0.0
        self.quit = False
        self.closed = False
        self.started = clock()
        self.config_file = config_path(args.config)
        self.history = History()
        self.watcher = Watcher()
        self.watcher.add(Watched('hardware', self.config_file, self.load_hardware, self.apply_hardware,
                                 subsystem='hardware'))
        self.config_note = self.config_file.name if self.config_file.exists() else 'defaults'

    @property
    def armed(self):
        return self.arming is not None and self.arming.armed

    def snapshot(self):
        snapshot = self.source.snapshot()
        if self.nav2_file is not None:
            snapshot['components'] = [self.nav2_file.row()]
        return snapshot

    # --- interlocks ------------------------------------------------------------

    def blockers(self):
        reasons = motion_blockers(self.snapshot(), self.profile, self.args.bench_override)
        if self.ros.fault:
            reasons.insert(0, 'ROS fault: ' + self.ros.fault)
        # A running survey staged the backup itself; it restores it when it ends or aborts.
        return reasons + ([] if self.running else self.staging().blockers())

    def staging(self):
        """The identity-correction backup for this session's hardware YAML (harness/apply.py)."""
        return Staging(self.args.config)

    def offer_restore(self):
        """At start: an unrestored backup means a survey died with identity C in the file."""
        if self.staging().pending() is None:
            return
        try:
            if self.choose('A survey was interrupted: restore the saved correction?',
                           ('restore', 'keep'), {'r': 0, 'k': 1},
                           message='Until restored, motion is refused.') == 'restore':
                self.staging().restore()
                self.watch_config()
        except Cancelled:
            pass

    def arm_blockers(self):
        """What the arm panel shows: the interlocks plus what arming itself would refuse."""
        rival = self.arming.rival() if self.arming and self.profile.outputs != 'none' else ''
        return (self.blockers() + ([rival] if rival else []) + self.plant.conflicts()
                + self.foreign_blockers())

    def foreign_instances(self):
        """Suite nodes on the graph that no live registry entry explains: (seen, confirmed live).

        A crashed instance leaves its DDS node for tens of seconds, so a newcomer is only an
        advisory for FOREIGN_GRACE; one still there after that is treated as live.
        """
        names = set(getattr(self.ros, 'instances', lambda: [])())
        state = self.registry.state() if self.registry else {}
        known = {entry['node'] for entry in [state.get('owner') or {}, *state.get('guests', {}).values()]
                 if 'node' in entry}
        now = self.clock()
        self.foreign_seen = {name: self.foreign_seen.get(name, now) for name in names - known}
        live = [name for name, since in self.foreign_seen.items() if now - since >= FOREIGN_GRACE]
        return sorted(self.foreign_seen), live

    def foreign_blockers(self):
        return [f'another instance {name} is live' for name in self.foreign_instances()[1]]

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
        self.source.tick(self.clock())
        if not self.source.live:
            return  # replay: no host reads, config watching or driving
        self.refresh_advisories()
        self.watch_config()
        if self.nav2_file is not None:
            self.notice = self.nav2_file.poll() or self.notice
        if self.test_pulses is not None and self.publish_pulses is not None:
            self.publish_pulses(self.test_pulses)
        if self.armed and self.arming.fault():
            self.stop(self.arming.fault(), by='fault')

    def refresh_advisories(self):
        """Sample the non-blocking checks at most once per ADVISORY_PERIOD."""
        if self.clock() - self.last_advisory < ADVISORY_PERIOD:
            return
        self.last_advisory = self.clock()
        seen, live = self.foreign_instances()
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
        self.monitor.advise('recording', 'OK' if self.processes.running('bag') else 'FAIL',
                            str(self.paths.bag))
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

    # --- hardware config: edits from the panel or any editor, hot-reloaded -------

    def watch_config(self):
        """Poll every watched file once; announce what changed (harness/watch.py)."""
        for change in self.watcher.poll():
            self.announce(change)
        rows = self.watcher.rows()
        bad = [name for name, ok, _, _ in rows if not ok]
        self.monitor.advise('Config', 'WARN' if bad else 'OK',
                            f'{bad[0]} error' if bad else f'{len(rows)} file' + 's' * (len(rows) != 1))

    def announce(self, change):
        if not change.ok:
            self.config_note = 'invalid edit'
            if change.report:
                self.notice = f'Config rejected, keeping the last good one: {change.error}'
            return
        self.config_note = self.config_file.name
        self.notice = f'Config reloaded from {self.config_file.name}'
        self.ros.emit('config_reloaded', file=change.name, sha=change.sha, result='ok', path=str(self.config_file))

    def config_popup(self):
        """popup.config: one row per watched surface, with its state."""
        rows = [(name, f'{theme.icon("ok" if ok else "bad").plain} {detail}'
                 + (f' {datetime.fromtimestamp(at):%H:%M}' if ok and at else ''))
                for name, ok, detail, at in self.watcher.rows()]
        self.show('Config', rows=rows)

    def load_hardware(self, _path):
        return load_config(self.args.config)

    def apply_hardware(self, config):
        self.args.hardware_config = config
        if self.arming is not None:
            self.arming.hardware_config = config  # arming saves this copy
        if self.armed and self.profile.outputs != 'none':
            # The driver hot-reloads its snapshot; keep it the session's config.
            save_config(dict(config, output_backend=self.profile.outputs), self.paths.hardware)

    def warn_history(self, message):
        self.notice = message

    def observe_config(self):
        """At start: log config files edited since the last session as external changes."""
        adapters.observe_all(self.history, {'hardware': self.config_file}, self.warn_history)

    def save_hardware(self, **changes):
        """Write ``changes`` into the operator's hardware YAML; the watcher applies it.

        Returns False, keeping the draft, when the file on disk is broken.
        """
        adapters.observe_file(self.history, 'hardware', self.config_file, self.warn_history)   # a hand edit first
        try:
            update_config(changes, self.args.config)
        except (OSError, ValueError, TypeError, yaml.YAMLError):
            self.notice = f'Fix hardware.yaml first: {self.config_file}'
            return False
        adapters.record_file(self.history, 'hardware', 'tui:hardware', self.config_file, self.warn_history)
        self.draft = self.grabbed = None
        self.announce(self.watcher.reload('hardware'))  # our own save skips the settle wait
        return True

    # --- rendering ---------------------------------------------------------------

    @property
    def tab(self):
        return self.tabs.current

    def panels(self):
        if self.tab != 'Drive':
            return [self.focus]
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
        hints = MANUAL_HINTS if self.manual else PANEL_HINTS.get(self.focus, ()) + GLOBAL_HINTS
        if self.nav2_file is not None and self.focus == 'status' and not self.manual:
            hints = PANEL_HINTS['status'] + [('e', 'Nav2')] + GLOBAL_HINTS
        return dict(
            tab=self.tab, focus=self.focus, cursors=self.cursors, modal=self.modal, running=self.running,
            activity=self.activity, now=self.monitor.clock(), notice=notice, hints=hints,
            position=f'{min(self.cursors[self.focus], count - 1) + 1}/{count}' if count else '',
            authority=('human', 'KEYS') if self.manual else ('ours', self.running.upper()) if self.running else None,
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
                              self.args.hardware_config, journal_events=journal.read(self.paths.journal),
                              conditions=self.conditions.summary())
        path = write_report(report, self.paths.report)
        self.notice = f'Report saved: {path}'
        return path

    # --- the loop ----------------------------------------------------------------

    def sync_guests(self):
        """Emit guest_attach / guest_detach as boat_trial guests join and leave the registry."""
        if self.registry is None or self.watching:
            return
        guests = self.registry.state().get('guests', {})
        for pid in sorted(set(guests) - self.guests_seen):
            self.ros.emit('guest_attach', pid=pid, **guests[pid])
        for pid in sorted(self.guests_seen - set(guests)):
            self.ros.emit('guest_detach', pid=pid)
        self.guests_seen = set(guests)

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
                self.offer_restore()
                while not self.quit:
                    try:
                        key = self.keys.read(timeout=0.1)
                        if key is not None:
                            try:
                                self.dispatch(key)
                            except STOPPING_ERRORS as error:
                                self.stop(str(error) or type(error).__name__, by='error')
                        self.tick()
                        self.refresh(force=key is not None)
                    except KeyboardInterrupt as error:
                        self.detaching = isinstance(error, Detach)
                        # Ctrl+C quits like q, minus the question: anything moving has
                        # already stopped at neutral (the runner neutralizes on any exit).
                        self.quit = True
                        self.modal = None
                        self.notice = 'Interrupted'
                    except (EOFError, OSError) as error:
                        if not terminal_gone(error):
                            raise
                        # docker exec never hangs up a dead client's pty; quit like q
                        # instead of lingering headless with the recorder and outputs.
                        self.quit = True
                        self.modal = None
                        self.notice = 'Terminal closed'
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
            if not self.save_hardware(channel_map=channel_map, reversed=reversed_):
                return
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
        if self.detaching:   # the children and the recording go on under the next owner
            steps = [('Outputs', self.close_outputs), ('Handover', self.close_handover)]
        if self.watching:    # not ours to stop: neutral, events and the report belong to the owner
            steps = [('Watch', lambda: f'left pid {self.watching} running')]
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

    def close_handover(self):
        """Leave the children running, recorded for the successor, and give up the seat."""
        children = self.processes.records()
        handover = (self.registry.state().get('handover') or {}) if self.registry else {}
        self.ros.emit('handover', to=handover.get('to'))
        self.export()
        if self.registry is not None:
            self.registry.set_children(children)
        self.processes.release(children)
        if self.registry is not None:
            self.registry.release(os.getpid())
        self.handed_to = handover.get('to')
        return f"{', '.join(children) or 'no children'} left running for pid {self.handed_to}"

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

    def switch_tab(self, step):
        """Move to the next or previous tab, restoring the focus and cursors it was left with."""
        if self.focus == 'thrusters':
            self.clear_pulses()  # test pulses only while their panel has focus
        self.grabbed = None
        self.focus, saved = self.tabs.switch(step, self.focus, self.cursors)
        self.cursors = {**{panel: 0 for panel in PANELS}, **saved}

    def move(self, step):
        count = self.rows(self.focus)
        if count:
            self.cursors[self.focus] = max(0, min(count - 1, self.cursors[self.focus] + step))

    def dispatch(self, key):
        """Apply one key on the page."""
        raw, key = key, fold(key)
        panels = self.panels()
        if key in ('[', ']'):
            self.switch_tab(-1 if key == '[' else 1)
        elif key in ('\t', 'backtab'):
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
        elif key == 'b':
            self.mark_note() if raw == 'B' else self.mark()
        elif key in ('+', '=', '-', '_'):
            step = 0.05 if key in ('+', '=') else -0.05
            self.settings.amplitude = round(min(1.0, max(0.05, self.settings.amplitude + step)), 2)
            self.notice = f'Amplitude {self.settings.amplitude:g} {self.profile.command_units}'
            settings.save(self.settings)
        elif (key in {p.key for p in PROCEDURES} and self.tab == 'Drive'
              and not (self.focus == 'thrusters' and key in ('1', '2', '3', 'c', 's'))
              and not (self.focus == 'activity' and key == 'v')
              and not (self.focus == 'status' and key == 'o')):  # Odometry popup; Perception stays on o elsewhere
            self.on_procedures(key)
        else:
            getattr(self, f'on_{self.focus}', lambda key: None)(key)

    # --- markers -----------------------------------------------------------------

    def moving(self):
        return bool(self.running or self.manual)

    def mark(self):
        """Journal a marker stamped now; ask for a note unless something is moving."""
        t_ns = time.time_ns()
        number = journal.next_marker_id(journal.read(self.paths.journal))
        self.ros.emit('marker', kind='user', id=number, note='', t_ns=t_ns)
        if self.moving():
            self.notice = f'{theme.pick("▾", "v")}{number} marked{theme.SEPARATOR}B adds a note'
        else:
            self.mark_note(number)

    def mark_note(self, number=None):
        """Type a note for marker ``number`` (default the last); never while moving."""
        found = journal.markers(journal.read(self.paths.journal))
        if number is None:
            number = found[-1]['id'] if found else None
        if number is None:
            self.notice = 'No marker yet; b adds one'
        elif self.moving():
            self.notice = 'Not while moving'
        else:
            note = self.text_input(f'Marker {number}', 'Note, Enter saves, Esc keeps it bare')
            if note:
                self.ros.emit('marker_note', id=number, note=note)

    def text_input(self, title, message):
        """A one-line text popup; the typed text on Enter, None on Esc."""
        modal = dict(kind='input', title=title, message=message, text='', width=64,
                     hint=f'Enter save{theme.SEPARATOR}Esc cancel')

        def handle(key):
            if key == 'enter':
                return modal['text'].strip()
            if key == 'escape':
                return ''
            if key in ('backspace', 'ctrl-h', '\x7f'):
                modal['text'] = modal['text'][:-1]
            elif key == 'space':
                modal['text'] += ' '
            elif len(key) == 1 and key.isprintable():
                modal['text'] += key
        return self.popup(modal, handle) or None

    # --- panel keys --------------------------------------------------------------

    def on_status(self, key):
        rows = status_rows(self.snapshot(), self.profile.sensors)
        if key == 'o':
            self.odometry()
        elif key == 'e':
            row = rows[min(self.cursors['status'], len(rows) - 1)] if rows else None
            if row and row['group'] == 'Components' and self.nav2_file is not None:
                self.nav2_popup()
        elif key == 'enter':
            if rows:
                row = rows[min(self.cursors['status'], len(rows) - 1)]
                if row['group'] == 'Sensors' and row['name'] == 'odometry':
                    return self.odometry()
                if row['name'] == 'Config':
                    return self.config_popup()
                self.show(row['name'], rows=[('Group', row['group']),
                                             ('State', row['state'].lower()),
                                             ('Detail', row['detail'])])

    def odometry(self):
        """Fused sources popup. Parameters and rows are read here, once, so nothing is
        computed while it is closed. ``ros.get_parameters(node)`` is optional (None: unreadable)."""
        fetch = getattr(self.ros, 'get_parameters', lambda node: None)
        params = {node: fetch(node) for node in odometry.ESTIMATES}
        sources = [(row['name'], row['state'].lower(), row['value'])
                   for row in status_rows(self.snapshot(), self.profile.sensors)
                   if row['group'] == 'Sensors']
        self.show('Odometry', rows=odometry.rows(params, sources), width=76)

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
            self.notice = ('Moving ' + dashboard.POSITIONS[position].upper() + f'; {UPDOWN} moves it, '
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
            if self.save_hardware(channel_map=channel_map, reversed=reversed_):
                self.notice = f'Mapping saved to {self.config_file.name}'
        elif key == 'u':
            self.draft = self.grabbed = None
            self.notice = 'Mapping changes undone'
        elif key == 'p':
            self.notice = pins.edit_pins(self)
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
        conflicts = self.plant.conflicts()
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

    # --- popups (see ui/popups) ------------------------------------------------------

    def popup(self, modal, handle):
        return popups.popup(self, modal, handle)

    scroll = staticmethod(popups.scroll)

    def show(self, title, message=None, rows=None, width=60):
        return popups.show(self, title, message, rows, width)

    def nav2_popup(self):
        """Preview the Nav2 file against what was applied; y applies, Esc leaves it pending."""
        source = self.nav2_file
        previewed = source.current  # apply exactly what is shown, even if the file changes meanwhile
        rows = [(key, f'{old if old is not None else theme.DASH} {theme.ARROW} '
                      f'{new if new is not None else theme.DASH}')
                for key, old, new in source.changes()]
        rows += [(key, f'removed from file {theme.ARROW} restart to reset') for key in source.removed()]
        rows = rows or [('', 'No changes pending')]
        try:
            answer = self.choose('Nav2 parameters', ['Apply', 'Close'], {'y': 0}, rows=rows,
                                 selected=0 if source.pending else 1, width=76)
        except Cancelled:
            return
        if answer == 'Apply':
            self.notice = source.apply(previewed)

    def prompt(self, message, title='Ready'):
        return popups.prompt(self, message, title)

    def choose(self, title, options, shortcuts=None, message=None, danger=False, rows=None,
               selected=0, width=64):
        return popups.choose(self, title, options, shortcuts, message, danger, rows, selected, width)

    def ask_text(self, title, message, text=''):
        return popups.ask_text(self, title, message, text)

    def driver_values(self, keys):
        """The thruster driver's live parameters for ``keys``, or None without an answer."""
        return self.ros.driver_parameters('thruster_pwm_node', keys)

    def type_word(self, title, message, word, rows=None, width=64):
        return popups.type_word(self, title, message, word, rows, width)

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

    def arm_panel(self):
        """One popup for both directions; Space still stops without it."""
        if not self.source.live:
            self.notice = 'Arming is disabled in review'
            return
        if self.watching:
            self.notice = f'Watching pid {self.watching}: no arming'
            return
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
        if self.plant is not None:
            self.plant.engage()
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

    def disarm(self, reason='disarmed', by='operator'):
        self.clear_pulses()
        if self.plant is not None:
            self.plant.neutral()
            self.plant.release()
        self.command = (0.0, 0.0, 0.0)
        if self.armed:
            self.arming.disarm()
            self.ros.emit('disarmed', reason=reason, by=by)
        self.notice = f'{reason}; outputs neutral'

    def stop(self, reason, by='operator'):
        """Neutral and disarmed, whatever happened; ``by`` is operator, fault or error.

        A watcher publishes neutral too: Space always works.
        """
        try:
            if self.watching and self.plant is not None:
                self.plant.engage()  # a second publisher also aborts the owner's motion
            self.disarm(reason, by)
        finally:
            self.monitor.event('STOP: ' + reason)

    # --- procedures ---------------------------------------------------------------

    def availability(self, procedure):
        if self.watching:
            return f'watching pid {self.watching}: no commands'
        reason = unavailable(procedure, self.profile, self.armed, self.args.bench_override)
        # Procedures that measure motion can't be picked without odometry and TF, nor
        # while another node commands the same topic.
        if not reason and 'sensors' in procedure.needs and not self.args.bench_override:
            blockers = self.motion_blockers() if self.armed else motion_blockers(
                self.snapshot(), self.profile)
            if blockers:
                reason = blockers[0]
        if not reason and self.engine is not None:
            blocking, _ = self.engine.gate(
                procedure.key, self.args.bench_override, 'armed' in procedure.needs)
            reason = blocking[0] if blocking else ''
        return reason

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

    def run_procedure(self, procedure):
        if not self.source.live:
            self.notice = 'Procedures are disabled in review'
            return
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
        self.run_id = runs.next_run_id(self.paths.runs, procedure.title)
        self.tags = dict(procedure=procedure.title, key=procedure.key, run_id=self.run_id)
        self.notice = f'Running {procedure.title}'
        self.health_snapshot('start', procedure)
        began = self.engine.clock() if self.engine else 0.0
        aborted = None
        try:
            try:
                procedure.run(self.procedure_context())
            except BaseException:
                self.health_window(procedure, began)   # before teardown, whatever the exit
                raise
            invalid = self.health_window(procedure, began)
        except STOPPING_ERRORS as error:
            aborted = str(error)
            if not isinstance(error, Cancelled):
                raise
            self.notice = f'{procedure.title} cancelled'
            self.disarm(f'{procedure.title} cancelled')
            return
        finally:
            self.running, self.tags = None, {}
            self.write_run(procedure, results, aborted)
        if len(self.snapshot()['results']) > results:  # show what it found
            self.focus, self.activity = 'activity', 'results'
            self.cursors['activity'] = 0
            if not invalid:
                self.notice = f'{procedure.title} complete'

    def write_run(self, procedure, start, aborted):
        """The one run file for a procedure run: every result it recorded since ``start``."""
        found = self.snapshot()['results'][start:]
        run_id, self.run_id = self.run_id, None
        if not found and aborted is None:
            return
        record = dict(procedure=procedure.title, run_id=run_id, results=found)
        if aborted is not None:
            record['reason'] = aborted
        runs.write_run(self.paths.runs, record, settings=vars(self.settings),
                       status=runs.ABORTED if aborted is not None else None)

    def health_window(self, procedure, began):
        """Snapshot health and flag the trial window invalid when a gated check left ok.

        Returns the invalid notice (also shown), or None.
        """
        self.health_snapshot('end', procedure)
        if self.engine is None:
            return None
        problems = self.journal.window_problems(
            self.engine, procedure.key, began, self.engine.clock(), 'armed' in procedure.needs)
        if problems:
            self.ros.emit('trial_window_invalid', procedure=procedure.key, reasons=problems)
            self.notice = f'{procedure.title}: window invalid, {problems[0]}'
            return self.notice
        return None

    def procedure_context(self):
        return host.context(
            self, self.profile, self.monitor, self.settings, self.args.stale_after,
            dict(
                conditions=self.conditions, nav2=self.nav2, task_watcher=self.task_watcher,
                start=self.processes.start,
                running=self.processes.running, stop=self.processes.stop,
                nav2_params=nav2_params(), nav2_overlay=self.paths.nav2,
                nav2_user=nav2_limits.path(),
                hardware_config=self.args.hardware_config, wall_ns=self.wall_ns,
                user_config=self.args.config,
                history=self.history, warn=self.warn_history,
                staging=self.staging(),
                disarm=self.disarm, pulses=self.set_pulses, save_hardware=self.save_hardware,
                manual=dict(plant=self.plant, blockers=self.manual_blockers,
                            open_events=self.open_events, show=self.manual_show, emit=self.emit,
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
                        emit=self.emit, clock=self.monitor.clock)
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

    def clock_ns(self):
        """Wall nanoseconds, the journal's time base, for a time on the monitor clock."""
        return self.wall_ns(self.monitor.clock())

    def wall_ns(self, t):
        if self._wall_offset is None:  # fixed once, so windows stay contiguous
            self._wall_offset = time.time_ns() - int(self.monitor.clock() * 1e9)
        return self._wall_offset + int(t * 1e9)

    def emit(self, phase, **details):
        """Journal an event stamped with the running procedure; explicit details win."""
        self.ros.emit(phase, **{**self.tags, **details})

    def record(self, result):
        if self.run_id:
            result = {**result, 'run_id': self.run_id}
        self.monitor.record_result(result)
        self.ros.emit('result', **result)
        self.export()

    def manual_show(self, command, amplitude, chords, done=False):
        """Redraw the page for manual control: Motion focused, manual keys in the footer."""
        self.manual = not done
        self.command = tuple(command)
        if 'motion' in self.panels():
            self.focus = 'motion'
        active = ' '.join(f'{axis} {value:+.2f}' for axis, value in
                          zip(('surge', 'sway', 'yaw'), command) if value) or 'neutral'
        self.notice = ('Manual control ended' if done else
                       f"Manual {amplitude:.0%}{dashboard.SEPARATOR}{active}{dashboard.SEPARATOR}"
                       + ('Chords' if chords else 'Single key'))
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
    parser.add_argument('--start-stack', action='store_true',
                        help="start the profile's components marked start; default attaches to "
                             'running topics (the other components start from the Status rows)')
    parser.add_argument('--fcu-url', default='',
                        help='MAVROS FCU URL for --start-stack (default: detect the flight controller)')
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                        help='override a setting for this run (see harness/settings.py), e.g. '
                             'amplitude=0.4, command_mode=velocity, bench_override=on')
    parser.add_argument('--output', type=Path, default=None,
                        help='session bag path (default $MHSEALS_DATA or <workspace>/bags, <profile>_<time>)')
    return parser


def parse_args(argv=None):
    parser = build_parser()
    try:
        args = configure_args(parser, argv)  # adds --config and the merged hardware_config
    except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
        # Refuse to start on a broken hardware YAML, naming the file instead of a traceback.
        probe = argparse.ArgumentParser(add_help=False)
        probe.add_argument('--config')
        path = config_path(probe.parse_known_args(argv)[0].config)
        parser.error(f'{path}: {error}')
    args.settings = settings.load()
    try:
        for assignment in args.set:
            args.settings.set(assignment)
    except ValueError as error:
        parser.error(f'--set {error}')
    args.stale_after, args.bench_override = args.settings.stale_after, args.settings.bench_override
    profile = load_profile(args.profile)
    if args.settings.command_mode == 'velocity':
        profile = profile.with_velocity_control()
    if args.start_stack and not profile.components:
        parser.error(f'profile {profile.name} has no components; start its stack separately')
    args.output = args.output or data_root() / (
        f"{profile.name.replace('+', '_')}_{datetime.now():%Y%m%d_%H%M%S}")
    return args, profile


def stack_overlay(profile, args):
    """Launch arguments the command line adds to the profile's components."""
    if not profile.uses_fcu:
        return {}
    # The flight controller's stable /dev/serial/by-id name when one is plugged in.
    return {'odometry': {'fcu_url': args.fcu_url or default_fcu_url()}}


def terminal_gone(error):
    """True for the key reader's EOF or EIO: the terminal is gone, not a stopping error."""
    return isinstance(error, EOFError) or error.errno == errno.EIO


def hang_up(*_):
    """SIGHUP and SIGTERM stop like Ctrl+C: neutral, disarm, flush, then exit."""
    raise KeyboardInterrupt


def detach(*_):
    """SIGUSR1: the next instance is taking over."""
    raise Detach


def main(argv=None):
    args, profile = parse_args(argv)
    # ROS imports stay here so the App (and its tests) never need rclpy.
    import rclpy

    from mhseals_testing.ros.arming import Arming
    from mhseals_testing.ros.health import subscribe_fcu
    from mhseals_testing.ros.mission import TaskWatcher
    from mhseals_testing.ros.nav2 import Nav2
    from mhseals_testing.ros.node import RosIO, node_name
    from mhseals_testing.ros.plant import Plant
    from mhseals_testing.harness.processes import ProcessManager, bag_command, unique_path

    from rclpy.signals import SignalHandlerOptions

    settings.save(args.settings)  # the last used values (not command mode or bench override)
    console = Console()
    registry = Registry()
    me = dict(pid=os.getpid(), node=node_name(), host=socket.gethostname(), started=time.time(),
              profile=profile.name, tty=os.ttyname(0) if os.isatty(0) else '')
    choice = decide(registry, console, me)
    if choice.mode == 'quit':
        return
    watching = choice.watching if choice.mode == 'watch' else 0
    # Adopted sessions keep the recording's directory, so the report and bag stay together.
    paths = SessionPaths.from_output(
        Path(choice.session) if choice.session and choice.mode != 'own' else unique_path(args.output))
    # rclpy's own SIGINT handler would shut ROS down before the graceful close could
    # send neutral and the final events; without it Ctrl+C is a KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for hangup in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(hangup, hang_up)
    signal.signal(signal.SIGUSR1, detach)
    monitor = BoatMonitor(args.stale_after, history_sensors=tuple(
        sensor.name for sensor in profile.sensors if sensor.kind == 'odometry'),
        deadlines={sensor.name: sensor.deadline for sensor in profile.sensors
                   if sensor.deadline is not None})
    ros = RosIO(profile, monitor, args.stale_after, paths.journal)
    # The keepers follow the registry owner, so a handover keeps the children for the adopter.
    processes = ProcessManager(paths.logs, registry=None if watching else registry)
    plant = Plant(ros, profile)
    stack = Stack(components(profile, stack_overlay(profile, args)), processes, time.monotonic)
    arming = Arming(profile, ros, processes, args.hardware_config, paths.hardware)
    app = None
    facts = Facts()
    try:
        for name, child in choice.children.items():
            processes.adopt(name, child)
        if choice.mode == 'own':               # a stale owner's leftovers: stop, start fresh
            processes.stop_all()
        if profile.uses_fcu:
            subscribe_fcu(ros, monitor, facts, profile.battery_cells)
        if watching:   # known to the owner, so it is not flagged as a foreign instance
            registry.add_guest(os.getpid(), me['node'], 'watch')
        else:
            registry.update(lambda state: state.update(session=str(paths.bag)))
            paths.bag.parent.mkdir(parents=True, exist_ok=True)
            if not processes.running('bag'):
                processes.start('bag', bag_command(paths.bag))
            if args.start_stack and not any(processes.running(key) for key in stack.start_order()):
                time.sleep(2.0)  # let discovery find an already-running stack
                if ros.publishers(profile.odometry):
                    raise RuntimeError(f'{profile.odometry} already has publishers; '
                                       'attach without --start-stack')
                for key in stack.start_order():
                    stack.start(key, ros.nodes())
            registry.set_children(processes.records())
        app = App(args, profile, monitor, ros, processes, paths, console, arming, plant,
                  nav2=lambda: Nav2(ros, profile.map_frame or profile.odom_frame,
                                    profile.base_frame),
                  task_watcher=lambda: TaskWatcher(ros), pulses=plant.test_pulses,
                  registry=registry, watching=watching, facts=facts,
                  nav2_file=Nav2File(nav2_limits.path()))
        if not watching:
            ros.emit('takeover' if choice.mode == 'adopt' else 'session_start',
                     profile=profile.name, bag=str(paths.bag))
            app.observe_config()
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
            processes.stop_all()   # a detach already released the children it hands on
            if watching:
                registry.remove_guest(os.getpid())
            else:
                registry.release(os.getpid(), keep_children=False)
            ros.close()
            rclpy.try_shutdown()
    if app is not None and app.detaching:
        console.print(f'Handed over to pid {getattr(app, "handed_to", "?")}')
    if app is not None:
        table = dashboard.theme.grid({'header': '', 'style': f'bold {dashboard.ACCENT}'},
                                     {'header': '', 'ratio': 1}, padding=(0, 2))
        table.show_header = False
        for label, value in app.summary():
            table.add_row(label, value)
        console.print(dashboard.theme.block(table, 'Session closed', expand=False))


if __name__ == '__main__':
    main()
