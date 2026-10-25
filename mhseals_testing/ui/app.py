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
import errno
import os
from pathlib import Path
import signal
import socket
import subprocess
import textwrap
import threading
import time

import yaml
from rich.console import Console, Group
from rich.live import Live
from rich.segment import Segments
from rich.text import Text

from mhseals_hardware.configuration import (
    config_path, config_sha, configure_args, load_config, save_config, update_config)
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.menu import Cancelled
from mhseals_hardware.thruster_mixer import IDENTITY, MAX_PWM, MIN_PWM, POSITIONS, validate_mixer
from mhseals_testing import calibration, data, nav2_limits
from mhseals_testing.analysis import classes as classes_module
from mhseals_testing.harness import host, index, journal, runs, session as session_module, settings
from mhseals_testing.harness.bags import MODES as REC_MODES, Recorder
from mhseals_testing.harness.apply import CORRECTION_ACK_REQUIRED, RuntimeStaging
from mhseals_testing.harness.components import Stack, components
from mhseals_testing.harness.remote import Units
from mhseals_testing.harness.unit import probe as unit_probe
from mhseals_testing.analysis.readiness import planned_directions
from mhseals_testing.harness.conditions import Conditions, STILL_FRESH_S, stillness
from mhseals_testing.runner import STALE_MUX_S, override_cause
from mhseals_testing.harness import nav2_edit
from mhseals_testing.harness.nav2_edit import Nav2File
from mhseals_testing.harness.recwatch import DISK_MARKER, RecWatch
from mhseals_testing.harness.registry import Registry
from mhseals_testing.harness.watch import Watched, Watcher
from mhseals_testing.health import config as health_config, legacy as health
from mhseals_testing.health.advisories import Health
from mhseals_testing.health.probes import Facts
from mhseals_testing.health.journal import Journal
from mhseals_testing.health.network import NetworkRow
from mhseals_testing.history import History, adapters, backends, revert
from mhseals_testing.monitor import BoatMonitor
from mhseals_testing.procedures import PROCEDURES, unavailable
from mhseals_testing.devices import DeviceError
from mhseals_testing.profiles import available_profiles, load_profile
from mhseals_testing.readiness import arm_nodes, arm_refusal, liveness_refusal, motion_blockers
from mhseals_testing.report import build_report, write_report
from mhseals_testing.ros.arming import EFFORT_TOPIC, command_input, muxed, session_hardware
from mhseals_testing.runner import Aborted, Runner, fold
from mhseals_testing.ros import camera_feed
from mhseals_testing.ui import (
    authority_chip, camera_view, feeds as feeds_module, graphics, layout, palette, parts,
    perception, popups, theme)
from mhseals_testing.ui.camera_view import CameraView
from mhseals_testing.ui.feeds import agent as agent_feed, authority as authority_feed, health as health_feed
from mhseals_testing.ui.feeds.conditions import ConditionsFeed
from mhseals_testing.ui.feeds.issues import IssuesFeed, settings as issue_settings
from mhseals_testing.ui.feeds.rtk import RtkFeed
from mhseals_testing.ui.feeds.toasts import ToastFeed
from mhseals_testing.ui.feeds.tracking import TrackingFeed
from mhseals_testing.ui.toasts import Toasts
from mhseals_testing.ui.layout import MINIMAP, ROW, TabMemory, body_rows, motion_shown
from mhseals_testing.ui.minimap import panel as minimap_panel
from mhseals_testing.ui.minimap.layerbox import LayerBox
from mhseals_testing.ui import keymap as keymap_module, rtk, rtk_icon, widgets
from mhseals_testing.harness import bags as bags_module
from mhseals_testing.ui.panels import PANELS, activity, agent_log, correction, scrubber
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import config as config_popup, odometry, pins, run as run_popup
from mhseals_testing.ui.popups import magcal as magcal_popup, stop as stop_popup
from mhseals_testing.ui.review import (
    BIG, find_session, load_session, Offline, review_metadata, SMALL)
from mhseals_testing.ui.source import LiveSource, ReviewSource
from mhseals_testing.ui.takeover import decide

# Ctrl+h/j/k/l and Ctrl+arrows move focus to the panel in that direction, by layout:
#   row (>= 90 columns): Status | Motion     | Conditions, Thrusters      stack: Status | Motion, Conditions,
#                                | Procedures | Activity                            | Thrusters, Procedures, Activity
DIRECTIONS = {'ctrl-h': 'left', 'ctrl-left': 'left', 'ctrl-l': 'right', 'ctrl-right': 'right',
              'ctrl-k': 'up', 'ctrl-up': 'up', 'ctrl-j': 'down', 'ctrl-down': 'down'}
NEIGHBOURS = {   # a name or the first of a tuple of names that is on screen
    True: {'status': {'right': 'motion'},
           'motion': {'left': 'status', 'right': 'conditions', 'down': 'procedures'},
           'conditions': {'left': 'motion', 'right': 'minimap', 'down': 'thrusters'},
           'thrusters': {'left': 'motion', 'right': 'minimap', 'up': 'conditions', 'down': 'activity'},
           'minimap': {'left': 'thrusters', 'down': 'activity'},
           'procedures': {'left': 'status', 'right': 'activity', 'up': 'motion'},
           'activity': {'left': 'procedures', 'up': 'thrusters'}},
    False: {'status': {'right': ('motion', 'conditions')},
            'motion': {'left': 'status', 'down': 'conditions'},
            'conditions': {'left': 'status', 'up': 'motion', 'down': 'thrusters'},
            'thrusters': {'left': 'status', 'up': 'conditions', 'down': 'procedures'},
            'procedures': {'left': 'status', 'up': 'thrusters', 'down': 'activity'},
            'activity': {'left': 'status', 'up': 'procedures'}},
}
ADVISORY_PERIOD = 1.0
MUX_SETTLE_S = 5.0       # s after arming for the mux's first state (it starts last, then 1 Hz)
HELD = 'applies once disarmed and idle'   # a health or settings edit read while armed or running
FOREIGN_GRACE = 30.0     # s: a dead DDS participant lingers until its lease expires; later it counts as live
HOLD_CONFIRM_S = 2.0      # s the mux state may take to show a requested watchdog hold
WATCHDOG_GRACE = 2.0     # s a required safety watchdog may be missing from the graph while armed (graph lag)
FRAME = 1 / 30           # s; continuous redraws (motion, manual) stop at 30 per second
MANUAL_DESK_S = 0.2      # s; the agent desk's tick from inside manual control
CAMERA_ROWS = 12         # cell rows of the perception popup's camera view
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
    'conditions': [('Enter', 'Details')],
    'thrusters': [(LEFTRIGHT, 'Pulse'), ('Enter', 'Move'), ('1-4', 'Output'), ('r', 'Reverse'),
                  ('s', 'Save'), ('u', 'Undo'), ('0', 'Neutral'), ('i', 'Identify'),
                  ('c', 'Clear decoupling'), ('P', 'Pin'), ('M', 'Pins')],
    'procedures': [('Enter', 'Run'), (UPDOWN, 'Select')],
    'activity': [('Enter', 'Run'), ('v', 'View')],
    'minimap': [('z/Z', 'Zoom'), ('0', 'Reset zoom'), ('u', 'Heading-up'), ('F', 'Fov')],
    'map': [('c', 'Costmap'), ('d', 'Tracks'), ('y', 'Trail'), ('z/Z', 'Zoom'), ('u', 'Heading-up'),
            ('F', 'Fov'), ('w', 'Wake'), ('g', 'Rings'), ('T', 'Tasks')],
    'list': [(UPDOWN, 'Select'), ('p', 'Pin'), ('n', 'Title'), ('d', 'Delete'), ('P', 'Prune'), ('F', 'Finalize')],
}
CORRECTION_HINTS = [(LEFTRIGHT, 'Direction'), ('Enter', 'Holds'), ('r', 'Rerun direction'),
                    ('R', 'Run all'), ('A', 'Apply pending'), ('I', 'Hardware changed')]
REVIEW_HINTS = [('p', 'Play/pause'), (f'{LEFTRIGHT} {{ }}', 'Seek 5 s / 30 s'),
                (', .', 'Prev/next mark'), ('< >', 'Rate'), ('Esc', 'Leave review'),
                ('?', 'Help'), ('q', 'Quit')]
REVIEW_KEYS = {'left': (SMALL, -1), 'h': (SMALL, -1), 'right': (SMALL, 1), 'l': (SMALL, 1),
               'shift-left': (BIG, -1), '{': (BIG, -1), 'shift-right': (BIG, 1), '}': (BIG, 1)}
MANUAL_HINTS = [(f'W/S {UPDOWN}', 'Surge'), ('A/D', 'Sway'), (LEFTRIGHT, 'Yaw'), ('+/-', 'Speed'),
                ('Space', 'Stop'), ('x / Esc', 'Leave manual')]
HELP = [
    ('Ctrl h j k l / arrows', 'Focus the panel to the left, below, above, right'),
    ('Tab / Shift Tab', 'Cycle focus through the panels'),
    (f'{UPDOWN} / j k', 'Move within the focused panel'),
    ('Procedure keys', 'Run from any panel; focused-panel edit keys take precedence'),
    ('Enter', 'Run, open or show details'),
    ('o', 'Pose: what the EKFs fuse, and the sources'),
    ('a', 'Arm panel: outputs, interlocks, arm or disarm'),
    ('Space', 'Stop: neutral and disarm'),
    ('m', 'Manual control'),
    ('V', 'Perception: pipeline, latency, drops and per-class rates'),
    ('+ / -', 'Trial amplitude'),
    ('x', 'Write the session report'),
    ('b / B', 'Mark the moment (journal) / add a note to the last marker'),
    (f'Thrusters {LEFTRIGHT}',
     f'Test pulse {PLUSMINUS}{PULSE_STEP} {MICRO}s on the selected thruster (armed)'),
    ('Thrusters 0', 'All test pulses back to neutral'),
    ('Thrusters Enter', f'Pick up the thruster; {UPDOWN} moves it, Enter drops it'),
    ('Thrusters 1-4 / r', 'Wire it to an output / reverse it (a draft until saved)'),
    ('Thrusters s / u',
     'Save the draft to the config (the next session starts from it) / undo it'),
    ('Thrusters M', 'Edit the PWM pins and MOSFET line; saved, then the driver restarts, sysfs '
                    'confirms and it stops (disarmed only; refused while armed or arming)'),
    ('Thrusters P', 'Move the thruster to another free PWM pin (a draft; s saves and reads it back)'),
    ('Thrusters i', 'Thruster set: identify outputs by pulsing each one'),
    ('Thrusters c', 'Clear the fitted decoupling (asks first)'),
    ('Activity v', 'Switch between results and the event log'),
    ('q', 'Quit: neutral, disarm, flush the bag'),
]


@dataclass(frozen=True)
class SessionPaths:
    """Where one session keeps its files: all inside ``sessions/<id>/`` (data_layout)."""
    directory: Path
    bag: Path       # bag/NN/ segments
    report: Path
    logs: Path
    hardware: Path  # snapshot of the hardware config the driver runs with
    nav2: Path      # Nav2 params with measured limits, written by characterization
    journal: Path   # append-only events: markers, phases, results
    runs: Path      # one runs/<run_id>.json per procedure run
    session: Path   # session.json: identity and clock domain
    truth: Path     # simulator course truth, copied at start

    @classmethod
    def from_output(cls, directory):
        """The paths of the session directory ``directory``."""
        d = Path(directory)
        return cls(d, d / 'bag', d / 'analysis' / 'report.json', d / 'logs',
                   d / 'snapshots' / 'hardware.yaml', d / 'snapshots' / 'nav2.yaml',
                   d / 'events.jsonl', d / 'runs', d / 'session.json', d / 'course_truth.json')


ARM_WRAP = 52  # arm-panel value column: the 72-wide popup less its label column


CURSORS = (*PANELS, 'list')   # every cursor an App keeps, the Bags tab's 'list' included


def map_key_maps(doc):
    """``(drive, map)`` key -> action of the map views: view.minimap on the Drive column, and on
    the Map tab with tab.map's toggles (w g) over it, which are Drive procedure keys elsewhere.
    view.minimap's own e (centre) shadows Interference on both, so a focused map never starts it."""
    view = keymap_module.key_map(doc.get('view.minimap'))
    return view, {**keymap_module.key_map(doc.get('tab.map')), **view}


class App:
    """The page, its key handling and the procedure context.

    ROS, processes, arming, the plant, keys, the live view and the raw-pulse
    publisher are injected so tests drive the whole loop with fakes.
    """

    rtk_link = None       # the running RTK base (ui/rtk.py): .base and .reload; None means no base

    def __init__(self, args, profile, monitor, ros, processes, paths, console=None,
                 arming=None, plant=None, open_keys=KeyReader, live=Live,
                 clock=time.monotonic, foreground=subprocess.call, nav2=None,
                 task_watcher=None, pulses=None, open_events=None, registry=None, watching=0,
                 facts=None, source=None, nav2_file=None, nav2_live=None,
                 review=None, network=None, recorder=None, ekf=None, quiet_run=None, stack=None,
                 issues=None):
        self.args = args
        self.stack = stack      # harness.components.Stack: the device parts' Status rows (ui/parts.py)
        self.issues = issues or IssuesFeed()   # device/part/transport problems past their grace
        self.keep_units = False   # Quit's 'keep remote parts': their leases run out instead
        self.stop_adopted = False  # Quit's 'also stop adopted parts' (units another TUI may own)
        self.ekf = ekf          # harness.ekf.EkfRestart, or None when the session cannot restart them
        self.registry, self.watching = registry, watching   # watching: pid of the owner we only watch
        self.detaching = False
        self.foreign_seen, self.guests_seen = {}, set()
        self._review_live = None
        self.review = review     # a ui.review.Review while a past session plays; live work is off
        self.profile = profile
        self.monitor = monitor
        self.network = network
        self.tracking = TrackingFeed()  # model ghost and live scores; session scores recorded at close
        self.rtk = RtkFeed(profile)
        # ui.feeds objects: ticked live, their view() merged into view()
        self.feeds = [health_feed.HealthFeed(), self.tracking, ConditionsFeed(), self.rtk, self.issues]
        self.ros = ros
        self.source = source or LiveSource(monitor, profile)
        self.processes = processes
        self.paths = paths
        self.console = console or Console()
        self.arming = arming
        self.plant = plant
        self.conditions = Conditions(planned=self._planned)   # fed by the ingest path (ts054); unfed it reads unknown
        self.open_keys = open_keys
        self.live_factory = live
        self.clock = clock
        self.foreground = foreground
        self.quiet_run = quiet_run or (lambda argv: subprocess.call(
            argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        self.finalizing, self.finalize_thread = None, None
        self.recording_lock = threading.RLock()
        self.publish_pulses = pulses
        self.journal = Journal()
        # Manual control needs key releases (Kitty protocol) to tell held keys from taps.
        self.open_events = open_events or (lambda: KeyReader(report_events=True))
        self.manual = False
        self.nav2_factory, self._nav2 = nav2, None
        self.nav2_file = nav2_file  # harness.nav2_edit.Nav2File, or None
        self.nav2_live_factory, self._nav2_live = nav2_live, None   # -> harness.nav2_live.Nav2Live
        if nav2_file is not None and nav2_file.applier is None and nav2_live is not None:
            nav2_file.applier = self.apply_nav2_live
        self.watcher_factory, self._watcher = task_watcher, None
        self.run_id = None  # the procedure run in progress; its results carry this id
        self.settings = args.settings
        self.tabs = TabMemory()
        self.minimap = minimap_panel.MiniMap(self.map_state)
        self.layerbox = LayerBox(getattr(self.ros, 'map', None), self.map_state, live=True)
        self.map_keys, self.map_tab_keys = map_key_maps(keymap_module.load()[0].doc)
        self.bag_recording_key = None
        self.bag_scan = (float('-inf'), [], (0, 0, 0))   # (time, sessions, disk) of the Bags tab; re-read every 5 s
        self.focus = 'status'
        self.cursors = dict.fromkeys(CURSORS, 0)
        self.column = None   # Correction tab column; None follows the worst one
        self.activity = 'results'
        self.modal = None
        self.toasts = Toasts()          # health transitions feed it; never takes keys
        self.running = None
        self._wall_offset = None
        self.tags = {}           # procedure, key and run_id of the run in progress
        self.test_pulses = None
        self.draft = None        # unsaved thruster mapping: channel_map, reversed and pwm_chips
        self.pins_unverified = False   # a saved pin change not read back yet: arming refused
        self.grabbed = None      # thruster row being moved with ↑↓
        self.notice = theme.OVERLAY_WARNING or 'Passive monitor; a opens the arm panel'
        self.command = (0.0, 0.0, 0.0)
        self.chords = False             # the manual mode, for the Command glyph's label
        self.manual_speed = None        # the last manual speed this session; None until manual ran
        self.last_advisory = float('-inf')
        self.watchdog_missing = None    # {since, asked, retried, held} while armed with a watchdog off the graph
        self.health = Health(profile, paths.bag.parent, facts, clock, hardware=lambda: self.args.hardware_config)
        self.feeds.append(ToastFeed(self.health))
        if getattr(ros, 'mission', None) is not None:   # ui.feeds.mission.MissionState, fed by ROS
            self.feeds.append(ros.mission)
        self.recwatch = RecWatch(self.clock)
        self.recorder = recorder or Recorder(processes, paths.bag, profile.name)
        self.rec = None                 # latest RecSample, drawn in the header strip
        self.pwm_readback = [theme.DASH] * 4
        self.keys = self.live = None
        self.last_frame = 0.0
        self.last_drawn = None   # segments of the last idle frame, to skip identical ones
        self.last_tick = clock()
        self.quit = False
        self.closed = False
        self.started = clock()
        self.config_file = config_path(args.config)
        self.history = History()
        self.config_backends = backends.file_backends(
            self.config_file, {'hardware': lambda: self.announce(self.watcher.reload('hardware'), observe=False)})
        self.watcher = Watcher()
        self.watcher.add(Watched('hardware', self.config_file, self.load_hardware, self.apply_hardware,
                                 subsystem='hardware'))
        self.watch_own_files()
        self.config_note = self.config_file.name if self.config_file.exists() else 'defaults'
        self.pending_settings = None

    @property
    def armed(self):
        return self.arming is not None and self.arming.armed

    def ack_toasts(self, id=None):
        """Dismiss one toast, or every standing one (the Health popup's x)."""
        self.toasts.ack(id)

    def _planned(self):
        """The directions the characterization has curves for: what the ceiling gates on until a plan narrows it."""
        config = getattr(self.args, 'hardware_config', None) or {}
        from mhseals_testing.procedures import characterize
        effort = characterize.load_settings()['levels'][0] * self.settings.amplitude     # a_hi, as characterize drives it
        return planned_directions(config.get('characterization', {}).get('speed_curves'), effort)

    def snapshot(self):
        snapshot = self.source.snapshot()
        if not self.source.live:
            return snapshot
        if self.nav2_file is not None:
            snapshot['components'] = [self.nav2_file.row()]
        if self.stack is not None:
            snapshot['components'] = snapshot.get('components', []) + parts.feed(self)
        if self.source.live:    # a past session shows no live control state
            snapshot['control'] = self.control()
            if not self.watching:
                snapshot['rec_mode'] = self.recorder.mode   # the Recording row (owner only)
        counts = health_feed.summary(self.engine)
        if counts:
            snapshot['health'] = counts
        if self.rtk.current is not None:
            snapshot['rtk_view'] = self.rtk.current
        return snapshot

    # --- interlocks ------------------------------------------------------------

    def blockers(self):
        reasons = motion_blockers(self.snapshot(), self.profile, self.args.bench_override)
        if self.ros.fault:
            reasons.insert(0, 'ROS fault: ' + self.ros.fault)
        # A file backup is not evidence of the running driver correction, even during a run.
        return reasons + self.staging().blockers()

    def fit_context(self):
        """What a fit records beside its numbers: session, run, battery volts and whether the conditions gate was muted."""
        facts, _ = self.health.facts.read()
        battery = facts.get('battery.voltage')
        muted = bool(self.health.engine.muted & {'conditions.poor', 'conditions.marginal'})
        return dict(session=self.paths.session.parent.name, run=self.run_id,
                    battery_v=getattr(battery, 'value', None), muted=muted)

    def staging(self):
        """Runtime transitions fail closed; the offline backup remains a durable blocker."""
        return RuntimeStaging(self.args.config)

    def offer_restore(self):
        """Keep an interrupted correction backup until actual runtime restoration is supported."""
        if self.staging().pending() is None:
            return
        self.notice = CORRECTION_ACK_REQUIRED

    def pin_refusal(self):
        """Arming waits for the sysfs readback after a pin change (s or M runs it)."""
        return pins.refusal(self)

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
        watch = getattr(self.ros, 'watch_beats', None)
        if watch is not None:
            watch(required)
        names = {node.rsplit('/', 1)[-1] for node in required}
        off = []
        if self.stack is not None:
            off = [f'{item.title} on {item.remote or "this computer"}' for key, item in self.stack.items.items()
                   if names & set(item.nodes) and self.stack.reported(key) is False]
        beats, runs = getattr(self.ros, 'beats', dict)(), getattr(self.ros, 'runs', dict)()
        return arm_refusal(required, nodes), liveness_refusal(required, beats, self.clock(), off,
                                                              runs=runs)

    def thruster_refusal(self):
        """Why the real thruster driver may not start on this computer, or ''.

        Only a device-registry host with the ``thrusters`` capability drives real pins; a laptop
        or the camera host running a boat profile must not, except by arming that host remotely
        (ros/arm_link.py, set up at session start). An unreadable registry refuses too.
        """
        if self.profile.outputs != 'odroid':
            return ''
        try:
            me = self.profile.placed[0]
        except DeviceError as error:
            return f'device registry: {error}'
        if 'thrusters' not in me.capabilities and getattr(self.arming, 'remote', None) is None:
            return f'{me.name} has no thrusters capability in devices.yaml; the driver runs on the boat computer'
        return ''

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
        return reasons + self.plant.conflicts() + self.staging().blockers()

    def motion_blockers(self):
        """When the suite commands, nothing else may publish the command topic."""
        return self.supervision_blockers() + (self.plant.conflicts() if self.armed else [])

    # --- background work, every tick ---------------------------------------------

    def tick(self):
        if getattr(self, '_runner_paused', False):
            self.poll_pause()
            return
        if self.review:
            self.tick_review()
            if self._review_live is not None and self._review_live['source'].live:
                replay = self.source
                try:
                    self.source = self._review_live['source']
                    self.tick_live()
                finally:
                    self.source = replay
            if self.args.review is None and self.armed and self.arming.fault():
                self.stop(self.arming.fault())  # review over a live session still stops on a fault
            return
        self.tick_live()

    def tick_review(self):
        """Advance the replay; reader, decode and ingest data errors end only the review."""
        if getattr(self, '_review_fault', False):
            return
        try:
            self.review.tick(self.clock(), self.clock() - self.last_tick)
            self.last_tick = self.clock()
            self.source.tick(self.clock())
        except (RuntimeError, OSError, ValueError, KeyError) as error:
            self.review.pause()
            self.review.due = None
            message = f'Review playback failed: {error}'
            if self.args.review is None and self._review_live is not None:
                self.leave_review()          # live owners and recording were never touched
            else:
                self._review_fault = True    # standalone replay: stay paused, stop reading
            self.notice = message

    def tick_live(self):
        """Keep the existing live owner work running while review only replaces its view."""
        self.source.tick(self.clock())
        if not self.source.live:
            return  # replay: no host reads, config watching or driving
        self.refresh_advisories()
        self.watch_config()
        if self.feeds:
            snapshot = self.snapshot()
            for feed in self.feeds:
                feed.tick(self, snapshot, self.clock())
        if self.nav2_file is not None:
            self.notice = self.nav2_file.poll() or self.notice
        if self.test_pulses is not None and self.publish_pulses is not None:
            self.publish_pulses(self.test_pulses)
        if self.armed and self.arming.fault():
            self.stop(self.arming.fault(), by='fault')
        self.watch_watchdogs()

    def watch_watchdogs(self):
        """While armed, hold the mux once a required safety watchdog is off the graph WATCHDOG_GRACE.

        The arming gate's own check (watchdog_refusal), read every tick. The hold is sticky: the
        operator releases it (U) only once the watchdogs are back. plant.hold is a fire-and-forget
        publish, so the mux state must show input 'hold' within HOLD_CONFIRM_S of a request: a
        silent one is requested once more, then the session disarms (the driver stops, the MOSFET
        cuts). Without a mux it disarms at once. A dead ``~/status`` beat (silent while another
        watchdog's beats on, so never a radio dropout) or a report of off holds without the grace:
        the beat's 2 s staleness already is one.
        """
        graph, live = self.watchdog_reasons() if self.armed else ('', '')
        reason = '; '.join(filter(None, (graph, live)))
        if not reason:
            self.watchdog_missing = None
            return
        now = self.clock()
        state = self.watchdog_missing
        if state is None:
            state = self.watchdog_missing = dict(since=now, asked=None, asked_mux=None,
                                                        retried=False, held=False)
        if live:
            state['since'] = min(state['since'], now - WATCHDOG_GRACE)
        if state['held'] or now - state['since'] < WATCHDOG_GRACE:
            return
        if self.plant is None or not muxed(self.profile):
            self.stop(reason, by='fault')
            return
        if state['asked'] is None:
            self.request_watchdog_hold(state, now, reason)
        elif self.mux_holding(state['asked_mux']):
            state['held'] = True
        elif now - state['asked'] >= HOLD_CONFIRM_S:
            if state['retried']:
                self.stop(f'{reason}; no mux hold seen', by='fault')
            else:
                state['retried'] = True
                self.request_watchdog_hold(state, now, reason)

    def request_watchdog_hold(self, state, now, reason):
        """Ask the mux to hold and note when: ``now`` on the app's clock for the confirm timeout,
        and the arbiter's own clock (wall time), which stamps the mux state, for the confirmation."""
        arbiter = getattr(self, 'arbiter', None)
        state['asked'], state['asked_mux'] = now, arbiter.clock() if arbiter is not None else None
        self.plant.hold('watchdog')
        self.ros.emit('watchdog_hold', reason=reason)
        self.monitor.event('HOLD: ' + reason)
        self.notice = f'Holding: {reason}'

    def mux_holding(self, since):
        """True when the mux reported input 'hold' at or after ``since`` (the arbiter's clock)."""
        arbiter = getattr(self, 'arbiter', None)
        if arbiter is None or since is None:
            return False
        with arbiter.lock:
            mux, at = arbiter.mux, arbiter.mux_at
        return bool(mux) and mux.get('input') == 'hold' and at is not None and at >= since

    def refresh_advisories(self):
        """Sample the non-blocking checks at most once per ADVISORY_PERIOD."""
        if self.clock() - self.last_advisory < ADVISORY_PERIOD:
            return
        self.last_advisory = self.clock()
        seen, live = self.foreign_instances()
        self.feed_map()
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
        network = self.network.poll() if self.network else None
        if network:
            self.monitor.advise('network', *network)
        if getattr(self.network, 'clocks', None):       # pre.clock_sync judges clock.spread
            self.health.facts.publish('clock_sync', self.network.clock_facts)
        self.monitor.advise('recording', 'OK' if self.processes.running('bag') else 'FAIL',
                            str(self.paths.bag))
        self.watch_recording()
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

    def feed_map(self):
        """The recorded costmap follows the Recorder mode and the session owner (map.md 4)."""
        feeds = getattr(self.ros, 'map', None)
        if feeds is not None:
            nav = any(n.rsplit('/', 1)[-1] == 'planner_server' for n in self.ros.nodes())
            feeds.want_record(not self.watching, self.recorder.mode != 'off', nav)

    def watch_recording(self):
        """Sample the recorder's health; below the free-disk floor stop it, mark it and say so."""
        self.rec = self.recwatch.sample(self.paths.bag, self.processes.running('bag'),
                                       self.recorder.mode)
        if self.rec.stop_for_disk:
            self.ros.emit('marker', kind='system', note=DISK_MARKER)   # first, so it lands in the bag
            self.set_recording('off', reason='disk')
            self.notice = f'{DISK_MARKER} ({self.rec.free_gb:.1f} GB free)'

    def set_recording(self, mode, reason='operator'):
        """Recording row: end the running segment (``rec_stop``), start the next unless ``off`` (``rec_start``)."""
        with self.recording_lock:
            if mode != 'off' and self.paths.directory.resolve() == self.finalizing:
                self.notice = 'Recording refused while this session is being finalized'
                return
            self._set_recording(mode, reason)

    def _set_recording(self, mode, reason):
        if self.recorder.segment is not None:
            self.ros.emit('rec_stop', segment=self.recorder.segment.name, reason=reason)
        segment = self.recorder.set_mode(mode)
        self.ros.recording = segment is not None  # events stamp ``rec`` from this (bags.md 1)
        self.rebuild_index()                      # INDEX.json at each segment close
        if segment is not None:
            self.ros.emit('rec_start', segment=segment.name, mode=mode)
        self.notice = f'Recording {mode}' + (f' {segment.name}' if segment else '')

    def rebuild_index(self):
        """Rewrite INDEX.json and README.md; a failure is logged to the notice, never fatal."""
        sessions = self.paths.directory.parent
        if self.watching or self.args.review is not None or sessions.name != 'sessions':
            return    # an explicit --output outside a data root has no index to keep
        try:
            index.rebuild(sessions.parent)
        except OSError as error:
            self.notice = f'Index not rebuilt: {error}'

    def step_recording(self, key):
        """left/right on the Recording row moves along lean, full, off without wrapping."""
        i = REC_MODES.index(self.recorder.mode) + (1 if key == 'right' else -1)
        if 0 <= i < len(REC_MODES):
            self.set_recording(REC_MODES[i])

    # --- hardware config: edits from the panel or any editor, pins restart the driver ---

    def watch_config(self):
        """Poll every watched file once; announce what changed (harness/watch.py)."""
        for change in self.watcher.poll():
            self.announce(change)
        self.adopt_health()
        self.monitor.advise('Config', *config_popup.summary(self.watcher, self.history))

    def announce(self, change, observe=True):
        """Show a reload's result and emit its event; returns the Change for callers that check ``ok``.

        ``observe`` logs a hand edit to the history; a revert passes False, it records itself."""
        self._announce(change, observe)
        return change

    def _announce(self, change, observe=True):
        watched = self.watcher.watches.get(change.name)
        if observe and change.ok and watched is not None and watched.subsystem:
            # A hand edit reaches the history now; our own save was recorded
            # already, so this finds no difference.
            adapters.observe_file(self.history, watched.subsystem, watched.path, self.warn_history)
        if not change.ok:
            if change.name == 'hardware':
                self.config_note = 'invalid edit'
            if change.report:
                self.notice = f'Config rejected, keeping the last good one: {change.error}'
            return
        if change.name == 'hardware':
            self.config_note = watched.path.name
        self.notice = (f'Config read from {watched.path.name}{theme.SEPARATOR}{watched.warning}' if watched.warning
                       else f'Config reloaded from {watched.path.name}')
        self.ros.emit('config_reloaded', file=change.name, sha=change.sha, result='ok',
                      path=str(watched.path))

    def watch_own_files(self):
        """Hot-reload the TUI's own YAML in the same poll; nothing here touches ROS."""
        def load_keymap(_path):
            keymap, warning = keymap_module.load(user=keymap_module.USER_FILE)
            if warning:
                raise ValueError(warning)
            return keymap

        def load_theme(_path):
            new, warning = palette.load_theme()
            if warning:
                raise ValueError(warning)
            return new

        def apply_keymap(keymap):
            self.map_keys, self.map_tab_keys = map_key_maps(keymap.doc)

        def apply_theme(new):
            theme.use_palette(new)
            self.minimap.redraw.mark()

        def apply_minimap(table):
            self.minimap.table = table
            self.minimap.redraw.mark()

        def apply_held(_value):     # read while armed or running: say it waits, not that it reloaded
            self.adopt_health()
            return dict(status='unknown', detail=HELD) if self.armed or self.running else None

        def apply_settings(new):    # procedures read settings each step: held while one runs
            self.pending_settings = new
            return apply_held(new)

        for watched in (
                Watched('keymap', keymap_module.USER_FILE, load_keymap, apply_keymap,
                        subsystem='keymap'),
                Watched('theme', palette.OVERLAY, load_theme, apply_theme),
                Watched('minimap', classes_module._default_path(), classes_module.load_classes,
                        apply_minimap),
                Watched('settings', settings.settings_path(),
                        lambda path: settings.load(path, strict=True), apply_settings),
                Watched('health', health_config.USER_FILE, lambda _path: self.health.reload(),
                        apply_held, subsystem='health')):
            self.watcher.add(watched)

    def adopt_health(self):
        """Health rules and settings are read while armed or running: edits wait for neither."""
        if self.armed or self.running:
            return
        for watched in self.watcher.watches.values():
            if watched.warning == HELD:
                watched.warning = None
        self.health.adopt_pending()
        new, self.pending_settings = self.pending_settings, None
        if new is not None:     # a key the file dropped goes back to its default
            for spec in settings.SPECS:
                if not spec.reset:
                    setattr(self.settings, spec.key, getattr(new, spec.key))

    def load_hardware(self, _path):
        return load_config(self.args.config)

    def apply_hardware(self, config):
        """Take ``config`` and report what the hardware did with it, not just that it was saved.

        Arming a remote thrusters host, the boat's hardware.yaml is the only config: a local edit or
        reload is refused and never replaces it (it is read from the boat at session start and each arm).
        """
        remote = getattr(self.arming, 'remote', None) if self.arming is not None else None
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
            self.manual_speed = None   # a new configured default wins over the session's last speed
        if self.arming is not None:
            self.arming.hardware_config = config  # arming saves this copy
        if not driven:
            return dict(status='saved')
        sha = config_sha(load_config(self.paths.hardware))
        end = time.monotonic() + 4.0   # the driver polls the file at 1 Hz
        while True:
            status = getattr(self.ros, 'driver_status', None) or {}
            if status.get('requested_config_sha') == sha and any(
                    name.endswith('/thruster_pwm_node') for name in self.ros.nodes()):
                break
            if time.monotonic() >= end:
                return dict(status='unknown', detail='driver did not confirm the edit')
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                time.sleep(0.1)
            if key in ('space', 'escape', 'press:space', 'press:escape'):
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
        self.notice = message

    def observe_config(self):
        """At start: log config files edited since the last session as external changes."""
        adapters.observe_all(self.history, {'hardware': self.config_file}, self.warn_history)

    def save_hardware(self, **changes):
        """Write ``changes`` into the operator's hardware YAML; the watcher applies it.

        Returns False, keeping the draft, when the file on disk is broken, or when a remote boat's
        hardware.yaml is the config (refused before any write).
        """
        remote = getattr(self.arming, 'remote', None) if self.arming is not None else None
        if remote is not None:
            self.notice = f"Not saved: the boat's hardware.yaml is the config, edit it on {remote.host.name}"
            return False
        adapters.observe_file(self.history, 'hardware', self.config_file, self.warn_history)   # a hand edit first
        try:
            update_config(changes, self.args.config)
        except (OSError, ValueError, TypeError, yaml.YAMLError):
            self.notice = f'Fix hardware.yaml first: {self.config_file}'
            return False
        adapters.record_file(self.history, 'hardware', 'tui:hardware', self.config_file, self.warn_history)
        change = self.announce(self.watcher.reload('hardware'))  # skip the settle wait
        if not change.ok:
            return False
        self.draft = self.grabbed = None
        return True

    # --- rendering ---------------------------------------------------------------

    @property
    def tab(self):
        return self.tabs.current

    def panels(self):
        if self.tab != 'Drive':
            return [self.focus]
        width = self.console.size.width
        body = body_rows(self.console.size.height, bool(self.review))
        return [p for p in PANELS if (p != 'motion' or motion_shown(width, body))
                and (p != 'minimap' or width >= MINIMAP)]

    def rows(self, panel):
        if panel == 'status':
            return len(status_rows(self.snapshot(), self.profile.sensors))
        if panel == 'thrusters':
            return 4
        if panel == 'list':
            return len(self.bags_view()[0])
        if panel == 'procedures':
            return len(PROCEDURES)
        if panel == 'activity':
            if self.activity == 'agent':
                return len(agent_log.rows(feeds_module.merged(self.feeds).get('agent_desk')))
            snapshot = self.snapshot()
            return len(snapshot['events' if self.activity == 'log' else 'results'])
        return 0

    def review_hints(self):
        """REVIEW_HINTS, with `z Loupe` only while Activity (where z acts) is focused."""
        i = next(k for k, h in enumerate(REVIEW_HINTS) if h[0] == 'Esc')
        return REVIEW_HINTS[:i] + ([('z', 'Loupe')] if self.focus == 'activity' else []) + REVIEW_HINTS[i:]

    def view(self):
        config = self.args.hardware_config
        blockers = ([] if self.review else
                    self.motion_blockers() if self.armed else self.blockers())
        notice = self.notice if not blockers or self.running else (
            f'{self.notice}{theme.SEPARATOR}blocked: {blockers[0]}')
        count = self.rows(self.focus)
        hints = (self.review_hints()
                 if self.review else MANUAL_HINTS if self.manual
                 else PANEL_HINTS.get(self.focus, []) + GLOBAL_HINTS)
        if self.tab == 'Correction' and not self.review:
            hints = CORRECTION_HINTS
        if self.nav2_file is not None and self.focus == 'status' and not (self.manual or self.review):
            hints = PANEL_HINTS['status'] + [('e', 'Nav2')] + GLOBAL_HINTS
        shared = {} if self.review else feeds_module.merged(self.feeds)
        own = dict(
            tab=self.tab, minimap=self.minimap, layerbox=self.layerbox,
            **(dict(zip(('bags', 'bag_usage'), self.bags_view())) if self.tab == 'Bags' else {}), focus=self.focus, cursors=self.cursors, modal=self.modal, running=self.running,
            activity=self.activity, now=self.monitor.clock(),
            toasts=self.toasts.visible(self.monitor.clock(), bool(self.modal)),
            toasts_more=self.toasts.hidden(self.monitor.clock()), notice=notice, hints=hints,
            position=f'{min(self.cursors[self.focus], count - 1) + 1}/{count}' if count else '',
            focus_title=self.correction_focus() if self.tab == 'Correction' else None,
            authority=self.driving(shared),
            agent=authority_chip.banner(shared.get('authority_record'), time.time(), desk=shared.get('agent_desk')),
            armed=self.armed, arm_label=('armed' + theme.SEPARATOR + self.profile.outputs)
            if self.armed else 'disarmed', command=self.command, chords=self.chords,
            speed=self.manual_speed if self.manual_speed is not None
            else (self.args.hardware_config or {}).get('manual_amplitude'),
            profile=self.profile.name, units=self.profile.command_units,
            plot_scale=self.profile.plot_scale,
            outputs=self.profile.outputs, specs=self.profile.sensors,
            deploy={} if self.review else self.health.deploy(), host=socket.gethostname(),
            domain=os.environ.get('ROS_DOMAIN_ID', '0'),
            recording=False if self.review else self.processes.running('bag'), rec=self.rec,
            bench_override=self.args.bench_override,
            hardware=config, matrix=validate_mixer(config['thruster_matrix']),
            test_pulses=self.test_pulses, pwm_readback=self.pwm_readback,
            draft=self.draft, grabbed=self.grabbed,
            config_note=self.config_note, review=self.review, correction=self.correction_state(),
            procedures=[dict(key=p.key, title=p.title, description=p.description, group=p.group,
                             reason=self.availability(p)) for p in PROCEDURES])
        if self.review and not self.source.live:
            own.update(profile=self.source.profile.name, specs=self.source.profile.sensors,
                       units=self.source.profile.command_units,
                       outputs=self.source.profile.outputs,
                       plot_scale=self.source.profile.plot_scale, now=self.source.now,
                       host=self.review.host, deploy={}, recording=False, rec=None,
                       pwm_readback=[theme.DASH] * 4, toasts=[], toasts_more=0)
        return {**shared, **own}  # App's own keys win a collision

    def driving(self, shared=None):
        """The authority chip: from the arbiter's record, stale-aware (ui U4); local guess without one."""
        return authority_chip.driving(shared if shared is not None else feeds_module.merged(self.feeds),
                                      getattr(self, 'arbiter', None), self.manual, self.running, time.time())

    def control(self):
        """``snapshot['control']`` for the Status Control rows."""
        record = feeds_module.merged(self.feeds).get('authority_record')
        return authority_chip.control(record, self.driving(), time.time(), self.settings.agents,
                                      agent_motion=self.settings.agent_motion)

    def render(self):
        return layout.render(self.snapshot(), self.view(),
                                self.console.size.width, self.console.size.height)

    def refresh(self, force=True):
        """Draw now. Continuous callers pass force=False to cap redraws at 1/FRAME."""
        if self.live is None:
            return
        now = time.monotonic()
        if not force and now - self.last_frame < FRAME:
            return
        self.last_frame = now
        frame = self.render()
        if not force:
            # Idle ticks: skip the write when the frame is byte-identical to the last one,
            # and hand Live the segments already laid out instead of rendering twice.
            drawn = list(self.console.render(frame))
            if drawn == self.last_drawn:
                return
            self.last_drawn = drawn
            frame = Segments(drawn)
        else:
            self.last_drawn = None
        self.live.update(frame, refresh=True)

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
                        self.refresh(force=key is not None)  # idle ticks redraw only on change
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
            if self.pins_moved():   # pins save only through s, which reads them back
                rows.append(('Pins', 'unsaved pin change is dropped'))
            options = ['Save mapping and quit', 'Quit without saving', 'Cancel']
        remote = [] if self.watching else parts.quit_rows(self)
        rows += remote        # asked once; Quit stops only what this TUI started
        if any(value.endswith('Quit stops them') for _, value in remote):
            options.insert(-1, 'Quit, keep remote parts running')
        if any(value.endswith('Quit keeps them running') for _, value in remote):
            options.insert(-1, 'Quit, also stop adopted parts')
        try:
            choice = self.choose('Quit', options, {'y': 0, 'n': len(options) - 1}, rows=rows,
                                 default=len(options) - 1, width=72)
        except Cancelled:
            return
        if choice == 'Cancel':
            return
        self.keep_units = choice == 'Quit, keep remote parts running'
        self.stop_adopted = choice == 'Quit, also stop adopted parts'
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
        self.health.close()
        if self.args.review is not None:
            return  # nothing of ours to flush; the session's report must not be rewritten
        steps = [('Outputs', self.close_outputs), ('Children', self.close_children),
                 ('Remote parts', self.close_units), ('Report', self.close_report), ('Tracking', self.close_tracking),
                 ('Calibration', self.close_calibration),
                 ('Recording', self.close_recording), ('Index', self.close_index)]
        if self.detaching:   # the children and the recording go on under the next owner
            steps = [('Outputs', self.close_outputs), ('Handover', self.close_handover)]
        if self.watching:    # not ours to stop: neutral, events and the report belong to the owner
            steps = [('Watch', lambda: f'left pid {self.watching} running')]
        rows = [[name, theme.icon('busy').plain + ' waiting'] for name, _ in steps]
        self.modal = dict(kind='message', title='Closing session', rows=rows, width=64,
                          hint='Ctrl+C again skips the rest')
        for row, (_, step) in zip(rows, steps):
            row[1] = theme.icon('busy').plain + ' working'
            self.refresh()
            try:
                row[1] = theme.icon('ok').plain + ' ' + step()
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

    def close_units(self):
        """Stop the units we started on other computers (Quit's default), or leave them to their
        lease; adopted ones are stopped only on the operator's explicit choice."""
        units = getattr(self.stack, 'units', None)
        return units.close(stop=not self.keep_units, adopted=self.stop_adopted) \
            if units is not None else 'none'

    def close_report(self):
        self.ros.emit('session_stop')
        return self.export().name

    def close_tracking(self):
        verdicts = self.tracking.close()
        return ', '.join(f"{a} {'stale' if v['flag'] else 'ok'}" for a, v in verdicts.items()) or 'no scores'

    def close_calibration(self):
        wrote = calibration.write_snapshot(self.args.hardware_config)
        return 'status written' if wrote else 'nothing fitted'

    def close_recording(self):
        if not self.processes.running('bag'):
            return 'was not running'
        self.set_recording('off', reason='quit')  # SIGINT lets ros2 bag finish the file
        return f'flushed {self.paths.directory.name}'

    def close_index(self):
        session_module.finish(self.paths.directory)
        self.rebuild_index()
        return 'INDEX.json rebuilt'

    def summary(self):
        """What the session left behind, printed once the page is gone."""
        results = self.snapshot()['results']
        passed = sum(1 for r in results if theme.kind(r.get('verdict', '')) == 'good')
        minutes = (self.clock() - self.started) / 60
        return [('Duration', f'{minutes:.1f} min'), ('Results', f'{passed}/{len(results)} good'),
                ('Bag', str(self.paths.bag)), ('Report', str(self.paths.report))]

    def focus_on(self, panel):
        if self.focus == 'thrusters' and panel != 'thrusters':
            self.clear_pulses()  # test pulses only while their panel has focus
        self.focus = panel

    def map_state(self):
        """The ``MapState`` the ROS feeds fill, or None without them."""
        if not self.source.live:
            return None
        return getattr(getattr(self.ros, 'map', None), 'state', None)

    def show_tab(self, name):
        """Switch to the tab ``name`` (focus and cursors are restored by TabMemory)."""
        step = self.tabs.tabs.index(name) - self.tabs.tabs.index(self.tab)
        if step:
            self.switch_tab(step)

    def on_minimap(self, key, keys=None):
        """view.minimap keys (Drive column and Map tab): zoom, and the Layers box toggles."""
        if key in ('z', 'Z'):
            self.minimap.zoom(1 if key == 'z' else -1)
        elif key == '0':
            self.minimap.reset()
        else:
            self.layerbox.press((keys or self.map_keys).get(key), self.minimap.half)

    def on_map(self, key):
        """The Map tab adds tab.map's toggles, whose keys are Drive procedures elsewhere."""
        self.on_minimap(key, self.map_tab_keys)

    def recording_paths(self):
        """Authoritative live session and exact writer segment, resolved at action time."""
        if not self.processes.running('bag'):
            return None, None
        segment = self.recorder.segment
        return self.paths.directory.resolve(), segment.resolve() if segment is not None else None

    def bag_protected(self, bag):
        session, segment = self.recording_paths()
        path = bag.path.resolve()
        return path == session or (segment is not None and (path == segment or path in segment.parents))

    def invalidate_bags(self):
        self.bag_scan = (float('-inf'), *self.bag_scan[1:])

    def bags_view(self):
        """One lazy cache; writer transitions invalidate even within the five-second interval."""
        now = self.clock()
        recording_key = self.recording_paths()
        if recording_key != self.bag_recording_key or now - self.bag_scan[0] >= 5:
            recording = {recording_key[0].name} if recording_key[0] is not None else ()
            try:
                self.bag_scan = (now, bags_module.scan(recording=recording), bags_module.disk())
            except OSError:
                self.bag_scan = (now, [], (0, 0, 0))
            self.bag_recording_key = recording_key
            self.cursors['list'] = max(0, min(self.cursors['list'], len(self.bag_scan[1]) - 1))
        return self.bag_scan[1], self.bag_scan[2]

    def selected_bag(self):
        found = self.bags_view()[0]
        if not found:
            self.notice = 'No sessions'
            return None
        return found[self.cursors['list']]

    def check_bag(self, bag, destructive=False, metadata=None):
        """Revalidate live paths and metadata; cached row flags never authorize a mutation."""
        if self.bag_protected(bag):
            raise PermissionError(f'{bag.id} is recording')
        if bag.path.resolve() == getattr(self, 'finalizing', None):
            raise PermissionError(f'{bag.id} is being finalized')
        header = metadata if metadata is not None else data.read_json(bag.meta_path, {})
        bag.pinned = bool(header.get('pinned'))
        if destructive and bag.pinned:
            raise PermissionError(f'{bag.id} is pinned')

    def refresh_bag_index(self, bag):
        root = bag.path.parent if bag.legacy else bag.path.parent.parent
        try:
            index.rebuild(root)
        finally:
            self.invalidate_bags()

    def on_list(self, key):
        handler = {'enter': self.open_bag_review, 'p': self.pin_bag,
                   'n': self.title_bag, 'd': self.delete_bag,
                   'P': self.prune_bags, 'F': self.finalize_bag}.get(key)
        if handler is None:
            return
        bag = None if key == 'P' else self.selected_bag()
        if key != 'P' and bag is None:
            return
        try:
            handler(bag)
        except Cancelled:
            pass
        except (OSError, ValueError) as error:
            self.notice = f'{key}: {error}'

    def pin_bag(self, bag):
        self.check_bag(bag)
        bags_module.set_meta(bag, pinned=not bag.pinned)
        self.refresh_bag_index(bag)
        self.notice = f'{"Pinned" if bag.pinned else "Unpinned"} {bag.id}'

    def title_bag(self, bag):
        self.check_bag(bag)
        title = self.ask_text('Title', f'Title for {bag.id}', bag.title)
        if title is not None:
            self.check_bag(bag)
            bags_module.set_meta(bag, title=title.strip())
            self.refresh_bag_index(bag)
            self.notice = f'Titled {bag.id}'

    def delete_bag(self, bag):
        self.check_bag(bag, destructive=True)
        if self.choose(f'Delete {bag.id}?', ('Delete', 'Cancel'), {'y': 0}, default=1,
                       danger=True, message='Session files are removed for good.') == 'Delete':
            self.check_bag(bag, destructive=True)
            try:
                bags_module.delete(bag)
            finally:
                self.refresh_bag_index(bag)
            self.notice = f'Deleted {bag.id}'

    def prune_bags(self, _bag):
        eligible = []
        for bag in self.bags_view()[0]:
            try:
                self.check_bag(bag, destructive=True)
            except PermissionError:
                continue
            eligible.append(bag)
        # Keep protected sessions in the total budget, but never among candidates.
        protected = [b for b in self.bags_view()[0] if b not in eligible]
        budget = bags_module.budget_bytes() - sum(b.size for b in protected)
        doomed = bags_module.prune_candidates(eligible, budget)
        if not doomed:
            self.notice = 'Nothing to prune'
            return
        if self.choose(f'Prune {len(doomed)} sessions?', ('Delete', 'Cancel'), {'y': 0},
                       default=1, danger=True, rows=[(b.id, '') for b in doomed]) != 'Delete':
            return
        attempted = False
        try:
            for bag in doomed:
                self.check_bag(bag, destructive=True)
                attempted = True
                bags_module.delete(bag)
            self.notice = f'Pruned {len(doomed)} sessions'
        finally:
            if attempted:
                self.refresh_bag_index(doomed[0])

    def finalize_bag(self, bag):
        """Claim one session before starting the quiet worker; recording cannot enter it."""
        with self.recording_lock:
            if self.armed or self.moving():
                self.notice = 'Finalize refused while armed or moving'
                return
            if self.finalizing is not None:
                self.notice = 'Finalize is still running'
                return
            self.check_bag(bag)
            segments = sorted({f.parent for f in bag.path.rglob('*.mcap')
                               if not (f.parent / 'metadata.yaml').exists()})
            if not segments:
                self.notice = f'{bag.id}: nothing to finalize'
                return
            self.finalizing = bag.path.resolve()
            self.notice = f'Finalize {bag.id}: {len(segments)} segments in the background'
            self.finalize_thread = threading.Thread(target=self._finalize, args=(bag, segments), daemon=True)
            try:
                self.finalize_thread.start()
            except Exception:
                self.finalizing = None
                raise

    @staticmethod
    def segment_stamp(segment):
        """Detect cleanup even if reindex fails after removing an empty MCAP."""
        paths = [*segment.glob('*.mcap'), segment / 'metadata.yaml']
        return [(p.name, p.stat().st_size, p.stat().st_mtime_ns) for p in paths if p.exists()]

    def _finalize(self, bag, segments):
        changed, results = False, []
        try:
            for segment in segments:
                if self.bag_protected(bag):
                    results.append('refused')
                    break
                before = self.segment_stamp(segment)
                try:
                    # Read the actual segment at execution time, never paths.bag.
                    result = bags_module.finalize(segment, self.quiet_run, self.recording_paths()[1])
                    results.append(result['status'])
                finally:
                    changed |= before != self.segment_stamp(segment)
            self.notice = f'Finalize {bag.id}: ' + ', '.join(
                f'{results.count(r)} {r}' for r in dict.fromkeys(results))
        except Exception as error:
            self.notice = f'Finalize {bag.id}: {error}'
        finally:
            try:
                if changed:
                    self.refresh_bag_index(bag)
            except Exception as error:
                self.notice = f'Finalize {bag.id}: index update failed: {error}'
            finally:
                with self.recording_lock:
                    self.finalizing = None

    def switch_tab(self, step):
        """Move to the next or previous tab, restoring the focus and cursors it was left with."""
        if self.focus == 'thrusters':
            self.clear_pulses()  # test pulses only while their panel has focus
        self.grabbed = None
        self.focus, saved = self.tabs.switch(step, self.focus, self.cursors)
        self.cursors = {**dict.fromkeys(CURSORS, 0), **saved}

    def move(self, step):
        count = self.rows(self.focus)
        if count:
            self.cursors[self.focus] = max(0, min(count - 1, self.cursors[self.focus] + step))

    def dispatch(self, key):
        """Apply one key on the page."""
        if self.review:
            folded = fold(key)
            if self.dispatch_review(folded) or not self.review_allows(folded):
                return
        if key == 'V' and not self.review:  # Shift-V; folded 'v' still toggles Results/log on Activity
            self.perception_popup(view=True)  # ui.md keymap camera: View already open
            return
        if key == 'H':  # Shift-H opens the history everywhere; folded 'h' still adjusts a setting
            self.open('history')
            return
        if key in ('E', 'U'):  # Shift bindings of the global scope; never the lowercase procedure letters
            if key == 'E':
                self.revoke_agent()
            else:
                self.release_hold()
            return
        raw, key = key, fold(key)
        if raw == 'M' and self.focus == 'thrusters' and not self.review:  # keymap pins: M; folded 'm' stays Manual
            self.notice = pins.edit_pins(self)
            return
        if raw == 'P' and self.focus == 'thrusters' and not self.review:  # keymap pin: P; p stays Perception
            self.notice = pins.edit_row_pin(self, self.cursors['thrusters'])
            return
        if raw == 'X' and self.focus == 'activity' and not self.review and self.activity == 'results':
            self.exclude_result()  # panel-level Exclude run; folded 'x' still writes the report
            return
        if key == 'o':  # the Pose popup is global
            self.open('odometry')
            return
        if raw == '!' and self.engine is not None:  # keymap global health
            self.open('health')
            return
        if self.tab == 'Correction' and not self.review and self.on_correction(raw):
            return
        panels = self.panels()
        if key in ('[', ']'):
            self.switch_tab(-1 if key == '[' else 1)
        elif key in ('\t', 'backtab'):
            index = panels.index(self.focus) if self.focus in panels else 0
            self.focus_on(panels[(index + (-1 if key == 'backtab' else 1)) % len(panels)])
        elif key in DIRECTIONS:
            wanted = NEIGHBOURS[self.console.size.width >= ROW].get(self.focus, {}).get(DIRECTIONS[key], ())
            neighbour = next((p for p in ((wanted,) if isinstance(wanted, str) else wanted) if p in panels), None)
            if neighbour:
                self.focus_on(neighbour)
        elif key in ('up', 'down', 'j', 'k'):
            step = -1 if key in ('up', 'k') else 1
            if self.focus == 'thrusters' and self.grabbed is not None and not self.review:
                self.move_grabbed(step)
            else:
                self.move(step)
        elif key == 'space':
            self.operator_stop()
        elif key == 'q':
            self.confirm_quit()
        elif key == 'escape':  # backs out; never quits
            if self.grabbed is not None:
                self.grabbed = None
                self.notice = 'Dropped'
        elif key == '?':
            self.show('Help', rows=[(self.profile.name, self.profile.description), *HELP], width=76)
        elif key == 'a':
            self.arm_panel()
        elif key == 'x' and raw == 'x':
            if not self.toasts.ack_newest(self.monitor.clock()):   # x dismisses a toast first
                self.export()
        elif key == 'b':
            self.mark_note() if raw == 'B' else self.mark()
        elif key in ('+', '=', '-', '_'):
            step = 0.05 if key in ('+', '=') else -0.05
            self.settings.amplitude = round(min(1.0, max(0.05, self.settings.amplitude + step)), 2)
            self.notice = f'Amplitude {self.settings.amplitude:g} {self.profile.command_units}'
            settings.save(self.settings)
            self.watcher.reload('settings')   # our own save is not a hand edit
        elif (key in {p.key for p in PROCEDURES} and raw == key and self.tab == 'Drive'  # Shift is never a procedure
              and not (self.focus == 'thrusters' and key in ('1', '2', '3', 'c', 's'))
              and not (self.focus == 'activity' and key == 'v')
              and not (self.focus == 'status' and key == 'e')  # Nav2 popup there; interference elsewhere
              and not (self.focus == 'minimap' and key in self.map_keys)):  # view.minimap wins (e centres)
            self.on_procedures(key)
        elif self.focus == 'list':
            self.on_list(raw)
        elif self.focus in ('minimap', 'map'):
            getattr(self, f'on_{self.focus}')(raw)
        elif raw == key:  # a Shift letter has no panel binding yet; folding it hit the lowercase ones
            getattr(self, f'on_{self.focus}', lambda key: None)(key)

    def revoke_agent(self):
        """E: revoke the agent's active claim (stops the boat first); no confirm."""
        arbiter = getattr(self, 'arbiter', None)
        holder = arbiter.table.holder() if arbiter is not None else None
        if holder is None or holder.get('owner') != 'agent':
            self.notice = 'No agent claim to revoke'
        else:
            arbiter.revoke(holder['id'])
            self.notice = 'Agent revoked'

    def release_refusal(self):
        """Only fresh, known mux state can authorize an operator release request."""
        watchdogs = self.watchdog_refusal() if self.armed else ''
        if watchdogs:
            return f'Release refused: {watchdogs}'
        arbiter = getattr(self, 'arbiter', None)
        if arbiter is None:
            return 'Release refused: mux state is unknown'
        with arbiter.lock:
            mux, at = arbiter.mux, arbiter.mux_at
        if (mux is None or at is None or not 0 <= arbiter.clock() - at <= STALE_MUX_S
                or mux.get('input') not in ('auto', 'idle', 'hold', 'manual', 'rc')):
            return 'Release refused: mux state is stale or unknown'
        rc = mux.get('rc') or {}
        mode = str(rc.get('mode') or '').upper()
        if rc.get('valid') and mode not in ('GUIDED', 'HOLD', 'MANUAL'):
            return 'Release refused: RC switch state is unknown'
        if mode in ('HOLD', 'MANUAL'):
            return f'Release refused: RC switch is {mode}'
        return ''

    def release_resumer(self):
        """Describe the prospective driver and retain its identity across confirmation."""
        holder = self.arbiter.table.holder()
        if holder:
            identity = tuple(holder.get(key) for key in ('id', 'owner', 'who', 'reason'))
            return ('claim', *identity), holder.get('who') or holder.get('reason') or holder['id']
        if self.running:
            return ('procedure', self.running, self.run_id), self.running
        with self.arbiter.lock:
            auto = (self.arbiter.mux or {}).get('auto') or {}
        # No claim does not establish idle: an unclaimed publisher can drive after release.
        label = 'unclaimed automation' if auto.get('active') else 'unknown automation (if present)'
        return ('unclaimed', auto.get('active'), auto.get('node')), label

    def release_hold(self):
        """Confirm who resumes, then request release once; only mux state proves release.

        True when the release was requested."""
        if getattr(self, '_releasing', False):
            return False
        release = getattr(self.plant, 'release_hold', None)
        if release is None:
            self.notice = 'Release hold is not available here'
            return False
        refusal = self.release_refusal()
        if refusal:
            self.notice = refusal
            return False
        identity, resumer = self.release_resumer()
        self._releasing = True
        requested = False
        try:
            answer = self.choose('Release hold?', ['Release', 'Cancel'],
                                 message=f'{resumer} resumes', default=1, danger=True)
            if answer == 'Release':
                refusal = self.release_refusal()
                if not refusal and self.release_resumer()[0] != identity:
                    refusal = 'Release refused: resumer changed; confirm again'
                requested = not refusal and release()
                self.notice = refusal or ('Release requested' if requested
                                          else 'Release hold is not available here')
        except Cancelled:
            pass
        finally:
            self._releasing = False
        return requested

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
            note = self.text_input(f'Marker {number}', 'Note for the marker (optional)')
            if note:
                self.ros.emit('marker_note', id=number, note=note)

    def text_input(self, title, message):
        """A one-line text popup; the typed text on Enter, None on Esc."""
        modal = dict(kind='input', title=title, message=message, text='', width=64,
                     placeholder='add a note', hint=f'Enter save{theme.SEPARATOR}Esc keeps it bare')

        def handle(key):
            if key == 'enter':
                return modal['text'].strip()
            if key == 'escape':
                return ''
            modal['text'] = popups.edit(modal['text'], key)
        return self.popup(modal, handle) or None

    # --- review ---------------------------------------------------------------------

    def dispatch_review(self, key):
        """Review's own keys (context global -> review). True when the key was consumed.

        Keys that would act on the boat (arm, procedures, export) are refused here, so the
        live paths never see them; focus, help and quit fall through to the page.
        """
        review, now = self.review, self.clock()
        if key in REVIEW_KEYS:
            seconds, sign = REVIEW_KEYS[key]
            review.seek(sign * seconds, now)
        elif key in (',', '.'):
            review.jump(-1 if key == ',' else 1, now)
        elif key in ('<', '>'):
            review.faster(-1 if key == '<' else 1)
        elif key == 'z' and self.focus == 'activity':        # review.activity zoom
            review.loupe = scrubber.next_loupe(review.loupe)
        elif key == 'p':
            review.toggle()
        elif key == 'space':
            self.operator_stop()  # Keep the global sticky hold and neutral/disarm safety.
            review.pause()
        elif key == 'escape':
            self.leave_review()
        elif key == 'q':
            if self.args.review is not None:
                self.quit = True        # nothing of ours to lose, no popup
            else:
                self.confirm_quit()
        elif key == 'v' and self.focus == 'activity':
            return False            # the view switch is read-only
        elif key == 'a' or key in {p.key for p in PROCEDURES} or key in ('x', 'm', '+', '=', '-', '_'):
            self.notice = 'Review: arming, procedures and edits are off'
        else:
            return False
        return True

    def review_allows(self, key):
        """Whitelist for keys that reach the page in review: focus, cursor, help and read-only details."""
        return (key in ('\t', 'backtab', 'up', 'down', 'j', 'k', '?', 'o') or key in DIRECTIONS
                or (key == 'enter' and self.focus == 'activity')
                or (key == 'v' and self.focus == 'activity'))

    def open_bag_review(self, bag):
        """Open the selected closed session without replacing live owners or callbacks."""
        if self.review:
            return
        if self.armed or self.manual or self.running or self.test_pulses is not None:
            self.notice = 'Review: disarm and stop motion or test pulses first'
            return
        header = review_metadata(bag.meta_path)
        self.check_bag(bag, metadata=header)
        selected = load_session(bag.path)
        name = header.get('profile')
        if isinstance(name, dict):
            name = name.get('name')
        profile = load_profile(name) if name else self.profile
        selected.host = header.get('host', selected.host)
        selected.title = header.get('title') or bag.id
        try:
            replay = ReviewSource(bag.path, profile, selected)
        except (ImportError, RuntimeError) as error:
            raise ValueError(f'cannot read session: {error}') from error
        self._review_live = {'source': self.source, 'tabs': self.tabs, 'focus': self.focus,
                             'cursors': self.cursors, 'activity': self.activity,
                             'minimap': self.minimap, 'layerbox': self.layerbox}
        self.source, self.review = replay, selected
        self._review_fault = False
        selected.load = replay.seek
        self.tabs, self.focus, self.cursors = TabMemory(), 'activity', dict.fromkeys(CURSORS, 0)
        self.minimap = minimap_panel.MiniMap(self.map_state)
        self.layerbox = LayerBox(None, self.map_state, live=False)
        self.activity = 'log'
        self.last_tick = self.clock()
        self.notice = f'Review {bag.id}; Esc returns to live'

    def leave_review(self):
        """Back to live; a session opened with --review has no live to return to, so it quits."""
        if self.args.review is not None:
            self.quit = True
        else:
            if self._review_live is not None:
                for name, value in self._review_live.items():
                    setattr(self, name, value)
                self._review_live = None
            self.review = None
            self.last_tick = self.clock()
            self.notice = 'Left review'

    # --- panel keys --------------------------------------------------------------

    def on_status(self, key):
        rows = status_rows(self.snapshot(), self.profile.sensors)
        if key in ('left', 'right') and self.agents_row(rows):
            self.toggle_agents()
        elif key in ('left', 'right') and self.row_setting(rows) == 'agent_motion':
            agent_feed.step_setting(self, self.row_setting(rows), key)
        elif key in ('left', 'right') and self.row_setting(rows) == 'recording':
            self.step_recording(key)
        elif key == 'e':
            row = rows[min(self.cursors['status'], len(rows) - 1)] if rows else None
            if row and row['name'] == nav2_edit.NAME and self.nav2_file is not None:
                self.nav2_popup()
        elif key == 'enter':
            if rows:
                row = rows[min(self.cursors['status'], len(rows) - 1)]
                if row.get('source') == 'sensor' and row['name'] == 'odometry':
                    return self.open('odometry')
                if row.get('source') == 'part' and not self.watching:
                    return parts.toggle(self, row['key'])
                if row['name'] == 'Driving':
                    return self.open('authority')
                if row['name'] == 'Hold':
                    return stop_popup.open_stop(self) if row.get('stop') else self.release_hold()
                if row['name'] in ('RTK', 'Source'):
                    return self.open('odometry')
                if row['name'] == 'Config':
                    return self.open('config')
                if row['name'].lower() == 'battery':   # the health advisory is named 'battery'
                    return self.open('power')
                return popups.inspect.open_inspect(self, row)   # the view fits the row (sensor, tf, ...)

    @property
    def facts(self):
        """The health facts by name, among them the perc.<source>.* ones of the perception probe."""
        return self.health.facts.read()[0]

    def power_facts(self):
        """The latest ``power.*`` facts (ros.health feeds them), or None without a voltage source."""
        return getattr(self.ros, 'power_facts', None)

    def row_setting(self, rows):
        row = rows[min(self.cursors['status'], len(rows) - 1)] if rows else None
        return row.get('setting') if row else None

    def agents_row(self, rows):
        return self.row_setting(rows) == 'agents'

    def toggle_agents(self):
        """Flip the Agents setting; a bound arbiter follows (block refuses new agent claims)."""
        self.settings.agents = 'block' if self.settings.agents == 'allow' else 'allow'
        arbiter = getattr(self, 'arbiter', None)
        if arbiter is not None:
            arbiter.allow_agents = self.settings.agents == 'allow'
        self.notice = f'Agents {self.settings.agents}'

    def open(self, action_id):
        """Open the popup registered for ``action_id`` in ``popups.OPENERS``; an unbound id does nothing."""
        opener = popups.OPENERS.get(action_id)
        if opener is None:
            return False
        opener(self)
        return True

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

    def apply_pending(self):
        """A: report unsupported runtime restoration without clearing the durable backup."""
        if self.staging().pending() is None:
            self.notice = 'Nothing pending'
            return
        self.notice = CORRECTION_ACK_REQUIRED

    def hardware_changed(self):
        """I: a typed note, then a confirm, then hardware_epoch + 1 (invalidates every fit); never while moving."""
        if self.moving():
            self.notice = 'Not while moving'
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
            self.notice = f'Hardware changed: epoch {epoch}, calibrations need a refit'
            change = self.history.head('hardware')
            if change:
                self.history.label(change['id'], note=note)

    def characterize(self, direction):
        """Route the temporarily unsupported survey without changing its settings."""
        procedure = next(p for p in PROCEDURES if p.title == 'Characterize')
        self.run_procedure(procedure)

    def on_motion(self, key):
        pass

    def on_conditions(self, key):
        if key == 'enter':
            self.open('conditions')

    def on_procedures(self, key):
        if key == 'enter':
            self.run_procedure(PROCEDURES[self.cursors['procedures']])
            return
        procedure = next((p for p in PROCEDURES if p.key == key), None)
        if procedure is not None:
            self.cursors['procedures'] = PROCEDURES.index(procedure)
            self.run_procedure(procedure)

    def perception_popup(self, view=False):
        """V: the perception view of ``self.facts`` (filled by the perception probe).

        ``v`` opens the camera view (its subscription exists only while it is open); ``view=True``
        opens it at once (ui.md keymap camera). Esc closes the view first, then the popup.
        Closing, resizing and detaching delete the kitty image.
        """
        modal = perception.modal(self.facts)
        state = dict(view=None, note='')
        idle_hint = modal['hint']

        def refresh():  # the probe fills facts while the popup is open; scroll is kept
            modal['rows'] = perception.modal(self.facts)['rows']
            current = state['view']
            if current is None:
                modal['hint'] = idle_hint
                if state['note']:
                    modal['body'] = Text(state['note'], style=theme.MUTED)
                return
            current.resize((min(self.console.size.width - 8, 96), CAMERA_ROWS))
            modal['hint'] = theme.SEPARATOR.join(('Esc close view', camera_view.KEYS_HINT))
            modal['body'] = Group(*current.lines(), Text(current.status(), style=theme.MUTED))

        def close_view():
            if state['view'] is not None:
                state['view'].close()
                state['view'] = None
                modal.pop('body', None)

        def open_view():
            feed = self.camera_feed()
            if feed is None:
                state['note'] = 'camera view needs a ROS node (none in this session)'
                self.notice = state['note']
                return
            state['note'] = ''
            reader = self.keys
            node = getattr(getattr(self, 'ros', None), 'node', None)
            self.camera_boxes = None if node is None else camera_feed.node_observations(  # subscribed while `d` is on
                node, lambda label: camera_view.class_rgb(self.minimap.table, label))
            state['view'] = CameraView(
                feed, (1, CAMERA_ROWS), self.write_terminal,
                kitty=graphics.kitty_enabled(getattr(reader, 'graphics', False),
                                             getattr(reader, 'xtversion', None)),
                save_dir=self.paths.directory / 'snapshots' / 'camera', detections=self.camera_detections,
                observations=self.camera_boxes)

        def handle(key):
            current = state['view']
            if current is not None and current.key(key):  # raw: d and D differ
                return None
            key = fold(key)
            if self.scroll(modal, key, ('k', 'j')):
                return None
            if key == 'v':
                close_view() if current is not None else open_view()
            elif key in ('escape', 'q'):
                if current is not None and key == 'escape':
                    close_view()
                else:
                    return True
        try:
            if view:
                open_view()
                refresh()
            self.popup(modal, handle, refresh)
        finally:
            close_view()

    def camera_feed(self):
        """A CameraFeed on the ROS node, or None without one."""
        node = getattr(self.ros, 'node', None)
        return None if node is None else camera_feed.profile_feed(node, self.profile.sensors)

    def camera_detections(self, size, floor, at=None):
        """Observation boxes in camera pixels with class colours (perception-v2 7.3); floor boxes stay []."""
        boxes = getattr(self, 'camera_boxes', None)
        return [] if floor or boxes is None else boxes.rects(size, at)

    def write_terminal(self, text):
        """Kitty uploads and deletes go straight to the terminal, after Rich's refresh."""
        self.console.file.write(text)
        self.console.file.flush()

    def on_activity(self, key):
        if key == 'v':
            desk = feeds_module.merged(getattr(self, 'feeds', ())).get('agent_desk')
            self.activity, self.cursors['activity'] = agent_log.next_view(self.activity, desk), 0
        elif key == 'enter' and self.activity == 'results':
            self.open_run()
        elif key == 'enter' and self.activity == 'agent':
            agent_log.detail(self, feeds_module.merged(self.feeds).get('agent_desk'), self.cursors['activity'])

    # Reasons offered by the run popup's X; an exclusion always carries one (critique K4).
    EXCLUDE_REASONS = ('operator error', 'wrong conditions', 'sensor fault', 'test run')

    def exclude_result(self, index=None):
        """Ask for a reason and exclude the result at oldest-first ``index`` (default: the cursor's)."""
        total = len(self.snapshot()['results'])
        if not total:
            return
        if index is None:
            index = total - 1 - min(self.cursors['activity'], total - 1)
        try:
            reason = self.choose('Exclude run', self.EXCLUDE_REASONS)
        except Cancelled:
            return
        if reason:
            self.monitor.exclude_result(index, run_popup.exclusion(reason))

    def open_run(self):
        """popup.run for the cursor's result: up/down step, X excludes, Esc closes; Space stops."""
        total = len(self.snapshot()['results'])
        if not total:
            return
        # Pinned by oldest-first index so a result landing mid-popup cannot change the run shown.
        state = dict(index=total - 1 - min(self.cursors['activity'], total - 1))

        def fill():
            result = self.snapshot()['results'][state['index']]
            modal.update(title=activity.result_label(result), rows=run_popup.run_rows(result),
                         hint=theme.SEPARATOR.join((f'{UPDOWN} prev/next', 'X exclude', 'Esc close')))

        def handle(key):
            if key == 'X':  # case matters here: lowercase x writes the report
                self.exclude_result(state['index'])
                self.modal = modal  # the nested popup cleared it
            else:
                key = fold(key)
                if key == 'space':
                    self.stop('stopped by operator')
                elif key in ('up', 'k', 'down', 'j'):
                    last = len(self.snapshot()['results']) - 1
                    state['index'] = min(max(state['index'] + (1 if key in ('up', 'k') else -1),
                                             0), last)
                    self.cursors['activity'] = last - state['index']
                elif key in ('escape', 'q'):
                    return True
            fill()

        modal = dict(kind='message', width=76)
        fill()
        self.popup(modal, handle)

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

    # --- popups (see ui/popups) ------------------------------------------------------

    def popup(self, modal, handle, refresh=None):
        return popups.popup(self, modal, handle, refresh)

    scroll = staticmethod(popups.scroll)

    def show(self, title, message=None, rows=None, width=60):
        return popups.show(self, title, message, rows, width)

    def live_nav2(self):
        """Return the live Nav2 applier, built when first needed (it subscribes to ROS state)."""
        if self._nav2_live is None and self.nav2_live_factory is not None:
            self._nav2_live = self.nav2_live_factory()
        return self._nav2_live

    def apply_nav2_live(self, requested):
        if self.watching or self.review:
            return {'refused': 'this instance only watches; the session owner applies'}
        return self.live_nav2().applier(requested)

    def nav2_popup(self):
        """Preview the Nav2 file against the running nodes; y applies it live, Esc leaves it."""
        source = self.nav2_file
        previewed = source.current  # apply exactly what is shown, even if the file changes meanwhile
        live = None if self.watching or self.review else self.live_nav2()
        if live is None:
            rows = [(key, f'{old if old is not None else theme.DASH} {theme.ARROW} '
                          f'{new if new is not None else theme.DASH}')
                    for key, old, new in source.changes()]
            refused = False
        else:
            try:
                found = live.preview(source.requested(previewed))
            except Exception as error:  # the page must survive a ROS failure
                self.notice = f'Nav2 preview failed: {error}'
                return
            rows = [(row['key'], f'{theme.DASH if row["old"] is None else row["old"]} '
                                 f'{theme.ARROW} {row["new"]}'
                                 + (' (derived)' if row['derived'] else '')) for row in found.rows]
            rows += [('refused', reason) for reason in found.refusals]
            refused = bool(found.refusals)
            if not rows:
                rows = [('', 'The running nodes already hold these values')]
        rows += [(key, f'removed from file {theme.ARROW} restart to reset') for key in source.removed()]
        rows = rows or [('', 'No changes pending')]
        label = 'Apply live' if live else 'Apply'
        options = ['Close'] if refused else [label, 'Close']
        try:   # Close is focused: Enter never applies, y does
            answer = self.choose('Nav2 parameters', options, {'y': 0} if len(options) > 1 else None,
                                 rows=rows, default=len(options) - 1, width=76,
                                 risky={0} if len(options) > 1 else set())
        except Cancelled:
            return
        if answer == label:
            self.notice = source.apply(previewed)

    def prompt(self, message, title='Ready'):
        # Owner decision 2026-10-26: a --sim-arm run whose checks passed (the desk sets agent_ready for it,
        # and only while it holds the arm) has its Ready prompts confirmed by the agent, journaled as such.
        who = getattr(self, 'agent_ready', None)
        if who and title == 'Ready' and self.armed:
            self.ros.emit('ready', by='agent', who=who, sim_arm=True, message=message)
            self.notice = f'Ready confirmed by agent {who} (sim auto-arm)'
            return ''
        return popups.prompt(self, message, title)

    def choose(self, title, options, shortcuts=None, message=None, danger=False, rows=None,
               default=0, width=64, risky=None, quiet=False):
        return popups.choose(self, title, options, shortcuts, message, danger, rows, default, width, risky, quiet)

    def ask_text(self, title, message, text='', check=None):
        return popups.ask_text(self, title, message, text, check)

    def type_word(self, title, message, word, rows=None, width=64, quiet=False):
        return popups.type_word(self, title, message, word, rows, width, quiet)

    # --- arming --------------------------------------------------------------------

    def arm_rows(self):
        """What arming means right now: outputs, driver, config and every interlock."""
        remote = getattr(self.arming, 'remote', None)
        outputs = {'odroid': 'ODROID PWM pins (real thrusters)' + (f' on {remote.describe()}' if remote else ''),
                   'topic': f'simulator over ROS ({self.args.hardware_config["pwm_topic"]})',
                   'none': 'commands straight to the plant'}[self.profile.outputs]
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
        """The typed word that starts the real driver, for arming and the pin readback alike."""
        return self.type_word(title, 'Propellers submerged, area clear, physical power '
                              'cutoff within reach.', 'arm', rows=self.arm_rows(), width=72, quiet=True)

    def arm_panel(self):
        """One popup for both directions; Space still stops without it."""
        if self.review:
            self.notice = 'Review: arming is off'
            return
        if not self.source.live:
            self.notice = 'Arming is disabled in review'
            return
        if self.watching:
            self.notice = f'Watching pid {self.watching}: no arming'
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
                self.notice = f'Arming refused: {refusal}'
                return
            if self.profile.outputs == 'odroid':
                ok = self.confirm_drive(title)
            else:
                ok = self.choose(title, ('Cancel', 'Arm'), {'n': 0, 'y': 1},
                                 rows=self.arm_rows(), width=72, quiet=True) == 'Arm'
        except Cancelled:
            ok = False
        if not ok:
            self.notice = 'Arming cancelled'
            return
        refusal = self.watchdog_refusal()      # one may have gone while the operator typed
        if refusal:
            self.notice = f'Arming refused: {refusal}'
            return
        # Another /cmd_vel source (Nav2) doesn't stop arming: Nav2 procedures need it, and
        # suite-commanded motion refuses to run beside it on every tick (motion_blockers).
        arbiter = getattr(self, 'arbiter', None) if muxed(self.profile) else None
        since = arbiter.clock() if arbiter is not None else None
        self.wait_for_driver()
        if self.plant is not None:
            self.plant.engage()
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware))
        self.notice = 'Armed at neutral'
        if arbiter is not None:
            self.settle_mux(arbiter, since)

    def agent_arm(self, who, checks):
        """Sim auto-arm for an agent run: the arm panel's path, its popup replaced by the desk's sim checks.

        Only the desk calls it, after every harness/sim_arm.py check passed. The panel's refusals and
        driver start are the same; ``armed`` is journaled with ``by: agent``, who and the checks passed.
        A hold the mux restores is never released: arming is undone and the hold named. Returns ''
        when armed with the mux on auto, else why not (and disarmed).
        """
        if self.review or not self.source.live or self.watching or self.arming is None:
            return 'this session cannot arm'
        refusal = self.thruster_refusal() or self.pin_refusal() or self.watchdog_refusal()
        if refusal:
            return refusal
        arbiter = getattr(self, 'arbiter', None) if muxed(self.profile) else None
        if arbiter is None:
            return 'no command mux to arm through'
        since = arbiter.clock()
        try:
            self.wait_for_driver()
        except (Cancelled, RuntimeError) as error:
            return str(error) or 'arming cancelled'
        if self.plant is not None:
            self.plant.engage()
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware), by='agent', who=who, sim_arm=list(checks))
        self.notice = f'Armed for agent {who} (sim auto-arm)'
        why = ''
        if not self.await_mux(arbiter, since, lambda mux: True):
            why = 'the mux did not report after arming'
        elif self.override_cause():
            with arbiter.lock:
                mux = arbiter.mux or {}
            cause = mux.get('cause') or ((mux.get('holds') or [{}])[0] or {}).get('cause') or self.override_cause()
            why = f'the mux holds ({cause}); only the operator releases it (U)'
        if why:
            self.disarm(f'sim auto-arm undone: {why}', by='agent')
        return why

    def settle_mux(self, arbiter, since):
        """After arming, wait for the mux's first state; a hold it restored is offered for release.

        The mux starts with the holds it saved (a Space before the disarm), so whatever drives
        next would meet a stale or held mux. Only the human releases, through U's own confirm.
        """
        if not self.await_mux(arbiter, since, lambda mux: True):
            self.notice = 'Armed; the mux has not reported yet'
        elif self.override_cause() == 'hold':
            since = arbiter.clock()
            if self.release_hold():
                self.await_mux(arbiter, since, lambda mux: mux.get('input') != 'hold')

    def await_mux(self, arbiter, since, done, timeout=MUX_SETTLE_S):
        """Repaint until the mux publishes a state after ``since`` that ``done(mux)`` accepts.

        False on timeout or Esc; Space stops the boat as everywhere. Each pass waits 0.1 s."""
        for _ in range(round(timeout / 0.1)):
            with arbiter.lock:
                mux, at = arbiter.mux, arbiter.mux_at
            if mux is not None and at is not None and at > since and done(mux):
                return True
            self.refresh()
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                self.arming.sleep(0.1)
            if key in ('space', 'press:space'):
                self.operator_stop()
                return False
            if key in ('escape', 'press:escape'):
                return False
        return False

    def arm(self):
        self.arm_panel()

    def wait_for_driver(self):
        """Start the driver and show it starting; Space or Esc cancels.

        The view keeps redrawing while the driver comes up, so arming reads as
        loading, never as a frozen screen.
        """
        self.ever_armed = True                   # a hold saved after this is this session's (sim stale holds)
        if getattr(self.arming, 'remote', None) is None:
            self.arming.begin()
        else:                                    # the ssh read ran off the key loop; None refuses
            self.arming.begin(self.read_boat_config())
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

    def read_boat_config(self):
        """A remote arm's ssh read of the boat's hardware.yaml, off the key loop (up to ~16 s on a bad
        link): the view redraws, Space stops, E revokes and Esc cancels meanwhile. None without a remote
        boat; a failed read leaves None and ``begin`` refuses with the reason."""
        remote = getattr(self.arming, 'remote', None)
        if remote is None:
            return None
        box = {}
        thread = threading.Thread(target=lambda: box.update(config=remote.fetch_config()),
                                  name='boat-config', daemon=True)
        thread.start()
        started = self.clock()
        while thread.is_alive():
            self.notice = (f'Arming: reading the boat config on {remote.host.name} '
                           f'{self.clock() - started:.0f} s{theme.SEPARATOR}Esc cancels')
            self.refresh()
            key = self.keys.read(0.1) if self.keys is not None else None
            if key is None and self.keys is None:
                thread.join(0.1)
            if key in ('space', 'press:space'):
                self.operator_stop()
                raise Cancelled('arming cancelled')
            if key in ('escape', 'press:escape'):
                raise Cancelled('arming cancelled')
            if key == 'E':
                self.revoke_agent()
        return box.get('config')

    def disarm(self, reason='disarmed', by='operator'):
        self.clear_pulses()
        if self.plant is not None:
            self.plant.neutral()
            self.plant.release()
        self.command = (0.0, 0.0, 0.0)
        if self.armed:
            self.arming.disarm()
            self.watcher.watches['hardware'].warning = None
            self.ros.emit('disarmed', reason=reason, by=by)
        self.notice = f'{reason}; outputs neutral'

    def operator_stop(self, agent=None):
        """Space requests a sticky manual hold, then always neutralizes and disarms. ``agent`` is the id of an
        agent whose ``stop`` this is (None: the operator's Space); ``stopped_by`` keeps it for the desk's hints."""
        self.command = (0.0, 0.0, 0.0)
        self.stopped_by = f'agent {agent}' if agent else 'operator'
        try:
            if self.plant is not None:
                self.plant.hold('keys')
        finally:
            self.stop(f'stopped by {self.stopped_by}')

    def stop(self, reason, by='operator'):
        """Neutral and disarmed, whatever happened; ``by`` is operator, fault or error.

        A watcher publishes neutral too: Space always works.
        """
        if getattr(self, '_runner_paused', False):
            self._runner_cancelled = True
        try:
            self.disarm(reason, by)
        finally:
            self.monitor.event('STOP: ' + reason)

    # --- procedures ---------------------------------------------------------------

    def availability(self, procedure):
        if self.watching:
            return f'watching pid {self.watching}: no commands'
        if procedure.key == 'f':
            return CORRECTION_ACK_REQUIRED
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
        if self.review:
            self.notice = f'{procedure.title}: procedures are off in review'
            return
        if not self.source.live:
            self.notice = 'Procedures are disabled in review'
            return
        if procedure.key == 'f':
            self.notice = f'{procedure.title}: {CORRECTION_ACK_REQUIRED}'
            return
        reason = self.availability(procedure)
        if reason and 'armed' in procedure.needs and not self.armed:
            self.arm_panel()  # offer to arm, then carry on with the procedure
            reason = self.availability(procedure)
        if reason:
            self.notice = f'{procedure.title}: {reason}'
            return
        self.clear_pulses()
        desk = getattr(self, 'desk', None)
        if desk is not None and procedure.key == 'm':
            desk.take_over()             # manual cancels an agent goal at once, as Space and E do
        # Only jump to Map once it has a body; otherwise the operator lands on a placeholder.
        if (self.settings.auto_tab and 'Map' in layout.BODIES
                and getattr(procedure, 'group', '') in ('Navigate', 'Mission')):
            self.show_tab('Map')
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
                       status=runs.ABORTED if aborted is not None else None,
                       provenance=session_module.provenance(
                           self.facts, self.history, getattr(self.args, 'profile', None)))

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
                session_dir=self.paths.directory, start=self.processes.start,
                running=self.processes.running, stop=self.processes.stop,
                returncode=self.processes.returncode,
                nav2_params=nav2_params(), nav2_overlay=self.paths.nav2,
                nav2_user=nav2_limits.path(),
                hardware_config=self.args.hardware_config, wall_ns=self.wall_ns,
                user_config=self.args.config,
                history=self.history, warn=self.warn_history,
                staging=self.staging(), fit_context=self.fit_context, ask_text=self.ask_text,
                driver_status=lambda: getattr(self.ros, 'driver_status', None),
                disarm=self.disarm, pulses=self.set_pulses, save_hardware=self.save_hardware,
                mux_input=self.mux_input,
                magcal=lambda: magcal_popup.ask_heading(self), type_word=self.type_word,
                task=getattr(self, 'agent_task', None),   # mission: the task an agent's run named
                reset_to_start=self.reset_to_start,   # mission: sim-only, gated by mission.sim_stack
                manual=dict(plant=self.plant, blockers=self.manual_blockers,
                            open_events=self.open_events, show=self.manual_show, emit=self.emit,
                            speed=lambda: self.manual_speed, clock=self.monitor.clock)))

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

    def motion(self, name, **details):
        """A Runner that commands the plant, inside the live page."""
        return self.session(name, self.motion_blockers, details)

    def supervise(self, name, **details):
        """A Runner for watching Nav2 drive; the suite does not command meanwhile."""
        return self.session(name, self.supervision_blockers, details)

    def observe(self, name, **details):
        """A passive measurement window; no arming or readiness needed."""
        return self.session(name, lambda: [], details)

    def resume_gate(self):
        """Fresh odometry, independent of the neutral-water estimator's selected source."""
        if self.armed and muxed(self.profile) and getattr(self, 'arbiter', None) is None:
            return ['authority is unknown']
        row = self.monitor.snapshot()['sensors'].get('odometry', {})
        age = row.get('age', float('inf'))
        if (row.get('status') != 'LIVE' or row.get('issue')
                or not 0 <= age <= STILL_FRESH_S):
            return ['pose is missing, stale or invalid']
        stamp_age = row.get('stamp_age')
        if stamp_age is not None and not -.25 <= stamp_age <= STILL_FRESH_S:
            return ['pose timestamp is invalid']
        values = row.get('values', {})
        return stillness(values.get('vx'), values.get('vy'), values.get('wz'))

    def poll_pause(self):
        """Refresh read-only sources and views; never replay pulses or watch/apply config."""
        self.source.tick(self.clock())
        arbiter = getattr(self, 'arbiter', None)
        if arbiter is not None:
            arbiter.tick()
        snapshot = self.snapshot()
        for feed in self.feeds:
            feed.tick(self, snapshot, self.clock())

    def runner_key(self, key, keymap):
        action = keymap.resolve(['runner'], key)
        if action is None and keymap.resolve(['runner'], fold(key)) == 'abort':
            action = 'abort'  # Caps Lock must not defeat abort; E/U remain case-sensitive.
        if action == 'abort':
            return 'abort'
        if action is None and keymap.resolve(['tab.drive'], key) == 'manual':
            # A manual takeover: the run (an agent's too) ends at neutral; m again drives.
            desk = getattr(self, 'desk', None)
            if desk is not None:
                desk.cancel('overridden: keyboard')
            return 'abort'
        if action in ('mark', 'revoke_agent', 'release_hold'):
            {'mark': self.mark, 'revoke_agent': self.revoke_agent,
             'release_hold': self.release_hold}[action]()
            if getattr(self, '_runner_cancelled', False):
                return 'abort'

    @contextmanager
    def session(self, name, blockers, details):
        def show(command, remaining, label):
            self._runner_paused = False
            self.command = command
            self.monitor.command(command)
            self.notice = f'{label or name}{theme.SEPARATOR}{remaining:.1f}s{theme.SEPARATOR}Space aborts'
            if self.test_pulses is not None and self.publish_pulses is not None:
                self.publish_pulses(self.test_pulses)
            self.refresh(force=False)

        self._runner_cancelled = False
        keymap, _ = keymap_module.load()

        def paused_tick(cause, elapsed):
            self._runner_paused = True
            if self._runner_cancelled:
                raise Aborted('motion cancelled by operator')
            self.poll_pause()
            self.command = (0.0, 0.0, 0.0)
            self.monitor.command(self.command)
            self.notice = (f'PAUSED{theme.SEPARATOR}{cause}{theme.SEPARATOR}{elapsed:.1f}s'
                           f'{theme.SEPARATOR}U release · E revoke · Space aborts')
            self.refresh(force=False)

        def require_runtime_c():
            if self.staging().pending() is not None:
                raise Cancelled(CORRECTION_ACK_REQUIRED)

        def on_pause():
            require_runtime_c()

        def on_resume():
            require_runtime_c()

        runner = Runner(self.plant, lambda: blockers() + agent_feed.blockers(self), self.keys, show=show,
                        emit=self.emit, clock=self.monitor.clock,
                        override=self.override_cause, gate=self.resume_gate,
                        paused_tick=paused_tick, key_action=lambda key: self.runner_key(key, keymap),
                        on_pause=on_pause, on_resume=on_resume)
        try:
            with runner.session(name, **details):
                yield runner
        finally:
            self._runner_paused = False
            self.command = (0.0, 0.0, 0.0)

    def override_cause(self):
        """Why the command mux is not under automation (derive's rules in harness/authority.py).

        '' when no arbiter is bound or the profile has no mux; when disarmed
        (the mux only runs as an arming child) a missing or stale state is '' too.
        """
        arbiter = getattr(self, 'arbiter', None)
        if arbiter is None or not muxed(self.profile):
            return ''
        with arbiter.lock:
            mux, at = arbiter.mux, arbiter.mux_at
        age = float('inf') if at is None else arbiter.clock() - at
        if not self.armed and (mux is None or age > STALE_MUX_S):
            return ''
        return override_cause(mux, age)

    def reset_to_start(self, timeout):
        """crane_ml's /crane/reset_to_start; mission.reset_to_start calls it on a sim stack only."""
        if getattr(self, '_start_reset', None) is None:
            from mhseals_testing.procedures.mission import START_RESET
            from mhseals_testing.ros.sim_reset import StartReset
            self._start_reset = StartReset(self.ros, yaml.safe_load(START_RESET.read_text())['service'])
        return self._start_reset(timeout)

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
                        help=f'plant profile name ({", ".join(available_profiles())}) or YAML path; '
                             'boat_trial profiles explains each path; ? shows the active profile')
    parser.add_argument('--fcu-url', default='',
                        help='explicit MAVROS FCU URL override (default: inherit endpoint YAML)')
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                        help='override a setting for this run (see harness/settings.py), e.g. '
                             'amplitude=0.4, command_mode=velocity, bench_override=on')
    parser.add_argument('--review', nargs='?', const='', default=None, metavar='SESSION',
                        help='play back a past session (path or name under the data root; '
                             'default the newest) with no ROS node and no outputs')
    parser.add_argument('--output', type=Path, default=None,
                        help='session directory (default a new sessions/<time>_<host>_<profile> under the data root)')
    return parser


def parse_args(argv=None, *, defer_hardware=False):
    parser = build_parser()
    try:
        if defer_hardware:
            parser.add_argument('--config', help='hardware YAML (or MHSEALS_HARDWARE_CONFIG)')
            parser.add_argument('--save-config', action='store_true')
            args = parser.parse_args(argv)
            args.hardware_config = None
        else:
            args = configure_args(parser, argv)  # explicit unmanaged/parser callers
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
    if profile.components and args.review is None:
        try:            # where each part runs: refused here, before a session directory exists
            profile.placed
        except DeviceError as error:
            raise SystemExit(f'boat_test: {error}') from None
    return args, profile


def stack_overlay(profile, args):
    """Launch arguments the command line adds to the profile's components."""
    if not profile.uses_fcu or not args.fcu_url:
        return {}
    from mhseals_testing.harness.components import ENDPOINT_LAUNCHES
    placed = profile.placed[1]
    return {key: {'fcu_url': args.fcu_url}
            for key, spec in (profile.components or {}).items()
            if placed[spec.get('part', key)].spec.get('launch') in ENDPOINT_LAUNCHES}


def terminal_gone(error):
    """True for the key reader's EOF or EIO: the terminal is gone, not a stopping error."""
    return isinstance(error, EOFError) or error.errno == errno.EIO


def hang_up(*_):
    """SIGHUP and SIGTERM stop like Ctrl+C: neutral, disarm, flush, then exit."""
    raise KeyboardInterrupt


def detach(*_):
    """SIGUSR1: the next instance is taking over."""
    raise Detach


def nav2_recorder(app):
    """Return the history writer for a live apply: the content Nav2File handed to it.

    Not the file as it is now, so an edit saved during the apply is not recorded as applied.
    """
    def record():
        applying = getattr(app().nav2_file, 'applying', None)
        if applying:
            adapters.record_value(app().history, 'nav2', 'tui:nav2-live', *applying,
                                  app().warn_history, applied='live-session')
    return record


def build_nav2_live(ros, registry, profile, app):
    """Build the live Nav2 applier for this session owner (``app``: a callable, built later)."""
    from mhseals_testing.harness.nav2_live import Nav2Live
    from mhseals_testing.ros.nav2_live import RosPorts
    ports = RosPorts(ros.node, registry, lambda phase, **fields: ros.emit(phase, **fields),
                     lambda: bool(app().armed) and muxed(profile))
    return Nav2Live(ports, me=os.getpid(), record=nav2_recorder(app))


def review_main(args, profile):
    """--review: the dashboard over a past session, with no ROS, bag recorder or outputs."""
    try:
        found = find_session(args.review)
        review = load_session(found)
    except ValueError as error:
        raise SystemExit(f'boat_test --review: {error}')
    console = Console()
    offline = Offline()
    app = App(args, profile, BoatMonitor(args.stale_after), offline, offline,
              SessionPaths.from_output(found), console, review=review)
    app.run()


def main(argv=None):
    """boat_test: a device registry the harness refuses ends with its message, not a traceback."""
    try:
        return session_main(argv)
    except DeviceError as error:
        raise SystemExit(f'boat_test: {error}') from None


def session_main(argv=None):
    args, profile = parse_args(argv, defer_hardware=True)
    if args.review is not None:
        args.hardware_config = load_config(args.config)
        return review_main(args, profile)
    # ROS imports stay here so the App (and its tests) never need rclpy.
    import rclpy

    from mhseals_testing.harness.simsession import start_session
    from mhseals_testing.ros import arm_link
    from mhseals_testing.ros.arming import Arming
    from mhseals_testing.ros.health import subscribe_fcu
    from mhseals_testing.ros.perception import subscribe_detections
    from mhseals_testing.ros.preflight import subscribe_tui as subscribe_preflight
    from mhseals_testing.ros.mission import TaskWatcher
    from mhseals_testing.ros.nav2 import Nav2
    from mhseals_testing.ros.node import RosIO, node_name
    from mhseals_testing.ros.plant import Plant
    from mhseals_testing.harness.processes import ProcessManager

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
    if choice.session and choice.mode != 'own':
        paths = SessionPaths.from_output(choice.session)
    else:
        paths = SessionPaths.from_output(session_module.create(
            profile.name, directory=args.output,
            clock={'domain': 'sim' if profile.use_sim_time else 'wall', 'time_scale': 1.0},
            git=session_module.running_sources(),
            kind='sim' if profile.use_sim_time else 'boat').directory)
    overlay = stack_overlay(profile, args)
    if args.save_config:
        if choice.mode != 'own':
            raise ValueError('--save-config requires a fresh owner session')
        save_config(load_config(args.config), args.config)
    args.hardware_config = load_config(args.config)
    # rclpy's own SIGINT handler would shut ROS down before the graceful close could
    # send neutral and the final events; without it Ctrl+C is a KeyboardInterrupt.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for hangup in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(hangup, hang_up)
    signal.signal(signal.SIGUSR1, detach)
    gyros = [sensor.name for sensor in profile.sensors if sensor.kind == 'imu']
    monitor = BoatMonitor(args.stale_after, gyro_sensor=gyros[0] if gyros else None,
                          history_sensors=tuple(
                              sensor.name for sensor in profile.sensors
                              if sensor.kind == 'odometry' or sensor.name == (gyros or [None])[0]),
                          deadlines={sensor.name: sensor.deadline for sensor in profile.sensors
                                     if sensor.deadline is not None},
                          future=profile.stamp_future)
    facts = Facts()
    ros = RosIO(profile, monitor, args.stale_after, paths.journal, facts)
    # The keepers follow the registry owner, so a handover keeps the children for the adopter.
    stack_items = components(profile, overlay)
    processes = ProcessManager(paths.logs, registry=None if watching else registry)
    plant = Plant(ros, profile)
    # Parts on other computers run as units there; a watcher reads them but keeps none alive.
    # Every computer reports its own parts (its process table), so DDS discovery never sets a state.
    status = issue_settings()
    units = Units(stack_items, by=dict(host=profile.placed[0].name if stack_items else '',
                                       pid=os.getpid(), session=str(paths.directory)),
                  renew=not watching, period=float(status.get('period', 5.0))).begin()
    stack = Stack(stack_items, processes, time.monotonic, units, scan=unit_probe)
    ros.ghosts = stack.ghosts              # node.unique ignores the lingering names of a child we stopped
    # Off the thrusters host, arming targets it over DDS + ssh (ros/arm_link.py); a watcher never arms.
    driver_host = None if watching else arm_link.target(profile)
    remote_driver = None if driver_host is None else arm_link.RemoteDriver(
        driver_host[0], arm_link.RosLink(ros.node), driver_host[1],
        by=dict(host=profile.placed[0].name, pid=os.getpid(), session=str(paths.directory)))
    if remote_driver is not None:   # one hardware.yaml: the boat's, read now and again at each arm
        args.hardware_config = remote_driver.fetch_config() or args.hardware_config   # shown only if unread

        def adopt(config):
            args.hardware_config = config
        remote_driver.adopt = adopt
    arming = Arming(profile, ros, processes, args.hardware_config, paths.hardware, remote=remote_driver)
    app = authority = desk = None
    try:
        for name, child in choice.children.items():
            processes.adopt(name, child)
        if choice.mode == 'own':               # a stale owner's leftovers: stop, start fresh
            processes.stop_all()
        subscribe_detections(ros, facts, profile)
        if profile.uses_fcu:
            subscribe_fcu(ros, monitor, facts, profile.battery)
        subscribe_preflight(ros, facts, profile, lambda: args.hardware_config, registry)
        if watching:   # known to the owner, so it is not flagged as a foreign instance
            registry.add_guest(os.getpid(), me['node'], 'watch')
        else:
            registry.update(lambda state: state.update(session=str(paths.directory)))
            start_session(profile, paths.session, paths.truth)
            # The profile's components marked start come up with the session; whatever of
            # them already runs is attached to (Stack.autostart). Off the simulator nothing
            # that moves the boat auto-starts.
            if stack.start_order():
                time.sleep(2.0)  # let discovery find an already-running stack
                sim = profile.use_sim_time is True and profile.outputs == 'topic'
                for line in stack.autostart(ros.nodes(), ros.publishers(profile.odometry), sim):
                    console.print(Text(line, style=theme.WARN))
                    ros.emit('autostart', detail=line)
            registry.set_children(processes.records())
        app = App(args, profile, monitor, ros, processes, paths, console, arming, plant,
                  nav2=lambda: Nav2(ros, profile.map_frame or profile.odom_frame,
                                    profile.base_frame),
                  task_watcher=lambda: TaskWatcher(ros), pulses=plant.test_pulses,
                  registry=registry, watching=watching, facts=facts,
                  nav2_file=Nav2File(nav2_limits.path()), network=NetworkRow(),
                  nav2_live=lambda: build_nav2_live(ros, registry, profile, lambda: app), stack=stack,
                  issues=IssuesFeed(status.get('grace_s', 15.0), status.get('discovery_grace_s', 20.0)))
        if not watching:
            ros.emit('takeover' if choice.mode == 'adopt' else 'session_start',
                     profile=profile.name, bag=str(paths.bag))
            app.observe_config()
            if processes.running('bag'):               # adopted: the recorder keeps going
                app.recorder.mode = 'lean'
            else:
                app.set_recording('lean')              # lean is the default at session start
            registry.set_children(processes.records())
        authority = None if watching else authority_feed.start(
            app, ros, subscribe=authority_feed.subscribe_cmd_vel, registry=registry)
        desk = None if watching else agent_feed.start(app, ros, registry=registry)
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
            if desk is not None:        # an act never outlives its TUI, nor passes to a successor
                desk.close()
            if authority is not None:   # claims end with the session unless a successor adopts them
                authority.arbiter.close(keep_claims=app.detaching)
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
        table = theme.grid({'header': '', 'style': f'bold {theme.ACCENT}'},
                                     {'header': '', 'ratio': 1}, padding=(0, 2))
        table.show_header = False
        for label, value in app.summary():
            table.add_row(label, value)
        console.print(theme.block(table, 'Session closed', expand=False))


if __name__ == '__main__':
    main()
