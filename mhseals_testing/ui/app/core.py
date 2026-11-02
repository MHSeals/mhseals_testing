"""App: the page's state and the coordinator of its parts. Behaviour lives in the mixins
(loop, view, actions/), each one area; App holds the state they share."""

from contextlib import contextmanager
import errno
import os
from pathlib import Path
import socket
import subprocess
import textwrap
import threading
import time
import traceback

import yaml
from rich.console import Console, Group
from rich.live import Live
from rich.segment import Segments
from rich.text import Text

from mhseals_hardware.configuration import config_path
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.menu import Cancelled
from mhseals_hardware.thruster_mixer import IDENTITY, MAX_PWM, MIN_PWM, POSITIONS, validate_mixer
from mhseals_testing import nav2_limits
from mhseals_testing.harness import host, operator_state, runs, session as session_module
from mhseals_testing.harness.bags import Recorder
from mhseals_testing.harness.apply import LiveStaging
from mhseals_testing.analysis.readiness import planned_directions
from mhseals_testing.harness.conditions import Conditions, STILL_FRESH_S, stillness
from mhseals_testing.runner import Aborted, Runner, STALE_MUX_S, fold, override_cause
from mhseals_testing.harness.recwatch import RecWatch
from mhseals_testing.harness.watch import Watched, Watcher
from mhseals_testing.health.advisories import Health
from mhseals_testing.health.journal import Journal
from mhseals_testing.history import History, backends
from mhseals_testing.procedures import PROCEDURES, unavailable
from mhseals_testing.devices import DeviceError
from mhseals_testing.readiness import arm_nodes, arm_refusal, liveness_refusal, motion_blockers
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ros import camera_feed
from mhseals_testing.ui import (
    agent_acts, authority_chip, camera_view, feeds as feeds_module, graphics, keymap as keymap_module, layout,
    palette, parts, perception, pixels, popups, progress, theme, widgets)
from mhseals_testing.ui.camera_view import CameraView
from mhseals_testing.ui.panels import PANELS, agent_log, fusion as fusion_panel
from mhseals_testing.ui.feeds import agent as agent_feed, health as health_feed
from mhseals_testing.ui.feeds.conditions import ConditionsFeed
from mhseals_testing.ui.feeds.issues import IssuesFeed
from mhseals_testing.ui.feeds.rtk import RtkFeed
from mhseals_testing.ui.feeds.toasts import ToastFeed
from mhseals_testing.ui.feeds.tracking import TrackingFeed
from mhseals_testing.ui.toasts import Toasts
from mhseals_testing.ui.layout import MINIMAP, ROW, TabMemory, body_rows, motion_shown
from mhseals_testing.ui.minimap import panel as minimap_panel
from mhseals_testing.ui.minimap.layerbox import LayerBox
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.panels.checklist import Checklist
from mhseals_testing.ui.popups import magcal as magcal_popup, pins
from mhseals_testing.ui.source import LiveSource

from mhseals_testing.ui.app.help import (
    CORRECTION_HINTS, FUSION_HINTS, GLOBAL_HINTS, HELP, MANUAL_HINTS, MICRO, PANEL_HINTS, REVIEW_HINTS, UPDOWN,
    help_rows, help_width, map_key_maps)
from mhseals_testing.ui.app.shared import CURSORS, DIRECTIONS, Detach, PULSE_STEP
from mhseals_testing.ui.app.actions.quit import Quit
from mhseals_testing.ui.app.actions.operator import Operator
from mhseals_testing.ui.app.actions.bags import Bags
from mhseals_testing.ui.app.actions.settings import Settings
from mhseals_testing.ui.app.actions.holds import Holds


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


MUX_SETTLE_S = 5.0       # s after arming for the mux's first state (it starts last, then 1 Hz)


FOREIGN_GRACE = 30.0     # s: a dead DDS participant lingers until its lease expires; later it counts as live


FRAME = 1 / 30           # s; continuous redraws (motion, manual) stop at 30 per second
IDLE_S = 1.0             # s; a still screen redraws only on a change, and this often for its ages and clocks
MANUAL_DESK_S = 0.2      # s; the agent desk's tick from inside manual control
CAMERA_ROWS = 12         # cell rows of the perception popup's camera view


NEUTRAL = 1500


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


def run_end(error):
    """How a procedure run that raised ``error`` ended, for its run record: ``outcome``
    (aborted, cancelled or error) and ``cause`` (the abort's cause, or what failed). A
    KeyboardInterrupt (Ctrl-C) aborts the run with cause ``interrupted``."""
    if isinstance(error, KeyboardInterrupt):
        return dict(outcome='aborted', cause='interrupted', by='operator')
    if isinstance(error, Aborted):
        return dict(outcome='aborted', cause=error.cause, by=error.by)
    if isinstance(error, Cancelled):
        return dict(outcome='cancelled', cause=str(error) or 'cancelled')
    return dict(outcome='error', cause=f'{type(error).__name__}: {error}')


def guard_end(cause, stopped_by):
    """``(reason, ended)`` for a run a watchdog ended (``cause``: the fence's ``geofence``), whoever stopped it."""
    return f'{cause}: the {cause} watchdog stopped the run', dict(outcome='aborted', cause=cause, by='guard',
                                                                   stopped_by=stopped_by)


ARM_WRAP = 52  # arm-panel value column: the 72-wide popup less its label column


def nav2_params():
    """mhseals_nav's checked-in Nav2 parameters, or None outside a ROS install."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_nav')) / 'config' / 'nav2_params.yaml'
    except (ImportError, KeyError):
        return None
    return path if path.exists() else None


def terminal_gone(error):
    """True for the key reader's EOF or EIO: the terminal is gone, not a stopping error."""
    return isinstance(error, EOFError) or error.errno == errno.EIO


class App(Holds, Settings, Bags, Operator, Quit):
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
        self.handed_by = self.restart_as = None   # the takeover's taker; 'take' or 'view' to restart as one
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
        self.fusion_keys = keymap_module.key_map(keymap_module.load()[0].doc.get('tab.fusion'))
        self.checklist = Checklist.from_env()   # boat_tui preview's sanity checklist, else None
        self.fusion_image = None   # the Fusion camera image: None until toggled (on locally, off over a link budget)
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
        self.last_key = None     # still_key() of the last frame (render only on change, D-20261101-26)
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

    armed_by = None   # who armed, for the ARMED chip: None is the operator, else {'who', 'sim'} (an agent)

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
            state = self.map_state()
            if state is not None and getattr(state, 'fence_status', None) is not None:
                snapshot['fence'] = self.fence_status(state)    # the Course area and On exit rows
            if not self.watching:
                snapshot['rec_mode'] = self.recorder.mode   # the Recording row (owner only)
        if self.live is not None:   # the Graphics row: this terminal's setting and drawing tier
            snapshot['graphics'] = (theme.PALETTE.graphics, pixels.TIER)
        counts = health_feed.summary(self.engine)
        if counts:
            snapshot['health'] = counts
        if self.rtk.current is not None:
            snapshot['rtk_view'] = self.rtk.current
        return snapshot

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
        """The one LiveStaging of this TUI: C on the running driver over its parameters, read back."""
        if getattr(self, '_staging', None) is None:
            def params():
                node = getattr(self.ros, 'node', None)
                if node is None:
                    return None
                from mhseals_testing.ros.params import Params
                return Params(self.ros)
            self._staging = LiveStaging(self.args.config, params=params)
        return self._staging

    def offer_restore(self):
        """At start, disarmed: an interrupted run's backed-up C goes back to the YAML the driver starts from."""
        if not self.armed and self.staging().recover():
            self.notice = 'An interrupted characterization was undone: the deployed C is back'

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
        here = getattr(getattr(self, 'stack', None), 'here', None)
        if here is not None:
            here(fresh=True)                # arming decides on a current process view
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

    def motion_blockers(self):
        """When the suite commands, nothing else may publish the command topic."""
        return self.supervision_blockers() + (self.plant.conflicts() if self.armed else [])

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
        if not self.watching:
            self.fence_alert()
        if self.nav2_file is not None:
            self.notice = self.nav2_file.poll() or self.notice
        if self.test_pulses is not None and self.publish_pulses is not None:
            self.publish_pulses(self.test_pulses)
        if self.armed and self.arming.fault():
            self.stop(self.arming.fault(), by='fault')
        self.watch_watchdogs()

    def step_graphics(self, key):
        """left/right on the Graphics row moves along off, auto, kitty, sixel without wrapping; the
        choice goes to the theme overlay, whose reload (apply_theme) switches the tier."""
        i = palette.GRAPHICS.index(theme.PALETTE.graphics) + (1 if key == 'right' else -1)
        if not 0 <= i < len(palette.GRAPHICS):
            return
        if not palette.save_graphics(palette.GRAPHICS[i]):
            self.notice = f'Graphics not saved: {palette.OVERLAY} is not writable'
            return
        change = self.watcher.reload('theme')   # our own save is not a hand edit
        if change is not None and not change.ok:
            self.announce(change)
            return
        self.notice = f'Graphics {theme.PALETTE.graphics}{theme.SEPARATOR}{pixels.TIER}'

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
        elif self.tab == 'Fusion' and not (self.review or self.manual):
            hints = FUSION_HINTS + GLOBAL_HINTS
        if self.nav2_file is not None and self.focus == 'status' and not (self.manual or self.review):
            hints = PANEL_HINTS['status'] + [('e', 'Nav2')] + GLOBAL_HINTS
        shared = {} if self.review else feeds_module.merged(self.feeds)
        own = dict(
            tab=self.tab, minimap=self.minimap, layerbox=self.layerbox, checklist=getattr(self, 'checklist', None),
            **(dict(zip(('bags', 'bag_usage'), self.bags_view())) if self.tab == 'Bags' else {}), focus=self.focus, cursors=self.cursors, modal=self.modal, running=self.running,
            activity=self.activity, now=self.monitor.clock(),
            toasts=self.toasts.visible(self.monitor.clock(), bool(self.modal)),
            toasts_more=self.toasts.hidden(self.monitor.clock()), notice=notice, hints=hints,
            position=f'{min(self.cursors[self.focus], count - 1) + 1}/{count}' if count else '',
            focus_title=self.correction_focus() if self.tab == 'Correction' else None,
            authority=self.driving(shared), operator=None if self.review else operator_state.current(),
            agent=authority_chip.banner(shared.get('authority_record'), time.time(), desk=shared.get('agent_desk')),
            armed=self.armed, armed_by=self.armed_by if self.armed else None,
            arm_label=('armed' + theme.SEPARATOR + self.profile.outputs)
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
        own.update(self.fusion_view())
        return {**shared, **own}  # App's own keys win a collision

    def fusion_view(self):
        """The Fusion tab's feed snapshot while the tab is shown (live only); its subscriptions end when it is not."""
        shown = self.tab == 'Fusion' and not self.review
        feed = getattr(self, 'fusion_feed', None)
        if feed is None and shown and getattr(self.ros, 'node', None) is not None:
            from mhseals_testing.ros import fusion_feed
            cfg = fusion_feed.load_config()
            camera = camera_feed.profile_feed(self.ros.node, self.profile.sensors, hz=cfg['feed']['camera_hz'])
            feed = self.fusion_feed = fusion_feed.node_feed(self.ros, camera, cfg) or False
        review = {'fusion': None, 'fusion_review': True} if self.tab == 'Fusion' and self.review else {}
        if not feed:
            return {'fusion': None} if shown else review
        if shown:
            # a stale or missing fusion_state keeps the last correction (the readout marks it stale), so the scan
            # is not drawn uncorrected under boxes a corrected fusion still ranges
            feed.applied = fusion_panel.fusion_status(self.facts).get('applied_deg') or feed.applied
        # the frame only as image tiles; over a link budget only once turned on (i), drawn in blocks within the budget
        image = self.fusion_on()
        feed.want(shown, camera=image and pixels.active() and pixels.images('fusion.camera'))
        return {'fusion': {**feed.snapshot(), 'image': image}, 'fusion_facts': self.facts} if shown else review

    def fusion_on(self):
        """Whether the Fusion camera image is on: as toggled (i), else on locally and off over a link budget."""
        return self.fusion_image if self.fusion_image is not None else pixels.BUDGET is None

    def driving(self, shared=None):
        """The authority chip: from the arbiter's record, stale-aware (ui U4); local guess without one."""
        return authority_chip.driving(shared if shared is not None else feeds_module.merged(self.feeds),
                                      getattr(self, 'arbiter', None) or getattr(self, 'authority_view', None),
                                      self.manual, self.running, time.time())

    def control(self):
        """``snapshot['control']`` for the Status Control rows."""
        record = feeds_module.merged(self.feeds).get('authority_record')
        return authority_chip.control(record, self.driving(), time.time(), self.settings.agents,
                                      agent_motion=self.settings.agent_motion)

    def render(self):
        return layout.render(self.snapshot(), self.view(),
                                self.console.size.width, self.console.size.height)

    def lively(self):
        """Something moves on screen (driving, a run, an act, a popup, a replay, the Fusion tab): idle ticks draw at
        the FRAME cap, as ever."""
        goals = getattr(self, 'goals', None)
        return bool(self.armed or self.running or self.manual or self.modal is not None or self.review
                    or getattr(getattr(self, 'desk', None), 'act', None) or getattr(goals, 'active', None)
                    or self.tab == 'Fusion')

    def still_key(self, now):
        """A cheap signature of a still screen: the notice, toasts, events, health, authority, the agent desk and the
        IDLE_S tick (ages); equal means no render. Anything it misses still shows within IDLE_S."""
        desk = getattr(self, 'desk', None)
        events = getattr(self.monitor, 'events', ())
        return (int(now / IDLE_S), str(self.notice or ''), self.tab, getattr(self, 'focus', None),
                tuple((t.id, t.kind, t.text) for t in getattr(self.toasts, '_items', {}).values()),
                len(events), events[-1] if events else None, id(getattr(self.engine, 'last', None)),
                getattr(getattr(self, 'arbiter', None), 'sent', None),   # the record's content (its timer is 4 Hz)
                (len(desk.recent), len(desk.notes), len(desk.queue), len(desk.notices)) if desk is not None else None,
                getattr(self.map_state(), 'fence_status_at', None) if hasattr(self, 'map_state') else None)

    def refresh(self, force=True):
        """Draw now. Continuous callers pass force=False to cap redraws at 1/FRAME; a still screen draws only when
        ``still_key`` changed (D-20261101-26)."""
        if self.live is None:
            return
        now = time.monotonic()
        if not force and now - self.last_frame < FRAME:
            return
        repaint = pixels.watch()    # tmux: a client came or went, or its images need repainting
        key = None if force or self.lively() else self.still_key(now)
        if key is not None and key == self.last_key and not repaint:
            return
        self.last_key = key
        self.last_frame = now
        if repaint:
            self.last_drawn = None
        frame = self.render()
        images = pixels.active()
        if not force or images:
            # Idle ticks: skip the write when the frame is byte-identical to the last one,
            # and hand Live the segments already laid out instead of rendering twice.
            # Image tiers always lay out first: pixels finds its tiles in the segments.
            drawn = list(self.console.render(frame))
            if not force and drawn == self.last_drawn:
                return
            self.last_drawn = drawn if not force else None
            frame = Segments(pixels.frame(drawn, tuple(self.console.size)))   # scanned only when it changed
        else:
            self.last_drawn = None
        pixels.before(self.write_terminal)
        self.live.update(frame, refresh=True)
        pixels.after(self.write_terminal)

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
                            self.refresh()               # the key's frame first, before the tick's work
                        self.tick()
                        self.refresh(force=False)        # idle ticks render only on change
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
        checklist = getattr(self, 'checklist', None)
        if checklist is not None and not self.review and checklist.handle(self, key):
            return
        if key == 'V' and not self.review:  # Shift-V; folded 'v' still toggles Results/log on Activity
            self.perception_popup(view=True)  # ui.md keymap camera: View already open
            return
        if key == 'H':  # Shift-H opens the history everywhere; folded 'h' still adjusts a setting
            self.open('history')
            return
        if key in ('E', 'U', 'O', 'S'):  # Shift bindings of the global scope; never the lowercase procedure letters
            if key == 'E':
                self.revoke_agent()
            elif key == 'O':
                self.take_control()
            elif key == 'S':
                self.operator_menu()
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
        if self.tab == 'Fusion' and not self.review and self.fusion_keys.get(raw) == 'camera_image':
            self.fusion_image = not self.fusion_on()
            self.notice = f"camera image {'on' if self.fusion_image else 'off'}"
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
            self.show('Help', rows=help_rows(self.profile, HELP), width=help_width(getattr(getattr(self, 'console', None), 'width', None)))
        elif key == 'a':
            self.arm_panel()
        elif key == 'x' and raw == 'x':
            if not self.toasts.ack_newest(self.monitor.clock()):   # x dismisses a toast first
                self.export()
        elif key == 'b':
            self.mark_note() if raw == 'B' else self.mark()
        elif key in ('+', '=', '-', '_'):
            step = 0.05 if key in ('+', '=') else -0.05
            self.set_setting('amplitude', round(min(1.0, max(0.05, self.settings.amplitude + step)), 2))
            self.notice = f'Amplitude {self.settings.amplitude:g} {self.profile.command_units}'
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

    def moving(self):
        return bool(self.running or self.manual)

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

    def characterize(self, direction):
        """r: re-refine ``direction`` (the cursor column); R: the whole survey. The setting is this run's only."""
        procedure = next(p for p in PROCEDURES if p.key == 'f')
        before, self.settings.direction = self.settings.direction, direction
        try:
            self.run_procedure(procedure)
        finally:
            self.settings.direction = before

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
                observations=self.camera_boxes, work=camera_view.background)   # decode and encode off the keys

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

    # Reasons offered by the run popup's X; an exclusion always carries one (critique K4).
    EXCLUDE_REASONS = ('operator error', 'wrong conditions', 'sensor fault', 'test run')

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
                if getattr(self.arming, 'pending', None) is not None:
                    self.arming.cancel()
                if self.plant is not None:
                    self.plant.neutral()
                    self.plant.release()
        finally:
            self.toasts.push('arm.error', 'fail', text, self.monitor.clock())
            self.notice = f'{text}; outputs neutral'

    def _arm_panel(self):
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
        self.armed_by = None                     # the operator's arm; an agent's sets {'who', 'sim'}
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware))
        self.notice = 'Armed at neutral'
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
        self.armed_by = {'who': who, 'sim': sim}   # the ARMED chip reads ◆ who [sim-arm]; disarm clears it
        self.ros.emit('armed', profile=self.profile.name, outputs=self.profile.outputs,
                      config=str(self.paths.hardware), by='agent', who=who, sim_arm=list(checks))
        what = 'sim auto-arm' if sim else 'agent arm'
        self.notice = f'Armed for agent {who} ({what})'
        why = ''
        if not self.await_mux(arbiter, since, lambda mux: True):
            why = 'the mux did not report after arming'
        elif self.override_cause():
            with arbiter.lock:
                mux = arbiter.mux or {}
            cause = mux.get('cause') or ((mux.get('holds') or [{}])[0] or {}).get('cause') or self.override_cause()
            why = f'the mux holds ({cause}); only the operator releases it (U)'
        if why:
            self.disarm(f'{what} undone: {why}', by='agent')
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
            self.armed_by = None
            self.watcher.watches['hardware'].warning = None
            self.ros.emit('disarmed', reason=reason, by=by)
        self.notice = f'{reason}; outputs neutral'

    def operator_stop(self, agent=None):
        """Space requests a sticky manual hold, then always neutralizes and disarms. ``agent`` is the id of an
        agent whose ``stop`` this is (None: the operator's Space); ``stopped_by`` keeps it for the desk's hints."""
        self.command = (0.0, 0.0, 0.0)
        self.stopped_by = f'agent {agent}' if agent else f'viewer {self.client_name()}' if self.watching \
            else 'operator'
        try:
            if self.plant is not None:
                self.plant.hold('keys', f'view {self.client_name()}') if self.watching else self.plant.hold('keys')
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

    def run_procedure(self, procedure):
        if self.review:
            self.notice = f'{procedure.title}: procedures are off in review'
            return
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
        desk = getattr(self, 'desk', None)
        if desk is not None and procedure.key == 'm':
            desk.take_over()             # manual cancels an agent goal at once, as Space and E do
        if procedure.key == 'm' and not operator_state.paused(operator_state.load()):
            self.set_operator_state('manual', reason='manual control (m)', notice=False)
        try:
            # Only jump to Map once it has a body; otherwise the operator lands on a placeholder.
            if (self.settings.auto_tab and 'Map' in layout.BODIES
                    and getattr(procedure, 'group', '') in ('Navigate', 'Mission')):
                self.show_tab('Map')
            self.running = procedure.title
            results = len(self.snapshot()['results'])
            self.run_id = runs.next_run_id(self.paths.runs, procedure.title)
            self.run_fields = {}   # run-level fields a procedure adds to its run record (mission)
            self.tags = dict(procedure=procedure.title, key=procedure.key, run_id=self.run_id)
            self.notice = f'Running {procedure.title}'
            self.health_snapshot('start', procedure)
            if procedure.key == 'f':           # characterization (f, and an agent's run f): the fence follows
                self.anchor_fence()
            began = self.engine.clock() if self.engine else 0.0
        except BaseException:              # manual never outlives the drive it was set for
            self.running, self.tags = None, {}
            self.end_manual(procedure)
            raise
        aborted, ended = None, dict(outcome='complete', cause=None)
        try:
            try:
                procedure.run(self.procedure_context())
            except BaseException:
                self.health_window(procedure, began)   # before teardown, whatever the exit
                raise
            invalid = self.health_window(procedure, began)
            stopped = next((r.get('verdict') for r in self.snapshot()['results'][results:]
                            if r.get('verdict') in runs.ABORT_VERDICTS), None)
            if stopped:      # its goal ended early (the fence's return goal preempts it): aborted, never complete
                guard = self.guard_cause()
                aborted, ended = guard_end(guard, 'guard') if guard else (
                    f'{procedure.title} {stopped.lower()}', dict(outcome='aborted', cause=stopped.lower(),
                                                                 by='procedure'))
        except BaseException as error:    # a KeyboardInterrupt is recorded too, then re-raised
            aborted, ended = str(error), run_end(error)
            guard = self.guard_cause() if isinstance(error, Aborted) else ''
            if guard and ended.get('cause') != guard:  # the stop came while the watchdog held: its cause, not a cancel
                aborted, ended = guard_end(guard, ended.get('by'))
            if not isinstance(error, Cancelled):
                raise
            self.notice = f'{procedure.title} cancelled'
            self.disarm(f'{procedure.title} cancelled')
            return
        finally:
            self.running, self.tags = None, {}
            self.end_manual(procedure)
            self.write_run(procedure, results, aborted, ended)
        if len(self.snapshot()['results']) > results:  # show what it found
            self.focus, self.activity = 'activity', 'results'
            self.cursors['activity'] = 0
            if aborted is not None:
                self.notice = f"{procedure.title} ABORTED: {ended.get('cause')}"
            elif not invalid:
                self.notice = f'{procedure.title} complete'

    def end_manual(self, procedure):
        """Back to normal when manual control (m) ends, if manual is still the one m set."""
        if procedure.key == 'm' and operator_state.load()['reason'] == 'manual control (m)':
            self.set_operator_state('normal', reason='manual control ended', notice=False)

    def write_run(self, procedure, start, aborted, ended=None):
        """The one run file for a procedure run: every result it recorded since ``start``, how the
        run ended (``ended``: outcome and cause, see run_end) and the procedure's run fields, which
        win (a mission's ``refused``). A run with neither results, an abort nor fields leaves none."""
        found = self.snapshot()['results'][start:]
        run_id, self.run_id = self.run_id, None
        fields = getattr(self, 'run_fields', None) or {}
        if not found and aborted is None and not fields:
            return
        key = agent_acts.procedure(procedure.key)[1]   # its agent name (mission); title: display
        record = dict(ended or {}, **fields, procedure=procedure.title, run_key=key,
                      run_id=run_id, results=found)
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
                mission_profile=getattr(self, 'agent_profile', None),   # and its nav profile (finals)
                run_id=getattr(self, 'run_id', None), run_fields=getattr(self, 'run_fields', {}),
                manual=dict(plant=self.plant, blockers=self.manual_now, checks=self.manual_checks,
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
            place = getattr(runner, 'place', None)
            if place is not None:     # a long run (characterization): bar, step, command against measured
                self.notice = (progress.line(place, command, self.measured_velocity(), remaining, theme.SEPARATOR)
                               + f'{theme.SEPARATOR}Space aborts')
            else:
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

        def on_pause():     # the human drives with the deployed C, the survey with identity
            if self.staging().active:
                self.staging().suspend()

        def on_resume():
            if self.staging().active:
                self.staging().resume()

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

    def measured_velocity(self):
        """(vx, vy, wz) from live odometry, or None."""
        row = self.monitor.snapshot()['sensors'].get('odometry', {})
        values = row.get('values') or {}
        found = [values.get(key) for key in ('vx', 'vy', 'wz')]
        return found if row.get('status') == 'LIVE' and all(isinstance(v, (int, float)) for v in found) else None

    def override_cause(self):
        """Why the command mux is not under automation (derive's rules in harness/authority.py).

        '' when no arbiter is bound or the profile has no mux; when disarmed
        (the mux only runs as an arming child) a missing or stale state is '' too.
        """
        arbiter = getattr(self, 'arbiter', None)
        if arbiter is None or not muxed(self.profile):
            return ''
        with arbiter.lock:
            mux, at, pause = arbiter.mux, arbiter.mux_at, arbiter.pause
        age = float('inf') if at is None else arbiter.clock() - at
        if not self.armed and (mux is None or age > STALE_MUX_S):
            return ''
        return override_cause(mux, age, pause)

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
