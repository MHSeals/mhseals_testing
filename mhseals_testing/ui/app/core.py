"""App: the page's state and the coordinator of its parts. Behaviour lives in the mixins
(loop, view, actions/), each one area; App holds the state they share."""

import subprocess
import threading
import time

from rich.console import Console
from rich.live import Live

from mhseals_hardware.configuration import config_path
from mhseals_hardware.keyboard import KeyReader
from mhseals_testing.harness.bags import Recorder
from mhseals_testing.analysis.readiness import planned_directions
from mhseals_testing.conditions import Conditions
from mhseals_testing.harness.recwatch import RecWatch
from mhseals_testing.harness.watch import Watched, Watcher
from mhseals_testing.health.advisories import Health
from mhseals_testing.health.journal import Journal
from mhseals_testing.history import History, backends
from mhseals_testing.ui import keymap as keymap_module, theme
from mhseals_testing.ui.feeds import health as health_feed
from mhseals_testing.ui.feeds.conditions import ConditionsFeed
from mhseals_testing.ui.feeds.issues import IssuesFeed
from mhseals_testing.ui.feeds.rtk import RtkFeed
from mhseals_testing.ui.feeds.toasts import ToastFeed
from mhseals_testing.ui.feeds.tracking import TrackingFeed
from mhseals_testing.ui.toasts import Toasts
from mhseals_testing.ui.layout import TabMemory
from mhseals_testing.ui.minimap import panel as minimap_panel
from mhseals_testing.ui.minimap.layerbox import LayerBox
from mhseals_testing.ui.panels.checklist import Checklist
from mhseals_testing.ui.source import LiveSource

from mhseals_testing.ui.app.shared import CURSORS
from mhseals_testing.ui.app.help import map_key_maps
from mhseals_testing.ui.app.actions.quit import Quit
from mhseals_testing.ui.app.actions.operator import Operator
from mhseals_testing.ui.app.actions.bags import Bags
from mhseals_testing.ui.app.actions.settings import Settings
from mhseals_testing.ui.app.actions.holds import Holds
from mhseals_testing.ui.app.actions.arm import Arm
from mhseals_testing.ui.app.actions.manual import Manual
from mhseals_testing.ui.app.actions.procedures import Procedures
from mhseals_testing.ui.app.actions.popups import Popups
from mhseals_testing.ui.app.view import View
from mhseals_testing.ui.app.loop import Loop


class App(Loop, View, Popups, Procedures, Manual, Arm, Holds, Settings, Bags, Operator, Quit):
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

    def offer_restore(self):
        """At start, disarmed: an interrupted run's backed-up C goes back to the YAML the driver starts from."""
        if not self.armed and self.staging().recover():
            self.notice = 'An interrupted characterization was undone: the deployed C is back'

    # Reasons offered by the run popup's X; an exclusion always carries one (critique K4).
    EXCLUDE_REASONS = ('operator error', 'wrong conditions', 'sensor fault', 'test run')

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
