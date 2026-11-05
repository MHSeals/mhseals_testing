"""App: the page's state and the coordinator of its parts. Behaviour lives in the mixins
(loop, view, actions/), each one area; App holds the state they share."""

import subprocess
import threading
import time

from rich.console import Console
from rich.live import Live

from mhseals_testing.ui import input as key_input
from mhseals_testing.harness.bags import Recorder
from mhseals_testing.analysis.readiness import planned_directions
from mhseals_testing.conditions import Conditions
from mhseals_testing.harness.recwatch import RecWatch
from mhseals_testing.health.advisories import Health
from mhseals_testing.health.journal import Journal
from mhseals_testing.ui import theme
from mhseals_testing.ui.layout import TabMemory
from mhseals_testing.ui.panels.checklist import Checklist
from mhseals_testing.ui.source import LiveSource

from mhseals_testing.ui.app.shared import CURSORS
from mhseals_testing.ui.app import state as app_state
from mhseals_testing.ui.app import mixins, wiring


# Who writes what (finding #4): each mixin owns an attribute group and is its one writer; another area (a mixin,
# a feed, a popup) reads it, through app.services where a container exists, and asks the owner's method to change
# it. SHARED is the page's own state that every part sets (the popup, focus, the Activity view, the cursors,
# quit; the footer notice only through App.say). APP is what App builds and hands out: startup.py and wiring.py
# build it, and after that App.rebuild is its one writer (a review swaps some of it). test_app_state_ownership
# fails on any write across owners in ui/**. The containers in ui/app/state hold the groups converted so far
# (render, bags behind aliases; manual, arm, holds and procedures as ``state.<group>``, their own); Labels keeps its LabelsState.
SHARED = ('notice', 'modal', 'focus', 'activity', 'cursors', 'quit')
APP = ('profile', 'source', 'ros', 'tabs', 'minimap', 'layerbox', 'review', 'arbiter', 'authority_view', 'desk')
OWNERS = {
    'View': ('last_frame', 'last_drawn', 'last_key', '_staging', 'fusion_feed'),
    'Loop': ('last_tick', 'fusion_image', 'detaching'),
    'Bags': ('bag_recording_key', 'bag_scan', 'finalizing', 'finalize_thread', 'rec', '_review_live', '_review_fault'),
    'Labels': ('_labels_state',),   # its LabelsState (actions/labels.py), read as app.labels
    'Manual': ('state.manual',),
    'Arm': ('state.arm',),
    'Holds': ('state.holds',),
    'Procedures': ('state.procedures',),
    'Popups': ('_nav2_live', 'camera_boxes'),
    'Annotate': ('flash', 'pick', 'bad_on', '_ids', '_queued'),
    'Quit': ('closed', 'guests_seen', 'keep_units', 'stop_adopted', 'keys', 'live'),
    'Operator': ('handed_by', 'handed_to', 'restart_as'),
    'Correction': ('column',),
    'Files': ('config_note', 'pending_settings', 'map_keys', 'map_tab_keys', 'fusion_keys'),
    'Health': ('last_advisory', 'pwm_readback'),
}


@app_state.install
class App(*mixins.ROSTER):
    """The page, its key handling and the procedure context.

    ROS, processes, arming, the plant, keys, the live view and the raw-pulse
    publisher are injected so tests drive the whole loop with fakes.
    """

    rtk_link = None       # the running RTK base (ui/rtk.py): .base and .reload; None means no base

    @property
    def services(self):
        """Read-only facades on other areas' state (ui/app/state/services.py): ``app.services.bags.scan``."""
        found = self.__dict__.get('_services')
        if found is None:
            found = self.__dict__['_services'] = app_state.facades(self)
        return found

    def __init__(self, args, profile, monitor, ros, processes, paths, console=None,
                 arming=None, plant=None, open_keys=key_input.KeyReader, live=Live,
                 clock=time.monotonic, foreground=subprocess.call, nav2=None,
                 task_watcher=None, pulses=None, open_events=None, registry=None, watching=0,
                 facts=None, source=None, nav2_file=None, nav2_live=None,
                 review=None, network=None, recorder=None, ekf=None, quiet_run=None, stack=None,
                 issues=None):
        self.state = app_state.AppState()   # each area's container (ui/app/state); OWNERS says whose
        self.args = args
        self.stack = stack      # harness.components.Stack: the device parts' Status rows (ui/parts.py)
        self.issues = wiring.issues(issues)   # device/part/transport problems past their grace
        self.keep_units = False   # Quit's 'keep remote parts': their leases run out instead
        self.stop_adopted = False  # Quit's 'also stop adopted parts' (units another TUI may own)
        self.ekf = ekf          # harness.ekf.EkfRestart, or None when the session cannot restart them
        self.registry, self.watching = registry, watching   # watching: pid of the owner we only watch
        self.detaching = False
        self.handed_by = self.restart_as = None   # the takeover's taker; 'take' or 'view' to restart as one
        self.guests_seen = set()
        self._review_live = None
        self.review = review     # a ui.review.Review while a past session plays; live work is off
        self.profile = profile
        self.monitor = monitor
        self.network = network
        wiring.live_feeds(self, profile)   # tracking, rtk, annotations and the feeds list
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
        self.recording_lock = threading.RLock()
        self.publish_pulses = pulses
        self.journal = Journal()
        # Manual control needs key releases (Kitty protocol) to tell held keys from taps: the
        # session's reader turns them on (ui/input.py), keeping its queue across manual mode.
        self.open_events = open_events or (lambda: key_input.events(self.keys))
        self.nav2_factory = nav2
        self.nav2_file = nav2_file  # harness.nav2_edit.Nav2File, or None
        self.nav2_live_factory, self._nav2_live = nav2_live, None   # -> harness.nav2_live.Nav2Live
        if nav2_file is not None and nav2_file.applier is None and nav2_live is not None:
            nav2_file.applier = self.apply_nav2_live
        self.watcher_factory = task_watcher
        self.settings = args.settings
        self.tabs = TabMemory()
        wiring.map_views(self)   # minimap, layerbox, map_keys, map_tab_keys, fusion_keys
        self.checklist = Checklist.from_env()   # boat_tui preview's sanity checklist, else None
        self.fusion_image = None   # the Fusion camera image: None until toggled (on locally, off over a link budget)
        self.focus = 'status'
        self.cursors = dict.fromkeys(CURSORS, 0)
        self.column = None   # Correction tab column; None follows the worst one
        self.activity = 'results'
        self.modal = None
        self.toasts = wiring.toasts()
        self._wall_offset = None
        self.say(theme.OVERLAY_WARNING or 'Passive monitor; a opens the arm panel')
        self.last_advisory = float('-inf')
        self.health = Health(profile, paths.bag.parent, facts, clock, hardware=lambda: self.args.hardware_config)
        self.feeds.append(wiring.toast_feed(self.health))
        if getattr(ros, 'mission', None) is not None:   # model.mission_status.MissionState, fed by ROS
            self.feeds.append(ros.mission)
        self.recwatch = RecWatch(self.clock)
        self.recorder = recorder or Recorder(processes, paths.bag, profile.name)
        self.rec = None                 # latest RecSample, drawn in the header strip
        self.pwm_readback = [theme.DASH] * 4
        self.keys = self.live = None
        self.last_tick = clock()
        self.quit = False
        self.closed = False
        self.started = clock()
        wiring.config_watch(self, args)   # config_file, history, config_backends, watcher
        self.watch_own_files()
        self.config_note = self.config_file.name if self.config_file.exists() else 'defaults'
        self.pending_settings = None

    @property
    def armed(self):
        return self.arming is not None and self.arming.armed

    arbiter = authority_view = desk = None   # the owner's arbiter and desk, a viewer's record (startup.py)

    def say(self, text):
        """The footer notice: the one writer of ``notice``, so every message is greppable."""
        self.notice = text

    def rebuild(self, **built):
        """Replace parts App built (APP): a review's source and views, a switched profile."""
        unknown = sorted(set(built) - set(APP))
        if unknown:
            raise AttributeError(f'not built by App: {unknown}')
        for name, value in built.items():
            setattr(self, name, value)

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
            self.say('An interrupted characterization was undone: the deployed C is back')

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
        self.ros.emit(phase, **{**self.services.procedures.tags, **details})

    def record(self, result):
        run_id = self.services.procedures.run_id
        if run_id:
            result = {**result, 'run_id': run_id}
        self.monitor.record_result(result)
        self.ros.emit('result', **result)
        self.export()
