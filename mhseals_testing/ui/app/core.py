"""App: the page's state and the coordinator of its parts. Behaviour lives in the mixins
(loop, view, actions/), each one area; App holds the state they share."""

import errno
import os
import socket
import subprocess
import threading
import time

from rich.console import Console
from rich.live import Live
from rich.segment import Segments

from mhseals_hardware.configuration import config_path
from mhseals_hardware.keyboard import KeyReader
from mhseals_hardware.thruster_mixer import validate_mixer
from mhseals_testing.harness import operator_state
from mhseals_testing.harness.bags import Recorder
from mhseals_testing.harness.apply import LiveStaging
from mhseals_testing.analysis.readiness import planned_directions
from mhseals_testing.harness.conditions import Conditions
from mhseals_testing.runner import fold
from mhseals_testing.harness.recwatch import RecWatch
from mhseals_testing.harness.watch import Watched, Watcher
from mhseals_testing.health.advisories import Health
from mhseals_testing.health.journal import Journal
from mhseals_testing.history import History, backends
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ros import camera_feed
from mhseals_testing.ui import (
    authority_chip, feeds as feeds_module, keymap as keymap_module, layout, palette, parts, pixels, theme)
from mhseals_testing.ui.panels import PANELS, agent_log, fusion as fusion_panel
from mhseals_testing.ui.feeds import health as health_feed
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
from mhseals_testing.ui.popups import pins
from mhseals_testing.ui.source import LiveSource

from mhseals_testing.ui.app.help import (
    CORRECTION_HINTS, FUSION_HINTS, GLOBAL_HINTS, HELP, MANUAL_HINTS, PANEL_HINTS, REVIEW_HINTS, help_rows,
    help_width, map_key_maps)
from mhseals_testing.ui.app.shared import CURSORS, DIRECTIONS, Detach, STOPPING_ERRORS
from mhseals_testing.ui.app.actions.quit import Quit
from mhseals_testing.ui.app.actions.operator import Operator
from mhseals_testing.ui.app.actions.bags import Bags
from mhseals_testing.ui.app.actions.settings import Settings
from mhseals_testing.ui.app.actions.holds import Holds
from mhseals_testing.ui.app.actions.arm import Arm
from mhseals_testing.ui.app.actions.manual import Manual
from mhseals_testing.ui.app.actions.procedures import Procedures
from mhseals_testing.ui.app.actions.popups import Popups


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


FRAME = 1 / 30           # s; continuous redraws (motion, manual) stop at 30 per second
IDLE_S = 1.0             # s; a still screen redraws only on a change, and this often for its ages and clocks


def terminal_gone(error):
    """True for the key reader's EOF or EIO: the terminal is gone, not a stopping error."""
    return isinstance(error, EOFError) or error.errno == errno.EIO


class App(Popups, Procedures, Manual, Arm, Holds, Settings, Bags, Operator, Quit):
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
