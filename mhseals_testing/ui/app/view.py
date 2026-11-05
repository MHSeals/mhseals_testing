"""Rendering: the panels and rows on screen, the view and what changed since the last frame."""

import os
import socket
import time

from rich.segment import Segments

from mhseals_hardware.thruster_mixer import validate_mixer
from mhseals_testing import tunables
from mhseals_testing.harness import operator_state
from mhseals_testing.harness.apply import LiveStaging
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ros import camera_feed
from mhseals_testing.ui import authority_chip, feeds as feeds_module, layout, palette, parts, perf, theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui import input as key_input
from mhseals_testing.ui.app import state as app_state
from mhseals_testing.ui.app.model import Frame, Snapshot, ViewState
from mhseals_testing.ui.panels import PANELS, agent_log, fusion as fusion_panel
from mhseals_testing.ui.feeds import health as health_feed
from mhseals_testing.ui.layout import MINIMAP, body_rows, motion_shown
from mhseals_testing.ui.panels.status import status_rows

from mhseals_testing.ui.app.help import (
    CORRECTION_HINTS, FUSION_HINTS, GLOBAL_HINTS, LABELS_HINTS, MANUAL_HINTS, PANEL_HINTS, REVIEW_HINTS)


FRAME = 1 / 30           # s; continuous redraws (motion, manual) stop at 30 per second


def idle_s():
    """s; a still screen redraws only on a change, and this often for its ages (input.yaml tune, read live)."""
    return tunables.tune('input')['idle_s']


class View:
    """Rendering, mixed into App."""

    def snapshot(self) -> Snapshot:
        """The boat's state for rendering (model.Snapshot): what a wave-2 backend would serve."""
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
            snapshot['graphics'] = (theme.PALETTE.graphics, pixels.TIER, pixels.WHY)
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
            return len(validate_mixer(self.args.hardware_config['thruster_matrix']))
        if panel == 'list':
            return len(self.bags_view()[0])
        if panel == 'labels':
            return self.labels_count()
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

    def view(self) -> ViewState:
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
        if self.tab == 'Labels' and not self.review:
            hints = LABELS_HINTS + [('Space', 'Stop')]     # the tab's own keys; ? and q stay in the help
        chip = None if self.review else self.review_chip()
        hints = ([chip] if chip else []) + hints
        pick = None if self.review else self.pick_hints()
        if pick:
            hints = pick + [('Space', 'Stop')]
        shared = {} if self.review else feeds_module.merged(self.feeds)
        thrusters = self.services.thrusters   # the Thrusters panel's edit, Manual's to write
        own = dict(
            tab=self.tab, minimap=self.minimap, layerbox=self.layerbox, checklist=getattr(self, 'checklist', None),
            **(dict(zip(('bags', 'bag_usage'), self.bags_view())) if self.tab == 'Bags' else {}), **({'labels': self.labels_model()} if self.tab == 'Labels' and not self.review else {}), focus=self.focus, cursors=self.cursors, modal=self.modal, running=self.running,
            activity=self.activity, now=self.monitor.clock(),
            toasts=self.toasts.visible(self.monitor.clock(), bool(self.modal)),
            toasts_more=self.toasts.hidden(self.monitor.clock()), notice=notice, hints=hints,
            flash=None if self.review else self.flashing(),
            position=f'{min(self.cursors[self.focus], count - 1) + 1}/{count}' if count else '',
            focus_title=self.correction_focus() if self.tab == 'Correction' else None,
            authority=self.driving(shared), operator=None if self.review else operator_state.current(),
            agent=authority_chip.banner(shared.get('authority_record'), time.time(), desk=shared.get('agent_desk')),
            armed=self.armed, armed_by=self.armed_by if self.armed else None,
            arm_label=('armed' + theme.SEPARATOR + self.profile.outputs)
            if self.armed else 'disarmed', command=self.command, chords=self.chords,
            chords_unavailable=key_input.chords_unavailable(self.keys),
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
            test_pulses=thrusters.test_pulses, pwm_readback=self.pwm_readback,
            draft=thrusters.draft, grabbed=thrusters.grabbed,
            config_note=self.config_note, review=self.review, correction=self.correction_state(),
            procedures=[dict(key=p.key, title=p.title, description=p.description, group=p.group,
                             reason=self.availability(p)) for p in PROCEDURES])
        if self.review and not self.source.live:
            own.update(profile=self.source.profile.name, specs=self.source.profile.sensors,
                       units=self.source.profile.command_units,
                       outputs=self.source.profile.outputs,
                       plot_scale=self.source.profile.plot_scale, now=self.source.now,
                       host=self.review.host, deploy={}, recording=False, rec=None,
                       pwm_readback=[theme.DASH] * len(own['matrix']), toasts=[], toasts_more=0)
        own.update(self.fusion_view())
        return {**shared, **own}  # App's own keys win a collision

    def fusion_view(self):
        """The Fusion tab's feed snapshot while the tab is shown (live only); its subscriptions end when it is not."""
        shown = self.tab == 'Fusion' and not self.review
        feed = getattr(self, 'fusion_feed', None)
        if feed is None and shown and getattr(self.ros, 'node', None) is not None:
            from mhseals_testing.fusion import ros_feed as fusion_feed
            cfg = fusion_feed.load_config()
            camera = camera_feed.profile_feed(self.ros.node, self.profile.sensors, hz=cfg['feed']['camera_rate'])
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

    def frame(self):
        """One draw's input (ui/app/model.py): the boat's snapshot and this station's view at the console's size."""
        return Frame(self.snapshot(), self.view(), self.console.size.width, self.console.size.height)

    def render(self):
        return self.frame().render()

    def lively(self):
        """Something moves on screen (driving, a run, an act, a popup, a replay, the Fusion tab): idle ticks draw at
        the FRAME cap, as ever."""
        goals = getattr(self.desk, 'goals', None)
        return bool(self.armed or self.running or self.manual or self.modal is not None or self.review
                    or getattr(getattr(self, 'desk', None), 'act', None) or getattr(goals, 'active', None)
                    or self.tab == 'Fusion')

    def still_key(self, now):
        """A cheap signature of a still screen: the notice, toasts, events, health, authority, the agent desk and the
        idle_s tick (ages); equal means no render. Anything it misses still shows within idle_s."""
        desk = getattr(self, 'desk', None)
        events = getattr(self.monitor, 'events', ())
        return (int(now / idle_s()), str(self.notice or ''), self.tab, getattr(self, 'focus', None),
                tuple((t.id, t.kind, t.text) for t in getattr(self.toasts, '_items', {}).values()),
                len(events), events[-1] if events else None, id(getattr(self.engine, 'last', None)),
                getattr(getattr(self, 'arbiter', None), 'sent', None),   # the record's content (its timer is 4 Hz)
                (len(desk.recent), len(desk.notes), len(desk.queue), len(desk.notices)) if desk is not None else None,
                getattr(self.map_state(), 'fence_status_at', None) if hasattr(self, 'map_state') else None)

    def repaint(self):
        """The next idle frame draws in full (the terminal's images restarted)."""
        app_state.state_of(self).render.last_drawn = None

    def refresh(self, force=True):
        """Draw now. Continuous callers pass force=False to cap redraws at 1/FRAME; a still screen draws only when
        ``still_key`` changed (D-20261101-26)."""
        if self.live is None:
            return
        render = app_state.state_of(self).render     # View's own redraw bookkeeping (ui/app/state)
        now = time.monotonic()
        if not force and now - self.last_frame < perf.period(self, FRAME):   # perf.yaml fps, slowed by slow frames
            return
        repaint = pixels.watch()    # tmux: a client came or went, or its images need repainting
        key = None if force or self.lively() else self.still_key(now)
        if key is not None and key == render.last_key and not repaint:
            return
        render.last_key = key
        render.last_frame = now
        if repaint:
            render.last_drawn = None
        frame = self.render()
        images = pixels.active()
        if not force or images:
            # Idle ticks: skip the write when the frame is byte-identical to the last one,
            # and hand Live the segments already laid out instead of rendering twice.
            # Image tiers always lay out first: pixels finds its tiles in the segments.
            drawn = list(self.console.render(frame))
            if not force and drawn == render.last_drawn:
                return
            render.last_drawn = drawn if not force else None
            frame = Segments(pixels.frame(drawn, tuple(self.console.size)))   # scanned only when it changed
        else:
            render.last_drawn = None
        pixels.before(self.write_terminal)
        self.live.update(frame, refresh=True)
        pixels.after(self.write_terminal)

    def map_state(self):
        """The ``MapState`` the ROS feeds fill, or None without them."""
        if not self.source.live:
            return None
        return getattr(getattr(self.ros, 'map', None), 'state', None)
