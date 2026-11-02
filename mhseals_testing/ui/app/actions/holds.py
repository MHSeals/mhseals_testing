"""Watchdog holds, their release and the geofence."""

import time

from mhseals_hardware.menu import Cancelled
from mhseals_testing.harness import control, course_area
from mhseals_testing.agent import grants as agent_grants
from mhseals_testing.runner import STALE_MUX_S
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import agent_acts, feeds as feeds_module


HOLD_CONFIRM_S = 2.0      # s the mux state may take to show a requested watchdog hold
WATCHDOG_GRACE = 2.0     # s a required safety watchdog may be missing from the graph while armed (graph lag)


class Holds:
    """Watchdog holds, their release and the geofence., mixed into App."""

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

    def revoke_agent(self):
        """E: revoke the agent's active claim (stops the boat first); no confirm.

        A viewer has no arbiter: it holds the boat itself at once, then asks the owner's desk to revoke.
        """
        if self.watching:
            self.operator_stop()
            control.request_revoke(self.client_name())
            self.notice = 'Stopped; revoke sent to the controller'
            return
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

    def fence_client(self):
        """fence_watchdog's course area and on_exit (ros/fence.py) on this owner's node; None for a watcher or review."""
        if self.watching or self.review or getattr(self.ros, 'node', None) is None:
            return None
        if getattr(self, '_fence_client', None) is None:
            from mhseals_testing.ros.fence import FenceClient
            self._fence_client = FenceClient(self.ros.node)
        return self._fence_client

    def anchor_fence(self):
        """/fence/anchor at a characterization's start: a map fence re-centres on the boat; a lat/lon course area
        stays ("course area fixed"). Journaled and logged; a failure never stops the run."""
        client = self.fence_client()
        if client is None:
            return
        try:
            ok, message = agent_acts.in_background(self, 'Fence anchor', client.anchor)
        except Exception as error:      # noqa: BLE001 - no fence_watchdog (a bench): logged, the run goes on
            ok, message = False, f'{type(error).__name__}: {error}'
        self.emit('fence_anchor', ok=ok, message=message)
        self.monitor.event(f"fence anchor: {message}")

    def fence_status(self, state):
        """The last /fence/status, marked stale when fence_watchdog went quiet (MapFeeds' own clock)."""
        clock = getattr(getattr(self.ros, 'map', None), 'clock', time.monotonic)
        return course_area.fresh(getattr(state, 'fence_status', None), getattr(state, 'fence_status_at', None),
                                 clock())

    def fence_alert(self):
        """Outside the course area: the notice and an error toast that stays until the boat is back inside (or x),
        with the live distance and what happens (on_exit); "U releases the hold" only while the mux holds."""
        state = self.map_state()
        status = self.fence_status(state) if state is not None else None
        outside = isinstance(status, dict) and status.get('inside') is False
        was = getattr(self, '_fence_out', False)
        if outside:
            text = course_area.status_line(status, held=bool(self.mux_holds()))[1]
            if not was:
                self.notice = text
                self.toasts.push('fence.outside', 'fail', text, self.monitor.clock())
                self.emit('fence_outside', distance=status.get('distance_to_edge'), on_exit=status.get('on_exit'))
            elif text != getattr(self, '_fence_text', None):
                if self.notice == getattr(self, '_fence_text', None):
                    self.notice = text
                self.toasts.update('fence.outside', text)
            self._fence_text = text
        back = was and isinstance(status, dict) and status.get('inside') is True
        if back:
            self.toasts.push('fence.outside', 'ok', 'Back inside the course area', self.monitor.clock())
        self._fence_out = outside or (was and not back)      # no position yet is not back inside

    def mux_holds(self):
        """The mux's holds as the authority record lists them (saved ones while the mux is stopped); [] when none."""
        record = feeds_module.merged(self.feeds).get('authority_record') if getattr(self, 'feeds', None) else None
        return list((record or {}).get('holds') or [])

    def guard_cause(self):
        """The watchdog holding the boat now ('' when none): a watchdog node's mux hold (the fence's ``geofence``),
        or ``geofence`` while the boat is outside the course area (on_exit return: the fence's goal preempts first)."""
        found = next((str(h.get('cause')) for h in self.mux_holds() if agent_grants.kind(h) == 'node'), '')
        return found or ('geofence' if getattr(self, '_fence_out', False) else '')
