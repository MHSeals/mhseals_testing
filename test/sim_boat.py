"""A boat in a fake clock: the real mux arbitration and authority derivation.

Wires ``Arbitration`` (mhseals_hardware), ``ClaimTable``/``derive``/``events``
(harness.authority) to a first-order 3-DOF plant, plus the two pieces that do
not exist yet as code: a fake ``Authority`` client and a fake ``Nav2Apply``
(spec authority_hotreload section 6 and A12). Everything is synchronous: the
agent code calls ``sim.step()`` through ``wait_until_released`` and ``run``, so
a scripted loop reads like the real one. ``at(t, fn)`` schedules operator
actions (RC switch, Space, U, E) on the fake clock.
"""

from mhseals_hardware.command_arbitration import Arbitration
from mhseals_hardware.configuration import load_defaults
from mhseals_testing.harness.authority import ClaimTable, derive, events

DT = 0.04                                  # one tick: the 25 Hz RC rate
GUIDED, HOLD, MANUAL = 1900, 1550, 1100    # mode channel PWM (MODE1..6 = 0,0,0,4,0,15)
T0 = 100.0


class Busy(RuntimeError):
    """Another automated claim is active."""


class Revoked(RuntimeError):
    """The claim was revoked by the operator."""


class Sim:
    def __init__(self):
        self.now, self.claims = T0, ClaimTable()
        self.arb = Arbitration.from_config(load_defaults()['command_mux'])
        self.rc = [1500, 1500, 1500, 1500, 1500, 1800, 1500, GUIDED]
        self.record, self.log, self.goals, self.applied = None, [], [], (0.0, 0.0, 0.0)
        self.speed, self._script, self._holds = [0.0] * 3, [], set()
        self.step()

    def at(self, t, fn):
        """Run ``fn(sim)`` on the first tick at or after ``T0 + t``."""
        self._script.append((T0 + t, fn))

    @property
    def t(self):
        return self.now - T0

    def step(self, auto=None):
        """One tick: script, mux, plant, derive, journal."""
        self.now += DT
        for item in [s for s in self._script if s[0] <= self.now]:
            self._script.remove(item)
            item[1](self)
        self.arb.update_rc(self.rc, self.now)
        if auto is not None:
            self.arb.update_auto(auto, self.now)
        _, self.applied = self.arb.select(self.now)
        self.speed = [v + DT / 0.5 * (c - v) for v, c in zip(self.speed, self.applied)]
        mux = self.arb.state(self.now)
        record = derive(mux, self.claims.claims, {'goals': self.goals}, None, None, None,
                        self.now, self.record, mux_age_s=0.0)
        self.log += self._hold_edges(mux) + events(self.record, record) + self.claims.drain()
        self.record = record

    def _hold_edges(self, mux):
        causes = {h['cause'] for h in mux['holds']}
        out = [{'phase': 'hold', 'rec': {'cause': c}} for c in sorted(causes - self._holds)]
        out += [{'phase': 'hold_release', 'rec': {'cause': c}} for c in sorted(self._holds - causes)]
        self._holds = causes
        return out

    def phases(self):
        return [e['phase'] for e in self.log]

    # Operator actions

    def switch(self, pwm, stick=1500):
        self.rc[7], self.rc[1] = pwm, stick

    def space(self):
        self.arb.update_manual((0.0, 0.0, 0.0), self.now)
        self.arb.request('hold', 'keys', 'tui', self.now)

    def release(self):
        """The U key: returns (ok, reason)."""
        return self.arb.request('release', 'keys', 'tui', self.now)

    def revoke(self):
        """The E key: Space, a revoke hold, cancel attributed goals, revoke the claim."""
        self.space()
        self.arb.request('hold', 'revoke', 'tui', self.now)
        for goal in self.goals:
            goal['status'] = 'canceled'
        for claim in self.claims.claims:
            self.claims.revoke(claim['id'], self.now)


class Authority:
    """Client side of the claim API over the in-memory bus (the sim)."""

    def __init__(self, sim, who, reason=''):
        self.sim, self.who, self.reason, self.claim_id = sim, who, reason, None

    def claim(self):
        claim, holder = self.sim.claims.claim('agent', self.who, self.sim.now, self.reason)
        if claim is None:
            raise Busy(holder)
        self.claim_id = claim['id']
        self.sim.step()
        return claim

    def release(self):
        self.sim.claims.release(self.claim_id, self.sim.now)

    def _mine(self):
        return next((c for c in self.sim.record['claims'] if c['id'] == self.claim_id), None)

    def ours(self):
        """Our claim is active and no human or hold has the output."""
        mine = self._mine()
        return bool(mine and mine['state'] == 'active'
                    and self.sim.record['input'] in ('auto', 'idle'))

    def paused(self):
        return not self.ours()

    def wait_until_released(self, timeout=None):
        """Step the sim while paused; False on timeout, Revoked if revoked."""
        end = None if timeout is None else self.sim.now + timeout
        while True:
            mine = self._mine()
            if mine is None or mine['state'] == 'revoked':
                raise Revoked(self.claim_id)
            if self.ours():
                return True
            if end is not None and self.sim.now >= end:
                return False
            self.sim.step()


class Nav2Apply:
    """Fake parameter applier: refuses unless the authority is ours, sends nothing then."""

    def __init__(self):
        self.sets = []

    def limits(self, overlay, authority):
        if not authority.ours():
            owner = authority.sim.record['owner']
            return {'blocked': [f'authority: {owner}'], 'applied': False}
        self.sets.append(dict(overlay))
        return {'blocked': [], 'applied': True}


def tuning_loop(sim, auth, nav2, overlays, run_s=5.0):
    """The spec section 6 loop; returns the overlays whose run was trusted."""
    trusted = []
    for overlay in overlays:
        auth.wait_until_released()
        if nav2.limits(overlay, authority=auth)['blocked']:
            continue
        end, overridden = sim.now + run_s, False
        while sim.now < end:
            sim.step(auto=(overlay['vx_max'], 0.0, 0.0))
            if auth.paused():
                overridden = True
                break
        if not overridden:
            trusted.append(overlay)
    return trusted
