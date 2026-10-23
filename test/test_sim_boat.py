"""An agent tuning loop against a manual override (spec authority_hotreload A12, section 9)."""

import pytest
from sim_boat import GUIDED, HOLD, MANUAL, Authority, Nav2Apply, Revoked, Sim, tuning_loop

OVERLAYS = [{'vx_max': 0.2 + 0.1 * i} for i in range(4)]


def start():
    sim = Sim()
    auth = Authority(sim, 'tune_mppi', 'vx_max sweep')
    auth.claim()
    return sim, auth, Nav2Apply()


def order(sim, *wanted):
    """The wanted phases appear in sim.log in this order."""
    phases = iter(sim.phases())
    return all(any(p == w for p in phases) for w in wanted)


def test_rc_override_pauses_the_loop_and_excludes_the_run():
    sim, auth, nav2 = start()
    seen = {}

    def manual(s):
        s.switch(MANUAL, stick=1800)
        s.arb.update_rc(s.rc, s.now)
        seen['rc'] = s.arb.select(s.now)

    def blocked(s):
        seen['blocked'] = nav2.limits({'vx_max': 9}, authority=auth)
        seen['sets'] = len(nav2.sets)

    sim.at(12.0, manual)
    sim.at(13.0, lambda s: s.switch(MANUAL))
    sim.at(13.5, blocked)
    sim.at(14.0, lambda s: s.switch(HOLD))
    sim.at(16.0, lambda s: s.switch(GUIDED))
    trusted = tuning_loop(sim, auth, nav2, OVERLAYS)

    assert seen['rc'][0] == 'rc' and seen['rc'][1][0] > 0   # rc is applied in the tick it arrives
    assert seen['blocked'] == {'blocked': ['authority: rc'], 'applied': False}
    assert seen['sets'] == 3                                # nothing was sent during the override
    assert trusted == [OVERLAYS[0], OVERLAYS[1], OVERLAYS[3]]
    assert sim.t > 16.0                                     # waited for GUIDED
    assert order(sim, 'authority', 'claim_suspend', 'override', 'hold_release', 'claim_resume')
    assert [e['rec']['by'] for e in sim.log if e['phase'] == 'override'] == ['rc']   # rc then hold is one override
    first = next(e['rec'] for e in sim.log if e['phase'] == 'authority' and e['rec']['to']['owner'] == 'rc')
    assert (first['from']['owner'], first['to']['owner']) == ('agent', 'rc')


def test_auto_ramps_in_over_one_second_after_the_release():
    sim, auth, _ = start()
    sim.at(1.0, lambda s: s.switch(HOLD))
    sim.at(2.0, lambda s: s.switch(GUIDED))
    steps = []
    while sim.t < 4.0:
        sim.step(auto=(0.4, 0.0, 0.0))
        steps.append((sim.t, sim.applied[0]))
    half = next(v for t, v in steps if t >= 2.5)
    full = next(v for t, v in steps if t >= 3.2)
    assert 0.1 < half < 0.3 and full == pytest.approx(0.4)


def test_space_holds_until_u_not_the_switch():
    sim, auth, nav2 = start()
    sim.at(3.0, lambda s: s.space())
    sim.at(4.0, lambda s: s.switch(HOLD))
    results = {}
    sim.at(5.0, lambda s: results.update(at_hold=s.release()))
    sim.at(6.0, lambda s: s.switch(GUIDED))
    while sim.t < 9.0:
        sim.step(auto=(0.3, 0.0, 0.0))
    assert results['at_hold'] == (False, 'mode switch is at HOLD')
    assert auth.paused() and sim.record['owner'] == 'hold'     # GUIDED did not resume
    assert auth.wait_until_released(timeout=1.0) is False
    assert sim.release() == (True, '')                         # U
    assert auth.wait_until_released(timeout=1.0) is True
    assert nav2.limits(OVERLAYS[0], authority=auth)['applied']
    assert order(sim, 'claim_suspend', 'hold_release', 'claim_resume')


def test_e_revokes_cancels_the_goal_and_the_loop_raises():
    sim, auth, nav2 = start()
    sim.goals.append({'id': 'g1', 'by': 'nav', 'status': 'executing'})
    sim.at(2.0, lambda s: s.revoke())
    with pytest.raises(Revoked):
        tuning_loop(sim, auth, nav2, OVERLAYS)
    assert sim.goals[0]['status'] == 'canceled'
    assert sim.record['owner'] in ('keyboard', 'hold')
    assert 'claim_revoke' in sim.phases()
    sim.at(sim.t + 1.0, lambda s: s.switch(GUIDED))
    assert sim.release()[0]
    for _ in range(50):
        sim.step()
    assert auth.paused()                                       # a revoked claim stays revoked
