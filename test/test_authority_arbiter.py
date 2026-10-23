"""The arbiter with fake transport: claim ack, handover, TUI-owner rule, revoke."""

import copy
import json

from mhseals_testing.ros.authority import Arbiter, exit_code


class FakeRegistry:
    """The Registry read/update interface over a dict."""

    def __init__(self):
        self.data = {}

    def read(self):
        return copy.deepcopy(self.data)

    def update(self, change):
        change(self.data)
        self.data = json.loads(json.dumps(self.data))  # what a file round trip does


class Rig:
    def __init__(self, registry=None, **kw):
        self.now, self.sent, self.events, self.calls, self.cancelled = 1000.0, [], [], [], []
        self.goals = []
        self.arbiter = Arbiter(self.sent.append, lambda phase, **f: self.events.append((phase, f)),
                               registry=registry, clock=lambda: self.now,
                               sources=lambda: {'nav': {'goals': self.goals}},
                               neutral=lambda: self.calls.append('neutral'),
                               hold=lambda cause, by: self.calls.append(('hold', cause, by)), **kw)
        self.arbiter.on_mux({'input': 'auto', 't_ns': int(self.now * 1e9)})

    def ask(self, **request):
        return self.arbiter.handle({'op': 'claim', 'request': 'r1', 'who': 'tune', **request})


def test_claim_is_acked_in_the_next_record_and_second_claim_is_busy():
    rig = Rig(tui_owner=True)
    ack = rig.ask(reason='sweep')
    assert ack['ok'] and ack['claim']['id'] == 'c1' and ack['id'] == 'c1'
    latest = rig.sent[-1]
    assert latest['ack']['request'] == 'r1' and latest['owner'] == 'agent'
    busy = rig.ask(request='r2', who='other')
    assert exit_code(busy) == 2 and busy['holder']['id'] == 'c1'
    assert busy['id'] is None and not busy['ok']
    assert exit_code(ack) == 0


def test_agent_claim_without_a_tui_owner_exits_4():
    rig = Rig(tui_owner=False)
    ack = rig.ask()
    assert ack['error'] == 'no_owner' and exit_code(ack) == 4
    assert rig.arbiter.table.claims == []


def test_blocked_agents_exit_5_and_revoke_over_the_topic_is_refused():
    rig = Rig(tui_owner=True, allow_agents=False)
    assert exit_code(rig.ask()) == 5
    rig = Rig(tui_owner=True)
    rig.ask()
    ack = rig.arbiter.handle({'op': 'revoke', 'id': 'c1', 'request': 'x'})
    assert ack['error'] == 'forbidden' and rig.arbiter.table.get('c1')['state'] == 'active'


def test_handover_keeps_claims_and_seq():
    registry = FakeRegistry()
    first = Rig(registry, tui_owner=True)
    first.ask()
    first.arbiter.close(keep_claims=True)
    second = Rig(registry, tui_owner=True)
    assert [c['id'] for c in second.arbiter.table.claims] == ['c1']
    second.arbiter.table.release('c1', second.now)
    assert second.ask(request='r3')['claim']['id'] == 'c2'  # never re-issues c1


def test_claims_end_with_the_session_unless_kept():
    registry = FakeRegistry()
    rig = Rig(registry, tui_owner=True)
    rig.ask()
    rig.arbiter.close()
    assert registry.data['authority']['claims'] == []
    assert registry.data['authority']['seq'] == 1


def test_revoke_stops_the_boat_cancels_goals_then_revokes():
    rig = Rig(tui_owner=True)
    rig.ask()
    rig.goals.append({'id': 'g1', 'status': 'executing', 'by': 'tune',
                      'cancel': lambda: rig.calls.append('cancel g1')})
    assert rig.arbiter.revoke('c1')
    assert rig.calls == ['neutral', ('hold', 'revoke', 'tui'), 'cancel g1']
    assert rig.arbiter.table.get('c1')['state'] == 'revoked'
    assert rig.sent[-1]['owner'] != 'agent' and 'cancel' not in json.dumps(rig.sent[-1])
    assert 'claim_revoke' in [phase for phase, _ in rig.events]


def test_record_is_republished_only_on_change_or_heartbeat():
    rig = Rig(tui_owner=True)
    rig.arbiter.tick()
    count = len(rig.sent)
    rig.now += 0.3
    rig.arbiter.on_mux({'input': 'auto', 't_ns': int(rig.now * 1e9)})
    rig.arbiter.tick()
    assert len(rig.sent) == count
    rig.now += 1.0
    rig.arbiter.on_mux({'input': 'auto', 't_ns': int(rig.now * 1e9)})
    rig.arbiter.tick()
    assert len(rig.sent) == count + 1
