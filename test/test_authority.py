"""Authority derivation: every rule of spec authority_hotreload section 3, no ROS."""

from mhseals_testing.harness.authority import ClaimTable, derive, events, is_unknown

NOW = 100.0


def mux(input='auto', **extra):
    return {'input': input, 't_ns': int(NOW * 1e9), 'rc': {'mode': 'GUIDED'}, **extra}


def goal(by='nav', status='executing'):
    return {'goals': [{'id': 'g1', 'by': by, 'status': status}]}


def run(state='auto', claims=(), nav=None, task=None, procedure=None, graph=None, **kw):
    state = mux(state) if isinstance(state, str) else state
    return derive(state, list(claims), nav, task, procedure, graph, kw.pop('now', NOW), **kw)


def table():
    return ClaimTable()


def test_human_inputs_outrank_everything_and_suspend_claims():
    claims = table()
    claim, _ = claims.claim('agent', 'tune', NOW)
    for state, owner in (('rc', 'rc'), ('manual', 'keyboard'), ('hold', 'hold')):
        record = run(mux(state), claims.claims)
        assert record['owner'] == owner
        assert record['claims'][0]['state'] == 'suspended'
    assert claims.get(claim['id'])['state'] == 'active'  # never dropped
    assert run(mux('auto'), claims.claims)['owner'] == 'agent'


def test_stale_or_missing_mux_is_unknown():
    assert run(None)['owner'] == 'unknown'
    assert run(now=NOW + 3.5)['owner'] == 'unknown'
    assert is_unknown(None, NOW) and is_unknown(run(), NOW + 3.5)
    assert not is_unknown(run(), NOW + 1)


def test_malformed_mux_state_is_unknown_not_a_crash():
    for bad in ({'t_ns': int(NOW * 1e9)}, {'input': 'auto'}, {}, 'junk'):
        record = derive(bad, [], None, None, None, None, NOW)  # bypass run(), which wraps str into a valid dict
        assert record['owner'] == 'unknown' and record['source'] == 'mux state malformed'
    assert run({'input': 'auto'}, mux_age_s=0.5)['owner'] != 'unknown'  # receipt age stands in for the stamp


def test_mux_rc_that_is_not_a_dict_is_malformed_not_a_crash():
    for rc in ('MANUAL', 5, ['MANUAL']):
        record = derive({'input': 'rc', 't_ns': int(NOW * 1e9), 'rc': rc}, [], None, None, None, None, NOW)
        assert record['owner'] == 'unknown' and record['source'] == 'mux state malformed'
    assert derive({'input': 'rc', 't_ns': int(NOW * 1e9), 'rc': None}, [], None, None, None, None, NOW)['owner'] == 'rc'


def test_staleness_uses_local_receipt_age_not_the_senders_clock():
    behind = mux() | {'t_ns': int((NOW - 5) * 1e9)}  # sender clock 5 s behind, just received
    ahead = mux() | {'t_ns': int((NOW + 5) * 1e9)}  # sender clock 5 s ahead, received 4 s ago
    assert run(behind, mux_age_s=0.2)['owner'] != 'unknown'
    assert run(ahead, mux_age_s=4.0)['owner'] == 'unknown'
    assert not is_unknown(run(), NOW + 50, age_s=0.5)
    assert is_unknown(run(), NOW, age_s=4.0)


def test_precedence_of_claim_owners_and_nesting():
    claims = table()
    claims.claim('procedure', 'sq', NOW, local=True)
    record = run(claims=claims.claims, nav=goal(by='external'), task={'status': 'RUNNING'})
    assert record['owner'] == 'procedure'  # goal and task belong to the claim
    assert not record['conflict']


def test_implicit_sources_without_a_claim():
    assert run(procedure={'name': 'square'})['owner'] == 'procedure'
    assert run(task={'status': 'RUNNING', 'name': 'gate'})['owner'] == 'task'
    assert run(task={'status': 'IDLE'})['owner'] == 'none'
    assert run(nav=goal())['owner'] == 'nav'
    assert run(nav=goal(status='succeeded'))['owner'] == 'none'
    assert run(nav=goal(), procedure={'name': 'square'})['owner'] == 'procedure'


def test_a_guest_is_an_agent():
    claims = table()
    claim, _ = claims.claim('procedure', 'attached', NOW, pid=99, host='tranquility')
    assert claim['owner'] == 'agent'
    assert run(claims=claims.claims)['owner'] == 'agent'


def test_only_local_claims_keep_their_owner():
    for kw in ({}, {'pid': 10}, {'pid': 10, 'host': 'tranquility'}):
        assert table().claim('procedure', 'sq', NOW, **kw)[0]['owner'] == 'agent'
    assert table().claim('procedure', 'sq', NOW, local=True)[0]['owner'] == 'procedure'


def test_an_unclaimed_goal_is_external():
    record = run(nav=goal(by='unknown'))
    assert record['owner'] == 'external'
    assert record['source'] == '? goal (unclaimed)'
    assert run(nav=goal(by='unknown'), task={'status': 'RUNNING'})['owner'] == 'task'


def test_a_cursor_goal_plus_an_unclaimed_goal_is_a_conflict():
    record = run(nav={'goals': [{'id': 'a', 'status': 'executing', 'by': 'nav'},
                                {'id': 'b', 'status': 'executing', 'by': 'unknown'}]})
    assert record['owner'] == 'external'
    assert record['conflict'] is True
    assert record['source'].startswith('? conflict')


def test_unclaimed_cmd_vel_is_external_and_filters_publishers_by_gid():
    graph = {'cmd_vel': {'age_s': 0.2, 'nonzero': True}, 'own_gids': ['g-own'],
             'publishers': [{'node': '/mhseals_testing', 'gid': 'g-own'},
                            {'node': '/rogue', 'gid': 'g-x'}]}
    record = run(graph=graph)
    assert (record['owner'], record['source']) == ('external', '? /rogue')
    graph['cmd_vel']['age_s'] = 1.5
    assert run(graph=graph)['owner'] == 'none'
    graph['cmd_vel'].update(age_s=0.2, nonzero=False)
    assert run(graph=graph)['owner'] == 'none'


def test_cmd_vel_from_only_our_own_publishers_is_not_external():
    graph = {'cmd_vel': {'age_s': 0.2, 'nonzero': True}, 'own_gids': ['A'],
             'publishers': [{'node': 'mhseals_testing_tui', 'gid': 'A'}]}
    assert run(graph=graph)['owner'] == 'none'


def test_cmd_vel_is_ignored_when_a_claim_is_active_or_mux_not_auto():
    graph = {'cmd_vel': {'age_s': 0.1, 'nonzero': True}, 'publishers': []}
    claims = table()
    claims.claim('agent', 'a', NOW)
    assert run(claims=claims.claims, graph=graph)['owner'] == 'agent'
    assert run(mux('hold'), graph=graph)['owner'] == 'hold'
    assert run(mux('idle'), graph=graph)['owner'] == 'none'


def test_two_automated_sources_are_a_conflict_shown_as_external():
    record = run(procedure={'name': 'square'}, task={'status': 'RUNNING', 'name': 'gate'})
    assert record['conflict'] and record['owner'] == 'external'
    assert record['source'].startswith('? conflict')
    both = [{'id': 'c1', 'owner': 'agent', 'who': 'a', 'reason': '', 'pid': 1},
            {'id': 'c2', 'owner': 'agent', 'who': 'b', 'reason': '', 'pid': 2}]
    assert run(claims=both)['conflict']


def test_procedure_beside_an_agent_claim_is_a_conflict():
    claims = table()
    claims.claim('agent', 'tune', NOW)
    record = run(claims=claims.claims, procedure={'name': 'characterize'})
    assert record['conflict'] and record['owner'] == 'external'


def test_second_claim_is_busy_and_names_the_holder():
    claims = table()
    first, _ = claims.claim('agent', 'tune', NOW)
    second, holder = claims.claim('procedure', 'sq', NOW)
    assert second is None and holder['id'] == first['id']


def test_leases_expire_but_leaseless_claims_stay():
    claims = table()
    leased, _ = claims.claim('agent', 'api', NOW, lease_s=10)
    claims.expire(NOW + 9)
    assert claims.get(leased['id'])
    claims.renew(leased['id'], NOW + 9)
    claims.expire(NOW + 18)
    assert claims.get(leased['id'])
    claims.expire(NOW + 20)
    assert claims.get(leased['id']) is None
    forever, _ = claims.claim('agent', 'cli', NOW + 20)
    claims.expire(NOW + 9999)
    assert claims.get(forever['id'])
    assert [e['phase'] for e in claims.drain()] == [
        'claim', 'claim_expire', 'claim']


def test_a_revoked_claim_stays_on_record_but_never_drives_or_blocks():
    claims = table()
    claim, _ = claims.claim('agent', 'tune', NOW)
    assert claims.revoke(claim['id'], NOW) and not claims.revoke(claim['id'], NOW)
    assert claims.drain()[-1]['phase'] == 'claim_revoke'
    record = run(claims=claims.claims)
    assert [c['state'] for c in record['claims']] == ['revoked'] and record['owner'] == 'none'
    assert run(mux('rc'), claims.claims)['claims'][0]['state'] == 'revoked'
    assert claims.claim('agent', 'next', NOW)[0]


def test_since_only_moves_when_the_owner_or_source_changes():
    first = run(procedure={'name': 'square'})
    later = derive(mux(), [], None, None, {'name': 'square'}, None, NOW + 1, prev=first)
    other = derive(mux(), [], None, None, {'name': 'station'}, None, NOW + 2, prev=later)
    assert later['since_ns'] == first['since_ns'] and other['since_ns'] > first['since_ns']


def test_events_for_override_suspend_resume_and_authority():
    claims = table()
    claims.claim('procedure', 'sq', NOW, local=True)
    auto = run(claims=claims.claims, procedure={'name': 'sq', 'run': 'r1'})
    rc = run(mux('rc', cause='rc_switch'), claims.claims, procedure={'name': 'sq', 'run': 'r1'})
    phases = [e['phase'] for e in events(auto, rc)]
    assert phases == ['authority', 'claim_suspend', 'override']
    assert [e['phase'] for e in events(rc, auto)] == ['authority', 'claim_resume']
    assert events(None, auto) == [] and events(auto, auto) == []
    assert events(auto, run(None)) == []  # an unknown record is no owner change


def test_claims_stay_in_the_record_while_the_mux_is_unknown():
    claims = table()
    claims.claim('agent', 'tune', NOW)
    for record in (run(None, claims.claims), run(claims=claims.claims, now=NOW + 3.5)):
        assert record['owner'] == 'unknown'
        assert [c['state'] for c in record['claims']] == ['suspended']
    live = run(claims=claims.claims)
    gap = run(None, claims.claims)
    assert [e['phase'] for e in events(live, gap)] == ['claim_suspend']
    assert [e['phase'] for e in events(gap, live)] == ['authority', 'claim_resume']


def test_holds_are_carried_as_a_list():
    two = [{'cause': 'keys', 'by': 'tui'}, {'cause': 'apply', 'by': 'cfg'}]
    record = run(mux('hold', holds=two))
    assert record['holds'] == two and record['source'] == 'hold keys by tui'
    assert run(mux('hold', hold=two[0]))['holds'] == two[:1]
    assert run()['holds'] == []


def test_override_survives_a_stale_gap():
    claims = table()
    claims.claim('agent', 'tune', NOW)
    procedure = {'name': 'sq', 'run': 'r1'}
    auto = run(claims=claims.claims)
    gap = run(claims=claims.claims, procedure=procedure, now=NOW + 4, prev=auto)
    assert gap['owner'] == 'unknown' and gap['last_owner'] == 'agent'
    assert run(claims=claims.claims, now=NOW + 8, prev=gap)['last_owner'] == 'agent'
    rc = derive(mux('rc', t_ns=int((NOW + 5) * 1e9)), claims.claims, None, None, procedure, None,
                NOW + 5, prev=gap)
    override = [e for e in events(gap, rc) if e['phase'] == 'override']
    assert override and override[0]['rec']['run'] == 'r1'


def test_a_human_gap_human_sequence_is_no_override():
    human = run(mux('rc'))
    gap = run(None, now=NOW + 4, prev=human)
    back = derive(mux('keyboard', t_ns=int((NOW + 5) * 1e9)), [], None, None, None, None, NOW + 5, prev=gap)
    assert not [e for e in events(gap, back) if e['phase'] == 'override']


def test_rebuilt_table_never_reuses_a_revoked_claims_id():
    table = ClaimTable()
    first, _ = table.claim('agent', 'a', NOW)
    table.revoke(first['id'], NOW)
    rebuilt = ClaimTable(table.claims)
    second, _ = rebuilt.claim('agent', 'b', NOW)
    assert second['id'] != first['id']
    assert rebuilt.get(first['id'])['state'] == 'revoked'
