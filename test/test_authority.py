"""Claim table: ownership, busy holder, leases and revocation, no ROS."""

from mhseals_testing.harness.authority import ClaimTable

NOW = 100.0


def table():
    return ClaimTable()


def test_a_guest_is_an_agent():
    claims = table()
    claim, _ = claims.claim('procedure', 'attached', NOW, pid=99, host='tranquility')
    assert claim['owner'] == 'agent'


def test_only_local_claims_keep_their_owner():
    for kw in ({}, {'pid': 10}, {'pid': 10, 'host': 'tranquility'}):
        assert table().claim('procedure', 'sq', NOW, **kw)[0]['owner'] == 'agent'
    assert table().claim('procedure', 'sq', NOW, local=True)[0]['owner'] == 'procedure'


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
    assert claims.get(claim['id'])['state'] == 'revoked'
    assert claims.claim('agent', 'next', NOW)[0]
