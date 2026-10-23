from mhseals_testing.health import dds

A, B = bytes([1, 15, 0xaa, 1]), bytes([1, 15, 0xbb, 2])      # host prefixes (bytes 0-3)


def gid(host, participant, entity):
    return host + bytes([0, 0, 0, 0, 0, 0, 0, participant]) + bytes([0, 0, 0, entity])


GIDS = [gid(A, 1, 3), gid(A, 1, 4), gid(A, 2, 3), gid(B, 1, 3), gid(B, 1, 5)]


def test_endpoints_of_one_participant_count_once():
    assert len(dds.participants(GIDS)) == 3


def test_counts_group_by_host_prefix_with_hex_fallback():
    assert dds.by_host(GIDS) == {A.hex(): 2, B.hex(): 1}


def test_node_globs_label_hosts():
    nodes = [('mavros', gid(A, 1, 3)), ('zed_wrapper', gid(B, 1, 3))]
    hosts = {'mavros': 'blastoise', 'zed*': 'squirtle'}
    assert dds.by_host(GIDS, hosts, nodes) == {'blastoise': 2, 'squirtle': 1}


def test_facts_total_and_per_host_and_short_gids_ignored():
    f = dds.facts(GIDS + [b'\x01\x02'], now=5.0)
    assert f['dds.participants'].value == 3
    assert f['dds.participants_by_host'].value == {A.hex(): 2, B.hex(): 1}
    assert f['dds.participants'].time == 5.0
