"""Boat-link facts from /proc fixtures, counter resets and address changes."""

import pytest

from mhseals_testing.health.netstat import Link, parse_dev, parse_snmp, rates

DEV = """Inter-|   Receive                                                |  Transmit
 face |bytes    packets errs drop fifo frame compressed multicast|bytes    packets errs drop fifo colls carrier compressed
    lo:     100       1    0    0    0     0          0         0      100       1    0    0    0     0       0          0
  end0: {rx}  10 {errs}  {drop}    0     0          0         0 {tx}  10    0    0    0     0       0          0
"""
# the boat's kernel has MemErrors after SndbufErrors; the dock's stops at InCsumErrors
SNMP_BOAT = """Ip: Forwarding DefaultTTL InReceives ReasmFails FragFails
Ip: 2 64 100 {reasm} 0
Udp: InDatagrams NoPorts InErrors OutDatagrams RcvbufErrors SndbufErrors InCsumErrors MemErrors IgnoredMulti
Udp: 5 {noports} {inerr} 5 {rcvbuf} 0 0 0 0
"""
SNMP_DOCK = SNMP_BOAT.replace(' MemErrors', '').replace('0 0 0 0\n', '0 0 0\n')


def proc(root, rx=0, tx=0, errs=0, drop=0, noports=0, inerr=0, rcvbuf=0, reasm=0, snmp=SNMP_BOAT, speed='1000'):
    """Write a fake /proc and /sys under root."""
    (root / 'proc/net').mkdir(parents=True, exist_ok=True)
    (root / 'sys/class/net/end0').mkdir(parents=True, exist_ok=True)
    (root / 'proc/net/dev').write_text(DEV.format(rx=rx, tx=tx, errs=errs, drop=drop))
    (root / 'proc/net/snmp').write_text(snmp.format(noports=noports, inerr=inerr, rcvbuf=rcvbuf, reasm=reasm))
    (root / 'sys/class/net/end0/speed').write_text(speed + '\n')


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return Clock()


def link(tmp_path, clock, address='192.168.0.152', environ=None):
    state = {'address': address}
    link = Link('192.168.0.0/24', root=tmp_path, environ=environ or {'XDG_RUNTIME_DIR': '/run'}, clock=clock,
                addresses=lambda names: {'end0': state['address']} if state['address'] and 'end0' in names else {})
    link.state = state
    return link


def test_parse_dev_reads_interfaces_by_column():
    parsed = parse_dev(DEV.format(rx=5, tx=7, errs=2, drop=3))
    assert parsed['end0']['rx_bytes'] == 5 and parsed['end0']['tx_bytes'] == 7
    assert parsed['end0']['rx_errs'] == 2 and parsed['end0']['rx_drop'] == 3 and 'lo' in parsed


@pytest.mark.parametrize('template', [SNMP_BOAT, SNMP_DOCK])
def test_parse_snmp_matches_by_header_on_both_kernels(template):
    parsed = parse_snmp(template.format(noports=900, inerr=2, rcvbuf=3, reasm=4))
    assert parsed['Udp']['NoPorts'] == 900 and parsed['Udp']['RcvbufErrors'] == 3
    assert parsed['Ip']['ReasmFails'] == 4
    assert ('MemErrors' in parsed['Udp']) == (template is SNMP_BOAT)


def test_rates_report_a_reset_as_none():
    assert rates({'a': 10, 'b': 10}, {'a': 30, 'b': 4}, 2) == {'a': 10.0, 'b': None}


def test_rates_over_two_samples(tmp_path, clock):
    net = link(tmp_path, clock)
    proc(tmp_path, rx=0, tx=0)
    assert 'net.rx_bps' not in net.sample()                  # one sample is no rate
    clock.now += 5
    proc(tmp_path, rx=1_250_000, tx=125_000, errs=5, drop=5, noports=1000, inerr=10, rcvbuf=15, reasm=5)
    facts = net.sample()
    assert facts['net.rx_bps'].value == pytest.approx(2e6)
    assert facts['net.tx_bps'].value == pytest.approx(2e5)
    assert facts['net.bw_pct'].value == pytest.approx(0.2)
    assert facts['net.errs_ps'].value == pytest.approx(2)
    assert facts['udp.drops_ps'].value == pytest.approx(6)    # (10 + 15 + 5) / 5
    assert facts['udp.noports_ps'].value == pytest.approx(200)


def test_counter_reset_drops_that_fact_then_recovers(tmp_path, clock):
    net = link(tmp_path, clock)
    proc(tmp_path, rx=5000, noports=500)
    net.sample()
    clock.now += 5
    proc(tmp_path, rx=100, noports=10)                        # both counters restarted
    facts = net.sample()
    assert 'net.rx_bps' not in facts and 'udp.noports_ps' not in facts
    clock.now += 5
    proc(tmp_path, rx=600, noports=60)
    facts = net.sample()
    assert facts['net.rx_bps'].value == pytest.approx(800) and facts['udp.noports_ps'].value == pytest.approx(10)


def test_unknown_link_speed_gives_no_percentage(tmp_path, clock):
    net = link(tmp_path, clock)
    proc(tmp_path, speed='-1')
    net.sample()
    clock.now += 1
    proc(tmp_path, rx=1000, speed='-1')
    facts = net.sample()
    assert 'net.rx_bps' in facts and 'net.bw_pct' not in facts


def test_no_boat_address_gives_no_facts(tmp_path, clock):
    net = link(tmp_path, clock, address='10.0.0.5')
    proc(tmp_path)
    assert net.sample() == {}


def test_address_change_against_the_pinned_address(tmp_path, clock):
    net = link(tmp_path, clock, environ={'ASTRO_DDS_ADDRESS': '192.168.0.147'})
    proc(tmp_path)
    assert net.sample()['net.addr_changed'].value == 1.0
    net.environ['ASTRO_DDS_ADDRESS'] = '192.168.0.152'
    assert net.sample()['net.addr_changed'].value == 0.0


def test_pinned_address_falls_back_to_the_state_file(tmp_path, clock):
    import os
    net = link(tmp_path, clock)
    state = tmp_path / 'run' / f'astro-dds-address-{os.getuid()}'
    state.parent.mkdir()
    state.write_text('192.168.0.147\n')
    net.environ['XDG_RUNTIME_DIR'] = '/run'
    proc(tmp_path)
    assert net.sample()['net.addr_changed'].value == 1.0


def test_address_events_count_changes_in_five_minutes(tmp_path, clock):
    net = link(tmp_path, clock)
    proc(tmp_path)
    assert net.sample()['net.addr_events_5m'].value == 0       # the first sighting is not a change
    net.state['address'] = '192.168.0.147'
    clock.now += 5
    assert net.sample()['net.addr_events_5m'].value == 1
    clock.now += 400
    assert net.sample()['net.addr_events_5m'].value == 0
