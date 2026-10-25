"""The Network row: netstat facts judged by the shipped config/health/network.yaml."""

from pathlib import Path

import pytest

from mhseals_testing.health.network import NetworkRow
from test_network import Clock, proc

NO_PINGS = dict(pinger=lambda peers: None)      # test_pinger.py covers the peer pings


@pytest.fixture
def clock():
    return Clock()


def row(tmp_path, clock, **kwargs):
    config = Path(__file__).resolve().parents[1] / 'config' / 'health'
    net = NetworkRow(config, root=tmp_path, clock=clock, environ={'ASTRO_DDS_ADDRESS': '192.168.0.152'},
                     addresses=lambda names: {'end0': '192.168.0.152'}, **NO_PINGS, **kwargs)
    return net


def test_row_is_hidden_without_a_boat_interface(tmp_path, clock):
    net = NetworkRow(Path(__file__).resolve().parents[1] / 'config' / 'health', root=tmp_path, clock=clock,
                     environ={}, addresses=lambda names: {}, **NO_PINGS)
    proc(tmp_path)
    assert net.poll() is None


def test_row_reads_ok_then_flags_udp_drops(tmp_path, clock):
    net = row(tmp_path, clock)
    proc(tmp_path)
    assert net.poll()[0] == 'OK'
    clock.now += 5
    proc(tmp_path, rx=625_000, tx=625_000)
    level, detail = net.poll()
    assert level == 'OK' and detail.startswith('2.0 Mb/s')
    clock.now += 5
    proc(tmp_path, rx=625_000, tx=625_000, rcvbuf=100)         # 20 /s
    net.poll()
    clock.now += 5
    proc(tmp_path, rx=625_000, tx=625_000, rcvbuf=200)         # still 20 /s: persists past the raise time
    level, detail = net.poll()
    assert (level, detail) == ('WARN', '20.00 /s; UDP drops')


def test_row_throttles_reads_to_its_period(tmp_path, clock):
    net = row(tmp_path, clock)
    proc(tmp_path)
    first = net.poll()
    proc(tmp_path, rcvbuf=10_000)
    clock.now += 1
    assert net.poll() == first


def test_row_reads_fail_when_the_address_is_lost_after_being_shown(tmp_path, clock):
    held = {'end0': '192.168.0.152'}
    net = NetworkRow(Path(__file__).resolve().parents[1] / 'config' / 'health', root=tmp_path, clock=clock,
                     environ={}, addresses=lambda names: dict(held), **NO_PINGS)
    proc(tmp_path)
    assert net.poll()[0] == 'OK'
    held.clear()                                   # cable pulled or DHCP lost
    clock.now += 5
    assert net.poll() == ('FAIL', 'no 192.168.0.0/24 address')
