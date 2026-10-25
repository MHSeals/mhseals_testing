"""Pinger: loss and rtt window math, packet framing and the no-privilege fallback."""

import socket
import struct

from mhseals_testing.health import pinger as pg


def test_checksum_of_built_packet_verifies():
    assert pg.checksum(pg.echo_request(7, 3)) == 0


def test_reply_seq_only_for_echo_replies():
    reply = struct.pack('!BBHHH', 0, 0, 0, 1, 42)
    assert pg.reply_seq(reply) == 42
    assert pg.reply_seq(struct.pack('!BBHHH', 8, 0, 0, 1, 42)) is None
    assert pg.reply_seq(b'\0') is None


def test_window_loss_and_rtt():
    w = pg.Window(size=4)
    assert w.loss_pct() is None and w.rtt_ms() is None
    for s in (0.002, None, 0.004, None):
        w.add(s)
    assert w.loss_pct() == 50.0
    assert abs(w.rtt_ms() - 3.0) < 1e-9
    w.add(0.001)                                    # oldest (0.002) drops out
    assert w.loss_pct() == 50.0
    for _ in range(3):
        w.add(0.001)                                # both losses drop out
    assert w.loss_pct() == 0.0


def test_window_all_lost_has_no_rtt():
    w = pg.Window(size=3)
    for _ in range(3):
        w.add(None)
    assert w.loss_pct() == 100.0 and w.rtt_ms() is None


class FakeSock:
    """Answers pings to addresses in ``alive`` after ``delay`` fake seconds."""

    def __init__(self, clock, alive, delay=0.003):
        self.clock, self.alive, self.delay = clock, alive, delay
        self.pending = None

    def sendto(self, packet, addr):
        self.pending = packet if addr[0] in self.alive else None

    def recv(self, n):
        if self.pending is None:
            self.clock.t += pg.TIMEOUT
            raise socket.timeout()
        self.clock.t += self.delay
        seq = struct.unpack('!H', self.pending[6:8])[0]
        self.pending = None
        return struct.pack('!BBHHH', 0, 0, 0, 0, seq)


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def test_rounds_give_per_peer_loss_and_rtt():
    clock = Clock()
    p = pg.Pinger({'jetson': '10.0.0.3', 'router': '10.0.0.1'}, clock=clock)
    sock = FakeSock(clock, alive={'10.0.0.1'})
    for _ in range(5):
        p.round(sock)
    facts = p.facts()
    assert facts['net.loss_pct.jetson'].value == 100.0
    assert 'net.rtt_ms.jetson' not in facts
    assert facts['net.loss_pct.router'].value == 0.0
    assert abs(facts['net.rtt_ms.router'].value - 3.0) < 1e-6


def test_stale_reply_for_another_seq_is_not_counted():
    clock = Clock()
    p = pg.Pinger({'a': '10.0.0.1'}, clock=clock)

    class Wrong(FakeSock):
        def recv(self, n):
            clock.t += 0.3
            return struct.pack('!BBHHH', 0, 0, 0, 0, 999)

    p.round(Wrong(clock, {'10.0.0.1'}))
    assert p.facts()['net.loss_pct.a'].value == 100.0


def test_forbidden_socket_yields_no_facts():
    def deny():
        raise PermissionError('ping_group_range')

    p = pg.Pinger({'a': '10.0.0.1'}, open_socket=deny)
    p.run()
    assert p.available is False and p.facts() == {}


def test_peer_addresses_come_from_the_lan_reservations_else_the_resolver(tmp_path):
    from mhseals_testing.health.network import lease_addresses, peer_addresses
    lan = tmp_path / 'boat-network.env'
    lan.write_text('ROUTER_IP=10.0.0.1\n'
                   'STATIC_LEASES="aa:bb=boat-pc=10.0.0.5 cc:dd=cam-host=10.0.0.6"  # reservations\n')
    assert lease_addresses(lan) == {'boat-pc': '10.0.0.5', 'cam-host': '10.0.0.6'}
    assert peer_addresses({'camera': 'cam-host', 'other': 'elsewhere.local'}, lan) == {
        'camera': '10.0.0.6', 'other': 'elsewhere.local'}
    assert lease_addresses(tmp_path / 'missing.env') == {}


class FakePinger:
    def __init__(self, peers):
        self.peers, self.started, self.values = peers, 0, {}

    def start(self):
        self.started += 1

    def facts(self):
        return dict(self.values)


def test_network_row_pings_the_camera_host_only_on_the_boat_link(tmp_path):
    from pathlib import Path
    from mhseals_testing.health.model import Fact
    from mhseals_testing.health.network import NetworkRow
    from test_network import Clock, proc
    lan = tmp_path / 'lan.env'
    lan.write_text('STATIC_LEASES="aa=blastoise-jetson=10.1.2.3"\n')
    made, held, clock = [], {}, Clock()

    def pinger(peers):
        made.append(FakePinger(peers))
        return made[-1]
    net = NetworkRow(Path(__file__).resolve().parents[1] / 'config' / 'health', root=tmp_path, clock=clock,
                     environ={}, addresses=lambda names: dict(held), lan_file=lan, pinger=pinger)
    assert made[0].peers == {'camera': '10.1.2.3'}
    proc(tmp_path)
    assert net.poll() is None and made[0].started == 0          # off the boat: never ping
    held['end0'] = '192.168.0.152'
    made[0].values = {'net.loss_pct.camera': Fact(100.0, label='camera')}
    for _ in range(4):                                          # past the check's raise time
        clock.now += 5
        proc(tmp_path)
        level, detail = net.poll()
    assert made[0].started == 1
    assert (level, detail) == ('FAIL', '100.00 %; camera host ping loss')
