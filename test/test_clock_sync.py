"""pre.clock_sync: SNTP offsets of the boat computers and their spread, judged at 50 ms."""

from dataclasses import replace
from pathlib import Path
import socket
import struct
import threading
import time

import pytest

from mhseals_testing.health import clock_sync
from mhseals_testing.health.config import load
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.network import NetworkRow
from mhseals_testing.profiles import load_profile
from test_network import Clock, proc

LAN = Path(__file__).resolve().parent / 'data' / 'boat-network.env'


def ntp(seconds):
    return struct.pack('!II', int(seconds + clock_sync.NTP_EPOCH), int(seconds % 1 * 2 ** 32))


@pytest.fixture
def server():
    """A one-shot SNTP server on localhost whose clock runs ``lead`` s ahead; yields (port, settings)."""
    settings = {'lead': 0.2, 'leap': 0, 'stratum': 2}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))

    def answer():
        _, sender = sock.recvfrom(48)
        now = time.time() + settings['lead']
        head = bytes([settings['leap'] << 6 | 4 << 3 | 4, settings['stratum']]) + bytes(30)
        sock.sendto(head + ntp(now) + ntp(now), sender)
    thread = threading.Thread(target=answer, daemon=True)
    thread.start()
    yield sock.getsockname()[1], settings
    sock.close()


def test_offset_reads_a_leading_host(server):
    port, _ = server
    assert clock_sync.offset('127.0.0.1', port=port) == pytest.approx(0.2, abs=0.02)


@pytest.mark.parametrize('field, value', [('leap', 3), ('stratum', 0)])
def test_an_unsynchronised_answer_is_no_offset(server, field, value):
    port, settings = server
    settings[field] = value
    assert clock_sync.offset('127.0.0.1', port=port) is None


def test_silence_is_no_offset():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as quiet:
        quiet.bind(('127.0.0.1', 0))
        assert clock_sync.offset('127.0.0.1', timeout=0.1, port=quiet.getsockname()[1]) is None


def test_spread_includes_this_computer_and_needs_every_host():
    fact = clock_sync.spread({'blastoise': 0.010, 'orin': -0.045})['clock.spread']
    assert fact.value == pytest.approx(0.055) and fact.label == 'blastoise vs orin'
    assert clock_sync.spread({'orin': 0.02})['clock.spread'].label == 'orin vs this computer'
    assert clock_sync.spread({'blastoise': 0.0, 'orin': None}) == {} == clock_sync.spread({})


def test_shipped_check_fails_over_50_ms(nav_share):
    """test_preflight.py covers ok/fail/unknown; here the warn band and the probe's roles."""
    config = load(profile=load_profile('boat'), user=False)
    spec = replace(next(s for s in config.specs if s.id == 'pre.clock_sync'), raise_after=0.0)
    assert config.probes['clock_sync']['roles'] == ['boat_computer', 'camera_host']
    engine = Engine([spec], {'clock_sync': 5}, clock=lambda: 100.0)
    facts = clock_sync.spread({'blastoise': 0.0, 'orin': 0.04})
    assert engine.tick(facts, {'clock_sync': (100.0, None)}).results[spec.id].status == 'warn'


def test_network_row_samples_clocks_on_the_boat_link(tmp_path):
    started, clock = [], Clock()

    class Peers(clock_sync.ClockPeers):
        def start(self):
            started.append(dict(self.peers))
            self.round()

    net = NetworkRow(Path(__file__).resolve().parents[1] / 'config' / 'health', root=tmp_path, clock=clock,
                     environ={'ASTRO_DDS_ADDRESS': '192.168.0.152'},
                     addresses=lambda names: {'end0': '192.168.0.152'}, pinger=lambda peers: None,
                     targets={}, env_file=LAN, clock_targets={'orin': '192.168.0.3'},
                     clocks=lambda peers, period: Peers(peers, period, query=lambda address: 0.08))
    proc(tmp_path)
    net.poll()
    assert started == [{'orin': '192.168.0.3'}]
    assert net.clock_facts['clock.spread'].value == pytest.approx(0.08)
