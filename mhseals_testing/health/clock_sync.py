"""Clock agreement of the boat computers, by SNTP from this one (the preflight check pre.clock_sync).

No stamp check can see a delay equal to a clock offset (they cancel), so the hosts' clocks are
compared directly: one SNTP query per host, the method of astro_dock's .devcontainer/astro-clock.py.
chrony answers it on hosts set up by ``setup.linux.sh --components clock``; a host that does not
answer (timesyncd serves no time) leaves the spread unknown, and preflight does not pass. The hosts
are those of the probe's ``roles`` (config/health/preflight.yaml), never this machine, whose own
clock enters the spread as 0, so an operator's TUI holds the boat to the operator's clock as well.
One daemon thread; ``facts()`` only copies under a lock.
"""

import socket
import struct
import threading
import time

from mhseals_testing import devices
from mhseals_testing.health.model import Fact

NTP_EPOCH = 2208988800      # seconds from 1900 to 1970
TIMEOUT = 1.0
PERIOD = 5.0
ROLES = ('boat_computer', 'camera_host')


def ntp_time(raw):
    seconds, fraction = struct.unpack('!II', raw)
    return seconds - NTP_EPOCH + fraction / 2 ** 32


def offset(address, timeout=TIMEOUT, port=123, now=time.time):
    """``address``'s clock minus this one's, in s; None without a synchronised answer."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout)
            t1 = now()
            sock.sendto(b'\x23' + bytes(39) + struct.pack('!II', int(t1 + NTP_EPOCH), int(t1 % 1 * 2 ** 32)),
                        (address, port))
            data = sock.recv(48)
            t4 = now()
    except OSError:
        return None
    if len(data) < 48 or data[0] >> 6 == 3 or not 0 < data[1] < 16:     # leap 3 or stratum 0/16: unsynchronised
        return None
    return ((ntp_time(data[32:40]) - t1) + (ntp_time(data[40:48]) - t4)) / 2


def targets(roles=ROLES, env_file=None):
    """{host: address} of the registry's ``roles``, never this machine; {} if the registry is refused."""
    try:
        registry = devices.load(env=env_file)
        me = devices.whoami(registry, devices.local_addresses())
    except devices.DeviceError:
        return {}
    return {host.name: host.address for role in roles for host in registry.hosts_for(role)
            if host.name not in (me.name, devices.FALLBACK) and host.address}


def spread(offsets):
    """``clock.spread`` (s, the largest gap among the hosts and this machine) once every host answered."""
    if not offsets or None in offsets.values():
        return {}
    readings = {'this computer': 0.0, **offsets}
    early, late = min(readings, key=readings.get), max(readings, key=readings.get)
    return {'clock.spread': Fact(readings[late] - readings[early], label=f'{late} vs {early}')}


class ClockPeers:
    """Queries each host once per ``period`` from a daemon thread."""

    def __init__(self, peers, period=PERIOD, query=offset):
        self.peers, self.period, self.query = dict(peers), period, query
        self.offsets = {}
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

    def start(self):
        threading.Thread(target=self.run, name='clock_sync', daemon=True).start()

    def stop(self):
        self.stop_event.set()

    def run(self):
        while not self.stop_event.is_set():
            self.round()
            self.stop_event.wait(self.period)

    def round(self):
        found = {name: self.query(address) for name, address in self.peers.items()}
        with self.lock:
            self.offsets = found

    def facts(self):
        with self.lock:
            return spread(self.offsets)
