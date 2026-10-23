"""Boat-link facts from /proc and /sys: throughput, errors, UDP drops, address changes.

Stdlib only, no root and no subprocess. ``Link.sample()`` returns ``{name: Fact}``
for the checks in ``config/health/network.yaml``. A rate needs two samples, and a
counter that went backwards (interface bounce, reboot, `nstat -r`) leaves its fact
out for that interval rather than reporting a negative or huge rate.
"""

from collections import deque
import fcntl
import ipaddress
import os
from pathlib import Path
import socket
import struct
import time

from mhseals_testing.health.model import Fact

SIOCGIFADDR = 0x8915
IGNORED = ('lo', 'docker', 'veth', 'br-')        # name prefixes that are never the boat link
EVENT_WINDOW = 300.0                             # seconds of address changes net.addr_events_5m counts
DEV_COLUMNS = ('rx_bytes', 'rx_packets', 'rx_errs', 'rx_drop', 'rx_fifo', 'rx_frame', 'rx_compressed',
               'rx_multicast', 'tx_bytes', 'tx_packets', 'tx_errs', 'tx_drop')


def parse_dev(text):
    """{interface: {column: counter}} from /proc/net/dev."""
    out = {}
    for line in text.splitlines()[2:]:
        name, _, rest = line.partition(':')
        values = rest.split()
        if len(values) >= len(DEV_COLUMNS):
            out[name.strip()] = dict(zip(DEV_COLUMNS, map(int, values)))
    return out


def parse_snmp(text):
    """{protocol: {counter: value}} from /proc/net/snmp, matched by header name.

    Columns differ between kernels (the boat's has Udp MemErrors, others do not),
    so values are never read by position.
    """
    lines, out = text.splitlines(), {}
    for head, body in zip(lines[::2], lines[1::2]):
        proto, _, names = head.partition(':')
        if proto == body.partition(':')[0]:
            out[proto] = dict(zip(names.split(), map(int, body.partition(':')[2].split())))
    return out


def rates(old, new, dt):
    """Per-second change of each counter; None for a counter that went backwards."""
    return {k: (new[k] - old[k]) / dt if new[k] >= old[k] else None for k in new if k in old}


def interface_addresses(names):
    """{interface: primary IPv4} by SIOCGIFADDR (secondary addresses are not reported)."""
    out = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for name in names:
            try:
                packed = fcntl.ioctl(sock.fileno(), SIOCGIFADDR, struct.pack('256s', name.encode()[:15]))
                out[name] = socket.inet_ntoa(packed[20:24])
            except OSError:
                pass
    return out


class Link:
    """Samples the interface that holds an address in ``subnet``."""

    def __init__(self, subnet, root='/', environ=None, clock=time.monotonic, addresses=interface_addresses):
        self.net = ipaddress.ip_network(subnet)
        self.root = Path(root)
        self.environ = os.environ if environ is None else environ
        self.clock = clock
        self.addresses = addresses
        self.last = None                         # (time, dev counters, udp/ip counters)
        self.known = None                        # address set at the last sample; None before the first
        self.changes = deque()                   # times of address-set changes

    def read(self, path):
        return (self.root / path).read_text()

    def find(self, dev):
        """(interface, address) of the boat link, else (None, None)."""
        found = self.addresses([n for n in dev if not n.startswith(IGNORED)])
        for name, address in found.items():
            if ipaddress.ip_address(address) in self.net:
                return name, address
        return None, None

    def pinned(self):
        """The address the DDS profile was pinned to at shell start, if any."""
        value = self.environ.get('ASTRO_DDS_ADDRESS')
        if value:
            return value
        base = Path(self.environ.get('XDG_RUNTIME_DIR') or '/tmp')
        try:
            return (self.root / str(base).lstrip('/') / f'astro-dds-address-{os.getuid()}').read_text().strip() or None
        except OSError:
            return None

    def sample(self):
        """Facts now; ``{}`` when no interface holds a boat-subnet address or /proc is unreadable."""
        now = self.clock()
        try:
            dev = parse_dev(self.read('proc/net/dev'))
            snmp = parse_snmp(self.read('proc/net/snmp'))
        except (OSError, ValueError):
            return {}
        name, address = self.find(dev)
        have = {address} if address else set()
        if self.known is not None and have != self.known:
            self.changes.append(now)
        self.known = have
        while self.changes and now - self.changes[0] > EVENT_WINDOW:
            self.changes.popleft()
        if name is None:
            self.last = None
            return {}
        facts = {'net.addr_events_5m': Fact(len(self.changes), name, time=now)}
        pin = self.pinned()
        facts['net.addr_changed'] = Fact(float(bool(pin and pin != address)), name, {'pinned': pin, 'now': address}, now)
        counters = {'rx_bytes': dev[name]['rx_bytes'], 'tx_bytes': dev[name]['tx_bytes'],
                    'errs': dev[name]['rx_errs'] + dev[name]['tx_errs'] + dev[name]['rx_drop'] + dev[name]['tx_drop'],
                    'drops': sum(snmp.get('Udp', {}).get(k, 0) for k in ('RcvbufErrors', 'InErrors'))
                    + snmp.get('Ip', {}).get('ReasmFails', 0),
                    'noports': snmp.get('Udp', {}).get('NoPorts', 0)}
        previous, self.last = self.last, (now, name, counters)
        if previous is None or previous[1] != name or now <= previous[0]:
            return facts
        rate = rates(previous[2], counters, now - previous[0])
        speed = self.speed(name)
        for key, fact in (('errs', 'net.errs_ps'), ('drops', 'udp.drops_ps'), ('noports', 'udp.noports_ps')):
            if rate[key] is not None:
                facts[fact] = Fact(rate[key], name, time=now)
        if rate['rx_bytes'] is not None and rate['tx_bytes'] is not None:
            rx, tx = rate['rx_bytes'] * 8, rate['tx_bytes'] * 8
            facts['net.rx_bps'], facts['net.tx_bps'] = Fact(rx, name, time=now), Fact(tx, name, time=now)
            if speed:
                facts['net.bw_pct'] = Fact(100 * max(rx, tx) / (speed * 1e6), name, {'link_mbps': speed}, now)
        return facts

    def speed(self, name):
        """Link speed in Mb/s, or None (virtual NICs and down links report -1 or nothing)."""
        try:
            value = int(self.read(f'sys/class/net/{name}/speed'))
        except (OSError, ValueError):
            return None
        return value if value > 0 else None
