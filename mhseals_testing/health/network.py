"""The Status `Network` row: boat-link facts judged by config/health/network.yaml.

``peers`` there names hosts to ping (the camera host); each name's address comes from the
DHCP reservations in the workspace's config/boat-network.env, else from the resolver.
"""

import shlex

from mhseals_testing import data
from mhseals_testing.health import config
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.netstat import Link
from mhseals_testing.health.pinger import Pinger

GROUP = 'Network & DDS'
LEVELS = {'ok': 'OK', 'info': 'OK', 'unknown': 'N/A', 'warn': 'WARN', 'fail': 'FAIL'}
PERIOD = 5.0                 # seconds between /proc reads


def lease_addresses(path):
    """{hostname: address} from STATIC_LEASES (``MAC=hostname=address ...``); {} if unreadable."""
    try:
        text = path.read_text()
    except OSError:
        return {}
    for line in text.splitlines():
        name, _, value = line.partition('=')
        if name.strip() == 'STATIC_LEASES':
            words = shlex.split(value, comments=True)
            leases = [word.split('=') for word in (words[0].split() if words else ())]
            return {lease[1]: lease[2] for lease in leases if len(lease) == 3}
    return {}


def peer_addresses(peers, lan_file=None):
    """{peer: address} for the pinger; a host with no reservation is left for the resolver."""
    leases = lease_addresses(lan_file or data.workspace() / 'config' / 'boat-network.env')
    return {name: leases.get(host, host) for name, host in peers.items()}


class NetworkRow:
    """``poll()`` returns (level, detail) for ``monitor.advise('network', ...)``, or None when hidden.

    Hidden while no interface has held an address in the subnet (laptop, simulation); once one has,
    losing it reads FAIL so the row cannot keep its last OK.
    """

    def __init__(self, directory=None, user=None, root='/', clock=None, lan_file=None, pinger=Pinger, **link):
        loaded = config.load(directory, user=user if user is not None else False)
        self.specs = [s for s in loaded.specs if s.group == GROUP]
        settings = loaded.probes.get('network', {})
        kwargs = {'clock': clock} if clock else {}
        self.link = Link(settings.get('subnet', '192.168.0.0/24'), root=root, **kwargs, **link)
        self.engine = Engine(self.specs, **kwargs)
        self.period = settings.get('period', PERIOD)
        peers = settings.get('peers') or {}
        self.pinger = pinger(peer_addresses(peers, lan_file)) if peers else None
        self.pinging = False         # started with the first boat-link sample, never off the boat
        self.next = None
        self.facts = {}
        self.shown = None
        self.seen = False            # a boat link has been shown; losing it must not go quiet

    def poll(self):
        now = self.link.clock()
        if self.next is not None and now < self.next:
            return self.shown
        self.next = now + self.period
        self.facts = self.link.sample()
        if self.facts and self.pinger:
            if not self.pinging:
                self.pinger.start()
                self.pinging = True
            self.facts.update(self.pinger.facts())
        if self.facts:
            self.seen, self.shown = True, self.render()
        elif self.seen and self.link.known == set():
            self.shown = ('FAIL', f'no {self.link.net} address')
        else:
            self.shown = None
        return self.shown

    def render(self):
        report = self.engine.tick(self.facts, {})
        titles = {s.id: s.title for s in self.specs}
        bad = [(i, r) for i, r in report.results.items() if r.status in ('warn', 'fail')]
        if bad:
            i, r = max(bad, key=lambda b: (b[1].status == 'fail',))
            return LEVELS[r.status], f'{r.value}; {titles[i]}'
        rx, tx = self.facts.get('net.rx_bps'), self.facts.get('net.tx_bps')
        iface = next(iter(self.facts.values())).label
        if rx is None or tx is None:
            return 'OK', f'{iface}; waiting for a second sample'
        return 'OK', f'{(rx.value + tx.value) / 1e6:.1f} Mb/s; {iface} link up'
