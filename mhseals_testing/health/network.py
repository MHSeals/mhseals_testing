"""The Status `Network` row: boat-link facts judged by config/health/network.yaml.

The boat subnet is ``BOAT_SUBNET`` in the workspace's config/boat-network.env (without it there is
no boat link to watch and the row stays hidden). The hosts pinged are those of the device
registry's ``ping:`` roles, never this machine (mhseals_testing.devices); the worst host's loss and
round trip are judged, and the row names that host.
"""

from mhseals_nav import boat_network

from mhseals_testing import devices
from mhseals_testing.health import clock_sync, config
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.netstat import Link
from mhseals_testing.health.pinger import Pinger

GROUP = 'Network & DDS'
LEVELS = {'ok': 'OK', 'info': 'OK', 'unknown': 'N/A', 'warn': 'WARN', 'fail': 'FAIL'}
PERIOD = 5.0                 # seconds between /proc reads
PEER_FACTS = ('net.loss_pct', 'net.rtt_ms')   # the worst pinged host's, labelled with its name


def ping_targets(env_file=None):
    """{host: address} of the registry's ping roles, never this machine; {} if the registry is refused."""
    try:
        registry = devices.load(env=env_file)
        return devices.ping_targets(registry, devices.whoami(registry, devices.local_addresses()))
    except devices.DeviceError:
        return {}


def worst(facts):
    """``facts`` plus ``net.loss_pct`` and ``net.rtt_ms``: the worst host's, labelled with it."""
    out = dict(facts)
    for name in PEER_FACTS:
        found = [fact for key, fact in facts.items() if key.startswith(name + '.')]
        if found:
            out[name] = max(found, key=lambda fact: fact.value)
    return out


class NetworkRow:
    """``poll()`` returns (level, detail) for ``monitor.advise('network', ...)``, or None when hidden.

    Hidden while no interface has held an address in the subnet (laptop, simulation); once one has,
    losing it reads FAIL so the row cannot keep its last OK.
    """

    def __init__(self, directory=None, user=None, root='/', clock=None, env_file=None,
                 targets=None, pinger=Pinger, clock_targets=None, clocks=clock_sync.ClockPeers, **link):
        loaded = config.load(directory, user=user if user is not None else False)
        self.specs = [s for s in loaded.specs if s.group == GROUP]
        settings = loaded.probes.get('network', {})
        kwargs = {'clock': clock} if clock else {}
        subnet = boat_network.values(env_file or devices.layer_paths()[3]).get('BOAT_SUBNET')
        self.link = Link(subnet, root=root, **kwargs, **link) if subnet else None
        self.engine = Engine(self.specs, **kwargs)
        self.period = settings.get('period', PERIOD)
        targets = ping_targets(env_file) if targets is None else targets
        self.pinger = pinger(targets) if targets and self.link else None
        sync = loaded.probes.get('clock_sync', {})     # config/health/preflight.yaml: pre.clock_sync
        if clock_targets is None and self.link:
            clock_targets = clock_sync.targets(tuple(sync.get('roles', clock_sync.ROLES)), env_file)
        self.clocks = (clocks(clock_targets, sync.get('period', clock_sync.PERIOD))
                       if clock_targets and self.link else None)
        self.clock_facts = {}        # clock.spread for the App's health facts
        self.pinging = False         # started with the first boat-link sample, never off the boat
        self.next = None
        self.facts = {}
        self.shown = None
        self.seen = False            # a boat link has been shown; losing it must not go quiet

    def poll(self):
        if self.link is None:
            return None
        now = self.link.clock()
        if self.next is not None and now < self.next:
            return self.shown
        self.next = now + self.period
        self.facts = self.link.sample()
        if self.facts and not self.pinging:
            for sampler in (self.pinger, self.clocks):
                if sampler:
                    sampler.start()
            self.pinging = True
        if self.facts and self.pinger:
            self.facts.update(worst(self.pinger.facts()))
        self.clock_facts = self.clocks.facts() if self.facts and self.clocks else {}
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
            host = (r.data or {}).get('label') if i.startswith('net.peer.') else None
            return LEVELS[r.status], f'{r.value}; {titles[i]}' + (f' {host}' if host else '')
        rx, tx = self.facts.get('net.rx_bps'), self.facts.get('net.tx_bps')
        iface = next(iter(self.facts.values())).label
        if rx is None or tx is None:
            return 'OK', f'{iface}; waiting for a second sample'
        return 'OK', f'{(rx.value + tx.value) / 1e6:.1f} Mb/s; {iface} link up'
