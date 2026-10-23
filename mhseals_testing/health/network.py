"""The Status `Network` row: boat-link facts judged by config/health/network.yaml."""

from mhseals_testing.health import config
from mhseals_testing.health.engine import Engine
from mhseals_testing.health.netstat import Link

GROUP = 'Network & DDS'
LEVELS = {'ok': 'OK', 'info': 'OK', 'unknown': 'N/A', 'warn': 'WARN', 'fail': 'FAIL'}
PERIOD = 5.0                 # seconds between /proc reads


class NetworkRow:
    """``poll()`` returns (level, detail) for ``monitor.advise('network', ...)``, or None when hidden.

    Hidden while no interface holds an address in the subnet (laptop, simulation).
    """

    def __init__(self, directory=None, user=None, root='/', clock=None, **link):
        loaded = config.load(directory, user=user if user is not None else False)
        self.specs = [s for s in loaded.specs if s.group == GROUP]
        settings = loaded.probes.get('network', {})
        kwargs = {'clock': clock} if clock else {}
        self.link = Link(settings.get('subnet', '192.168.0.0/24'), root=root, **kwargs, **link)
        self.engine = Engine(self.specs, **kwargs)
        self.period = settings.get('period', PERIOD)
        self.next = None
        self.facts = {}
        self.shown = None

    def poll(self):
        now = self.link.clock()
        if self.next is not None and now < self.next:
            return self.shown
        self.next = now + self.period
        self.facts = self.link.sample()
        self.shown = self.render() if self.facts else None
        return self.shown

    def render(self):
        report = self.engine.tick(self.facts, {})
        titles = {s.id: s.title for s in self.specs}
        bad = [(i, r) for i, r in report.results.items() if r.status in ('warn', 'fail')]
        if bad:
            i, r = max(bad, key=lambda b: (b[1].status == 'fail',))
            return LEVELS[r.status], f'{r.value}; {titles[i]} {r.detail}'
        rx, tx = self.facts.get('net.rx_bps'), self.facts.get('net.tx_bps')
        iface = next(iter(self.facts.values())).label
        if rx is None or tx is None:
            return 'OK', f'{iface}; waiting for a second sample'
        return 'OK', f'{(rx.value + tx.value) / 1e6:.1f} Mb/s; {iface} link up'
