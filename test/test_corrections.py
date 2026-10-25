"""Source priority (local base vs NTRIP) and the gate that lets only the active source reach the FCU."""

import threading

import pytest

from mhseals_testing.corrections import Corrections, choose


@pytest.mark.parametrize('priority,base_ok,fixed,ntrip_ok,km,expected', [
    ('auto', True, False, True, 8.0, 'ntrip'),        # near station, base only surveying: NTRIP
    ('auto', True, True, True, 8.0, 'base'),          # base fixed at a surveyed site: base
    ('auto', True, False, True, 22.0, 'base'),        # station too far: base
    ('auto', True, False, True, None, 'base'),
    ('auto', False, False, True, 40.0, 'ntrip'),      # no base: NTRIP at any distance
    ('auto', True, False, False, None, 'base'),       # no internet or no account: base
    ('auto', False, False, False, None, None),
    ('base', True, False, True, 2.0, 'base'),
    ('ntrip', True, True, True, 25.0, 'ntrip'),
    ('ntrip', True, True, False, None, 'base'),
])
def test_choose(priority, base_ok, fixed, ntrip_ok, km, expected):
    assert choose(priority, base_ok, fixed, ntrip_ok, km) == expected


class FakeBase:
    def __init__(self, phase='surveying'):
        self.phase = phase

    def snapshot(self):
        return dict(phase=self.phase, detail='', received=0, delivered=0, errors=0, model='F9P', survey=None,
                    base_position=None, base_satellites=None, site='dock' if self.phase == 'fixed' else None,
                    rough=dict(latitude=27.37, longitude=-82.45, height=3.0, h_acc=1.2), routes=[])


class FakeNtrip:
    def __init__(self, km=8.0):
        self.km, self.wanted = km, threading.Event()

    @property
    def available(self):
        return self.km is not None

    def best_km(self):
        return self.km

    def snapshot(self):
        return dict(phase='streaming' if self.wanted.is_set() else 'standby', detail='C/TAMP', mount='TAMP',
                    caster='C', km=self.km, vrs=False, station=dict(latitude=27.4, longitude=-82.5), candidates=[],
                    failovers=0, last_error='', received=3, last=None, gga_sent=0)


class Rough:
    def __init__(self):
        self.offers = []

    def offer(self, name, lat, lon, alt):
        self.offers.append((name, lat, lon, alt))


def make(base=None, net=None, priority='auto'):
    now, out = [0.0], []
    mux = Corrections([lambda f, t: out.append(f)], priority, switch_after=30, dead_after=5, clock=lambda: now[0])
    rough = Rough()
    mux.attach(base, net, rough)
    return mux, now, out, rough


def test_only_the_active_source_reaches_the_sinks_and_ntrip_is_wanted_only_when_chosen():
    base, net = FakeBase('fixed'), FakeNtrip(8.0)
    mux, now, out, rough = make(base, net)
    assert mux.evaluate() == 'base' and not net.wanted.is_set()
    mux.gate('ntrip')(b'N', 0.0)
    mux.gate('base')(b'B', 0.0)
    assert out == [b'B']
    assert rough.offers[0][0] == 'base'          # the base's own fix feeds the NTRIP picker
    state = mux.snapshot()
    assert state['source']['kind'] == 'base' and state['source']['site'] == 'dock'


def test_a_dead_active_source_switches_at_once_a_live_one_only_after_the_hold():
    base, net = FakeBase('ready'), FakeNtrip(30.0)
    mux, now, out, _ = make(base, net)
    mux.gate('base')(b'B', 0.0)
    assert mux.active == 'base'
    net.km = 5.0                    # the boat came near a station: NTRIP is now preferred
    mux.evaluate()
    assert net.wanted.is_set() and mux.active == 'base'
    for t in range(1, 40):
        now[0] = float(t)
        mux.gate('base')(b'B', now[0])
        mux.gate('ntrip')(b'N', now[0])
        mux.evaluate()
        if mux.active == 'ntrip':
            break
    assert mux.active == 'ntrip' and now[0] >= 30          # held 30 s while both streamed
    assert mux.switches == 1
    # NTRIP dies (internet lost) while the base still streams: back to the base at once
    net.km = None
    for t in range(int(now[0]) + 1, int(now[0]) + 8):
        now[0] = float(t)
        mux.gate('base')(b'B', now[0])
        mux.evaluate()
    assert mux.active == 'base' and not net.wanted.is_set()


def test_ntrip_snapshot_reports_the_mountpoint_in_the_base_session_shape():
    mux, now, out, _ = make(None, FakeNtrip(8.0))
    assert mux.evaluate() == 'ntrip'
    state = mux.snapshot()
    assert state['model'] == 'NTRIP' and state['phase'] == 'ready' and state['survey'] is None
    assert state['source']['ntrip']['mount'] == 'TAMP' and state['source']['kind'] == 'ntrip'


def test_a_failing_sink_does_not_stop_the_others():
    def broken(frame, stamp):
        raise OSError('radio gone')
    got = []
    mux = Corrections([broken, lambda f, t: got.append(f)], 'base', clock=lambda: 0.0)
    mux.attach(FakeBase('ready'))
    mux.gate('base')(b'B', 0.0)
    assert got == [b'B'] and mux.routes[0]['failed'] == 1 and mux.routes[1]['sent'] == 1
