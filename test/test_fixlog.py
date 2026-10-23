from mhseals_testing.analysis.fixlog import Fixlog, state_of
from mhseals_testing.ui.rtk_pane import key_action, rtk_pane


def ev(t, fix):
    return dict(phase='gps_fix', t=t, fix=fix)


EVENTS = [ev(0, 'none'), ev(10, '3D'), ev(40, 'FLOAT'), ev(100, 'FIXED'), ev(130, 'FLOAT'),
          ev(160, 'FIXED'), dict(phase='other', t=5, fix='FIXED')]


def test_totals_use_the_fake_clock():
    log = Fixlog(EVENTS, now=200)
    assert log.seconds == dict(none=10, **{'3D': 30}, DGPS=0, FLOAT=90, FIXED=70)
    assert log.entries['FIXED'] == 2 and log.entries['FLOAT'] == 2
    assert log.first_fixed == 100 and log.current == 'FIXED'
    assert Fixlog(EVENTS, now=300).seconds['FIXED'] == 170  # open state keeps growing


def test_bins_and_fix_type_numbers():
    assert Fixlog(EVENTS, now=200).bins(4) == ['3D', 'FLOAT', 'FIXED', 'FIXED']
    assert state_of(dict(fix_type=5)) == 'FLOAT' and state_of(dict(fix_type=6)) == 'FIXED'


def test_empty_and_repeats():
    log = Fixlog([ev(0, 'FIXED'), ev(5, 'FIXED')], now=10)
    assert log.entries['FIXED'] == 1 and log.first_fixed == 0
    assert Fixlog([], now=3).current is None


def test_pane_render():
    lines = rtk_pane(EVENTS, 200, width=20, ascii=True)
    assert len(lines[0]) == 20 and lines[0][-1] == '#'
    assert 'FIXED    1:10  x2' in lines and 'first FIXED 1:40' in lines
    assert lines[-1] == 'r  base controls'
    assert rtk_pane([], 0)[0] == 'no fix events yet'
    assert key_action('r') == 'base_controls' and key_action('x') is None
