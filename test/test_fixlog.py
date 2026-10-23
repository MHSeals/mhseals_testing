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
    assert Fixlog(EVENTS, now=200).bins(4) == ['none', 'FLOAT', 'FLOAT', 'FLOAT']  # worst per bin
    assert state_of(dict(fix_type=5)) == 'FLOAT' and state_of(dict(fix_type=6)) == 'FIXED'


def test_empty_and_repeats():
    log = Fixlog([ev(0, 'FIXED'), ev(5, 'FIXED')], now=10)
    assert log.entries['FIXED'] == 1 and log.first_fixed == 0
    assert Fixlog([], now=3).current is None


def test_pane_render():
    lines = rtk_pane(EVENTS, 200, width=20, ascii=True)
    assert len(lines[0]) == 20 and lines[0][-1] == '#'
    fancy = rtk_pane([ev(5, 'none')], 10, width=4)[0]
    assert fancy == '▁' * 4 and rtk_pane([ev(5, 'none')], 10, width=4, start=0)[0][0] == '·'
    assert rtk_pane([ev(5, 'none')], 10, 4, ascii=True, start=0)[0][0] == '.'
    assert 'FIXED    1:10  x2' in lines and 'first FIXED 1:40' in lines
    assert lines[-1] == 'r  base controls'
    assert rtk_pane([], 0)[0] == 'no fix events yet'
    assert key_action('r') == 'base_controls' and key_action('x') is None


def test_short_dip_shows_in_its_bin():
    log = Fixlog([ev(0, 'FIXED'), ev(50, 'FLOAT'), ev(52, 'FIXED')], now=100)
    assert log.bins(4) == ['FIXED', 'FIXED', 'FLOAT', 'FIXED']


def test_explicit_start_clips_entries_and_first_fixed():
    log = Fixlog([ev(0, 'FIXED'), ev(20, 'FLOAT'), ev(30, 'FIXED')], now=40, start=10)
    assert log.entries['FIXED'] == 1 and log.entries['FLOAT'] == 1
    assert log.seconds['FIXED'] == 20 and log.first_fixed == 20
    assert Fixlog([ev(0, 'FIXED')], now=40, start=10).first_fixed is None


def test_2d_is_not_a_3d_fix():
    assert state_of(dict(fix_type=2)) == 'none' and state_of(dict(fix='2D')) == 'none'
    assert state_of(dict(fix_type=3)) == '3D'
