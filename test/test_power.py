import math
from types import SimpleNamespace

from mhseals_testing.analysis import power as p


def msg(volt, amp=0.0, pct=math.nan, cells=()):
    return SimpleNamespace(voltage=volt, current=amp, percentage=pct, cell_voltage=list(cells),
                           temperature=math.nan)


def feed(pw, volt, amp, t0, seconds, **kw):
    out = None
    for i in range(int(seconds)):
        out = pw.update(p.from_mavros(msg(volt, amp, **kw), t0 + i))
    return out


def test_absent_and_nan():
    assert p.from_mavros(msg(0.4), 0) is None
    assert p.from_mavros(msg(math.nan), 0) is None
    s = p.from_mavros(msg(15.8, math.nan), 0)
    assert s['amp'] is None and s['pct'] is None


def test_detect_chemistries():
    assert p.detect_cells(15.8, 'lipo') == 4
    assert p.detect_cells(10.0, 'lifepo4') == 3
    assert p.detect_cells(0.9) is None


def test_4s_lipo_sag_and_pct():
    pw = p.Power()
    f = feed(pw, 15.8, 0.1, 0, 12)
    assert f['cells'] == 4 and f['sag_mV'] == 0
    f = feed(pw, 14.8, 12.0, 12, 3)
    assert f['cells'] == 4 and f['cells_match'] is None
    assert math.isclose(f['sag_mV'], 250, abs_tol=1) and f['loaded']
    assert f['pct_estimated'] and f['watts'] == 14.8 * 12.0


def test_3s_lifepo4_configured():
    pw = p.Power({'chemistry': 'lifepo4', 'cells': 3, 'capacity_mah': 5000})
    f = feed(pw, 9.9, 0.2, 0, 3)
    assert math.isclose(f['cell_min'], 3.3) and f['cells_match'] is True
    assert p.estimate_pct(3.65, 'lifepo4') == 100 and p.estimate_pct(2.7, 'lifepo4') == 0
    f = feed(pw, 9.6, 10.0, 3, 2)
    assert f['remaining_min'] is not None and f['remaining_min'] < 30


def test_no_rest_window_no_sag_and_mismatch():
    pw = p.Power({'cells': 3})
    f = feed(pw, 14.8, 8.0, 0, 15)
    assert f['sag_mV'] is None and f['cells_detected'] is None
    f = feed(pw, 15.8, 0.1, 100, 2)
    assert f['cells'] == 3


def test_pct_is_lower_of_reported_and_estimate():
    pw = p.Power()
    f = feed(pw, 15.8, 0.1, 0, 2, pct=0.2)
    assert f['pct'] == 20.0
    f = feed(pw, 13.2, 0.1, 0, 2, pct=0.9)
    assert f['pct'] < 90


def test_cell_voltage_reported():
    pw = p.Power()
    f = feed(pw, 15.8, 0.1, 0, 2, cells=[3.9, 3.95, 3.95, 3.95])
    assert f['cell_min'] == 3.9


def test_mah_integrates_and_pack_swap_resets():
    pw = p.Power({'capacity_mah': p.DEFAULT_CAPACITY})
    assert pw.capacity is None                       # 3300 means unknown
    f = feed(pw, 15.0, 3.6, 0, 11)
    assert math.isclose(f['mah_used'], 10.0, abs_tol=0.1)
    assert f['remaining_min'] is None
    f = feed(pw, 15.0, 0.1, 20, 1)                   # rest after use
    assert f['mah_used'] > 9
    f = feed(pw, 16.6, 0.1, 21, 1)                   # higher rest: a fuller pack
    assert f['mah_used'] == 0
    f = feed(pw, 16.6, 0.1, 22, 11)
    assert f['sag_mV'] == 0


def test_long_run_then_release_keeps_mah():
    pw = p.Power()
    feed(pw, 16.4, 0.1, 0, 12)
    feed(pw, 16.0, 20.0, 12, 300)
    feed(pw, 15.2, 20.0, 312, 300)
    f = pw.update(p.from_mavros(msg(15.4, 0.2), 612))
    assert f['mah_used'] > 3000


def test_start_under_load_then_release_keeps_mah():
    pw = p.Power()
    feed(pw, 15.0, 20.0, 0, 60)
    f = pw.update(p.from_mavros(msg(15.8, 0.2), 60))
    assert f['mah_used'] > 300


def test_release_after_load_is_not_a_swap():
    pw = p.Power()
    feed(pw, 16.0, 0.1, 0, 12)                       # rest window closes
    f = feed(pw, 15.0, 20.0, 12, 60)                 # loaded run: sags 1 V
    assert f['mah_used'] > 300 and f['sag_mV'] > 200
    f = pw.update(p.from_mavros(msg(15.8, 0.2), 72))  # throttle released
    assert f['mah_used'] > 300
    assert f['sag_mV'] is not None and pw.baseline is not None


def test_swaps_are_counted_and_survive_the_reset():
    pw = p.Power({'cells': 4})
    s = lambda t, v, a: p.from_mavros(SimpleNamespace(voltage=v, current=a, percentage=float('nan')), t)
    assert pw.update(s(0, 15.0, 0.1))['swaps'] == 0
    assert pw.update(s(5, 15.9, 0.1))['swaps'] == 1


def test_fcu_unknown_values_are_absent():
    """65.535 V (UINT16_MAX) is no voltage and a negative percentage is unknown, as in the row."""
    assert p.from_mavros(msg(65.535, 1.0, pct=-0.01), 0) is None
    assert p.from_mavros(msg(65.5), 0) is None
    s = p.from_mavros(msg(15.8, 1.0, pct=-0.01), 0)
    assert s['volt'] == 15.8 and s['pct'] is None
    assert p.from_mavros(msg(15.8, 1.0, pct=0.5), 0)['pct'] == 50.0
