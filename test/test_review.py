import pytest

from mhseals_testing.ui import scrubber
from mhseals_testing.ui.review import BIG, DEBOUNCE, Review, timeline


def events():
    def at(t, phase, **details):
        return dict(t_ns=int(t * 1e9), phase=phase, **details)
    return [at(100, 'session_start'), at(110, 'characterize_start'),
            at(130, 'characterize_stop', outcome='pass'), at(140, 'marker', note='wobble'),
            at(150, 'rec_stop'), at(160, 'rec_start'), at(170, 'marker', kind='system'),
            at(180, 'health', state='fail'), at(200, 'session_stop')]


def review(loaded=None):
    return Review(*timeline(events()), load=loaded)


def test_timeline_folds_windows_gaps_markers_and_fails():
    t0, t1, markers, windows, gaps, fails = timeline(events())
    assert (t0, t1) == (100, 200)
    assert [m[:2] for m in markers] == [(140, 'wobble'), (170, '')]
    assert windows == [(110, 130, 'PASS', 'characterize')] and gaps == [(150, 160)]
    assert fails == [180]


def test_seeks_clamp_and_the_window_loads_once_after_the_debounce():
    seen = []
    r = review(seen.append)
    for i in range(5):                       # a held key: only the playhead moves
        r.seek(BIG, now=i * 0.05)
        r.tick(i * 0.05, 0.05)
    assert seen == [] and r.playhead == 200
    r.tick(0.2 + DEBOUNCE + 0.01, 0.0)
    r.tick(5.0, 0.0)
    assert seen == [200]                     # exactly one load, at the rest position
    r.go(0, now=6.0)
    assert r.playhead == 100


def test_jump_walks_markers_in_order_and_play_does_not_reseek():
    seen = []
    r = review(seen.append)
    r.jump(1, 0); assert r.playhead == 140
    r.jump(1, 0); assert r.playhead == 170
    r.jump(1, 0); assert r.playhead == 170   # none beyond
    r.jump(-1, 0); assert r.playhead == 140
    r.tick(1.0, 0); seen.clear()
    r.toggle(); r.faster(); r.tick(2.0, 1.0)
    assert r.playhead == pytest.approx(142) and seen == []
    r.tick(3.0, 100.0)
    assert r.playhead == 200 and not r.playing


def lines(width, **kw):
    r = review()
    r.go(140, 0)
    return [t.plain for t in scrubber.rows(r, width, **kw)]


@pytest.mark.parametrize('width', [120, 80, 48])
def test_scrubber_rows_fit_and_mark_the_playhead_windows_and_gaps(width):
    *top, status = lines(width)
    ticks, track = top if width >= scrubber.NARROW else (None, top[0])
    assert len(top) == scrubber.height(width) - 1 and all(len(row) <= width for row in top + [status])
    assert len(track) == width
    assert track.count(scrubber.HEAD) == 1 and scrubber.WINDOW in track and scrubber.GAP in track
    if ticks:
        assert ticks.count(scrubber.MARK) == 2 and scrubber.FAIL in ticks
    else:
        assert scrubber.MARK in track          # ticks fold into the track
    assert '0:40 / 1:40' in status and '1/2 wobble' in status


def test_status_names_the_procedure_under_the_playhead():
    r = review()
    r.go(120, 0)
    assert 'characterize PASS' in scrubber.rows(r, 120)[-1].plain
    r.go(190, 0)
    assert 'characterize' not in scrubber.rows(r, 120)[-1].plain


def test_session_host_comes_from_the_journal(tmp_path):
    import json
    from mhseals_testing.ui.review import load_session
    lines_ = [dict(e, host='odroid') if e['phase'] == 'session_start' else e for e in events()]
    (tmp_path / 'events.jsonl').write_text('\n'.join(json.dumps(e) for e in lines_) + '\n')
    assert load_session(tmp_path).host == 'odroid'
    (tmp_path / 'session.json').write_text('{"host": "blastoise"}')
    assert load_session(tmp_path).host == 'blastoise'
