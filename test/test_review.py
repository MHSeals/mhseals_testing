import pytest

from mhseals_testing.ui.panels import scrubber
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
    return [t.plain for t in scrubber.review_footer(r, width, **kw)]


@pytest.mark.parametrize('width', [120, 80, 48])
def test_strip_footer_marks_the_loupe_playhead_and_status(width):
    events_row, skyline, axis, status = lines(width, notice='hello')
    assert {len(row) for row in (events_row, skyline, axis, status)} == {width}
    assert axis.count('\u251c') == axis.count('\u2524') == 1 and axis.count('\u25cf') == 1
    assert '0:40 / 1:40' in status and '\u25be1/2 wobble' in status
    assert status.endswith('loupe 12m') or status.rstrip().endswith('loupe 12m')
    assert width < 80 or 'hello' in status


def test_loupe_draws_the_playhead_and_the_window_end():
    from rich.console import Console
    r = review()
    r.go(140, 0)
    console = Console(width=60, record=True, color_system=None)
    console.print(scrubber.Loupe(r))
    out = console.export_text().splitlines()
    assert sum(row.count('\u25cf') for row in out) == 1 and '\u25cf' in out[3]
    assert 'now' not in out[-1] and out[-1].strip().endswith(('1:40', '1:30'))


def test_session_length_is_m_ss_under_an_hour_and_h_mm_after():
    t = scrubber.status(0, 0, 3 * 3600 + 12 * 60, 720.0).plain
    assert '0 / 3h12' in t
    assert '/ 38:02' in scrubber.status(0, 0, 38 * 60 + 2, 720.0).plain


def test_status_names_the_procedure_under_the_playhead():
    r = review()
    r.go(120, 0)
    assert 'characterize PASS' in scrubber.review_footer(r, 120)[-1].plain
    r.go(190, 0)
    assert 'characterize' not in scrubber.review_footer(r, 120)[-1].plain


def test_session_host_comes_from_the_journal(tmp_path):
    import json
    from mhseals_testing.ui.review import load_session
    lines_ = [dict(e, host='odroid') if e['phase'] == 'session_start' else e for e in events()]
    (tmp_path / 'events.jsonl').write_text('\n'.join(json.dumps(e) for e in lines_) + '\n')
    assert load_session(tmp_path).host == 'odroid'
    (tmp_path / 'session.json').write_text('{"host": "blastoise"}')
    assert load_session(tmp_path).host == 'blastoise'


def test_flat_session_journal_beside_the_folder_gives_markers(tmp_path):
    import json
    from mhseals_testing.ui.review import load_session
    session = tmp_path / 'crane_full_stack_20261023'
    session.mkdir()
    (tmp_path / (session.name + '.events.jsonl')).write_text(
        '\n'.join(json.dumps(e) for e in events()) + '\n')
    review = load_session(session)
    assert [m[1] for m in review.markers] == ['wobble', ''] and review.windows


def test_report_events_keep_their_phase_when_there_is_no_journal(tmp_path):
    import json
    from mhseals_testing.ui.review import load_session
    session = tmp_path / 'crane_full_stack_20261023'
    session.mkdir()
    (tmp_path / (session.name + '.report.json')).write_text(json.dumps({'events': events()}))
    assert [m[1] for m in load_session(session).markers] == ['wobble', '']


def test_first_rec_start_is_no_window_and_marker_note_edits_fold():
    def at(t, phase, **details):
        return dict(t_ns=int(t * 1e9), phase=phase, **details)
    evs = [at(100, 'session_start'), at(101, 'rec_start'), at(110, 'marker', id=1, note=''),
           at(120, 'marker_note', id=1, note='wake'), at(200, 'session_stop')]
    _, _, markers, windows, gaps, _ = timeline(evs)
    assert windows == [] and gaps == []
    assert markers == [(110.0, 'wake', 'user')]
