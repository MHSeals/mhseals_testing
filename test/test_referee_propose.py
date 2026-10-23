"""Course proposal from tracked objects: never invents, splits shared classes, reports shortfalls."""

from mhseals_testing.referee import propose


def buoy(label, x, y, score=0.9):
    return {'label': label, 'x': x, 'y': y, 'score': score}


def gate(y, x=0.0, w=2.0):
    return [buoy('red', x, y), buoy('green', x + w, y)]


NAV = {'red': 'red', 'green': 'green', 'max_width': 4.0, 'count': 2}


def test_never_invents_elements():
    objs = gate(0) + gate(5) + gate(10)
    c, _ = propose.course({'nav': NAV, 'follow': dict(NAV, count=3)}, objs, (0, 0))
    ids = {(e.x, e.y) for e in c.elements}
    assert ids <= {(o['x'], o['y']) for o in objs}
    assert len(c.elements) <= len(objs) and c.source == 'proposed'


def test_shared_class_split_by_approach():
    objs = gate(0) + gate(20)
    pack = {'nav': dict(NAV, count=1, approach=(1, 0)),
            'speed': dict(NAV, count=1, approach=(1, 20))}
    c, short = propose.course(pack, objs, (0, 0))
    ys = {ch: {e.y for e in c.elements if e.challenge == ch} for ch in pack}
    assert ys == {'nav': {0}, 'speed': {20}} and short == []


def test_short_count_reported_and_low_score_dropped():
    objs = gate(0) + [buoy('red', 0, 9, 0.2), buoy('green', 2, 9, 0.2)]
    c, short = propose.course({'nav': NAV}, objs, (0, 0))
    assert len(c.elements) == 2
    assert short == ['nav: 1/2 gates (expect 2)']


def test_object_used_once_and_cursor_anchor():
    objs = gate(0) + gate(20)
    pack = {'a': dict(NAV, count=1), 'b': dict(NAV, count=2)}
    c, short = propose.course(pack, objs, (0, 0), cursor=(1, 20))
    assert {e.y for e in c.elements if e.challenge == 'a'} == {20}
    assert {e.y for e in c.elements if e.challenge == 'b'} == {0}
    assert short == ['b: 1/2 gates (expect 2)']
