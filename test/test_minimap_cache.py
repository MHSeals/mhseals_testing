"""Layer cache invalidation, the redraw caps and the cached-frame time guard (map.md 9)."""
import time

from mhseals_testing.ui.minimap import cache
from mhseals_testing.ui.minimap.canvas import Canvas


def _layer(n):
    c = Canvas(56, 30)
    for i in range(n):
        c.dot(i % 112, (i * 7) % 120, 'costmap')
        c.tint((i * 3) % 56, i % 30, 'cost')
    return c


def test_layer_builds_once_per_key_and_rebuilds_on_view_or_version_change():
    calls, lc = [], cache.LayerCache()

    def build():
        calls.append(1)
        return Canvas(2, 2)

    for key in [('v1', 1), ('v1', 1), ('v2', 1), ('v2', 2), ('v2', 2)]:
        lc.get('cost', key, build)
    assert len(calls) == 3


def test_frame_paint_does_not_touch_the_cached_layer():
    lc = cache.LayerCache()
    layers = [('cost', 1, lambda: _layer(50))]
    first = lc.frame(56, 30, layers)
    first.dot(0, 0, 'boat')
    first.glyph(1, 1, 'X', 'label')
    assert lc.frame(56, 30, layers).text() == _layer(50).text()


def test_overlapping_layers_keep_every_dot_and_the_higher_glyph_in_any_order():
    def first():
        c = Canvas(2, 1)
        c.glyph(0, 0, 'B', 'boat')
        c.dot(0, 0, 'costmap')
        return c

    def second():
        c = Canvas(2, 1)
        c.glyph(0, 0, 'c', 'costmap')
        c.dot(1, 3, 'wall')
        return c

    for order in ((('a', first), ('b', second)), (('b', second), ('a', first))):
        frame = cache.LayerCache().frame(2, 1, [(n, 1, f) for n, f in order])
        assert frame._glyphs[(0, 0)][:2] == ('B', 'boat')
        assert frame._dots[(0, 0)] == {0x01: 'costmap', 0x80: 'wall'}


def test_redraw_caps_hidden_and_budget_fallback():
    r = cache.Redraw()
    assert r.due(0.0)
    r.done(0.0, 1.0)
    assert not r.due(1.0)            # nothing changed
    r.mark()
    assert not r.due(0.1)            # 5 Hz cap
    assert r.due(0.2)
    assert not r.due(0.2, visible=False)
    assert not r.due(0.2, on_map_tab=False) and r.due(0.5, on_map_tab=False)  # Drive 2 Hz
    r.done(0.2, 40.0)
    r.mark()
    assert r.slow and not r.due(0.5) and r.due(0.7)  # over budget: 2 Hz on the Map tab


def test_cached_frame_within_3_ms():
    lc = cache.LayerCache()
    layers = [('cost', 1, lambda: _layer(1500)), ('ref', 1, lambda: _layer(300))]
    lc.frame(56, 30, layers)
    best = float('inf')
    for _ in range(20):
        t = time.perf_counter()
        f = lc.frame(56, 30, layers)
        f.dot(10, 10, 'boat')
        f.cells()
        best = min(best, (time.perf_counter() - t) * 1000)
    assert best <= 3.0, best
