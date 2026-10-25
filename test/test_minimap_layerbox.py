"""The Layers box: c and r lease the costmap for the view's half-width and drop it when both are off."""
import io
from pathlib import Path

import pytest
from rich.console import Console

from mhseals_testing.ros import minimap_feeds as feeds
from mhseals_testing.ui import theme
from mhseals_testing.ui.minimap.layerbox import LayerBox
from mhseals_testing.ui.minimap.state import MapState

OBJECT_GLYPHS = '●○▲▴▼■▪⊠◉◆◇△⊕'      # as test_ui_grammar

GOLDEN = Path(__file__).parent / 'golden' / 'minimap' / 'layers_32.txt'


class FakeNode:
    def __init__(self):
        self.live = []

    def create_subscription(self, _type, topic, _callback, _qos):
        self.live.append(topic)
        return topic

    def destroy_subscription(self, sub):
        self.live.remove(sub)


@pytest.fixture
def node():
    return FakeNode()


@pytest.fixture
def map_feeds(node):
    """A MapFeeds with its real want_layers and LazySubs but no ROS node behind it."""
    real = object.__new__(feeds.MapFeeds)
    real.costmaps, real.state, real.layer_topic = {}, MapState(), None
    real.subs = {t: feeds.LazySub(node, None, t, None, None) for t in (feeds.LOCAL, feeds.GLOBAL)}
    return real


def test_c_leases_the_costmap_for_the_view_half_width_and_off_releases_it(map_feeds, node):
    box = LayerBox(map_feeds)
    box.press('layer_costmap', 5.0)
    assert node.live == [feeds.LOCAL]
    box.sync(20.0)                                   # zoomed out: the global costmap takes over
    assert node.live == [feeds.GLOBAL]
    box.press('layer_costmap', 20.0)
    assert node.live == []


def test_the_costmap_stays_leased_until_both_c_and_r_are_off(map_feeds, node):
    box = LayerBox(map_feeds)
    box.press('layer_costmap', 20.0)
    box.press('layer_hits', 20.0)
    box.press('layer_costmap', 20.0)
    assert node.live == [feeds.GLOBAL]
    box.press('layer_hits', 20.0)
    assert node.live == []


def test_toggles_call_want_layers_with_the_flags_and_the_source():
    calls = []

    class Fake:
        def want_layers(self, *args):
            calls.append(args)

    box = LayerBox(Fake())
    box.press('layer_costmap', 8.0)
    box.press('costmap_source', 8.0)
    box.press('layer_costmap', 8.0)
    assert calls == [(True, False, 8.0, 'auto'), (True, False, 8.0, 'local'), (False, False, 8.0, 'local')]
    assert box.press('goal_mode', 8.0) is False and len(calls) == 3


def test_d_cycles_floor_and_trail_restart_reach_the_map_state():
    state = MapState()
    state.ingest('pose_map', t=0.0, x=0.0, y=0.0)
    box = LayerBox(type('F', (), {'want_layers': lambda *a: None})(), state)
    box.press('detection_floor', 5.0)
    box.press('trail_restart', 5.0)
    assert state.floor == 0.3 and not state.crumbs


def paint(panel, width=32):
    out = io.StringIO()
    Console(width=width, file=out, color_system=None).print(panel)
    return out.getvalue()


@pytest.mark.skipif(theme.ASCII, reason='the Unicode golden')
def test_layers_box_golden_with_mixed_states(request):
    box = LayerBox(type('F', (), {'want_layers': lambda *a: None})(), truth=True)
    for action in ('layer_costmap', 'costmap_source', 'layer_detections', 'layer_detections',
                   'detection_floor', 'detection_floor', 'trail_mode', 'referee'):
        box.press(action, 5.0)
    text = paint(box.render(focused=True, up='N', zoom='fit', centre='boat'))
    if request.config.getoption('--update-goldens'):
        GOLDEN.parent.mkdir(parents=True, exist_ok=True)
        GOLDEN.write_text(text)
    assert text == GOLDEN.read_text()


def test_truth_is_listed_only_when_the_profile_has_course_truth():
    fake = type('F', (), {'want_layers': lambda *a: None})()
    assert 'truth' in paint(LayerBox(fake, truth=True).render())
    assert 'truth' not in paint(LayerBox(fake).render())


def test_render_holds_no_buoy_glyphs_in_either_theme(monkeypatch):
    box = LayerBox(type('F', (), {'want_layers': lambda *a: None})(), truth=True)
    box.press('layer_costmap', 5.0)
    for ascii_mode in (False, True):
        monkeypatch.setattr(theme, 'ASCII', ascii_mode)
        assert not set(paint(box.render())) & set(OBJECT_GLYPHS)


def test_labels_carry_the_pinned_source_and_the_detection_floor():
    box = LayerBox(type('F', (), {'want_layers': lambda *a: None})())
    assert 'costmap' in paint(box.render())
    box.press('costmap_source', 5.0)
    box.press('layer_detections', 5.0)
    box.press('detection_floor', 5.0)
    text = paint(box.render(up='H', zoom='10m', centre='goal'))
    assert 'local' in text and 'raw .3' in text and 'u H' in text and 'z 10m' in text and 'e goal' in text
    assert len(text.splitlines()) == 7               # border + 5 interior rows + border


def test_trail_restart_is_live_only():
    state = MapState()
    state.ingest('pose_map', t=0.0, x=0.0, y=0.0)
    box = LayerBox(None, state, live=False)
    assert box.press('trail_restart', 5.0) is True and state.crumbs
    LayerBox(None, state).press('trail_restart', 5.0)
    assert not state.crumbs


def test_review_has_no_feeds_so_nothing_is_leased():
    box = LayerBox(None, live=False)
    assert box.press('layer_costmap', 5.0) and box.costmap
    box.sync(20.0)                                   # no AttributeError on None


def test_fov_wake_rings_and_up_toggle_and_fov_starts_from_the_yaml():
    box = LayerBox(type('F', (), {'want_layers': lambda *a: None})())
    assert box.fov is None and box.objects == 'tracks' and box.up == 'N'
    for action in ('layer_fov', 'layer_wake', 'layer_rings', 'up_mode'):
        assert box.press(action, 5.0)
    assert (box.fov, box.wake, box.rings, box.up) == (False, False, False, 'H')
    box.press('layer_fov', 5.0)
    assert box.fov is True and 'u H' in paint(box.render())
