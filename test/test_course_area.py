"""The course area in the TUI (owner decision 2026-11-01): corners, the fence file, status, minimap, apply."""

from types import SimpleNamespace

import pytest

from mhseals_testing.harness import course_area
from mhseals_testing.ros.fence import FenceClient
from mhseals_testing.ui import popups
from mhseals_testing.ui.minimap.canvas import Canvas
from mhseals_testing.ui.minimap.layers import Frame, draw_fence
from mhseals_testing.ui.panels.status import fence_rows
from mhseals_testing.ui.popups import course

BOX = [(42.0, -71.0), (42.0, -70.999), (42.001, -70.999), (42.001, -71.0)]


def test_corners_add_at_the_fix_move_nudge_and_remove():
    edit = course_area.Corners()
    for corner in BOX[:3]:
        edit.add(corner)
    assert edit.items == BOX[:3] and edit.at == 2
    edit.nudge(10.0, 0.0)
    east, north = course_area.enu(BOX[2], edit.items[2])
    assert north == pytest.approx(10.0, abs=0.01) and east == pytest.approx(0.0, abs=0.01)
    edit.move(-1)
    assert edit.at == 1 and edit.items[2] == BOX[1]
    edit.set(BOX[3])
    assert edit.items[1] == BOX[3]
    edit.remove()
    assert len(edit.items) == 2 and course_area.problem(edit.items)


def test_the_fence_file_keeps_the_rest_and_reads_back(tmp_path):
    path = tmp_path / 'fence.yaml'
    path.write_text('fence_frame: map\nfence_points: [0, 0, 1, 0, 1, 1]\n')
    course_area.save(BOX, path)
    assert course_area.load(path) == BOX
    assert 'fence_points' in path.read_text()
    assert course_area.load(tmp_path / 'none.yaml') == []


@pytest.mark.parametrize('status, state, words', [
    ({'inside': True, 'distance_to_edge': 12.0, 'on_exit': 'alert'}, 'ok', 'inside'),
    ({'inside': False, 'distance_to_edge': 3.5, 'on_exit': 'alert'}, 'bad', 'drive the boat back inside'),
    ({'inside': False, 'distance_to_edge': 3.5, 'on_exit': 'return', 'returning': True}, 'bad', 'returning inside'),
    ({'inside': False, 'distance_to_edge': 3.5, 'on_exit': 'return', 'returning': False}, 'bad', 'drive the boat back'),
    ({'inside': None, 'on_exit': 'alert'}, 'warn', 'no position')])
def test_the_status_line_says_what_to_do(status, state, words):
    found, text = course_area.status_line(status)
    assert found == state and words in text
    if status.get('distance_to_edge'):
        assert f"{status['distance_to_edge']:.1f} m" in text


def test_the_status_area_has_course_area_and_an_on_exit_toggle():
    assert fence_rows({}, ()) == []
    rows = fence_rows({'fence': {'inside': False, 'distance_to_edge': 4.0, 'on_exit': 'alert'}}, ())
    assert [r['name'] for r in rows] == ['Course area', 'On exit']
    assert rows[0]['state'] == 'FAIL' and 'OUTSIDE' in rows[0]['value'] and rows[1]['setting'] == 'on_exit'


def layers(canvas):
    return {layer for bits in canvas._dots.values() for layer in bits.values()}


def test_the_minimap_draws_the_fence_red_outside_and_the_preview():
    square = [(-5, -5), (5, -5), (5, 5), (-5, 5), (-5, -5)]
    inside, outside = Canvas(20, 10, k=1), Canvas(20, 10, k=1)
    draw_fence(Frame(inside, (0.0, 0.0), 10.0), (square, False))
    draw_fence(Frame(outside, (0.0, 0.0), 10.0), (square, True), square[:4])
    assert layers(inside) == {'fence'} and layers(outside) == {'fence.out', 'fence.edit'}
    assert course_area.preview(BOX, BOX[0], (10.0, 20.0))[0] == (10.0, 20.0)


class Params:
    def __init__(self, keep=True):
        self.values, self.keep = {}, keep

    def set_atomically(self, node, values):
        assert node == '/fence_watchdog'
        if self.keep:
            self.values.update(values)
        return True, ''

    def get(self, node, names):
        return {n: self.values[n] for n in names if n in self.values}


def test_apply_sets_the_corners_reads_them_back_and_on_exit():
    client = FenceClient(node=None, params=Params())
    assert client.set_corners(BOX) == (True, '')
    assert client.params.values['course.corners'] == course_area.flatten(BOX)
    assert client.set_on_exit('return') == (True, '') and client.set_on_exit('drift')[0] is False
    lost = FenceClient(node=None, params=Params(keep=False))
    ok, why = lost.set_corners(BOX)
    assert not ok and 'readback' in why


def test_the_popup_builds_corners_from_the_fix_and_applies(tmp_path, monkeypatch):
    saved = []
    monkeypatch.setattr(course.course_area, 'save', lambda corners=None, on_exit=None: saved.append(corners or on_exit)
                        or tmp_path / 'f.yaml')
    state = SimpleNamespace(fence_status={'on_exit': 'alert'}, fix_ll=None, pose=(0, 1.0, 2.0, 0.0),
                            fence_preview=[])
    client = FenceClient(node=None, params=Params())
    events = []
    app = SimpleNamespace(map_state=lambda: state, fence_client=lambda: client,
                          emit=lambda phase, **f: events.append(phase))
    monkeypatch.setattr(course.agent_acts, 'in_background', lambda app, title, work, **k: work())
    keys = []
    for corner in BOX[:3]:
        keys += [('fix', corner), 'g']
    keys += ['n', 'o', 'y']
    previews = []

    def drive(app_, modal, handle, refresh=None):
        for key in keys:
            if isinstance(key, tuple):
                state.fix_ll = key[1]
                continue
            previews.append(list(state.fence_preview))
            result = handle(key)
            if result is not None:
                return result
    monkeypatch.setattr(popups, 'popup', drive)
    notice = course.open_course(app)
    assert notice.startswith('Course area applied (3 corners)'), notice
    assert len(previews[-1]) == 3 and state.fence_preview == []
    assert client.params.values['on_exit'] == 'return' and 'course_area' in events and saved[0] == 'return'
    assert len(saved[1]) == 3
    assert course_area.pairs(client.params.values['course.corners'])[2][0] > BOX[2][0]     # nudged north


def test_leaving_the_area_raises_a_standing_error_toast_until_back_inside():
    from mhseals_testing.ui.app import App
    from mhseals_testing.ui.toasts import Toasts
    now = [100.0]
    state = SimpleNamespace(fence_status={'inside': True, 'on_exit': 'alert'}, fence_status_at=100.0)
    events = []

    class Fake:
        fence_alert, fence_status = App.fence_alert, App.fence_status
        holds = [{'cause': 'geofence'}]

        def mux_holds(self):
            return self.holds
        ros = SimpleNamespace(map=SimpleNamespace(clock=lambda: now[0]))
        toasts, monitor, notice = Toasts(), SimpleNamespace(clock=lambda: now[0]), ''

        def map_state(self):
            return state

        def emit(self, phase, **fields):
            events.append(phase)
    app = Fake()

    def tick(status, at=None):
        state.fence_status, state.fence_status_at = status, now[0] if at is None else at
        app.fence_alert()
        return [(t.kind, t.text) for t in app.toasts.visible(now[0])]
    assert tick({'inside': True, 'on_exit': 'alert'}) == []
    shown = tick({'inside': False, 'distance_to_edge': 3.2, 'on_exit': 'alert'})
    assert shown[0][0] == 'fail' and 'drive the boat back inside' in shown[0][1] and events == ['fence_outside']
    now[0] += 60.0       # an error toast never expires while outside, and is raised once; its distance is live
    app.holds = []
    shown = tick({'inside': False, 'distance_to_edge': 4.0, 'on_exit': 'alert'})
    assert shown[0][0] == 'fail' and '4.0 m' in shown[0][1] and 'U releases' not in shown[0][1]
    assert tick({'inside': None, 'on_exit': 'alert'})[0][0] == 'fail' and events == ['fence_outside']
    assert tick({'inside': True, 'on_exit': 'alert'})[0][0] == 'ok'
    # a status fence_watchdog stopped sending is stale, never "inside"
    stale = app.fence_status(SimpleNamespace(fence_status={'inside': True}, fence_status_at=now[0] - 30))
    assert stale['inside'] is None and course_area.status_line(stale)[0] == 'warn'


pytestmark = pytest.mark.smoke   # tools/smoke
