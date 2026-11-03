"""A headless session end to end: keys and live ticks journal marks and automatic edges, close writes the manifest."""

import pytest

from mhseals_testing.annotate import manifest

pytestmark = pytest.mark.smoke   # tools/smoke


def test_a_session_becomes_a_manifest(tmp_path):
    from test_app import journaling, make_app
    app, _ = make_app(tmp_path)
    journaling(app)
    clock = [1000.0]
    app.annotations.wall = lambda: int(clock[0] * 1e9)

    def tick(fix=6, holds=()):
        snapshot = {'sensors': {'gpsraw': {'status': 'LIVE', 'values': {'fix': fix, 'lat': 1.0, 'lon': 2.0}},
                                'odometry': {'values': {'x': 3.0, 'y': 4.0, 'yaw': 0.5, 'vx': 0.4, 'vy': 0.0,
                                                        'wz': 0.0}}},
                    'control': {'holds': [{'cause': c} for c in holds]}}
        app.annotations.tick(app, snapshot, clock[0])

    app.dispatch('L')
    app.dispatch('right')                        # wind 1, before the run
    app.dispatch('[')
    app.arming.armed = True
    app.emit('armed', by='operator')
    tick()
    app.manual, app.command = True, (0.4, 0.0, 0.0)
    for _ in range(10):                          # 10 s of a held command: a calibration candidate
        clock[0] += 1.0
        tick()
    app.command = (0.0, 0.0, 0.0)
    tick()
    app.dispatch('b')
    app.dispatch('1')                            # wake
    app.dispatch('"')
    app.dispatch('5')                            # slow down
    tick(holds=('link lost',))
    tick()
    app.manual = False
    app.arming.armed = False
    app.emit('disarmed', by='operator')
    app.close_labels()
    doc = manifest.load(app.paths.directory)
    kinds = {s['kind'] for s in doc['segments']}
    assert {'armed', 'steady', 'mark', 'link', 'fix'} <= kinds
    steady = next(s for s in doc['segments'] if s['kind'] == 'steady')
    assert steady['detail']['command'] == [0.4, 0.0, 0.0] and steady['conditions']['wind'] == 1
    assert doc['commands'][0]['text'] == 'slow down' and 'x' in doc['commands'][0]['pose']
    assert doc['pending'] >= 2 and [c['id'] for c in doc['calibration']] == [steady['id']]
