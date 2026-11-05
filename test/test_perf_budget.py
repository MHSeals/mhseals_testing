"""The TUI's speed budgets (config/perf.yaml): a Drive frame lays out and writes inside budget.frame at p99, an
unchanged frame writes nothing, and the Meter reports p99s, bytes and its CPU split, naming a budget it passes."""
import io
import time

from rich.console import Console

from mhseals_testing.ui import perf
from mhseals_testing.ui.difflive import DiffLive
from test_app import make_app

FRAMES = 60


def test_a_drive_frame_fits_the_frame_budget_and_an_unchanged_one_writes_nothing(tmp_path):
    app, boat = make_app(tmp_path)
    limits = perf.budget()
    out = io.StringIO()
    console = Console(file=out, force_terminal=True, width=120, height=40, color_system='256')
    meter = perf.Meter(limits, knobs=perf.settings())
    live = DiffLive(app.render(), console=console, screen=True, auto_refresh=False,
                    settings={'full_every': 0.0, 'sync': True})
    with live:
        for i in range(FRAMES):
            boat.feed((0.1 * (i % 7), 0.0, 0.05))       # moving data: the Motion plots and Status rates change
            app.monitor.clock.now += 1 / 30
            start, cpu = time.perf_counter(), time.thread_time()
            live.update(app.render(), refresh=True)
            meter.frame(time.perf_counter() - start, live.sent, time.thread_time() - cpu)
        live.update(app.render(), refresh=True)
        live.update(app.render(), refresh=True)
        assert live.sent == 0                           # nothing changed: nothing written
    report = meter.report()
    assert report['frames'] == FRAMES
    assert report['frame_p99'] < limits['frame'], f"frame p99 {report['frame_p99']:.3f} s over {limits['frame']} s"


def test_the_meter_reports_percentiles_bytes_cpu_and_the_budgets_it_passes():
    now = [0.0]
    meter = perf.Meter({'frame': 0.04, 'key': 0.1, 'window': 10.0}, clock=lambda: now[0], cpu=lambda: now[0] / 2,
                       knobs={'report_every': 1.0})
    for i in range(100):
        now[0] = i * 0.1
        if i % 10 == 0:
            meter.key()
        meter.frame(0.01 if i < 98 else 0.09, 1000 if i % 2 else 0, 0.001)
    report = meter.report()
    assert report['frames'] == 100 and report['unchanged'] == 50 and report['keys'] == 10
    assert report['frame_p50'] == 0.01 and report['frame_p99'] == 0.09
    assert report['cpu'] == 50.0 and 0 < report['bytes_per_s'] < 10_000
    assert report['over'] == ['frame 0.090 > 0.040']
    meter.ros_cpu(1.0)
    meter.ros_cpu(3.0)
    assert meter.report()['cpu_ros'] > 0


def test_perf_yaml_loads_through_its_schema():
    knobs, limits = perf.settings(), perf.budget()
    assert knobs['executor'] in ('events', 'single') and isinstance(knobs['diff'], bool)
    assert 0 < limits['frame'] < limits['key']
