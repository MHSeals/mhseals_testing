"""The Odometry popup lists estimates then sources, and renders through the generic popup."""
import io

from rich.console import Console

from mhseals_testing.ui import layout
from mhseals_testing.ui.popups import odometry
from test_fusion import PARAMS


def test_rows_list_estimates_then_sources():
    out = odometry.rows({'ekf_local': PARAMS, 'ekf_global': None}, [('FCU IMU', 'LIVE', '20 Hz')])
    labels = [label for label, _ in out]
    assert labels[0] == 'Estimates' and labels.index('ekf_local') < labels.index('Sources')
    assert dict(out)['ekf_global'] == 'parameters unavailable'
    assert dict(out)['  odom0'] == '/odom/mavros  vx vy'
    assert out[-1] == ('FCU IMU', 'LIVE 20 Hz')


def test_trust_line_only_with_two_sources():
    sigmas = {'ekf_local': {'vx': {'odom0': 1.0, 'odom1': 1.0}, 'yaw': {'imu0': 1.0}}}
    out = dict(odometry.rows({'ekf_local': PARAMS}, [], sigmas))
    assert out['  vx'] == 'odom0 50% odom1 50%' and '  yaw' not in out


def test_popup_render():
    modal = dict(kind='message', title='Pose', rows=odometry.rows({'ekf_local': PARAMS}, []),
                 width=76, hint='Esc close')
    panel, width = layout.popup(modal, 100, 40)
    console = Console(width=width, file=io.StringIO(), record=True)
    console.print(panel)
    assert '/odom/mavros' in console.export_text()


def test_e_restarts_the_ekfs_once_and_journals_it(tmp_path):
    from test_app import make_app
    from test_ekf_restart import make
    log = []
    ekf, events = make(tmp_path, log, carry=False)
    app, _ = make_app(tmp_path)
    app.ekf = ekf
    app.restart_ekf()
    assert [entry[:2] for entry in log if entry[0] in ('stop', 'start')] == [('stop', 'ekf'), ('start', 'ekf')]
    assert [name for name, _ in events] == ['ekf_restart', 'map_redatumed']
    assert app.notice == 'EKFs restarted'


def test_restart_is_refused_with_the_reason_while_a_goal_is_active(tmp_path):
    from test_app import make_app
    from test_ekf_restart import make
    log = []
    ekf, events = make(tmp_path, log, goal=True)
    app, _ = make_app(tmp_path)
    app.ekf = ekf
    app.restart_ekf()
    assert log == [] and events == []
    assert app.notice == 'EKF restart refused: a navigation goal is active'
