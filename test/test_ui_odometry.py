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
    modal = dict(kind='message', title='Odometry', rows=odometry.rows({'ekf_local': PARAMS}, []),
                 width=76, hint='Esc close')
    panel, width = layout.popup(modal, 100, 40)
    console = Console(width=width, file=io.StringIO(), record=True)
    console.print(panel)
    assert '/odom/mavros' in console.export_text()
