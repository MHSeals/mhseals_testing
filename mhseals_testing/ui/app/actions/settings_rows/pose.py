"""The Pose popup (odometry): EKF restart, the RTK base controls, and the revert environment."""

import time

from mhseals_testing.model import Cancelled
from mhseals_testing.harness import journal
from mhseals_testing.history import revert
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ui import rtk, rtk_icon, theme
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import odometry


class Pose:
    """The Pose popup (odometry), mixed into Settings."""

    def revert_env(self):
        """The revert blocker probes over this session, and the mux hold used while armed.

        ``ros.mux_hold(token)`` (a context manager) is optional; without it an armed
        hardware revert is refused (revert.blockers).
        """
        hold = getattr(self.ros, 'mux_hold', None)
        return revert.Env(
            probes={'procedure': lambda: self.running is not None, 'pulses': lambda: self.test_pulses is not None},
            armed=lambda: self.armed,
            **({'hold': hold} if hold else {}))

    def odometry(self):
        """Fused sources popup. Parameters and rows are read here, once, so nothing is
        computed while it is closed. ``ros.get_parameters(node)`` is optional (None: unreadable)."""
        if self.review:
            profile = getattr(self.source, 'profile', self.profile)
            rows = [(row['name'], row['state'], row['value'])
                    for row in status_rows(self.snapshot(), profile.sensors)
                    if row.get('source') == 'sensor']
            self.show('Pose (review)', rows=rows, width=76)
            return
        fetch = getattr(self.ros, 'get_parameters', lambda node: None)
        params = {node: fetch(node) for node in odometry.ESTIMATES}
        sources = [(row['name'], row['state'].lower(), row['value'])
                   for row in status_rows(self.snapshot(), self.profile.sensors)
                   if row.get('source') == 'sensor']
        rows = odometry.rows(params, sources)
        events = [e for e in journal.read(self.paths.journal) if e.get('phase') == 'gps_fix']
        if self.rtk.current is not None:     # the correction source: its icon, ring and label
            rows += [('Source' if i == 0 else '', line)
                     for i, line in enumerate(rtk_icon.pane_lines(self.rtk.current, theme.ASCII))]
        if events or self.rtk_link is not None:
            rows += rtk.pane_rows(events, time.time(), 44, theme.ASCII)
        try:   # r opens the RTK base controls next to e (no base says so)
            answer = self.choose('Pose', ['Restart EKFs', 'Base controls', 'Magcal', 'Close'],
                                 {'e': 0, 'r': 1, 'c': 2}, default=3, rows=rows, width=76)
        except Cancelled:
            return
        if answer == 'Restart EKFs':
            self.restart_ekf()
        elif answer == 'Magcal':          # keymap popup.odometry c: the compass calibration (g)
            self.run_procedure(next(p for p in PROCEDURES if p.key == 'g'))
        elif answer == 'Base controls':
            rtk.open_controls(self)

    def restart_ekf(self):
        """Restart the EKF child (popup.odometry e); refused with the reason while the boat is busy.

        The EKF and navsat rows share one keeper child, so either row restarts both (HR9).
        """
        if self.ekf is None:
            self.notice = 'EKF restart unavailable in this session'
            return
        result = self.ekf.restart()
        self.notice = ('EKFs restarted' if result['restarted']
                       else 'EKF restart refused: ' + '; '.join(result['blocked']))
