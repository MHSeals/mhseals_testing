"""Restart the EKFs and navsat_transform as one keeper child (HR9). Pure and ROS-free.

mhseals_nav's ``ekf.launch.py`` runs ekf_local, ekf_global and navsat_transform in a
single launch, so the EKF row and the navsat row of the Odometry popup both restart
that one child: the profiles' ``ekf`` component, with ``start_ekf:=false`` on ``odometry`` so
only one set runs. MAVROS and Nav2 stay up. Restarting navsat_transform makes it pick a
new map datum from the next GPS fix, which would jump the ``map`` frame; with
``carry_datum`` the old origin is read back (``toLL(0, 0)``) before the stop and sent
as ``datum`` after the start. Until that is bench-verified the flag is off and the
jump is logged as a ``map_redatumed`` event instead.

The ROS calls (``to_ll``, ``datum_ready``, ``set_datum``) and the goal/procedure/velocity
checks are passed in. ``datum_ready`` is polled (up to ``DATUM_WAIT_S``) before the datum is
sent, because the service only exists once navsat_transform is back up.
"""

from pathlib import Path
import time

import yaml

from mhseals_testing.strict_yaml import load_strict

CHILD = 'ekf'
SUBSYSTEM = 'ekf'
DATUM_WAIT_S = 10.0
DATUM_POLL_S = 0.2
OVERLAY = Path('~/.config/mhseals/ekf.yaml')


def command(sim, overlay=OVERLAY, startup_arguments=None):
    """Preserve explicit owner launch selectors; no runtime restart route is enabled here."""
    args = {'sim': str(sim).lower(), 'ekf_overlay_file': str(Path(overlay).expanduser()),
            **(startup_arguments or {})}
    return ['ros2', 'launch', 'mhseals_nav', 'ekf.launch.py',
            *(f'{key}:={value}' for key, value in args.items())]


class EkfRestart:
    """Blockers, the restart sequence and the history record for the EKF child."""

    def __init__(self, processes, history, *, sim, goal_active, procedure_active, velocity_active,
                 emit, to_ll=None, datum_ready=None, set_datum=None, carry_datum=False,
                 overlay=OVERLAY, sleep=time.sleep, clock=time.monotonic, startup_arguments=None):
        self.processes, self.history, self.sim = processes, history, sim
        self.goal_active, self.procedure_active = goal_active, procedure_active
        self.velocity_active, self.emit = velocity_active, emit
        self.to_ll, self.datum_ready, self.set_datum = to_ll, datum_ready, set_datum
        self.startup_arguments = dict(startup_arguments or {})
        self.carry_datum = carry_datum
        self.overlay = Path(self.startup_arguments.get('ekf_overlay_file', overlay)).expanduser()
        self.sleep, self.clock = sleep, clock

    def blockers(self):
        """Why a restart is refused now; the filters feed Nav2 and the velocity controller."""
        reasons = []
        if self.goal_active():
            reasons.append('a navigation goal is active')
        if self.procedure_active():
            reasons.append('a procedure is running')
        if self.velocity_active():
            reasons.append('velocity control is running')
        return reasons

    def _overlay(self):
        """(parsed overlay, raw text) of the user's EKF overlay; ({}, None) when missing."""
        try:
            raw = self.overlay.read_text()
        except OSError:
            return {}, None
        return load_strict(raw, self.overlay) or {}, raw    # YAMLError, ValueError: restart()

    def _wait_ready(self):
        """Poll until navsat_transform answers again; False after ``DATUM_WAIT_S``."""
        end = self.clock() + DATUM_WAIT_S
        while not (self.datum_ready is None or self.datum_ready()):
            if self.clock() >= end:
                return False
            self.sleep(DATUM_POLL_S)
        return True

    def restart(self):
        """Stop and relaunch the EKF child. Returns {'restarted', 'blocked', 'datum'}."""
        blocked = self.blockers()
        if blocked:
            return {'restarted': False, 'blocked': blocked, 'datum': None}
        try:
            value, raw = self._overlay()
        except (ValueError, yaml.YAMLError) as error:
            reason = f'{self.overlay} is not valid YAML ({str(error).splitlines()[0]})'
            return {'restarted': False, 'blocked': [reason], 'datum': None}
        # observe records an outside edit so it is not merged into ours; the restart itself
        # changes nothing in the overlay, so there is no second record to make.
        self.history.observe(SUBSYSTEM, value, raw)
        datum = self.to_ll() if self.carry_datum and self.to_ll else None
        self.processes.stop(CHILD)
        self.processes.start(CHILD, command(self.sim, self.overlay, self.startup_arguments))
        carried = False
        if datum is not None and self.set_datum and self._wait_ready():
            self.set_datum(*datum)
            carried = True
        self.emit('ekf_restart', datum=list(datum) if carried else None)
        if not carried:
            self.emit('map_redatumed', carried=False)
        return {'restarted': True, 'blocked': [], 'datum': datum if carried else None}
