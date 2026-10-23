"""fcu probe: facts from the flight controller's MAVROS topics.

Message-agnostic on purpose: the ROS callbacks in ros/ pass plain numbers to the
``on_*`` methods, so this runs and is tested without ROS. Nothing here writes to
the FCU. Facts: fcu.connected/armed/mode, battery.voltage/percent/diag_level
(MAVROS "Battery" diagnostic level, 0 OK 1 WARN 2 ERROR 3 STALE, message as
label), gps.fix_type/sats/h_acc, mag.field_mG, imu.accel_norm, nav.speed_disagreement.
"""

from collections import deque
import math

from mhseals_testing.health.model import Fact

TESLA_TO_MG = 1e7              # sensor_msgs/MagneticField is in tesla
MOVING = 0.2                   # m/s; speed disagreement is only judged while either speed exceeds this
WINDOW = 10.0                  # s of speed samples in the RMS


class FcuProbe:
    def __init__(self):
        self._facts = {}
        self._speeds = deque()                  # (time, fcu speed - gps speed) while moving

    def _put(self, name, value, now, label=''):
        self._facts[name] = Fact(value, label=label, time=now)

    def on_state(self, connected, armed, mode, now):
        self._put('fcu.connected', bool(connected), now)
        self._put('fcu.armed', bool(armed), now)
        self._put('fcu.mode', mode, now)

    def on_battery(self, voltage, percentage, now):
        self._put('battery.voltage', voltage, now)
        self._put('battery.percent', percentage, now)

    def on_diagnostic(self, name, level, message, now):
        """A /diagnostics status; only the MAVROS battery one is kept (its text is the low-voltage reason)."""
        if 'battery' in name.lower():
            self._put('battery.diag_level', int.from_bytes(level, 'little') if isinstance(level, bytes)
                      else int(level), now, label=message)

    def on_gps_raw(self, fix_type, satellites, h_acc_m, now):
        self._put('gps.fix_type', int(fix_type), now)
        self._put('gps.sats', int(satellites), now)
        if h_acc_m is not None and math.isfinite(h_acc_m):
            self._put('gps.h_acc', h_acc_m, now)

    def on_mag(self, x, y, z, now):
        """Field components in tesla."""
        self._put('mag.field_mG', math.sqrt(x * x + y * y + z * z) * TESLA_TO_MG, now)

    def on_imu(self, ax, ay, az, now):
        """Specific force in m/s^2: about 9.8 at rest, near 0 in free fall."""
        self._put('imu.accel_norm', math.sqrt(ax * ax + ay * ay + az * az), now)

    def on_speeds(self, fcu_speed, gps_speed, now):
        """Ground speed from the FCU's odometry and from the GPS, sampled together."""
        if max(fcu_speed, gps_speed) > MOVING:
            self._speeds.append((now, fcu_speed - gps_speed))

    def facts(self, now):
        """Every fact published so far, plus the rolling speed disagreement (0 while nothing moves)."""
        while self._speeds and now - self._speeds[0][0] > WINDOW:
            self._speeds.popleft()
        rms = (math.sqrt(sum(d * d for _, d in self._speeds) / len(self._speeds))
               if self._speeds else 0.0)
        return {**self._facts, 'nav.speed_disagreement': Fact(rms, time=now)}
