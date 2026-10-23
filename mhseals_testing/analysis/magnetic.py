"""Compass and GPS quality (ROS-free): field strength, heading and yaw trust.

The expected field is the World Magnetic Model at the test site (Fort Worth,
32.612 N 97.120 W): |B| 470 mG, horizontal 235 mG, vertical 405 mG, declination
+2.8 deg. ``SITE`` holds these; a profile may override them.
"""

import math

TESLA_TO_MG = 1e7
SITE = dict(field_mg=470.0, horizontal_mg=235.0, declination_deg=2.8)
FIELD_RATIO = (0.7, 1.3)     # |B| / expected outside this makes yaw unreliable
DISAGREE_MAX = 5.0           # deg, compass vs integrated gyro
RESIDUAL_MAX = 10.0          # deg, magcal residual
SIGMA_MAX = 0.5              # rad, FCU yaw standard deviation


def field_mg(x, y, z):
    """Field magnitude in mG of a MagneticField vector given in tesla."""
    return math.sqrt(x * x + y * y + z * z) * TESLA_TO_MG


def field_ratio(x, y, z, site=SITE):
    """Measured |B| over the site's expected |B| (1.0 is a clean compass)."""
    return field_mg(x, y, z) / site['field_mg']


def heading(mag, roll, pitch):
    """Tilt-compensated magnetic heading in degrees [0, 360); ``mag`` is (x, y, z) in the FRD body frame."""
    x, y, z = mag
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    xh = x * cp + y * sr * sp + z * cr * sp
    yh = y * cr - z * sr
    return math.degrees(math.atan2(-yh, xh)) % 360


def wrap(deg):
    """Angle in degrees wrapped to [-180, 180)."""
    return (deg + 180) % 360 - 180


def compass_gyro(history_yaw, history_gyro, window=10.0):
    """Worst disagreement in degrees between compass yaw change and the gyro integral.

    Both are lists of (time, value): yaw in degrees, gyro in rad/s. Over each
    ``window`` seconds, compare the compass change with the integrated rate.
    The caller trims both histories to the span that matters (tui.md §13: the last 60 s).
    """
    if len(history_yaw) < 2 or len(history_gyro) < 2:
        return 0.0
    worst = 0.0
    for i, (t0, y0) in enumerate(history_yaw):
        later = [(t, y) for t, y in history_yaw[i + 1:] if t - t0 >= window]
        if not later:
            break
        t1, y1 = later[0]
        turned = 0.0
        for (ta, wa), (tb, _) in zip(history_gyro, history_gyro[1:]):
            lo, hi = max(ta, t0), min(tb, t1)
            if hi > lo:
                turned += wa * (hi - lo)
        worst = max(worst, abs(wrap(wrap(y1 - y0) - math.degrees(turned))))
    return worst


def yaw_quality(disagree, magcal_residual, yaw_sigma, field_ratio):
    """('ok' | 'unreliable', reason) from the compass-gyro angle, magcal residual (deg), yaw sigma (rad) and |B| ratio.

    Any input may be None when unknown; unknowns do not fail the rule.
    """
    reasons = []
    if disagree is not None and disagree > DISAGREE_MAX:
        reasons.append(f'comp–gyro {disagree:.0f}°')
    if magcal_residual is not None and magcal_residual > RESIDUAL_MAX:
        reasons.append(f'magcal {magcal_residual:.0f}°')
    if yaw_sigma is not None and yaw_sigma >= SIGMA_MAX:
        reasons.append(f'σ {math.degrees(yaw_sigma):.0f}°')
    if field_ratio is not None and not FIELD_RATIO[0] <= field_ratio <= FIELD_RATIO[1]:
        reasons.append(f'|B| {field_ratio:.1f}×')
    return ('unreliable', ', '.join(reasons)) if reasons else ('ok', '')
