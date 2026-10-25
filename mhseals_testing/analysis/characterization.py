"""Nav2 limits and hold windows from a characterization session.

The survey and per-direction column refinement live in procedures/characterize.py;
this module only turns held windows into the measured limits Nav2 plans with. ROS-free.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Mapping

import numpy

from mhseals_hardware.thruster_mixer import AXES
from mhseals_testing.analysis.thrust import ACTIVE, step_response
from mhseals_testing.strict_yaml import load_strict


# Share of the boat's limits Nav2 may plan with: all of it. (A share held back for the
# velocity loop slowed the boat without helping; MPPI's circling was its timing.)
HEADROOM = 1.0


def nav2_profile(result, source, headroom=HEADROOM):
    """Return only measured Nav2 limits, preserving the configured reverse cap."""
    base = load_strict(Path(source).read_text(), source)
    profile = {key: {"ros__parameters": {}} for key in
               ("controller_server", "velocity_smoother", "behavior_server")}
    limits = result["motion_limits"]
    velocity = {axis: headroom * value for axis, value in limits["velocity"].items()}
    acceleration = {axis: headroom * value for axis, value in limits["acceleration"].items()}
    deceleration = {axis: headroom * value for axis, value in limits["deceleration"].items()}
    controller = profile["controller_server"]["ros__parameters"]
    # MPPI samples inside these bounds, so it never plans what the boat can't do.
    follow_base = base["controller_server"]["ros__parameters"].get("FollowPath", {})
    follow = controller.setdefault("FollowPath", {})
    follow.update(
        vx_max=velocity["surge"], vx_min=max(follow_base.get("vx_min", -.2), -velocity["surge"]), vy_max=velocity["sway"],
        wz_max=velocity["yaw"], ax_max=acceleration["surge"], ax_min=-deceleration["surge"],
        ay_max=acceleration["sway"], az_max=acceleration["yaw"])
    # navigation.launch.py derives pruning distance from its current MPPI horizon.
    behavior = profile["behavior_server"]["ros__parameters"]
    if "behavior_server" in base:
        behavior["max_rotational_vel"] = velocity["yaw"]
        behavior["rotational_acc_lim"] = acceleration["yaw"]
    smoother = profile["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [velocity[axis] for axis in AXES]
    smoother["min_velocity"] = [max(-velocity[axis], bound) for axis, bound in zip(
        AXES, base["velocity_smoother"]["ros__parameters"].get(
            "min_velocity", [-.2, -velocity["sway"], -velocity["yaw"]]))]
    smoother["max_accel"] = [acceleration[axis] for axis in AXES]
    smoother["max_decel"] = [-deceleration[axis] for axis in AXES]
    return profile


def _vector(value, name):
    if isinstance(value, Mapping):
        value = [value.get(axis, 0.0) for axis in AXES]
    if len(value) != 3:
        raise ValueError(f"{name} must contain surge, sway, yaw")
    values = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in values):
        raise ValueError(f"{name} must contain finite values")
    return values


def acceleration_limits(samples, span=0.5, percentile=0.99):
    """Per-axis acceleration and deceleration the boat showed, robust to sensor noise.

    Velocity changes are taken over at least ``span`` seconds, not between
    neighbouring samples (differentiating 50 Hz odometry amplifies its noise into
    accelerations no thruster could produce), and the ``percentile`` of them is
    kept rather than the single largest.
    """
    rates = {axis: ([], []) for axis in AXES}
    later = 0
    for earlier, (_, velocity, stamp, _) in enumerate(samples):
        later = max(later, earlier + 1)
        while later < len(samples) and samples[later][2] - stamp < span:
            later += 1
        if later >= len(samples) or samples[later][2] - stamp > 2 * span:
            continue  # a gap between windows, not a measured change
        dt = samples[later][2] - stamp
        for index, axis in enumerate(AXES):
            change = (samples[later][1][index] - velocity[index]) / dt
            rates[axis][0 if change > 0 else 1].append(abs(change))

    def quantile(values):
        return sorted(values)[int(percentile * (len(values) - 1))] if values else 0.0
    return ({axis: quantile(up) for axis, (up, _) in rates.items()},
            {axis: quantile(down) for axis, (_, down) in rates.items()})


def _normalize(samples):
    """(command, velocity, time, yaw) tuples from sample mappings."""
    normalized = []
    for sample in samples:
        command = _vector(sample.get("command", sample.get("cmd_vel")), "command")
        velocity = _vector(sample.get("velocity", sample.get("odom")), "velocity")
        stamp = float(sample.get("time", sample.get("stamp", 0.0)))
        if not math.isfinite(stamp):
            stamp = 0.0
        yaw = sample.get("yaw")
        yaw = float(yaw) if yaw is not None and math.isfinite(float(yaw)) else None
        normalized.append((command, velocity, stamp, yaw))
    return normalized


def samples_from_windows(history, windows, transient=1.0):
    """Pair each odometry sample with the command held when it was measured.

    ``history`` is (time, values) odometry; ``windows`` are (command, start,
    end) from the suite's runner. The first ``transient`` seconds of each
    window are skipped, so the fit sees the response, not the switching.
    Samples with a missing or non-finite vx/vy/wz are dropped (one NaN would
    otherwise abort the whole fit), as are windows with a non-finite command.
    """
    samples = []
    for command, start, end in windows:
        if not all(math.isfinite(c) for c in command):
            continue
        for stamp, values in history:
            if start + transient <= stamp <= end and all(
                    values.get(key) is not None and math.isfinite(values[key])
                    for key in ('vx', 'vy', 'wz')):
                samples.append({"time": stamp, "command": tuple(command),
                                "velocity": (values["vx"], values["vy"], values["wz"]),
                                "yaw": values.get("yaw")})
    return samples


def neutral_noise(history, windows, transient=1.0, coast=0.0, t95=None, holds=None):
    """Per-sample SD of the neutral velocity as (yaw, surge, sway), the tracking score's noise floor.

    Each neutral window is demeaned on its own, so a drift or a bias between windows is not noise.
    A window that follows a hold starts with the coast, a decay demeaning cannot remove, so it
    skips max(transient, ``coast``, the ``t95`` of each axis that hold drove); ``t95`` maps
    surge/sway/yaw to seconds. ``holds`` are the windows to look for that hold in (default
    ``windows``; pass the unfiltered list when rejected holds were dropped from ``windows``).
    This is the per-sample level (decisions 2026-10-23), not the SD of a window mean.
    """
    residuals = {key: [] for key in ('wz', 'vx', 'vy')}
    holds = windows if holds is None else holds
    for command, start, end in windows:
        if any(command):
            continue
        before = [w for w in holds if any(w[0]) and w[2] <= start + 1e-6]
        skip = transient
        if before:
            last = max(before, key=lambda w: w[2])[0]
            skip = max(transient, coast, *(t95.get(axis, 0.0) for axis, c in zip(AXES, last) if c and t95))
        rows = samples_from_windows(history, [(command, start, end)], skip)
        if len(rows) < 2:
            continue
        for key, index in (('vx', 0), ('vy', 1), ('wz', 2)):
            values = numpy.array([row["velocity"][index] for row in rows])
            residuals[key].extend(values - values.mean())
    return [float(numpy.sqrt(numpy.sum(numpy.square(residuals[key])) / (len(residuals[key]) - 1)))
            if len(residuals[key]) > 1 else 0.0 for key in ('wz', 'vx', 'vy')]


def hold_t95(history, hold, key):
    """Seconds for ``key`` to reach 95 % of its steady change in one hold, or None if it never did.

    ``hold`` is (command, start, end); ``history`` must reach back before ``start`` for the baseline.
    """
    rows = [(stamp, values[key]) for stamp, values in history
            if values.get(key) is not None and math.isfinite(values[key])]
    if len(rows) < 3:
        return None
    t, y = zip(*rows)
    return step_response(t, y, hold[1], hold[2]).t95


def motion_limits(samples, motion, efforts):
    """Speeds, accelerations and decelerations the boat showed, in the shape ``nav2_profile`` reads.

    ``samples`` are the settled hold samples (peak speed); ``motion`` includes the switching
    transients (acceleration). ``efforts`` is the effort each axis was driven at, kept so a
    limit is never read as more than what was tested.
    """
    normalized = _normalize(samples)
    if not normalized:
        raise ValueError("no usable hold samples")
    velocity = {axis: max(abs(row[1][index]) for row in normalized) for index, axis in enumerate(AXES)}
    acceleration, deceleration = acceleration_limits(_normalize(motion))
    return dict(velocity=velocity, acceleration=acceleration, deceleration=deceleration,
                measured_at_effort=dict(zip(AXES, efforts)))


# A hold with fewer samples than this share of the typical sample density, or with
# a velocity that never changed, says the odometry dropped out or froze.
MIN_COVERAGE = 0.5


def print_warning(message):
    print(message, file=sys.stderr)


def audit_holds(history, windows, transient=1.0, minimum=MIN_COVERAGE):
    """One record per hold: samples, coverage against the median density, and whether it is usable.

    Odometry gaps (replay, bags) leave holds with few or no samples, and a frozen
    sensor with fresh stamps never moves, not even through the switching
    transient of a commanded hold; either would silently bend the speed curves.
    Holds no longer than ``transient`` leave nothing to fit and are not judged
    for coverage.
    """
    holds = []
    for command, start, end in windows:
        samples = samples_from_windows(history, [(command, start, end)], transient)
        everything = samples_from_windows(history, [(command, start, end)], 0.0)
        span = end - start - transient
        holds.append(dict(command=list(command), start=start, end=end, samples=len(samples),
                          density=len(samples) / span if span > 0 else None,
                          flat=len(everything) > 1 and any(abs(value) > ACTIVE for value in command)
                          and not numpy.ptp([s["velocity"] for s in everything], axis=0).any()))
    densities = [hold["density"] for hold in holds if hold["density"] is not None]
    typical = float(numpy.median(densities)) if densities else 0.0
    for hold in holds:
        density = hold.pop("density")
        hold["coverage"] = 1.0 if density is None else (density / typical if typical else 0.0)
        hold["ok"] = hold["coverage"] >= minimum and not hold["flat"]
    return holds
