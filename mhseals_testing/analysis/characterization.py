"""Fit a boat velocity model from a ROS bag or simple JSON/CSV samples.

See docs/math.tex, "Least-squares characterization". The fitter is
deliberately ROS-independent.  A sample is a mapping with
``command`` (surge, sway, yaw), ``velocity`` (measured vx, vy, wz), and an
optional ``time`` in seconds.  The CLI can read a rosbag2 directory when ROS
is installed, or newline-delimited JSON for offline CI and field analysis.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
from typing import Iterable, Mapping

import numpy

from mhseals_hardware.configuration import config_path, load_config, update_config
from mhseals_hardware.thruster_mixer import AXES, IDENTITY, validate_decoupling
from mhseals_hardware.velocity_law import decoupling_for, raw_gain
from mhseals_testing.history import History, adapters
from mhseals_testing.analysis.thrust import ACTIVE, speed_curves


# Share of the boat's limits Nav2 may plan with: all of it. (A share held back for the
# velocity loop slowed the boat without helping; MPPI's circling was its timing.)
HEADROOM = 1.0


def nav2_profile(result, source, headroom=HEADROOM):
    """Return only measured Nav2 limits, preserving the configured reverse cap."""
    import yaml
    base = yaml.safe_load(Path(source).read_text())
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


# Headings closer together than this (mean resultant length of the unit
# heading vectors) can't tell a world-fixed drift from a body-fixed bias.
SEPARATION_SPREAD = 0.9
MIN_SAMPLES = 8


def heading_spread(yaws):
    """1 for a single heading, 0 for headings spread evenly round the circle."""
    return math.hypot(sum(map(math.cos, yaws)), sum(map(math.sin, yaws))) / len(yaws)


def _design(samples, drift):
    """Stack one row per sample and measured axis for v = b_m + R(yaw)^T d + G e.

    Unknowns: b_m (3), G row-major (9), then d_x, d_y in the odom frame when
    ``drift`` is set. R(yaw)^T d = (d_x cos + d_y sin, -d_x sin + d_y cos), and
    yaw rate has no drift term.
    """
    rows, values = [], []
    for command, velocity, _, yaw in samples:
        c, s = (math.cos(yaw), math.sin(yaw)) if drift else (0.0, 0.0)
        for axis in range(3):
            row = [0.0] * (14 if drift else 12)
            row[axis] = 1.0
            row[3 + 3 * axis:6 + 3 * axis] = command
            if drift and axis < 2:
                row[12:14] = (c, s) if axis == 0 else (-s, c)
            rows.append(row)
            values.append(velocity[axis])
    return numpy.array(rows), numpy.array(values)


def _normal_equations(samples, drift):
    """Least squares through the normal equations X^T X theta = X^T y."""
    rows, values = _design(samples, drift)
    gram = rows.T @ rows + 1e-8 * numpy.eye(rows.shape[1])
    # Do not turn a one-axis bag into a seemingly valid 3-axis calibration.
    if numpy.linalg.matrix_rank(gram, tol=1e-6) < rows.shape[1]:
        raise ValueError('samples must excite neutral, surge, sway, and yaw')
    return numpy.linalg.solve(gram, rows.T @ values)


def _headed(samples):
    """The samples with a heading, when enough of them remain to fit on."""
    headed = [sample for sample in samples if sample[3] is not None]
    return headed if len(headed) >= MIN_SAMPLES else []


def fit(samples):
    """Fit the model; separate drift from bias only when the headings allow it.

    Returns (mechanical bias, gain, drift (x, y) or None).
    """
    # Samples without a heading can't be placed in the world frame: leave them
    # out of the fit instead of letting one of them switch drift separation off.
    samples = _headed(samples) or samples
    yaws = [sample[3] for sample in samples]
    drift = all(yaw is not None for yaw in yaws) and heading_spread(yaws) < SEPARATION_SPREAD
    theta = _normal_equations(samples, drift)
    bias = tuple(float(value) for value in theta[:3])
    gain = tuple(tuple(float(value) for value in theta[3 + 3 * i:6 + 3 * i]) for i in range(3))
    return bias, gain, (float(theta[12]), float(theta[13])) if drift else None


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


def check_wiring(gain):
    """Reject a main response with the wrong sign: that axis is wired backwards.

    ``gain`` is the fitted velocity gain G. Cross terms are not gated: they mix
    m/s with rad/s, so only the sign of each diagonal is comparable.
    """
    for i, axis in enumerate(AXES):
        if gain[i][i] <= 0:
            raise ValueError(f"{axis} responds backwards (gain {gain[i][i]:.2f}): "
                             "check wiring, run Thrusters")


def characterize_samples(samples: Iterable[Mapping], decoupling=sum(IDENTITY, ()),
                         source="samples", motion=None):
    """Return calibration data suitable for writing to hardware YAML.

    Samples are paired command and odometry observations, with the heading
    (``yaw``) when known. Commands are the normalized effort consumed by
    ``thruster_pwm_node``; velocities are SI units from odometry.
    ``decoupling`` is the matrix the driver applied while they were recorded,
    so the fit can divide it back out. The drift is today's current and wind:
    it is reported but the controller estimates it live, so it is never
    calibration. ``motion`` is the full odometry, switching transients
    included, for the acceleration limits (default: the samples themselves).
    """
    normalized = _normalize(samples)
    if len(normalized) < MIN_SAMPLES:
        raise ValueError("at least eight command/odometry samples are required")
    bias, gain, drift = fit(normalized)
    used = numpy.array(validate_decoupling(decoupling))
    gain_map = {axis: {name: gain[i][j] for j, name in enumerate(AXES)}
                for i, axis in enumerate(AXES)}
    try:
        recommended = decoupling_for(raw_gain(dict(
            velocity_gain=gain_map, fitted_with_decoupling=used.flatten().tolist())))
        recommended = [float(value) for value in recommended.flatten()]
    except numpy.linalg.LinAlgError:
        recommended = list(sum(IDENTITY, ()))
    check_wiring(gain)
    # Limits describe observed motion; do not extrapolate untested full-throttle dynamics.
    effort = {axis: max(abs(row[0][index]) for row in normalized) or 1.0
              for index, axis in enumerate(AXES)}
    velocity_limit = {axis: max(abs(row[1][index]) for row in normalized)
                      for index, axis in enumerate(AXES)}
    acceleration, deceleration = acceleration_limits(
        _normalize(motion) if motion is not None else normalized)
    combined = {}
    for command, velocity, _, _ in normalized:
        active = tuple(axis for axis, value in zip(AXES, command) if abs(value) > .02)
        if len(active) < 2:
            continue
        direction = ','.join(f'{axis}={1 if command[AXES.index(axis)] > 0 else -1}'
                             for axis in active)
        speed = math.sqrt(sum(velocity[index] ** 2 for index in range(3)))
        combined[direction] = max(combined.get(direction, 0.0), speed)
    return {
        "version": 1,
        "source": source,
        "identified_at": datetime.now(timezone.utc).isoformat(),
        "sample_count": len(normalized),
        "mechanical_bias": {axis: bias[i] for i, axis in enumerate(AXES)},
        "velocity_gain": gain_map,
        "speed_curves": speed_curves(((c, v, yaw) for c, v, _, yaw in (_headed(normalized) or normalized)), drift),
        "drift": (dict(x=drift[0], y=drift[1], speed=math.hypot(*drift),
                       toward=math.degrees(math.atan2(drift[1], drift[0])),
                       frame="odom", separated=True)
                  if drift else dict(separated=False)),
        "fitted_with_decoupling": [float(value) for value in used.flatten()],
        "decoupling": recommended,
        "motion_limits": {
            "velocity": velocity_limit,
            "acceleration": acceleration,
            "deceleration": deceleration,
            "combined": dict(velocity_limit),
            "combined_directions": combined,
            "measured_at_effort": effort,
        },
    }


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


def read_json_samples(path: Path):
    text = path.read_text()
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        value = [json.loads(line) for line in text.splitlines() if line.strip()]
    if isinstance(value, Mapping):
        value = value.get("samples", [])
    return value


def read_csv_samples(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for row in csv.DictReader(stream):
            rows.append({"time": row.get("time", row.get("stamp", 0)),
                         "command": [row.get("cmd_surge", row.get("surge", 0)),
                                     row.get("cmd_sway", row.get("sway", 0)),
                                     row.get("cmd_yaw", row.get("yaw", 0))],
                         "velocity": [row.get("vx", 0), row.get("vy", 0), row.get("wz", 0)],
                         "yaw": row.get("yaw") or None})
    return rows


def read_rosbag(path: Path, command_topic="/cmd_vel", odom_topic="/odom/local", transient=1.0):
    """Read command/odometry pairs from a rosbag2 directory.

    Each odometry sample is paired with the command held when it was measured;
    ``command_age`` records seconds since the last command change. Samples
    younger than ``transient`` are dropped so the fit sees steady responses.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
    except ImportError as error:
        raise RuntimeError("ROS 2 rosbag dependencies are required for --bag") from error
    reader = rosbag2_py.SequentialReader()
    storage_id = "sqlite3"
    metadata = path / "metadata.yaml"
    if (metadata.exists() and "storage_identifier: mcap" in metadata.read_text()) or \
            any(path.glob("*.mcap")):
        storage_id = "mcap"
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if command_topic not in topics or odom_topic not in topics:
        raise ValueError(f"bag must contain {command_topic} and {odom_topic}")
    command, since, samples = None, None, []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in (command_topic, odom_topic):
            continue
        cls = Twist if topic == command_topic else Odometry
        msg = deserialize_message(data, cls)
        timestamp = stamp / 1e9
        # Pair only with the latest preceding command, never a future one.
        if topic == command_topic:
            value = (msg.linear.x, msg.linear.y, msg.angular.z)
            if value != command:
                command, since = value, timestamp
        elif command is not None and timestamp - since >= transient:
            twist, q = msg.twist.twist, msg.pose.pose.orientation
            samples.append({"time": timestamp, "command": command,
                            "command_age": timestamp - since,
                            "velocity": (twist.linear.x, twist.linear.y, twist.angular.z),
                            "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y),
                                              1 - 2 * (q.y * q.y + q.z * q.z))})
    return samples


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path, help="rosbag2 directory")
    source.add_argument("--samples", type=Path, help="JSON or CSV samples")
    parser.add_argument("--config", type=Path,
                        help="hardware YAML to update; its decoupling must be the one "
                             "active when the bag was recorded")
    parser.add_argument("--output", type=Path, help="write calibration YAML")
    parser.add_argument("--apply-decoupling", action="store_true",
                        help="also apply the fitted decoupling to the driver")
    parser.add_argument("--nav2-params", type=Path,
                        help="source Nav2 YAML when generating --nav2-overlay")
    parser.add_argument("--nav2-overlay", type=Path,
                        help="write measured limits for navigation.launch.py limits_file")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom/local")
    args = parser.parse_args(argv)
    if args.bag:
        motion = read_rosbag(args.bag, args.cmd_topic, args.odom_topic, transient=0.0)
        samples = [sample for sample in motion if sample["command_age"] >= 1.0]
    elif args.samples.suffix.lower() == ".csv":
        samples = read_csv_samples(args.samples)
    else:
        samples = read_json_samples(args.samples)
    config = load_config(args.config)
    result = characterize_samples(samples, config["decoupling"],
                                 source="rosbag2" if args.bag else "samples",
                                 motion=motion if args.bag else None)
    target = args.output or args.config
    if target:
        changes = {"characterization": result}
        if args.apply_decoupling:
            changes["decoupling"] = list(result["decoupling"])
        # A separate output retains explicit pin overrides from the input file.
        if args.config and target != args.config:
            import yaml
            changes = dict(yaml.safe_load(args.config.read_text()), **changes)
        operator = Path(target) == Path(config_path())     # only the operator's file is versioned
        if operator:
            adapters.observe_file(History(), 'hardware', target, print_warning)
        update_config(changes, target)
        if operator:
            adapters.record_file(History(), 'hardware', 'cli:characterization', target, print_warning)
    if args.nav2_overlay:
        if not args.nav2_params:
            parser.error('--nav2-overlay requires --nav2-params')
        import yaml
        args.nav2_overlay.parent.mkdir(parents=True, exist_ok=True)
        args.nav2_overlay.write_text(yaml.safe_dump(
            nav2_profile(result, args.nav2_params), sort_keys=False))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
