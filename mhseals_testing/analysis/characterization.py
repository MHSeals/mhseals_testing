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
from typing import Iterable, Mapping

import numpy

from mhseals_hardware.configuration import load_config, save_config
from mhseals_hardware.thruster_mixer import AXES, IDENTITY, validate_decoupling
from mhseals_hardware.velocity_law import decoupling_for, raw_gain


# Share of the boat's limits Nav2 may plan with: all of it. (A share held back for the
# velocity loop slowed the boat without helping; MPPI's circling was its timing.)
HEADROOM = 1.0


def nav2_profile(result, source, headroom=HEADROOM):
    """Return a complete Nav2 profile with the measured limits in MPPI and the smoother."""
    import yaml
    profile = yaml.safe_load(Path(source).read_text())
    limits = result["motion_limits"]
    velocity = {axis: headroom * value for axis, value in limits["velocity"].items()}
    acceleration = {axis: headroom * value for axis, value in limits["acceleration"].items()}
    deceleration = {axis: headroom * value for axis, value in limits["deceleration"].items()}
    controller = profile["controller_server"]["ros__parameters"]
    # MPPI samples inside these bounds, so it never plans what the boat can't do.
    follow = controller["FollowPath"]
    follow.update(
        vx_max=velocity["surge"], vx_min=-velocity["surge"], vy_max=velocity["sway"],
        wz_max=velocity["yaw"], ax_max=acceleration["surge"], ax_min=-deceleration["surge"],
        ay_max=acceleration["sway"], az_max=acceleration["yaw"])
    # The path MPPI scores against must reach past its own trajectories, or near the
    # end of a path they run off it and the boat circles instead of arriving.
    horizon = follow.get("time_steps", 56) * follow.get("model_dt", 0.05)
    follow["prune_distance"] = max(follow.get("prune_distance", 0.0),
                                   round(1.2 * velocity["surge"] * horizon, 2))
    behavior = profile.get("behavior_server", {}).get("ros__parameters", {})
    if behavior:
        behavior["max_rotational_vel"] = velocity["yaw"]
        behavior["rotational_acc_lim"] = acceleration["yaw"]
    smoother = profile["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [velocity[axis] for axis in AXES]
    smoother["min_velocity"] = [-velocity[axis] for axis in AXES]
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


def fit(samples):
    """Fit the model; separate drift from bias only when the headings allow it.

    Returns (mechanical bias, gain, drift (x, y) or None).
    """
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
    if len(normalized) < 8:
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
    # Limits at full effort, from runs at any effort: thrust grows with effort squared
    # and drag at speed with speed squared, so speed scales with effort and acceleration
    # with effort squared. ``effort`` is the largest effort driven on each axis.
    effort = {axis: max(abs(row[0][index]) for row in normalized) or 1.0
              for index, axis in enumerate(AXES)}
    velocity_limit = {axis: max(abs(row[1][index]) for row in normalized) / effort[axis]
                      for index, axis in enumerate(AXES)}
    acceleration, deceleration = acceleration_limits(
        _normalize(motion) if motion is not None else normalized)
    acceleration = {axis: value / effort[axis] ** 2 for axis, value in acceleration.items()}
    deceleration = {axis: value / effort[axis] ** 2 for axis, value in deceleration.items()}
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


def samples_from_windows(history, windows, transient=1.0):
    """Pair each odometry sample with the command held when it was measured.

    ``history`` is (time, values) odometry; ``windows`` are (command, start,
    end) from the suite's runner. The first ``transient`` seconds of each
    window are skipped, so the fit sees the response, not the switching.
    """
    samples = []
    for command, start, end in windows:
        for stamp, values in history:
            if start + transient <= stamp <= end and all(
                    values.get(key) is not None for key in ('vx', 'vy', 'wz')):
                samples.append({"time": stamp, "command": tuple(command),
                                "velocity": (values["vx"], values["vy"], values["wz"]),
                                "yaw": values.get("yaw")})
    return samples


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
    samples within ``transient`` seconds of a command change are dropped, as
    in the suite's own procedure, so the fit sees responses and not switching.
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
    messages = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in (command_topic, odom_topic):
            cls = Twist if topic == command_topic else Odometry
            messages.append((stamp / 1e9, topic, deserialize_message(data, cls)))
    # Messages are in time order, so the command held is simply the latest one;
    # pairing with a future command would invent response at phase boundaries.
    command, since, samples = None, None, []
    for timestamp, topic, msg in messages:
        if topic == command_topic:
            value = (msg.linear.x, msg.linear.y, msg.angular.z)
            if value != command:
                command, since = value, timestamp
        elif command is not None and timestamp - since >= transient:
            twist, q = msg.twist.twist, msg.pose.pose.orientation
            samples.append({"time": timestamp, "command": command,
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
                        help="write a complete Nav2 YAML with measured limits")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom/local")
    args = parser.parse_args(argv)
    if args.bag:
        samples = read_rosbag(args.bag, args.cmd_topic, args.odom_topic)
        motion = read_rosbag(args.bag, args.cmd_topic, args.odom_topic, transient=0.0)
    elif args.samples.suffix.lower() == ".csv":
        samples = read_csv_samples(args.samples)
    else:
        samples = read_json_samples(args.samples)
    config = load_config(args.config) if args.config else load_config()
    result = characterize_samples(samples, config["decoupling"],
                                 source="rosbag2" if args.bag else "samples",
                                 motion=motion if args.bag else None)
    if args.apply_decoupling:
        config["decoupling"] = list(result["decoupling"])
    config["characterization"] = result
    target = args.output or args.config
    if target:
        save_config(config, target)
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
