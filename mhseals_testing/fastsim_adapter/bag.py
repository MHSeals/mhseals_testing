"""Recorded sessions without ROS: MCAP messages decoded with mcap_ros2, and a TF buffer over them."""

import bisect
from pathlib import Path

import numpy as np


def quat_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def stamp(msg, fallback):
    h = getattr(msg, 'header', None)
    t = h.stamp.sec + h.stamp.nanosec * 1e-9 if h is not None else 0.0
    return t if t > 0 else fallback


class Tf:
    """A TF buffer from recorded /tf and /tf_static: ``pose_at(t, target, source) -> (R, p)``
    (target <- source), translations and normalised quaternions interpolated linearly."""

    def __init__(self):
        self.parent, self.static, self.series = {}, {}, {}

    def add(self, msg, static):
        for tr in msg.transforms:
            child, parent = tr.child_frame_id.lstrip('/'), tr.header.frame_id.lstrip('/')
            self.parent[child] = parent
            t, r = tr.transform.translation, tr.transform.rotation
            row = (np.array([t.x, t.y, t.z]), np.array([r.x, r.y, r.z, r.w]))
            if static:
                self.static[child] = row
            else:
                times, rows = self.series.setdefault(child, ([], []))
                at = stamp(tr, 0.0)
                k = bisect.bisect(times, at)
                times.insert(k, at)
                rows.insert(k, row)

    def edge(self, child, t):
        if child in self.static:
            p, q = self.static[child]
            return quat_matrix(q), p
        times, rows = self.series.get(child, ([], []))
        if not times or t < times[0] - 0.2 or t > times[-1] + 0.2:
            raise LookupError(f'no transform for {child} at {t:.3f}')
        k = min(max(bisect.bisect(times, t), 1), len(times) - 1) if len(times) > 1 else 0
        if len(times) == 1:
            p, q = rows[0]
        else:
            t0, t1 = times[k - 1], times[k]
            a = min(max((t - t0) / (t1 - t0), 0.0), 1.0) if t1 > t0 else 0.0
            (p0, q0), (p1, q1) = rows[k - 1], rows[k]
            q1 = q1 if np.dot(q0, q1) >= 0 else -q1
            p, q = p0 + a * (p1 - p0), q0 + a * (q1 - q0)
        return quat_matrix(q / np.linalg.norm(q)), p

    def to_root(self, frame, t):
        R, p, seen = np.eye(3), np.zeros(3), set()
        while frame in self.parent and frame not in seen:
            seen.add(frame)
            Re, pe = self.edge(frame, t)
            R, p = Re @ R, Re @ p + pe
            frame = self.parent[frame]
        return frame, R, p

    def pose_at(self, t, target, source):
        ra, Ra, pa = self.to_root(target, t)
        rb, Rb, pb = self.to_root(source, t)
        if ra != rb:
            raise LookupError(f'{target} and {source} are not connected')
        return Ra.T @ Rb, Ra.T @ (pb - pa)


def messages(session, topics):
    """(topic, t, msg) of ``topics`` from every MCAP under ``session``, in log-time order."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    files = sorted(Path(session).rglob('*.mcap')) if Path(session).is_dir() else [Path(session)]
    rows = []
    for f in files:
        with open(f, 'rb') as fh:
            for _, channel, message, decoded in make_reader(fh, decoder_factories=[DecoderFactory()]) \
                    .iter_decoded_messages(topics=list(topics)):
                rows.append((message.log_time, channel.topic, decoded))
    rows.sort(key=lambda r: r[0])
    for log_time, topic, msg in rows:
        yield topic, stamp(msg, log_time * 1e-9), msg
