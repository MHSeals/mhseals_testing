"""Read rosbag2 sessions across segments, and replay them (spec bags B9).

A session holds one bag directory per recorder run (``bag_00``, ``bag_01``,
...), each with its own ``metadata.yaml`` and split files. ``read`` walks them
in order as one stream; ``BagSource`` adds a playhead with seek, play and rate
for review. ROS is imported inside the functions, so the module loads anywhere;
tests inject ``opener`` to avoid it. Times are wall nanoseconds (``t_ns``), the
receive stamp rosbag2 writes, which is also the journal's axis.
"""

import re
from pathlib import Path

WINDOW_S = 10.0


def _is_bag(path):
    return (path / 'metadata.yaml').exists() or any(path.glob('*.mcap')) or any(path.glob('*.db3'))


def segments(path):
    """Bag directories under ``path`` in recording order.

    ``path`` is a single bag (it has ``metadata.yaml`` or storage files), or a
    session whose ``NN`` children (recorder layout, also under ``bag/``) or
    legacy ``bag_NN`` children are the segments.
    """
    path = Path(path)
    if _is_bag(path):
        return [path]
    if (path / 'bag').is_dir():
        path = path / 'bag'
        if _is_bag(path):
            return [path]
    return sorted(child for child in path.iterdir()
                  if child.is_dir() and (child.name.startswith('bag_') or re.fullmatch(r'\d{2,}', child.name))
                  ) if path.is_dir() else []


def open_segment(path):
    """A rosbag2 SequentialReader over one segment and its topic->type map."""
    import rosbag2_py
    path = Path(path)
    metadata = path / 'metadata.yaml'
    mcap = any(path.glob('*.mcap')) or (metadata.exists()
                                         and 'storage_identifier: mcap' in metadata.read_text())
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap' if mcap else 'sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader, {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def read_raw(path, topics=None, start_ns=None, end_ns=None, opener=open_segment):
    """Yield ``(topic, type, data, t_ns)`` over all segments, ``start_ns <= t < end_ns``.

    ``topics`` filters by name (None keeps all). A segment that cannot be
    opened (a crashed one with no metadata) is skipped, not fatal.
    """
    wanted = None if topics is None else set(topics)
    for segment in segments(path):
        try:
            reader, types = opener(segment)
        except Exception:  # noqa: BLE001 - rosbag2 raises RuntimeError for a broken segment
            continue
        if start_ns is not None and hasattr(reader, 'seek'):
            reader.seek(int(start_ns))
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            if wanted is not None and topic not in wanted:
                continue
            if start_ns is not None and t_ns < start_ns:
                continue
            if end_ns is not None and t_ns >= end_ns:
                break
            yield topic, types.get(topic, ''), data, t_ns


def read(path, topics=None, start_ns=None, end_ns=None, opener=open_segment, decode=None):
    """Like ``read_raw`` but yields ``(topic, message, t_ns)`` with decoded messages."""
    if decode is None:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        def decode(kind, data):
            return deserialize_message(data, get_message(kind))
    for topic, kind, data, t_ns in read_raw(path, topics, start_ns, end_ns, opener):
        yield topic, decode(kind, data), t_ns


def bounds(path, opener=open_segment):
    """``(first_ns, last_ns)`` over the session from its segment metadata, else None."""
    import yaml
    starts, ends = [], []
    for segment in segments(path):
        try:
            info = yaml.safe_load((segment / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
            start = info['starting_time']['nanoseconds_since_epoch']
            starts.append(start)
            ends.append(start + info['duration']['nanoseconds'])
        except (OSError, KeyError, TypeError, yaml.YAMLError):
            continue
    return (min(starts), max(ends)) if starts else None


class BagSource:
    """A playhead over a session: seek, play at a rate, ingest what it passes.

    ``reset()`` clears the consumer (the monitor); ``ingest(topic, msg, t_s)``
    feeds one message. A seek resets and replays ``[t - window, t]`` so the
    consumer's history is warm; ``tick(dt)`` then reads forward from the
    playhead (re-reading the bag from there, so it never keeps a file open).
    """

    def __init__(self, path, topics, ingest, reset, window_s=WINDOW_S, rate=1.0, span=None, **reader_args):
        self.path, self.topics = Path(path), tuple(topics)
        self.ingest, self.reset, self.window_s, self.rate = ingest, reset, window_s, rate
        self.reader_args = reader_args
        self.span = span or bounds(self.path)
        self.t_ns = self.span[0] if self.span else 0
        self.playing = False

    def _feed(self, start_ns, end_ns):
        for topic, message, t_ns in read(self.path, self.topics, start_ns, end_ns, **self.reader_args):
            self.ingest(topic, message, t_ns / 1e9)

    def seek(self, t_ns):
        """Move the playhead (clamped to the session) and rebuild the window."""
        if self.span:
            t_ns = min(max(t_ns, self.span[0]), self.span[1])
        self.t_ns = int(t_ns)
        self.reset()
        self._feed(self.t_ns - int(self.window_s * 1e9), self.t_ns + 1)

    def play(self, playing=True):
        self.playing = playing

    def set_rate(self, rate):
        self.rate = max(float(rate), 0.0)

    def tick(self, dt):
        """Advance by ``rate * dt`` seconds while playing; returns the new playhead."""
        if not self.playing:
            return self.t_ns
        end = self.t_ns + int(self.rate * dt * 1e9)
        if self.span and end >= self.span[1]:
            end, self.playing = self.span[1], False
        self._feed(self.t_ns + 1, end + 1)
        self.t_ns = end
        return end
