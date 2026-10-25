"""Segment reading and BagSource: fake-reader tests run anywhere, the written bag needs ROS."""

import pytest

from mhseals_testing.harness import bagread

S = 1_000_000_000


class FakeReader:
    def __init__(self, rows):
        self.rows, self.index = rows, 0

    def has_next(self):
        return self.index < len(self.rows)

    def read_next(self):
        self.index += 1
        return self.rows[self.index - 1]


def session(tmp_path, layout):
    """Make bag_NN dirs; returns an opener serving ``layout[name]`` rows."""
    for name in layout:
        (tmp_path / name).mkdir()
        (tmp_path / name / 'metadata.yaml').write_text('x')
    return lambda segment: (FakeReader(layout[segment.name]), {'/a': 'T', '/b': 'T'})


def test_segments_are_ordered_and_a_single_bag_is_itself(tmp_path):
    session(tmp_path, {'bag_01': [], 'bag_00': []})
    assert [p.name for p in bagread.segments(tmp_path)] == ['bag_00', 'bag_01']
    assert bagread.segments(tmp_path / 'bag_00') == [tmp_path / 'bag_00']


def test_window_spans_a_segment_boundary_and_filters_topics(tmp_path):
    opener = session(tmp_path, {
        'bag_00': [('/a', b'', 1 * S), ('/b', b'', 2 * S), ('/a', b'', 3 * S)],
        'bag_01': [('/a', b'', 5 * S), ('/a', b'', 7 * S)]})
    got = [t for _, _, _, t in bagread.read_raw(tmp_path, ['/a'], 2 * S, 7 * S, opener)]
    assert got == [3 * S, 5 * S]


def test_broken_segment_is_skipped(tmp_path):
    good = session(tmp_path, {'bag_00': [('/a', b'', S)], 'bag_01': [('/a', b'', 2 * S)]})

    def opener(segment):
        if segment.name == 'bag_00':
            raise RuntimeError('no metadata')
        return good(segment)
    assert [t for *_, t in bagread.read_raw(tmp_path, None, opener=opener)] == [2 * S]


def test_bag_source_seek_resets_and_plays_at_rate(tmp_path):
    rows = [('/a', b'', i * S) for i in range(1, 31)]
    opener = session(tmp_path, {'bag_00': rows[:15], 'bag_01': rows[15:]})
    # The fake reader cannot be rewound, so open a fresh one per read.
    fresh = lambda segment: (FakeReader(list(opener(segment)[0].rows)), {'/a': 'T'})
    seen, resets = [], []
    source = bagread.BagSource(tmp_path, ['/a'], lambda *m: seen.append(m[2]),
                               lambda: (resets.append(1), seen.clear()),
                               opener=fresh, decode=lambda kind, data: data, span=(S, 30 * S))
    source.seek(16 * S)
    assert resets and seen == [float(t) for t in range(6, 17)]  # 10 s back, across bag_00/bag_01
    source.play()
    source.set_rate(2.0)
    seen.clear()
    assert source.tick(1.0) == 18 * S and seen == [17.0, 18.0]
    source.seek(10 ** 12)
    assert source.t_ns == 30 * S
    source.tick(5.0)
    assert source.t_ns == 30 * S and not source.playing


def test_written_bag_reads_across_segments_and_seeks(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from std_msgs.msg import Int32

    def write(name, stamps):
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(tmp_path / name), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name='/n', type='std_msgs/msg/Int32', serialization_format='cdr'))
        for stamp in stamps:
            writer.write('/n', serialize_message(Int32(data=stamp)), stamp * S)
        del writer
    write('bag_00', range(1, 6))
    write('bag_01', range(6, 11))
    assert [m.data for _, m, _ in bagread.read(tmp_path, ['/n'], 4 * S, 8 * S)] == [4, 5, 6, 7]
    seen = []
    source = bagread.BagSource(tmp_path, ['/n'], lambda t, m, s: seen.append(m.data), seen.clear,
                               window_s=3)
    source.seek(7 * S)
    assert seen == [4, 5, 6, 7]


def test_segments_find_recorder_layout_session_bag_nn(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    session(bag, {'01': [], '00': []})
    (bag / 'notes').mkdir()
    expect = [bag / '00', bag / '01']
    assert bagread.segments(tmp_path) == expect
    assert bagread.segments(bag) == expect


TYPES = {'/a': 'std_msgs/msg/String', '/zed': 'zed_msgs/msg/ObjectsStamped'}
ROWS = [('/zed', b'z1', S), ('/a', b'a1', 2 * S), ('/zed', b'z2', 3 * S)]


def zed_less_read(tmp_path, topics=None):
    """read() on a host without zed_msgs."""
    session(tmp_path, {'bag_00': ROWS})

    def opener(segment):
        return FakeReader(list(ROWS)), TYPES

    def lookup(kind):
        if kind.startswith('zed_msgs/'):
            raise ModuleNotFoundError("No module named 'zed_msgs'")
        return kind

    def deserialize(data, cls):
        return cls, data
    decode = bagread.ros_decoder(lookup, deserialize)
    return list(bagread.read(tmp_path, topics, opener=opener, decode=decode))


def test_a_type_this_host_cannot_import_is_skipped_with_one_warning(tmp_path):
    with pytest.warns(UserWarning, match='zed_msgs/msg/ObjectsStamped') as caught:
        got = zed_less_read(tmp_path)
    assert got == [('/a', ('std_msgs/msg/String', b'a1'), 2 * S)]
    assert len(caught) == 1


def test_a_named_topic_that_cannot_be_decoded_raises(tmp_path):
    with pytest.raises(LookupError, match='/zed'):
        zed_less_read(tmp_path, ['/a', '/zed'])
