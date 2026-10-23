"""Sim detection boxes land on the buoys (ROS-free fixture), plus a ROS-gated live check."""

import json
import os
import time
from pathlib import Path

import numpy as np
import pytest

from mhseals_testing.perception.project import to_frame, to_optical
from mhseals_testing.ui.overlay import box_corners, rect_3d

FRAME = json.loads((Path(__file__).parent / 'fixtures' / 'crane_frame.json').read_text())
K = [500, 0, 320, 0, 500, 240, 0, 0, 1]


def rects():
    tf = to_optical(FRAME['tf_static']['translation'], FRAME['tf_static']['quaternion'])
    return [rect_3d(to_frame(box_corners(d['center'], d['size']), tf), FRAME['K'], FRAME['image'])
            for d in FRAME['detections']]


def test_boxes_land_on_the_buoys():
    for rect, buoy in zip(rects(), FRAME['buoys']):
        u, v = buoy['pixel']
        assert rect[0] < u < rect[2] and rect[1] < v < rect[3], buoy['name']
        assert abs((rect[0] + rect[2]) / 2 - u) < 3 and abs((rect[1] + rect[3]) / 2 - v) < 3


def test_farther_buoys_draw_smaller():
    widths = [r[2] - r[0] for r in rects()]
    assert widths == sorted(widths, reverse=True)


def test_to_optical_inverts_the_static_transform():
    rot, trans = to_optical([1, 2, 3], [0, 0, 0, 1])
    assert to_frame([[1, 2, 3]], (rot, trans)) == pytest.approx(np.zeros((1, 3)))


def test_non_finite_detection_has_no_rect():
    tf = to_optical([0, 0, 0], [-0.5, 0.5, -0.5, 0.5])
    assert rect_3d(to_frame(box_corners([float('nan'), 0, 5], [1, 1, 1]), tf), K, (640, 480)) is None


@pytest.mark.skipif(not os.environ.get('MHS_CRANE'), reason='needs a running crane_ml (set MHS_CRANE=1)')
def test_live_sim_detections_project_into_the_image():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    from tf2_msgs.msg import TFMessage
    from vision_msgs.msg import Detection3DArray

    rclpy.init()
    node = Node('ts140_perception_check')
    got, static = {}, {}
    latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    try:
        node.create_subscription(CameraInfo, '/front_camera/camera_info',
                                 lambda m: got.setdefault('info', m), qos_profile_sensor_data)
        node.create_subscription(Detection3DArray, '/detections',
                                 lambda m: m.detections and got.setdefault('dets', m), 10)
        node.create_subscription(TFMessage, '/tf_static', lambda m: static.update(
            {(t.header.frame_id, t.child_frame_id): t.transform for t in m.transforms}), latched)
        deadline = time.time() + 15
        while len(got) < 2 and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        assert len(got) == 2, 'no camera_info and non-empty /detections from the sim'
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        src, dst = got['dets'].header.frame_id, got['info'].header.frame_id
        if src == dst:
            tf = (np.eye(3), np.zeros(3))
        elif (src, dst) in static:
            t = static[(src, dst)]
            tf = to_optical([t.translation.x, t.translation.y, t.translation.z],
                            [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        else:
            pytest.skip(f'no /tf_static entry {src} -> {dst}')
        size = (got['info'].width, got['info'].height)
        hits = 0
        for d in got['dets'].detections:
            c, s = d.bbox.center.position, d.bbox.size
            corners = to_frame(box_corners([c.x, c.y, c.z], [s.x, s.y, s.z]), tf)
            hits += rect_3d(corners, list(got['info'].k), size) is not None
        assert hits, 'no detection projects into the image'
    finally:
        node.destroy_node()
        rclpy.shutdown()
