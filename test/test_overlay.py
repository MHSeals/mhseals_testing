import numpy as np

from mhseals_testing.ui import overlay

K = [700.0, 0.0, 640.0, 0.0, 700.0, 360.0, 0.0, 0.0, 1.0]
SIZE = (1280, 720)


def test_known_box_gives_expected_rect():
    # 1 m cube, centre 10 m ahead and 2 m right: x in [1.5, 2.5], y in [-0.5, 0.5], z in [9.5, 10.5]
    rect = overlay.rect_3d(overlay.box_corners([2, 0, 10], [1, 1, 1]), K, SIZE)
    # u: 640 + 700*1.5/10.5 = 740 (far, inner) .. 640 + 700*2.5/9.5 = 824 (near, outer); v: 360 -+ 700*0.5/9.5
    assert rect == (740, 323, 824, 397)


def test_project_principal_point():
    assert overlay.project([[0, 0, 5]], K).tolist() == [[640.0, 360.0]]


def test_box_behind_camera_has_no_rect():
    assert overlay.rect_3d(overlay.box_corners([0, 0, 0.2], [1, 1, 1]), K, SIZE) is None


def test_rect_is_clipped_and_empty_dropped():
    assert overlay.rect_2d((-20, 10, 100, 2000), SIZE) == (0, 10, 100, 720)
    assert overlay.rect_2d((1300, 10, 1400, 50), SIZE) is None


def test_scale_for_reduced_decode():
    assert overlay.scale_rect((740, 323, 824, 397), 0.5) == (370, 162, 412, 198)


def test_non_finite_input_has_no_rect():
    nan, inf = float("nan"), float("inf")
    assert overlay.rect_3d(overlay.box_corners([nan, 0, 10], [1, 1, 1]), K, SIZE) is None
    assert overlay.rect_3d(overlay.box_corners([0, 0, inf], [1, 1, 1]), K, SIZE) is None
    assert overlay.rect_2d((0, 0, nan, 50), SIZE) is None
    assert overlay.rect_2d((0, 0, inf, 50), SIZE) is None
