"""Structural guard always runs; request a serialized timing window before CAMERA_BENCH=1."""
import io
import os

import numpy as np
import pytest
from PIL import Image

from bench_camera import benchmark, fixture, require_budget, require_fast_octree
from mhseals_testing.ui.camera_view import encode_png


def test_camera_benchmark_guards_actual_production_method():
    array, _ = fixture('natural')
    require_fast_octree(encode_png, array)

    def default_method(array):
        out = io.BytesIO()
        Image.fromarray(array).quantize(64).save(out, 'PNG')
        return out.getvalue()

    with pytest.raises(AssertionError):
        require_fast_octree(default_method, array)
    with pytest.raises(AssertionError):
        require_budget({'fixtures': {'default_mutation': {'median_ratio': 1.0}}})
    assert array.shape == (270, 480, 3) and array.dtype == np.uint8
    assert np.array_equal(array, fixture('natural')[0])


@pytest.mark.skipif(os.environ.get('CAMERA_BENCH') != '1', reason='requires exclusive timing window')
def test_camera_production_median_is_at_most_half_explicit_mediancut():
    import json
    report = benchmark()
    print(json.dumps(report, indent=2), flush=True)
    require_budget(report)
