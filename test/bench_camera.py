"""Serial camera guard: PYTHONPATH=.:test python test/bench_camera.py > camera.json.

Fixtures follow the field pipeline: seeded 1280x720 JPEG -> 480x270 -> eight
one-pixel boxes. Paired serializers receive the identical prepared RGB array.
Run only in an exclusive host timing window; this is not an ODROID measurement.
"""
import io
import json
import platform
from pathlib import Path
import statistics
import time
from unittest.mock import patch

import numpy as np
from PIL import Image, __version__ as pillow_version

from mhseals_testing.ui import camera_view

SEED = 20261023
SAMPLES = 25


def fixture(kind):
    rng = np.random.default_rng(SEED)
    if kind == 'noise':
        rgb = rng.integers(0, 256, (720, 1280, 3), dtype=np.uint8)
    else:
        y, x = np.mgrid[0:720, 0:1280]
        # Sky, distant shoreline, rippled water and distinct coloured buoy shapes.
        water = y > 310 + 12 * np.sin(x / 85)
        rgb = np.stack([80 + 50 * y / 720, 145 + 35 * x / 1280,
                        190 + 30 * np.sin(x / 160)], axis=-1)
        rgb[water] *= np.array([.45, .65, .72])
        rgb += rng.normal(0, 7, rgb.shape)
        for i, colour in enumerate([(235, 30, 25), (25, 200, 40), (240, 210, 20)]):
            rgb[350:430, 200 + i * 300:240 + i * 300] = colour
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    jpeg = io.BytesIO()
    Image.fromarray(rgb).save(jpeg, 'JPEG', quality=85)
    array = camera_view.scale(camera_view.decode_jpeg(jpeg.getvalue()), 480, 270).copy()
    for i in range(8):
        x, y = 12 + i * 54, 25 + i % 3 * 65
        camera_view.draw_box(array, (x, y, x + 30, y + 45))
    return array, len(jpeg.getvalue())


def mediancut(array):
    out = io.BytesIO()
    Image.fromarray(array).quantize(64, method=Image.Quantize.MEDIANCUT).save(out, 'PNG')
    return out.getvalue()


def default_quantize(array):
    """Mutation control: omission of method restores Pillow's MEDIANCUT default."""
    out = io.BytesIO()
    Image.fromarray(array).quantize(64).save(out, 'PNG')
    return out.getvalue()


def require_fast_octree(encoder, array):
    calls = []
    original = Image.Image.quantize

    def spy(self, colors=256, method=None, **kwargs):
        calls.append((colors, method))
        return original(self, colors, method=method, **kwargs)

    with patch.object(Image.Image, 'quantize', spy):
        encoder(array)
    assert calls == [(64, Image.Quantize.FASTOCTREE)], calls


def measure(encoder, array):
    start = time.perf_counter_ns()
    png = encoder(array)
    return (time.perf_counter_ns() - start) / 1e6, len(png)


def summary(times, sizes):
    return {'raw_ms': times, 'median_ms': statistics.median(times),
            'p95_ms': sorted(times)[int(np.ceil(.95 * len(times))) - 1],
            'max_ms': max(times), 'bytes': sizes}


def benchmark(samples=SAMPLES, production=camera_view.encode_png):
    assert samples >= 20
    results = {'environment': {'python': platform.python_version(), 'pillow': pillow_version,
                              'numpy': np.__version__, 'machine': platform.machine(),
                              'platform': platform.platform(), 'processor': platform.processor(),
                              'cpu_model': next((line.split(':', 1)[1].strip()
                                  for line in Path('/proc/cpuinfo').read_text().splitlines()
                                  if line.startswith('model name')), 'unknown')},
               'seed': SEED, 'samples': samples, 'timed_stage': 'production PNG serializer',
               'fixture': '1280x720 JPEG quality85 -> 480x270 nearest -> 8 one-pixel boxes',
               'fixtures': {}}
    for kind in ('noise', 'natural'):
        array, jpeg_bytes = fixture(kind)
        require_fast_octree(production, array)
        for encoder in (production, mediancut, default_quantize):
            encoder(array)  # warm each path without retaining a best-of-retries result
        timings, sizes = [[], [], []], [[], [], []]
        for i in range(samples):
            # Rotate paired order to avoid always charging one path first.
            for j in ((i + offset) % 3 for offset in range(3)):
                elapsed, length = measure((production, mediancut, default_quantize)[j], array)
                timings[j].append(elapsed)
                sizes[j].append(length)
        fast, reference, default = (summary(timings[j], sizes[j]) for j in range(3))
        results['fixtures'][kind] = {'jpeg_bytes': jpeg_bytes, 'shape': list(array.shape),
                                    'production': fast, 'mediancut': reference, 'default_mutation': default,
                                    'default_median_ratio': default['median_ms'] / reference['median_ms'],
                                    'median_ratio': fast['median_ms'] / reference['median_ms']}
    return results


def require_budget(results):
    for kind, result in results['fixtures'].items():
        assert result['median_ratio'] <= .5, (kind, result)
        assert result['default_median_ratio'] > .5, (kind, result)


if __name__ == '__main__':
    report = benchmark()
    print(json.dumps(report, indent=2), flush=True)
    require_budget(report)  # emit every raw sample even when the guard fails
