"""The Fusion tab: camera-lidar alignment at a glance (perception-calibration.md 2, lidar-fusion-design).

Three panels from ``ros.fusion_feed.FusionFeed.snapshot()`` plus the fusion status facts:
- **Camera**: the scan projected into the image (with lidar_fusion's applied correction) coloured by
  range on the spd ramp, over the camera frame in grey while the image is on (``i``; image tiers only: the
  frame is drawn with the boxes and scan of its own stamp, so all three are what the camera saw), with each box in its class
  colour and its range label (``L`` lidar, ``R`` ray mode, ``H`` held, ``S`` stereo, then the stereo
  range when it differs). A misaligned extrinsic shows as points sitting off their boxes.
- **Around**: top-down in camera axes (x right, z ahead): FOV, range rings, the scan, each box at its
  fused position with a +-2 sigma range bar, a line to its stereo position and its range +-2 sigma;
  ray hits are diamonds.
- **Alignment**: the live check (fusion_calib's check rule over the live samples), the applied and
  estimated correction, the du/dv pixel residuals with their trend, box sources and fusion health.

One paint per drawing serves every tier: an image tier paints a pixels.Raster at the cell size and
shows its tiles; text paints the same Raster at braille resolution (2 x 4 dots a cell), each cell in
its highest layer's colour; ASCII uses one character per layer. Labels are terminal text (holes).
Pure: no ROS, no clock.

The package: ``common`` (tokens, sizes, the one paint), ``camera``, ``topdown`` (Around) and ``residual``
(Alignment); this module lays the tab out and re-exports every name the one-file fusion.py had.
"""

# Every name fusion.py had stays fusion.X (unused here: noqa F401 throughout).
import math

import numpy as np
from rich.console import Group
from rich.layout import Layout
from rich.text import Text

from mhseals_testing.ui import palette, theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.panels.fusion.common import (  # noqa: F401
    AROUND_MIN, AROUND_SHARE, ASCII, BITS, CLASS_TOKENS, DEG, GREYS, Inks, LINK_BLOCKS, LINK_SHARE, OVER_S, PM,
    RAMP, RATE_S, READOUT_ROWS, SOURCE_MARK, SPARK_COLUMNS, STALE, STAMP_MATCH_S, START, STEP, TEXT_CELL, _HZ,
    _LABELS, _TEXT, _WEIGHTS, class_token, draw, draw_rate, ink_style, label_cells, link_block, link_stride,
    optical_xz, ramp_index, range_label, scan_stale, text_cells)
from mhseals_testing.ui.panels.fusion.camera import (  # noqa: F401
    JOIN_STEPS, _JOINED, _STARTED, block_mean, camera_layers, camera_model, camera_rows, camera_subtitle,
    frame_shown, image_note, paint_camera, paint_frame)
from mhseals_testing.ui.panels.fusion.topdown import (  # noqa: F401
    FAN_MARGIN, around_labels, around_rows, fan, half_fov, paint_around, reach)
from mhseals_testing.ui.panels.fusion.residual import (  # noqa: F401
    _READOUT, _readout, check, fusion_status, median, readout, spark, trend)


# --- the tab --------------------------------------------------------------------------------------

def camera_size(model, max_cols, max_rows):
    """(cols, rows) of the camera drawing that keeps the image aspect in a terminal cell's shape."""
    w, h = model.get('size') or (16, 9)
    cw, ch = pixels.CELL if pixels.active() else (1, 2)
    rows = max(3, max_rows)
    cols = round(rows * ch * w / (h * cw))
    if cols > max_cols:
        cols, rows = max_cols, max(3, min(rows, round(max_cols * h * cw / (w * ch))))
    return max(8, cols), rows


_CFG = []


def config():
    """fusion_panel.yaml, loaded once."""
    if not _CFG:
        from mhseals_testing.fusion import load_config
        _CFG.append(load_config())
    return _CFG[0]


def body(view, width, height, cfg=None):
    """The tab's body; ``view['fusion']`` is the feed snapshot (None: no ROS node in this session)."""
    cfg = cfg or config()
    model = view.get('fusion')
    focused = view.get('focus') == 'fusion'
    if model is None:
        why = ('Fusion is live only (not in review)' if view.get('fusion_review')
               else 'Fusion needs a ROS node (none in this session)')
        return theme.block(Text(why, style=theme.MUTED), 'Camera', focused)
    status = fusion_status(view.get('fusion_facts'))
    readout_rows = READOUT_ROWS
    side = max(AROUND_MIN, int(width * AROUND_SHARE))
    cam_cols, cam_rows = camera_size(model, width - side - 8, max(3, height - readout_rows - 2))
    around = (width - cam_cols - 8, height - 2)
    sub = camera_subtitle(model, cfg)
    camera = theme.block(Text('\n').join(camera_rows(model, cfg, cam_cols, cam_rows)), 'Camera', focused, sub)
    around_panel = theme.block(Text('\n').join(around_rows(model, cfg, *around)), 'Around', False)
    align = theme.block(Group(*readout(model, cfg, status, cam_cols)), 'Alignment', False)
    left = Layout(size=cam_cols + 4)
    left.split_column(Layout(camera, size=cam_rows + 2), Layout(align))
    page = Layout()
    page.split_row(left, Layout(around_panel))
    return page
