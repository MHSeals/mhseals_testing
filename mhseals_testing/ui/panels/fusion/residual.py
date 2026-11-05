"""The Alignment panel: the live check, the applied and estimated correction, the du/dv residuals with their
trend, box sources and fusion health."""

import math

from rich.text import Text

from mhseals_testing.ui import palette, theme
from mhseals_testing.ui import imaging as pixels
from mhseals_testing.ui.panels.fusion.common import DEG, SPARK_COLUMNS


# --- Alignment ------------------------------------------------------------------------------------

def median(values):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    return values[len(values) // 2] if values else None


def check(model, cfg):
    """``(level, reason)``: fusion_calib's check rule over the live samples; level None while collecting.

    Hits and L/S come from the feed's ``grades``: the near boxes lidar_fusion samples (stereo < near, a sample
    class, clear of the edge, isolated) of the frames the fusion processed, pooled per object over the residual
    window; an object fails only if it never got a hit. With no graded box at all the check WARNs (``near 0``)."""
    c = cfg['check']
    res = model['residuals']
    if len(res) < c['min_samples']:
        return None, f'collecting {len(res)}/{c["min_samples"]} samples'
    du, dv = median([r[1] for r in res]), median([r[2] for r in res])
    rank = ['PASS', 'WARN', 'FAIL']
    level = 'PASS'

    def worse(lv):
        nonlocal level
        level = max(level, lv, key=rank.index)
    for name, off in (('du', du), ('dv', dv)):     # dv: vertical (pitch) offset, same limits
        if abs(off) > c[name + '_warn_px']:
            worse('FAIL')
        elif abs(off) > c[name + '_pass_px']:
            worse('WARN')
    why = [f'du {du:+.1f} dv {dv:+.1f} px']
    objects = {}                  # backend id -> ([hit], [L/S]) over the window, as fusion_calib's per-object medians
    for _, key, hit, ratio in model.get('grades', ()):
        hits, ratios = objects.setdefault(key, ([], []))
        hits.append(hit)
        if ratio is not None:
            ratios.append(ratio)
    if not objects:               # no near box paired with its stereo array: du/dv alone is not fusion_calib's check
        why.append('near 0')
        worse('WARN')
    else:
        hit = [o for o in objects.values() if any(o[0])]
        why.append(f'near {len(hit)}/{len(objects)}')
        if len(hit) < len(objects):
            worse('FAIL')         # an object that never got a hit
        elif any(sum(h) / len(h) < c['hit_rate_min'] for h, _ in hit):
            worse('WARN')
    ratio = max((median(r) for _, r in objects.values() if r), key=lambda v: abs(v - 1), default=None)
    if ratio is not None:
        why.append(f'L/S {ratio:.2f}')
        if abs(ratio - 1) > c['range_fail_tol']:
            worse('FAIL')
        elif abs(ratio - 1) > c['range_tol']:
            worse('WARN')
    return level, theme.SEPARATOR.join(why)


def spark(values, scale):
    bars = theme.SPARK
    values = values[-SPARK_COLUMNS:]
    if not values:
        return theme.MIDDOT * SPARK_COLUMNS
    return ''.join(bars[min(len(bars) - 1, int(abs(v) / scale * (len(bars) - 1)))] for v in values).rjust(SPARK_COLUMNS)


def trend(res, index, buckets=SPARK_COLUMNS):
    """Median |residual| per time bucket over the window, oldest first."""
    if not res:
        return []
    t0 = min(r[0] for r in res)
    step = (0 - t0) / buckets or 1.0
    groups = [[] for _ in range(buckets)]
    for r in res:
        groups[min(buckets - 1, int((r[0] - t0) / step))].append(r[index])
    return [median(g) for g in groups if g]


def readout(model, cfg, status, width=None):
    """Alignment's lines; at ``width`` the check drops ``live`` then its du/dv (the rows below show them), and the
    correction drops its axis letters, then puts the estimate on its own line. Memoised on the data it reads (the
    window's ends, not the version, which every scan bumps) and recomputed each second for the trend's buckets."""
    res, grades = model['residuals'], model.get('grades') or [()]
    key = (len(res), res[-1][1:] if res else None, res[0][1:] if res else None, len(grades), grades[-1][1:],
           grades[0][1:], model.get('applied'), width, tuple(sorted(status.items())), theme.ASCII,
           palette.detect_tier(), int(pixels.clock()))
    if _READOUT and _READOUT[0] == key:
        return _READOUT[1]
    lines = _readout(model, cfg, status, width)
    _READOUT[:] = [key, lines]
    return lines


_READOUT = []


def _readout(model, cfg, status, width):
    width = width or 1 << 30
    level, why = check(model, cfg)
    style = {'PASS': theme.GOOD, 'WARN': theme.WARN, 'FAIL': theme.BAD}.get(level, theme.MUTED)
    head = Text.assemble(('check ', theme.MUTED), (level or theme.MIDDOT, f'bold {style}'.strip()), ' ')
    short = why.split(theme.SEPARATOR, 1)[1] if level and theme.SEPARATOR in why else why
    for tail in (f'{why}{theme.SEPARATOR}live', why, short):
        if len(head) + len(tail) <= width:
            break
    lines = [Text.assemble(head, (tail, theme.MUTED))]
    applied, est = status.get('applied_deg'), status.get('extrinsic_deg')
    stale = not applied and any(model.get('applied') or ())     # the feed keeps the last one for the points
    applied = applied or (model.get('applied') if stale else None)
    if applied:
        est_text = f'est {" ".join(f"{v:+.2f}" for v in est)}' if est else ''
        for names in (('ox', 'oy', 'oz'), ('', '', '')):      # optical axes: +oy is down, so +oy turns starboard
            corr = Text('corr  ', style=theme.MUTED)
            corr.append(' '.join(f'{n} {v:+.2f}'.strip() for n, v in zip(names, applied)) + DEG)
            if stale:
                corr.append(' stale', style=theme.WARN)
            if len(corr) + len(theme.SEPARATOR) + len(est_text) <= width:
                break
        if est_text and len(corr) + len(theme.SEPARATOR) + len(est_text) <= width:
            corr.append(f'{theme.SEPARATOR}{est_text}', style=theme.MUTED)
            lines.append(corr)
        else:
            lines += [corr, Text(f'      {est_text}', style=theme.MUTED)] if est_text else [corr]
    else:
        lines.append(Text('corr  none applied', style=theme.MUTED))
    res = model['residuals']
    scale = cfg['check']['du_warn_px']
    for name, index in (('du', 1), ('dv', 2)):
        values = trend(res, index)
        mid = median([r[index] for r in res])
        hot = mid is not None and abs(mid) > cfg['check'][name + '_pass_px']
        lines.append(Text.assemble((f'{name}    ', theme.MUTED), (spark(values, scale), theme.WARN if hot else theme.FG),
                                   f' {mid:+5.1f} px' if mid is not None else f' {theme.MIDDOT}',
                                   (' raw' if mid is not None else '', theme.MUTED)))   # before the applied corr
    counts = {}
    for det in model['dets']:
        counts[det['source']] = counts.get(det['source'], 0) + 1
    lines.append(Text.assemble(('boxes ', theme.MUTED), theme.SEPARATOR.join(
        f'{"ray" if s == "lidar_ray" else s} {n}' for s, n in sorted(counts.items())) or theme.MIDDOT))
    if not model['fused']:
        lines.append(Text('fusion off: stereo ranges only', style=theme.MUTED))
    else:
        state = status.get('state') or 'stale'
        hue = {'ok': theme.GOOD, 'warn': theme.WARN, 'error': theme.BAD}.get(state, theme.MUTED)
        parts = [f"{status['hz']:.1f} Hz"] if status.get('hz') is not None else []
        parts += [f"lidar {status['share']:.0%}"] if status.get('share') is not None else []
        lines.append(Text.assemble(('fusion ', theme.MUTED), (state, hue), (theme.SEPARATOR, theme.MUTED),
                                   theme.SEPARATOR.join(parts)))
    for line in lines:
        line.no_wrap, line.overflow = True, theme.OVERFLOW
    return lines


def fusion_status(facts):
    """The applied/estimated correction, state, rate and lidar share from ``perc.<src>.fusion_*`` facts."""
    out = {}
    for name, fact in (facts or {}).items():
        if not name.startswith('perc.'):
            continue
        key = name.rsplit('.', 1)[-1]
        if key == 'fusion_state':
            out['state'] = fact.value
            for field in ('applied_deg', 'extrinsic_deg'):
                try:
                    out[field] = tuple(float(v) for v in str(fact.data.get(field, '')).split())[:3] or None
                except ValueError:
                    pass
        elif key == 'fusion_hz':
            out['hz'] = fact.value
        elif key == 'fusion_share':
            out['share'] = fact.value
    return out
