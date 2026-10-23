"""Correction tab: the six-direction thrust correction table (ui spec 6.7, thrust.md section 6).

Pure Rich, no ROS. ``correction_view`` turns the mixer, the C+/C- columns and the per-direction
fit records into rows of strings; ``render`` draws them at a width and height; ``details`` is the
holds popup for the cursor column. Columns are surge+ surge- sway+ sway- yaw+ yaw-; the chain the
table shows is thrust matrix -> C (driver adds) -> speed curves (controller inverts).
"""
from __future__ import annotations

import math
import os
from functools import lru_cache
from pathlib import Path

import yaml
from rich.console import Group
from rich.table import Table
from rich.text import Text

from mhseals_testing import calibration
from mhseals_testing.ui import fitrows, theme
from mhseals_testing.ui.theme import MUTED, WARN

AXES = ('surge', 'sway', 'yaw')
CROSS = ('sw', 'ψ', 'su')   # cross-entry rows: sway, yaw, surge (the axis index of each is below)
CROSS_AXIS = (1, 2, 0)
DIRECTIONS = tuple(f'{axis}{sign}' for axis in AXES for sign in '+-')
SCOPE = 'italic bright_black'   # the slate hw / cal tag
MISSING = theme.DASH


def load_settings(path=None) -> dict:
    """correction.yaml, source tree first, then the installed share directory."""
    source = Path(__file__).resolve().parents[3] / 'config' / 'correction.yaml'
    if path is None and not source.exists():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('mhseals_testing')) / 'config' / 'correction.yaml'
    return yaml.safe_load(Path(path or source).read_text())


def _square(flat):
    """A flat row-major 9-list as 3x3 rows; None when the config has none."""
    return [list(flat[i:i + 3]) for i in (0, 3, 6)] if flat and len(flat) == 9 else None


def correction_data(hardware, fits=None, running=None, side=None):
    """The ``data`` of ``correction_view`` from the hardware config and the state.json fit records.

    C+/C- and the thruster matrix come from ``hardware``; ``fits`` is {direction: record}, to which
    the speed curves of ``hardware['characterization']`` (d1, d2) are added when a record has none.
    """
    result = hardware.get('characterization') or {}
    fits = {k: dict(v) for k, v in (fits or {}).items()}
    for key, curve in (result.get('speed_curves') or {}).items():
        fit = fits.setdefault(key, {})
        for name in ('d1', 'd2'):
            fit.setdefault(name, curve.get(name))
    stamp = str(result.get('identified_at') or '')
    fitted = f'{stamp[5:10]} {stamp[11:16]}' if len(stamp) >= 16 else None
    flat = hardware.get('thruster_matrix')   # the config holds 12 values; a list of rows also passes
    matrix = [flat[i:i + 3] for i in range(0, 12, 3)] if flat and not hasattr(flat[0], '__len__') else flat
    return dict(matrix=matrix, c_positive=_square(hardware.get('decoupling')),
                c_negative=_square(hardware.get('decoupling_negative')), fits=fits,
                header=dict(fitted=fitted, activity=running), **(side or {}))


def readiness_data(summary, blocking=(), warnings=(), floor=None):
    """The Readiness box model from ``Conditions.summary`` and ``engine.gate('characterize')``.

    Rows are drift (|d| m/s), floor (the wave floor, yaw/surge/sway) and ceiling; each gate reason
    becomes a line with its status glyph (blocking fail, warning warn, none: one good line).
    """
    drift, ceiling = summary.get('drift'), summary.get('ceiling')
    rows = [('drift', MISSING if not drift else _num(math.hypot(*drift), 3, False) + ' m/s'),
            ('floor', MISSING if not floor else ' '.join(_num(x, 3, False) for x in floor)),
            ('ceiling', MISSING if ceiling is None else f'{ceiling * 100:.0f}%')]
    gates = [('bad', r) for r in blocking] + [('warn', r) for r in warnings] or [('good', 'gates clear')]
    return dict(verdict=summary.get('verdict', MISSING), rows=rows, gates=gates)


def tracking_data(scores, sessions, provenance, version=None):
    """The Tracking box model: per axis the live score, this session's mean and the stored trend.

    ``scores`` are the feed's live scores (surge, sway, yaw; None gated), ``sessions`` its
    {axis: [score sum, windows, trust sum]}, ``provenance`` calibration/current.json's
    ``provenance.tracking``. The staleness line names the axes whose verdict is flagged.
    """
    pct = lambda v: MISSING if v is None else f'{v * 100:.0f}%'   # noqa: E731
    rows, stale = [], []
    for axis, now in zip(AXES, scores):
        total, n, _ = sessions.get(axis, (0, 0, 0))
        entry = (provenance.get(axis) or {}).get('staleness') or {}
        slope = entry.get('slope')
        trend = MISSING if slope is None else theme.pick('↘' if slope < 0 else '↗', '-' if slope < 0 else '+')
        rows.append((axis, pct(now), pct(total / n if n else None), trend))
        if entry.get('flag'):
            stale.append(f"{axis} {entry.get('reason', '')}".strip())
    line = 'stale: ' + ', '.join(stale) if stale else 'calibration current'
    return dict(rows=rows, notes=[line], version=version or '')


def _num(value, digits=3, signed=True):
    """+.031 style: no leading zero, explicit sign, em dash when missing."""
    if value is None:
        return MISSING
    text = f'{value:+.{digits}f}' if signed else f'{value:.{digits}f}'
    sign = text[0] if text[0] in '+-' else ''
    body = text[len(sign):]
    if body.startswith('0.'):
        body = body[1:]
    return sign.replace('-', theme.pick('−', '-')) + body


def _signs(matrix, axis, sign):
    """Thruster signs for a command along ``axis`` at ``sign``: + forward, - reversed."""
    return ''.join('·' if not row[axis] else ('+' if row[axis] * sign > 0 else theme.pick('−', '-'))
                   for row in matrix)


def _percent(value):
    return MISSING if value is None else f'{value:.0f}'


def _round(value):
    return None if value is None else round(value)


def _fit_record(fit):
    """The fitrows record (verdict, ceiling at fit) of a state.json fit; empty when never fitted."""
    return dict(verdict=fit.get('fit_label'), ceiling=_round(fit.get('fit')))


def column(key, data, matrix):
    """The string cells of one direction column plus its flags (below_trust handled later)."""
    axis, sign = AXES.index(key[:-1]), 1 if key[-1] == '+' else -1
    fit = data.get('fits', {}).get(key, {})
    c = (data.get('c_positive') if sign > 0 else data.get('c_negative') or data.get('c_positive'))
    limited = set(fit.get('limited', ()))
    cross = []
    for name, k in zip(CROSS, CROSS_AXIS):
        value = None if c is None or k == axis else c[k][axis]
        cross.append(MISSING if value is None else _num(value) + (' L' if name in limited else ''))
    lo, hi = fit.get('trust_lo'), fit.get('trust_hi')
    up = fitrows.better(fit.get('fit'), fit.get('ceiling'), min((t for t in (hi, lo) if t is not None), default=None))
    peak = fit.get('peak')
    return dict(
        key=key, hi=hi, lo=lo, mark=fit.get('mark', ''), up=up,
        cells={
            'mixer': _signs(matrix, axis, sign),
            'peak': _num(peak, 2, False) if peak is not None else MISSING,
            'cross': cross,
            'sigma': (f"{_num(fit['sigma'], 3, False)} {fit['age']}" if fit.get('sigma') is not None
                      else MISSING),
            'trust': MISSING if hi is None else f"{_percent(hi)}/{_percent(lo)}" + ('%' if lo is not None else ''),
            'residual': fit.get('residual', MISSING),
            'iterations': (f"{fit['iterations']}" + (' lim' if limited else '')) if 'iterations' in fit else MISSING,
            'fit': fitrows.fit_cell(_fit_record(fit)),
            'ceiling': fitrows.now_cell(dict(ceiling=_round(fit.get('ceiling')), hdg=fit.get('hdg'), up=up)),
            'speed': (f"{_num(fit['v_lo'], 2, False)}/{_num(fit['v_hi'], 2, False)}"
                      if fit.get('v_lo') is not None else MISSING),
            'drag': f"{fit['d1']:.1f}·{fit['d2']:.1f}" if fit.get('d1') is not None else MISSING,
            't95': (f"{fit['t95']:.1f} {_num(fit['vmax'], 2, False)}" if fit.get('t95') is not None else MISSING),
            'tracking': _percent(None if fit.get('tracking') is None else fit['tracking'] * 100),
        },
        holds=fit.get('holds', []))


def correction_view(data, cursor=0, settings=None):
    """The model ``render`` draws.

    ``data``: ``matrix`` (4x3 mixer), ``c_positive`` / ``c_negative`` (3x3, negative optional),
    ``fits`` ({direction: record}), ``header`` (fitted, activity, plan), ``readiness`` and
    ``tracking`` for the side column. Missing records show as em dashes, so the tab is
    readable before the first characterization.
    """
    settings = settings or load_settings()
    columns = [column(key, data, data.get('matrix') or ()) for key in DIRECTIONS]
    trusts = [min(t for t in (c['hi'], c['lo']) if t is not None) for c in columns
              if c['hi'] is not None or c['lo'] is not None]
    header = dict(data.get('header', {}))
    header['min_trust'] = min(trusts) if trusts else None
    header['verify'] = sum(1 for c in columns if c['mark'] == '?')
    return dict(columns=columns, cursor=max(0, min(cursor, len(columns) - 1)), header=header,
                readiness=data.get('readiness'), tracking=data.get('tracking'),
                plan=data.get('plan'), settings=settings)


def tab_view(hardware, state=None, running=None):
    """The model of the tab from the config and ``state`` ({'fits', 'column'}); column None is the worst."""
    state = state or {}
    side = {k: state[k] for k in ('readiness', 'tracking', 'plan') if state.get(k)}
    view = correction_view(correction_data(hardware, state.get('fits'), running, side))
    column = state.get('column')
    view['cursor'] = worst_column(view) if column is None else max(0, min(column, len(DIRECTIONS) - 1))
    return view


def move(view, delta):
    """The cursor column after ← / →, clamped to the six directions."""
    return max(0, min(view['cursor'] + delta, len(view['columns']) - 1))


def worst_column(view):
    """Default focus: the column with the least trust, or 0 before any fit."""
    scored = [(min(t for t in (c['hi'], c['lo']) if t is not None), i)
              for i, c in enumerate(view['columns']) if c['hi'] is not None or c['lo'] is not None]
    return min(scored)[1] if scored else 0


# (label, config key whose scope tags the first row of its block, how to read the cells)
_ROWS = (
    ('Thrusters M', 'thruster_matrix', 'mixer'), ('peak |t|', '', 'peak'),
    ('C cross sw', 'decoupling', ('cross', 0)), ('ψ', '', ('cross', 1)), ('su', '', ('cross', 2)),
    ('± σ / age', '', 'sigma'), ('Trust hi/lo', '', 'trust'), ('residual', '', 'residual'),
    ('iterations', '', 'iterations'), ('fit under', '', 'fit'), ('ceiling now', '', 'ceiling'),
    ('Drag v lo/hi', 'characterization.curves', 'speed'), ('d1 · d2', '', 'drag'), ('t95 · vmax', '', 't95'),
    ('tracking', '', 'tracking'))
TAGS = {'hardware': 'hw', 'calibration': 'cal'}


@lru_cache(maxsize=None)
def scope_tag(key):
    """``hw`` / ``cal`` for a config key from the fields.yaml scope (ts077), '' for a setting or no key.

    Memoized: the scope is static config and load_fields would re-parse fields.yaml on every render.
    """
    return TAGS.get(calibration.scope_of(key)[0], '') if key else ''


# Dropped first when the panel is short: later entries go before earlier ones.
_DROP = ('tracking', 'ceiling', 'fit', 'iterations', 'residual', 't95', 'drag', 'peak')


def _tint():
    """Cursor stripe style: a tint on truecolor and 256 colours, underline otherwise (ui U10)."""
    if theme.ASCII or os.environ.get('NO_COLOR'):
        return 'underline'
    if os.environ.get('COLORTERM') in ('truecolor', '24bit'):
        return 'on #232A36'
    return 'on grey15' if '256' in os.environ.get('TERM', '') else 'underline'


def _cell(row, col, settings):
    value = col['cells'][row[2]] if isinstance(row[2], str) else col['cells']['cross'][row[2][1]]
    style = None
    if row[2] == 'trust' and col['hi'] is not None:
        lowest = min(t for t in (col['hi'], col['lo']) if t is not None)
        style = f'bold {WARN}' if lowest < settings['trust_warn'] else None
    return value, style


def _table(view, width, height):
    s, cursor = view['settings'], view['cursor']
    inner = width - 4
    label = s['label_width']
    cell = min(s['cell_max'], (inner - label - 4) // 6)
    if cell < s['cell_min']:
        label = 14
        cell = min(s['cell_max'], (inner - label - 4) // 6)
    tint = _tint()
    rows = list(_ROWS)
    for name in _DROP:   # 9 lines of box, header, blank and footer around the rows
        if len(rows) + 9 <= height:
            break
        rows = [r for r in rows if r[2] != name]

    def line(tag, name, cells):
        out = Text(f'{tag:<4}', style=SCOPE) + Text(f'{name:<{label - 4}}' if len(name) <= label - 4 else name[:label - 4])
        for i, (value, style) in enumerate(cells):
            text = Text(f'{value:<{cell}}'[:cell], style=style)
            if i == cursor:
                text.stylize(tint)
            out.append_text(text)
        return out

    heads = []
    for col in view['columns']:
        mark = {'?': ' ?', '✗': ' ' + theme.pick('✗', 'x')}.get(col['mark']) or (' ' + fitrows.UP if col['up'] else '')
        heads.append((col['key'].replace('-', theme.pick('−', '-')) + mark, 'bold'))
    lines = [line('', '', heads)]
    for row in rows:
        lines.append(line(scope_tag(row[1]), row[0], [_cell(row, c, s) for c in view['columns']]))
    return lines


def _title(view):
    h = view['header']
    parts = ['Correction']
    parts.append(f"C± fitted {h['fitted']}" if h.get('fitted') else 'C± not fitted')
    if h.get('min_trust') is not None:
        parts.append(f"min {h['min_trust']:.0f}%")
    if h.get('verify'):
        parts.append(f"{h['verify']} verify")
    return parts


def _side(view, width, room):
    """Readiness and Tracking text blocks for the side column."""
    blocks = []
    r = view.get('readiness')
    if r:
        body = [Text(f"{k:<9} {v}") for k, v in r.get('rows', [])]
        body += [Text.assemble('  ', theme.icon(state), f' {label}'[:width - 6]) for state, label in r.get('gates', [])]
        blocks.append(theme.block(Group(*body), f"Readiness{theme.SEPARATOR}{r.get('verdict', MISSING)}"))
    t = view.get('tracking')
    if t:
        body = [Text('axis   now   session  trend', style=MUTED)]
        body += [Text(f'{a:<6} {n:<5} {s:<8} {tr}') for a, n, s, tr in t.get('rows', [])]
        body += [Text(x, style=MUTED) for x in t.get('notes', [])]
        blocks.append(theme.block(Group(*body), f"Tracking{theme.SEPARATOR}{t.get('version', '')}".rstrip(theme.SEPARATOR)))
    holds = details(view, width, max(6, room - 14 * len(blocks)))
    # Drop Tracking, then Readiness, until the column fits the height (holds always stay).
    while blocks and sum(len(b.renderable.renderables) + 2 for b in blocks + [holds]) > room:
        blocks.pop()
    return Group(*blocks, holds)


def hold_rows(view):
    """(label, text) rows of the cursor column's holds for a popup."""
    holds = view['columns'][view['cursor']]['holds']
    return [(str(n), f"a {_num(h.get('a'), 2, False)}  main {_num(h.get('main'), 2)}  "
                     f"cross {_num(h.get('cross'), 3)}  J {_num(h.get('j'), 2, False)}"
                     + (' L' if h.get('limited') else '')) for n, h in enumerate(holds, 1)] or [('', 'no holds yet')]


def details(view, width=34, height=None):
    """The holds popup (and side box) of the cursor column: one line per hold."""
    col = view['columns'][view['cursor']]
    body = [Text(f"{'#':<3}{'a':<5}{'main':<7}{'cross ψ':<9}J", style=MUTED)]
    for n, h in enumerate(col['holds'], 1):
        body.append(Text(f"{n:<3}{_num(h.get('a'), 2, False):<5}{_num(h.get('main'), 2):<7}"
                         f"{_num(h.get('cross'), 3):<9}{_num(h.get('j'), 2, False)}{' L' if h.get('limited') else ''}"))
    if not col['holds']:
        body.append(Text('no holds yet', style=MUTED))
    if height is not None:
        body = body[:max(2, height - 2)]
    name = col['key'].replace('-', theme.pick('−', '-'))
    return theme.block(Group(*body), f'{name}{theme.SEPARATOR}holds')


def render(view, width=120, height=40, focused=True):
    """The tab body: the table box, plus the side column from ``side_min_width`` up."""
    s = view['settings']
    side = width >= s['side_min_width']
    table_width = width - s['side_width'] if side else width
    body = [Text(MISSING if not view['header'].get('activity') else view['header']['activity'])]
    body.append(Text('thrust M ' + theme.ARROW + ' C± (driver adds) ' + theme.ARROW +
                     ' speed curves (controller inverts)', style=MUTED))
    lines = _table(view, table_width, height)
    footer = []
    if view.get('plan'):
        footer.append(Text('Plan  ' + view['plan']))
    footer.append(Text.assemble(('scope ', MUTED), (TAGS['hardware'], SCOPE), ' hardware: refit when rebuilt  ',
                                (TAGS['calibration'], SCOPE), ' calibration: refit each test'))
    pad = max(0, height - 2 - len(body) - len(lines) - len(footer) - 2)   # the box fills the body
    main = theme.block(Group(*body, Text(''), *lines, *[Text('')] * (pad + 1), *footer),
                       theme.SEPARATOR.join(_title(view)), focused=focused)
    if not side:
        return main
    grid = Table.grid(expand=False)
    grid.add_column(width=table_width)
    grid.add_column(width=s['side_width'])
    grid.add_row(main, _side(view, s['side_width'], height))
    return grid
