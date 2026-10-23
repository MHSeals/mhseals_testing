"""Correction tab fit rows (conditions_minimap.md section 5.2, CD7): pure text, no ROS, no Rich.

``fit under`` says what the ceiling was when the direction was fitted, ``ceiling now`` what it is
today; a column header carries ``x`` (invalid), ``?`` (verify) or the up mark when today is clearly
better than at fit. Scope glyphs are plain DIM text for the caller to style.
"""

from mhseals_testing.ui import theme

LABEL = 16          # row label column, after the two-cell scope glyph
CELL = 9            # one direction column
GAIN = 10           # ceiling points above the fit that make the up mark (R_now >= fit + 0.10)
TRUST_MAX = 90      # ... unless the direction is already trusted at this percent
VERDICTS = ('good', 'marg', 'poor')

HARDWARE = theme.pick('▪', 'H')
CALIBRATION = theme.pick('△', 'C')
UP = theme.pick('↑', '^')
INVALID = theme.pick('×', 'x')
HDG = 'hdg'


def _cell(text):
    return text.ljust(CELL)


def _row(label, cells, scope=' '):
    return f'{scope} {label:<{LABEL - 2}}' + ''.join(_cell(c) for c in cells).rstrip()


def mark(state, better=False):
    """Column header mark for a calibration state ('invalid', 'verify', 'ok') and the better flag."""
    return {'invalid': INVALID, 'verify': '?'}.get(state) or (UP if better else '')


def better(fit_ceiling, now, trust=None):
    """True when the ceiling now is at least GAIN above the fit's and trust is below TRUST_MAX."""
    if fit_ceiling is None or now is None:
        return False
    return now - fit_ceiling >= GAIN and (trust is None or trust < TRUST_MAX)


def header(directions, states, flags=()):
    """The column header line; ``states`` maps a direction to its status, ``flags`` are 'better' ones."""
    return (' ' * LABEL + ''.join(_cell((d + ' ' + mark(states.get(d, 'ok'), d in flags)).rstrip()) for d in directions)).rstrip()


def fit_cell(fit):
    """``<verdict> <ceiling at fit>`` of one direction's fit record, a dash when it was never fitted."""
    if not fit or fit.get('verdict') is None or fit.get('ceiling') is None:
        return theme.DASH
    return f"{fit['verdict']} {fit['ceiling']}"


def now_cell(now):
    """``ceiling now`` cell: the ceiling, then ``hdg`` and the up mark; a dash when unknown."""
    if not now or now.get('ceiling') is None:
        return theme.DASH
    return f"{now['ceiling']}" + (' ' + HDG if now.get('hdg') else '') + (UP if now.get('up') else '')


def fit_under(directions, fits):
    """``fit under`` row: ``<verdict> <ceiling at fit>``; a direction never fitted shows a dash."""
    return _row('fit under', [fit_cell(fits.get(d)) for d in directions])


def ceiling_now(directions, now):
    """``ceiling now`` row; ``now[d]`` is {'ceiling': int, 'hdg': bool, 'up': bool}, absent when unknown."""
    return _row('ceiling now', [now_cell(now.get(d)) for d in directions])


def legend():
    """One DIM line for the history popup footer."""
    return f'{HARDWARE} hardware  {CALIBRATION} calibration  (run|site|build)  no glyph: setting'


def consequence(change_id, effects):
    """History detail line for a hardware change: ``▪ change → △ C± · curves × (fitted under c0131)``.

    The affected items are listed once and one mark follows, the worst state winning (invalid over verify).
    """
    names = f' {theme.MIDDOT} '.join(name for name, _ in effects)
    worst = INVALID if any(state == 'invalid' for _, state in effects) else '?'
    return f'{HARDWARE} change {theme.ARROW} {CALIBRATION} {names} {worst} (fitted under {change_id})'
