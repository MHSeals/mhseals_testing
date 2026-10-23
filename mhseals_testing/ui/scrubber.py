"""The review footer rows: marker ticks, the session track and the status line."""

from rich.text import Text

from mhseals_testing.ui import theme

TRACK, WINDOW, GAP, HEAD = theme.pick('━', '-'), theme.pick('▬', '='), theme.pick('╌', '.'), theme.pick('┃', '|')
MARK, FAIL = theme.pick('▾', 'v'), theme.BAR
PLAY, PAUSE, TIMES = theme.pick('▶', '>'), theme.pick('‖', '||'), theme.pick('×', 'x')
OUTCOME = {'PASS': theme.GOOD, 'FAIL': theme.BAD, 'ABORTED': theme.WARN}   # else busy (running)
ROWS = 3


def clock(seconds):
    seconds = int(max(seconds, 0))
    return f'{seconds // 60}:{seconds % 60:02d}'


def rows(review, width, notice=''):
    """Tick row, track row and status line as Rich Text, each `width` columns or fewer."""
    def col(t):
        return min(width - 1, max(0, round((t - review.t0) / max(review.length, 1e-9) * (width - 1))))

    ticks, track = [(' ', None)] * width, [(TRACK, theme.MUTED)] * width
    for a, b in review.gaps:
        track[col(a):col(b) + 1] = [(GAP, theme.MUTED)] * (col(b) + 1 - col(a))
    for a, b, outcome, _ in review.windows:
        track[col(a):col(b) + 1] = [(WINDOW, OUTCOME.get(outcome, theme.BUSY))] * (col(b) + 1 - col(a))
    for t in review.fails:
        ticks[col(t)] = (FAIL, theme.BAD)
    for t, _, kind in review.markers:
        ticks[col(t)] = (MARK, theme.ACCENT if kind == 'user' else theme.MUTED)
    track[col(review.playhead)] = (HEAD, f'bold {theme.ACCENT}')
    status = Text(f" {PLAY if review.playing else PAUSE} {review.rate:g}{TIMES}", style='bold')
    status.append(f"{theme.SEPARATOR}{clock(review.playhead - review.t0)} / {clock(review.length)}")
    near = review.nearest()
    if near:
        status.append(f'{theme.SEPARATOR}{MARK}{near[0]}/{near[1]} {near[2]}'.rstrip())
    if notice:
        status.append(f'{theme.SEPARATOR}{notice}', style=theme.MUTED)
    status.truncate(width, overflow='ellipsis')
    return [Text.assemble(*line) for line in (ticks, track)] + [status]
