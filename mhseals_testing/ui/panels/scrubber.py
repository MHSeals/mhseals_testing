"""Review scrubber and loupe (timeline.md section 3.7).

The footer scrubber is the whole session on one exact-width axis drawn by
`strip.rows(form='footer')`: events, a one-row skyline and the axis with `├ ┤`
around the loupe window and `●` at the playhead, then a status line. The
Activity panel shows the same window linearly (the loupe); `z` cycles its size.
"""

from rich.text import Text

from .. import theme
from ..strip import FAIL, Band, Point, Span, rows
from ..timeaxis import Axis, clock

LOUPES = (720.0, 1800.0, 120.0)     # seconds: 12 m, 30 m, 2 m


def next_loupe(size):
    """The loupe size after `size` in the z cycle (unknown sizes restart it)."""
    return LOUPES[(LOUPES.index(size) + 1) % len(LOUPES)] if size in LOUPES else LOUPES[0]


def loupe_window(play, size, t0, end):
    """(a, b) of `size` seconds centred on the playhead, shifted to stay inside the session."""
    size = min(size, max(end - t0, 1e-6))
    a = min(max(play - size / 2, t0), end - size)
    return a, a + size


def loupe_axis(play, size, t0, end, width):
    """Linear, fold-free axis of the loupe window for the Activity panel."""
    return Axis(t0, end, width, [], window=loupe_window(play, size, t0, end))


def label(size):
    """Loupe size as shown in the status line and subtitle: 2m, 12m."""
    return f'{size / 60:g}m'


def status(play, t0, end, size, speed=1.0, playing=True, note=None, fmt=clock,
           ascii_mode=None, width=None):
    """Status line: rate, position over length, an optional note, loupe size.

    With `width` the note is ellipsised to fit so the loupe item stays whole.
    """
    ascii_ = theme.ASCII if ascii_mode is None else ascii_mode
    total = int(end - t0)
    length = (f'{total // 3600}h{total // 60 % 60:02d}' if total >= 3600
              else f'{total // 60}:{total % 60:02d}')
    parts = [f'{"▶" if playing else "⏸"} {speed:g}×' if not ascii_
             else f'{">" if playing else "|"} {speed:g}x',
             f'{fmt(play)} / {length}']
    sep = ' | ' if ascii_ else ' · '
    tail = f'loupe {label(size)}'
    if note and width is not None:
        room = width - len(sep.join(parts + [tail])) - len(sep)
        note = note if len(note) <= room else (note[:room - 1] + ('~' if ascii_ else '…') if room >= 2 else '')
    return Text(sep.join(parts + ([note] if note else []) + [tail]), no_wrap=True)


def footer(width, t0, end, play, size, spans=(), points=(), bands=(), armed=(), holds=(),
           fmt=clock, ascii_mode=None, color=None, **status_args):
    """Four lines of `width` cells: events, skyline, axis with loupe and playhead, status."""
    window = loupe_window(play, size, t0, end)
    busy = ([(s.a, s.b) for s in spans] + [(b.a, b.b) for b in bands] + list(armed)
            + [(p.t, p.t) for p in points] + [window])    # the loupe is never folded away
    axis = Axis(t0, end, width, busy, exact=True)
    lines = rows(axis, spans=spans, points=points, armed=armed, holds=holds, bands=bands,
                 form='footer', fmt=fmt, ascii_mode=ascii_mode, color=color,
                 window=window, play=play)
    status_line = status(play, t0, end, size, fmt=fmt, ascii_mode=ascii_mode, width=width,
                         **status_args)
    status_line.truncate(width, overflow='crop')
    status_line.pad_right(width - status_line.cell_len)
    return lines + [status_line]


def scene(review):
    """The strip's spans, points and bands for a ui.review.Review (windows, marks, fails, gaps)."""
    ends = {'FAIL': 'fault', 'ABORTED': 'abort'}
    spans = [Span(a, b, name[:1], end=ends.get(outcome), running=not outcome)
             for a, b, outcome, name in review.windows]
    points = ([Point(t, 'mark') for t, _, kind in review.markers if kind == 'user']
              + [Point(t, 'health', FAIL) for t in review.fails])
    return dict(spans=spans, points=points, bands=[Band(a, b, 'recoff') for a, b in review.gaps])


def review_note(review, notice=''):
    """What is under the playhead for the status line: procedure, nearest mark, then the notice."""
    parts = []
    under = review.window_at()
    if under:
        parts.append(f'{under[0]} {under[1] or "running"}'.rstrip())
    near = review.nearest()
    if near:
        parts.append(f'{theme.timeline()["mark"]}{near[0]}/{near[1]} {near[2]}'.rstrip())
    return theme.SEPARATOR.join(parts + ([notice] if notice else []))


def elapsed_fmt(review):
    """Time formatter showing m:ss since the session start (no timezone in goldens)."""
    return lambda t: f'{int(t - review.t0) // 60}:{int(t - review.t0) % 60:02d}'


def review_footer(review, width, notice=''):
    """The four footer lines of a Review at `width`."""
    elapsed = elapsed_fmt(review)
    return footer(width, review.t0, review.t1, review.playhead, review.loupe, speed=review.rate,
                  playing=review.playing, note=review_note(review, notice), fmt=elapsed,
                  **scene(review))


class Loupe:
    """Rich renderable of the Activity loupe: the strip over the loupe window at the panel width."""

    def __init__(self, review):
        self.review = review

    def __rich_console__(self, console, options):
        r = self.review
        axis = loupe_axis(r.playhead, r.loupe, r.t0, r.t1, options.max_width)
        fmt = elapsed_fmt(r)
        yield from rows(axis, play=r.playhead, fmt=fmt, end_text=fmt(loupe_window(r.playhead, r.loupe, r.t0, r.t1)[1]), **scene(r))
