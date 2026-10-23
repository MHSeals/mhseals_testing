"""The Send popup (bags.md 7, B13): a pure model of one transfer plus its frame, flow and guards.

Nothing here reads keys, a clock or rsync: the caller feeds ``update`` the worker's progress and
passes ``now`` to ``frame``, so the stall rule and the easing are testable with a fake clock.
"""

import dataclasses

from rich.console import Group
from rich.text import Text

from mhseals_testing.ui import theme

STALL_S = 5.0          # no new bytes for this long turns "stalled" yellow
ETA_AFTER_S = 2.0      # the ETA appears once the transfer has run this long
EASE = 0.35            # fraction of the gap to the target the bar closes per 5 Hz frame
EMA = 0.7              # weight of the old rate
WIDTH = 58
EIGHTHS = ' ▏▎▍▌▋▊▉█'
SPIN = '⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏'
SPIN_ASCII = '|/-\\'
SPIN_S = 0.16


@dataclasses.dataclass
class Transfer:
    """One session send. ``state``: running, done, failed or cancelled."""
    session: str
    target: str
    detail: str = ''          # procedures in the session, shown after the id
    address: str = ''         # resolved IP, muted
    state: str = 'running'
    stage: str = 'index'      # index, reports or bags
    passes: tuple = (0, 0)    # closed segments sent / total
    done: int = 0
    total: int = 0
    file: str = ''
    error: str = ''           # last rsync stderr line on failure
    rate: float = 0.0         # bytes/s, EMA
    shown: float = 0.0        # eased fraction actually drawn
    started: float | None = None
    last_bytes: float | None = None   # time the byte count last changed

    @property
    def fraction(self):
        return min(1.0, self.done / self.total) if self.total else 0.0

    def update(self, now, done=None, total=None, file=None, stage=None, passes=None):
        """Take a progress line. The rate is an EMA over bytes since the last change."""
        if self.started is None:
            self.started = self.last_bytes = now
        if total is not None:
            self.total = total
        if stage:
            self.stage = stage
        if passes:
            self.passes = passes
        if file:
            self.file = file
        if done is not None and done != self.done:
            if done > self.done and now > self.last_bytes:
                inst = (done - self.done) / (now - self.last_bytes)
                self.rate = inst if not self.rate else EMA * self.rate + (1 - EMA) * inst
            self.done, self.last_bytes = done, now

    def finish(self, state, error=''):
        self.state, self.error = state, error
        if state == 'done':
            self.done = self.total

    def stalled(self, now):
        return self.state == 'running' and self.last_bytes is not None and now - self.last_bytes > STALL_S

    def eta(self, now):
        if self.started is None or now - self.started < ETA_AFTER_S or self.rate <= 0:
            return None
        return max(0, int((self.total - self.done) / self.rate))

    def tick(self):
        """One 5 Hz render step: ease the drawn fraction toward the target."""
        self.shown = 1.0 if self.state == 'done' else self.shown + (self.fraction - self.shown) * EASE


def size(n):
    for unit in ('B', 'KiB', 'MiB', 'GiB'):
        if n < 1024 or unit == 'GiB':
            return f'{n:.0f} {unit}' if unit == 'B' else f'{n:.1f} {unit}'
        n /= 1024


def bar(fraction, width):
    """Eighth-block bar, or ``[####----]`` in ASCII mode."""
    if theme.ASCII:
        full = round(fraction * (width - 2))
        return '[' + '#' * full + '-' * (width - 2 - full) + ']'
    cells = fraction * width
    whole = int(cells)
    tail = EIGHTHS[int((cells - whole) * 8)] if whole < width else ''
    return ('█' * whole + tail).ljust(width, '░')


def frame(t, now, width=WIDTH):
    """The popup as (panel, panel_width), fitting a terminal ``width``. Reads nothing but ``t``."""
    panel_width = max(20, min(width - 4, WIDTH))
    inner = panel_width - 4
    spin = SPIN_ASCII if theme.ASCII else SPIN
    glyph = spin[int(now / SPIN_S) % len(spin)]
    ok, cur = theme.pick('✓', '+'), glyph
    mark = lambda name, active: f"{cur if active else ok} {name}"   # noqa: E731
    stage = t.stage
    steps = '   '.join((mark('index', stage == 'index'), mark('reports', stage == 'reports'),
                        f"{mark('bags', stage == 'bags')} {t.passes[0]}/{t.passes[1]}"))
    lines = [Text(f'{glyph} {t.session}' + (theme.SEPARATOR + t.detail if t.detail else '')),
             Text('  ' + steps + (f'   {t.address}' if t.address else ''), style=None)]
    if t.address:
        lines[1].stylize(theme.MUTED, len(lines[1].plain) - len(t.address))
    lines.append(Text(''))
    pct = f'{round(t.shown * 100):3d}%'
    lines.append(Text(bar(t.shown, max(4, inner - 6)) + '  ' + pct, style=theme.GOOD if t.state == 'done' else None))
    if t.state == 'done':
        lines.append(Text(f'sent {t.passes[0]}/{t.passes[1]}{theme.SEPARATOR}sizes ok', style=theme.GOOD))
    elif t.state in ('failed', 'cancelled'):
        lines.append(Text(t.error or t.state, style=theme.BAD if t.state == 'failed' else theme.WARN))
    else:
        info = Text(f'{size(t.done)} / {size(t.total)}')
        info.append(f'{theme.SEPARATOR}{size(t.rate)}/s', style=theme.MUTED)
        eta = t.eta(now)
        if eta is not None:
            info.append(f'{theme.SEPARATOR}eta {eta // 60}:{eta % 60:02d}')
        if t.stalled(now):
            info.append(f'{theme.SEPARATOR}stalled', style=theme.WARN)
        lines += [info, Text(t.file, style=theme.MUTED)]
    keys = {'running': 'Esc hide · c cancel', 'done': 'Esc close', 'failed': 'Esc hide · r retry',
            'cancelled': 'Esc hide · r retry'}[t.state]
    lines += [Text(''), Text(keys.replace(' · ', theme.SEPARATOR), style=theme.MUTED)]
    for line in lines:
        line.no_wrap, line.overflow = True, 'crop'
    return theme.block(Group(*lines), f'{theme.FOCUS} Send {theme.ARROW} {t.target}'), panel_width


def send_steps(targets, recording, title):
    """The ``y`` flow as ordered step names: target (if several), stop choice, title, then send."""
    steps = []
    if len(targets) > 1:
        steps.append('target')
    if recording:
        steps.append('stop_recording')
    if not title:
        steps.append('title')
    return steps + ['index', 'send']


def quit_guard(transfers):
    """The choose-popup spec shown on quit while a transfer runs, or None when nothing is sending."""
    if not any(t.state == 'running' for t in transfers):
        return None
    return dict(title='Transfer running', options=['Wait', 'Cancel (resumable)'],
                message='Cancelling keeps the partial files; send again to resume.')


def completion_toast(t):
    """(kind, text) for the toast raised when a transfer ends."""
    if t.state == 'done':
        return 'ok', f'Sent {t.session} {theme.ARROW} {t.target}'
    if t.state == 'failed':
        return 'fail', f'Send failed: {t.session}'
    return 'warn', f'Send cancelled: {t.session}'
