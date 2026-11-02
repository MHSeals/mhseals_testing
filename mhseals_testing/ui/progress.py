"""One footer line for a long run's place: bar, percent, step, the command against the measured velocity."""

WIDTH = 12


def bar(done, total, width=WIDTH):
    filled = round(width * min(max(done / total, 0.0), 1.0)) if total else 0
    return '█' * filled + '░' * (width - filled)


def line(place, command, velocity, remaining, separator=' · '):
    """``▕████░░░░▏ 42% 5/12 sway+ · cmd +0.40 +0.00 +0.00 · vel +0.31 +0.02 -0.01 · 3.2s``.

    ``place`` is the runner's (done, total, label); ``velocity`` (vx, vy, wz) or None when odometry is quiet."""
    done, total, label = place
    total = max(total, 1)
    head = f'▕{bar(done, total)}▏ {100 * done // total:3d}% {done}/{total} {label}'.rstrip()
    cmd = ' '.join(f'{v:+.2f}' for v in command)
    vel = ' '.join(f'{v:+.2f}' for v in velocity) if velocity else '—'
    return separator.join((head, f'cmd {cmd}', f'vel {vel}', f'{remaining:.1f}s'))
