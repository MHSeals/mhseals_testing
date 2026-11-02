"""tmux as a transport (docs/tmux-graphics.md): which tier tmux can carry to every client viewing the
pane's window, and the passthrough that wraps kitty's escapes for the terminal outside tmux.
"""

import re
import subprocess

from mhseals_testing.ui.imaging.kitty import KITTY_NAMES

TMUX_FORMAT = ('#{window_id}\t#{allow-passthrough}', '#{window_id}\t#{client_created}\t#{client_termtype}\t'
               '#{client_termfeatures}\t#{client_cell_width}\t#{client_cell_height}')


_APC = re.compile('\x1b_G[^\x1b]*\x1b\\\\')


def tmux_view(env, run=subprocess.run):
    """What the tmux server says about the pane in ``env`` ($TMUX, $TMUX_PANE), or None (no answer):
    ``passthrough`` and the ``clients`` viewing its window, each ``(created, termtype, features,
    cell)``."""
    pane = env.get('TMUX_PANE')
    if not env.get('TMUX') or not pane:
        return None
    try:
        out = run(['tmux', 'display', '-p', '-t', pane, TMUX_FORMAT[0], ';', 'list-clients', '-F', TMUX_FORMAT[1]],
                  capture_output=True, text=True, timeout=0.5, env=dict(env)).stdout.splitlines()
    except (OSError, subprocess.SubprocessError):
        return None
    head = out[0].split('\t') if out else []
    if len(head) != 2 or not head[0].startswith('@'):
        return None
    clients = []
    for line in out[1:]:
        f = line.split('\t')
        if len(f) == 6 and f[0] == head[0]:
            cell = (int(f[4]), int(f[5])) if f[4].isdigit() and f[5].isdigit() else (0, 0)
            clients.append((f[1], f[2], frozenset(f[3].split(',')), cell))
    return dict(window=head[0], passthrough=head[1] in ('on', 'all'), clients=tuple(sorted(clients)))


def _tmux_tier(setting, caps, view, colours):
    """The tier tmux can carry to every client viewing the window (text without one): tmux keeps
    the sixel images (built with sixel: its DA1 says 4) for clients with the sixel feature and draws
    'SIXEL IMAGE' text for the rest; kitty escapes pass through untouched to kitty clients only."""
    clients = (view or {}).get('clients')
    if not clients:
        return 'text'
    cells = {c[3] for c in clients}
    nested = any(c[1].lower().startswith(('tmux', 'screen')) for c in clients)
    if len(cells) != 1 or 0 in next(iter(cells)) or nested:
        return 'text'
    if (setting in ('auto', 'kitty') and view['passthrough'] and colours in ('256', 'truecolor', 'eight_bit')
            and all(c[1].lower().startswith(KITTY_NAMES) for c in clients)):
        return 'kitty'
    if setting in ('auto', 'sixel') and caps.get('sixel') and all('sixel' in c[2] for c in clients):
        return 'sixel'
    return 'text'


def wrap(data):
    """Kitty escapes for the terminal outside tmux: each in a passthrough DCS, its ESCs doubled."""
    return _APC.sub(lambda m: '\x1bPtmux;' + m.group().replace('\x1b', '\x1b\x1b') + '\x1b\\', data)
