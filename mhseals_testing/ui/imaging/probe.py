"""The one capability probe and the tier it selects.

`probe` asks the terminal once a session (XTVERSION, the cell size, the kitty keyboard flags for the
input layer's Terminal and DA1, ``4`` = sixel; then the
kitty graphics query only if XTVERSION names kitty or ghostty), each round ending at the DA1 reply
or PROBE_S. `choose` turns the replies, theme.yaml ``graphics`` and the environment into a tier:
under tmux what tmux.py says it carries, else the first backend (protocol.py) that accepts.
"""

import os
import re
import select
import time

from mhseals_testing.ui.imaging.kitty import KITTY_NAMES, _kitty_env
from mhseals_testing.ui.imaging.protocol import backends
from mhseals_testing.ui.imaging.tmux import _tmux_tier

PROBE_ID = 4242


PROBE = ('\x1b[>0q'                                              # XTVERSION
         '\x1b[16t'                                              # cell size in pixels
         '\x1b[?u'                                               # kitty keyboard flags (key events)
         '\x1b[c')                                               # DA1: everyone answers, so it ends the probe


# Asked only of a terminal that named itself kitty or ghostty: tmux takes an unknown APC for a
# window title, so the query never goes out blind.
PROBE_KITTY = f'\x1b_Gi={PROBE_ID},s=1,v=1,a=q,t=d,f=24;AAAA\x1b\\\x1b[c'


PROBE_S = 0.5               # s; the probe normally ends at the DA1 reply, one round trip



_REPLY = re.compile(rb'\x1b_G([^\x1b]*)\x1b\\|\x1bP>\|([^\x1b]*)\x1b\\|\x1b\[6;(\d+);(\d+)t|\x1b\[\?([\d;]*)c|\x1b\[\?(\d+)u')


def parse(data):
    """``(caps, rest)`` from probe reply bytes: caps has ``done`` (DA1 seen), ``kitty``, ``sixel``,
    ``cell`` (w, h px or None), ``xtversion`` and ``keyboard`` (kitty keyboard flags or None); ``rest`` is what was not a reply (typed keys)."""
    caps = dict(done=False, kitty=False, sixel=False, cell=None, xtversion=None, keyboard=None)
    for m in _REPLY.finditer(data):
        if m[6] is not None:
            caps['keyboard'] = int(m[6])
        elif m[1] is not None:
            caps['kitty'] = caps['kitty'] or m[1].startswith(b'i=%d;OK' % PROBE_ID)
        elif m[2] is not None:
            caps['xtversion'] = m[2].decode('utf-8', 'replace')
        elif m[3] is not None:
            if int(m[3]) > 0 and int(m[4]) > 0:
                caps['cell'] = (int(m[4]), int(m[3]))
        else:
            caps['done'] = True
            caps['sixel'] = '4' in m[5].decode().split(';')[1:]
    return caps, _REPLY.sub(b'', data)


def _ask(fd, write, query, timeout):
    write(query)
    data, deadline = b'', time.monotonic() + timeout
    while True:
        caps, rest = parse(data)
        left = deadline - time.monotonic()
        if caps['done'] or left <= 0:
            return caps, rest
        if select.select([fd], [], [], left)[0]:
            chunk = os.read(fd, 4096)
            if not chunk:
                return caps, rest
            data += chunk



def probe(fd, write, timeout=PROBE_S, env=None):
    """Ask the terminal on ``fd`` (XTVERSION, cell size, DA1), then the kitty graphics query if it
    is kitty or ghostty; each round ends at the DA1 reply or ``timeout`` s. ``(caps, rest)``."""
    env = os.environ if env is None else env
    caps, rest = _ask(fd, write, PROBE, timeout)
    name = (caps['xtversion'] or '').lower()
    if caps['done'] and (name.startswith(KITTY_NAMES) or (not name and _kitty_env(env))):
        more, extra = _ask(fd, write, PROBE_KITTY, timeout)
        caps['kitty'], rest = more['kitty'], rest + extra
    return caps, rest


def _tmux(env, caps):
    """Under tmux: $TMUX, TERM_PROGRAM=tmux (tools/boat_tui hands it through docker exec) or tmux's XTVERSION."""
    return (bool(env.get('TMUX')) or env.get('TERM_PROGRAM') == 'tmux'
            or (caps.get('xtversion') or '').lower().startswith('tmux'))


def blind(env, caps):
    """A tmux the TUI can't ask about its clients (docker exec or ssh from a tmux pane: no $TMUX)."""
    return _tmux(env, caps) and not env.get('TMUX')


def choose(setting, caps, env=None, colours='truecolor', view=None):
    """The tier for theme.yaml's ``graphics`` setting, the probe's ``caps`` and the environment.

    Under tmux only what tmux reports for its clients (``view``, from `tmux_view`) picks an image
    tier: tmux can't vouch for a client through ssh or docker, and screen never. Nor in a dumb
    terminal, nor sixel unless the terminal gave its cell size: a guessed cell lets tiles spill
    past their cells, over the arm and stop popups."""
    from mhseals_testing.ui import palette
    env = os.environ if env is None else env
    name = (caps.get('xtversion') or '').lower()
    if (setting == 'off' or env.get('NO_COLOR') or palette.ascii_mode(env) or env.get('TERM') == 'dumb'
            or env.get('STY') or name.startswith('screen')):
        return 'text'
    if _tmux(env, caps):
        if setting in ('auto', 'sixel', 'kitty') and view is None and not env.get('TMUX'):
            # A tmux the TUI can't ask (docker exec or ssh from a tmux pane): a forced tier is the
            # user's word for the viewers. Sixel (auto too) trusts tmux's own DA1 (built with sixel:
            # tmux draws it, text for a client without sixel) and cell size; kitty trusts
            # allow-passthrough (tools/boat_tui sets it on its own server).
            if setting == 'kitty':
                return 'kitty' if colours in ('256', 'truecolor', 'eight_bit') else 'text'
            return 'sixel' if caps.get('sixel') and caps.get('cell') else 'text'
        return _tmux_tier(setting, caps, view, colours)
    for backend in backends():
        if backend.probe(setting, caps, env, colours):
            return backend.name
    return 'text'


def reason(setting, caps, env=None, colours='truecolor', view=None):
    """Why `choose` draws text for an image setting, in a few words for the operator ('' when it doesn't)."""
    from mhseals_testing.ui import palette
    env = os.environ if env is None else env
    if setting == 'off' or choose(setting, caps, env, colours, view) != 'text':
        return ''
    name = (caps.get('xtversion') or '').lower()
    if env.get('NO_COLOR') or palette.ascii_mode(env):
        return 'NO_COLOR or ASCII mode'
    if env.get('TERM') == 'dumb' or env.get('STY') or name.startswith('screen'):
        return 'dumb terminal or screen'
    if not caps.get('done'):
        return 'the terminal did not answer the probe'
    if _tmux(env, caps):
        if view is None and env.get('TMUX'):
            return 'tmux did not answer'
        if view is None:
            return ('tmux has no sixel: kitty users set Graphics kitty' if not caps.get('sixel')
                    else 'tmux gave no cell size yet')
        if not view.get('clients'):
            return 'no client attached'
        if not view.get('passthrough'):
            return 'tmux allow-passthrough is off and a client lacks sixel'
        return 'a tmux client is not kitty and has no sixel'
    if setting == 'sixel' or caps.get('sixel'):
        return 'the terminal gave no cell size'
    return 'the terminal has neither kitty graphics nor sixel'
