"""Backend selection for ui.imaging: probe the terminal once a session and pick the tier (text, kitty,
sixel), follow tmux clients, and the tier's switches (use, forget, repaint) and link-rate meter.

The state these set stays on the package (``imaging.TIER``, ``imaging.BUDGET``, ...): every global is read and
written as ``pkg.NAME``, so readers of the package and tests that patch it see one copy.
"""
import os
import sys
import threading
import time

pkg = sys.modules['mhseals_testing.ui.imaging']   # imported at the end of the package, which it serves


# --- detection ---------------------------------------------------------------------------------


def start(keys, console, write, setting='off', env=None):
    """Probe the terminal on the key reader's fd (once a session) and pick the tier; returns it.

    ``off`` still probes (the camera view's kitty reads it, `kitty_camera`); NO_COLOR and MHS_ASCII
    probe nothing unless a tier is wanted. Bytes that were not probe replies (keys typed meanwhile) go back to the
    reader's buffer. In sixel the console's stream is wrapped so markers become cursor-forward.
    Without a tty on the reader's fd every setting is text."""
    pkg._reset()
    pkg._listen(keys)
    pkg._session.update(keys=keys, console=console, write=write)
    pkg.GENERATION += 1
    pkg.SETTING = setting
    env = os.environ if env is None else env
    fd = getattr(keys, 'fd', None)
    caps = pkg._caps
    try:
        tty = fd is not None and os.isatty(fd)
    except OSError:
        tty = False
    if tty and setting != 'off' and env.get('TMUX'):
        pkg.tmux.passthrough(env)            # the TUI's own pane passes kitty's escapes (cx's tmux too)
    view = pkg.tmux_view(env) if tty and setting != 'off' and env.get('TMUX') else None
    # Under tmux the probe (tmux answers it itself) runs detached too: a client may attach later.
    hopeful = view and dict(view, passthrough=True, clients=(('', 'kitty', frozenset({'sixel'}), pkg.DEFAULT_CELL),))
    # The one capability probe of the session: it runs for the camera view's kitty too (kitty_camera),
    # so graphics: off still asks, as the key reader's own probe did; NO_COLOR and ASCII ask nothing.
    plain = env.get('NO_COLOR') or env.get('MHS_ASCII')
    wanted = pkg.choose(setting, {'sixel': True, 'cell': pkg.DEFAULT_CELL}, env, view=hopeful) != 'text'
    if not caps and tty and (wanted or not plain):
        try:
            caps, rest = pkg.probe(fd, write, env=env)
            pkg._caps.update(caps)
            if rest and hasattr(keys, 'buffer'):
                keys.buffer.extend(rest)
                keys.escape_started = time.monotonic()
        except OSError:
            caps = {}
    system = getattr(console, 'color_system', None) or 'truecolor'
    pkg.TIER = pkg.choose(setting, caps, env, system, view) if tty else 'text'
    pkg.WHY = (pkg.reason(setting, caps, env, system, view) or '') if tty else ('no terminal' if setting != 'off' else '')
    pkg.BUDGET = pkg.SSH_BUDGET if env.get('SSH_CONNECTION') else None
    pkg.CELL = caps.get('cell') or pkg.DEFAULT_CELL
    if pkg.TIER != 'text' and view:          # tmux's 16t answer may be stale (asked while detached)
        pkg.CELL = view['clients'][0][3]
    pkg.PASSTHROUGH = pkg.TIER == 'kitty' and pkg._tmux(env, caps)
    pkg._mux.clear()
    if view is not None:
        pkg._mux.update(env=env, view=view, next=pkg.clock() + pkg.TMUX_POLL_S, painted=pkg.clock(), token=object())
    elif tty and setting != 'off' and pkg.blind(env, caps):
        # A tmux it can't ask: repaint now and then, and ask tmux again (DA1, cell size) when a client
        # attaches (the window resizes) or while it draws text: boat_tui starts detached.
        pkg._mux.update(blind=True, env=env, painted=pkg.clock(), size=pkg._size_of(console), reprobe=pkg.clock() + pkg.TMUX_REPAINT_S)
    pkg.K = 1 if pkg.TIER == 'text' else max(1, min(4, round(pkg.CELL[0] / 3.5)))
    if pkg.TIER == 'sixel' and console is not None and not isinstance(console.file, pkg._Stream):
        pkg._stream = console.file = pkg._Stream(console.file)
    return pkg.TIER


def stop(console, write):
    """Free kitty's images and unwrap the stream; widgets draw braille again."""
    if pkg.TIER == 'kitty' and pkg._by_id:
        write(pkg._wrap(pkg.by_name('kitty').clear(pkg._by_id)))
    if console is not None and isinstance(console.file, pkg._Stream):
        console.file = console.file.inner
    pkg.TIER, pkg.K = 'text', 1
    pkg._mux.clear()
    pkg._reset()


def restart(keys, console, write, setting, env=None):
    """A new ``graphics`` setting mid-session: free the old tier, start the new one (the terminal
    is probed at most once a session). Clears the screen when images were up; returns the tier."""
    shown = pkg.TIER != 'text'
    pkg.stop(console, write)
    tier = pkg.start(keys, console, write, setting, env)
    if shown:
        write('\x1b[2J')
    return tier


def _spawn(run):
    """Run ``run`` off the UI thread (tests swap in a synchronous call)."""
    threading.Thread(target=run, name='pixels-tmux', daemon=True).start()


def _ask_tmux(env, token):
    """The background tmux query: its answer lands in ``_mux`` only if the session that asked still runs."""
    view = pkg.tmux_view(env)
    if pkg._mux.get('token') is token:
        pkg._mux.update(answer=view, busy=False)


def watch():
    """Under tmux, at most every TMUX_POLL_S: ask tmux about the clients again, on a background thread
    (tmux can take up to its 0.5 s timeout, and a key must never wait on it), and apply its last answer
    before laying out a frame. A new tier restarts; any other change, or TMUX_REPAINT_S, repaints every
    image. True when the next frame must be written even if its text did not change."""
    if not pkg._mux:
        return False
    now = pkg.clock()
    if pkg._mux.get('blind'):
        if pkg._reprobe(now):
            return True
        if pkg.TIER == 'text' or now - pkg._mux['painted'] < pkg.TMUX_REPAINT_S:
            return False
        pkg._mux['painted'] = now
        pkg.repaint()
        return True
    if now >= pkg._mux['next'] and not pkg._mux.get('busy'):
        pkg._mux.update(next=now + pkg.TMUX_POLL_S, busy=True)
        pkg._spawn(lambda env=pkg._mux['env'], token=pkg._mux['token']: pkg._ask_tmux(env, token))
    if 'answer' not in pkg._mux:
        return False
    env, view = pkg._mux['env'], pkg._mux.pop('answer')
    console = pkg._session.get('console')
    system = getattr(console, 'color_system', None) or 'truecolor'
    if pkg.choose(pkg.SETTING, pkg._caps, env, system, view) != pkg.TIER or (pkg.TIER != 'text' and view['clients'][0][3] != pkg.CELL):
        pkg.restart(pkg._session['keys'], console, pkg._session['write'], pkg.SETTING, env)
        return True
    if view != pkg._mux['view'] or now - pkg._mux['painted'] >= pkg.TMUX_REPAINT_S:
        pkg._mux.update(view=view, painted=now)
        pkg.repaint()
        return pkg.TIER != 'text'
    return False


def _listen(keys):
    """Copy the terminal replies the key reader drops (`KeyReader.absorb`) into a re-probe round in flight."""
    inner = getattr(keys, 'absorb', None)
    if inner is None or getattr(inner, 'imaging', False):
        return

    def absorb(value):
        hit = inner(value)
        if hit and 'round' in pkg._mux:
            pkg._mux['round'][1].extend(value)
        return hit
    absorb.imaging = True
    keys.absorb = absorb


def _reprobe(now):
    """Behind a tmux it can't ask: send the probe again on a resize (a client attached) or every
    TMUX_REPAINT_S while drawing text; tmux answers it itself, and its replies come back through the key
    reader (`_listen`), so nothing waits. A settled round with a new tier or cell restarts the tier."""
    session = pkg._session
    if 'round' in pkg._mux:
        started, data = pkg._mux['round']
        got, _ = pkg.parse(bytes(data))
        if not got['done'] and now - started < 4 * pkg.PROBE_S:
            return False
        del pkg._mux['round']
        if not got['done']:
            return False
        pkg._caps.update({k: got[k] for k in ('sixel', 'cell', 'xtversion') if got[k] is not None})
        pkg._caps['done'] = True
        console = session.get('console')
        system = getattr(console, 'color_system', None) or 'truecolor'
        tier = pkg.choose(pkg.SETTING, pkg._caps, pkg._mux['env'], system)
        if tier != pkg.TIER or (tier != 'text' and pkg._caps.get('cell') != pkg.CELL):
            pkg.restart(session['keys'], console, session['write'], pkg.SETTING, pkg._mux['env'])
            return True
        return False
    size = pkg._size_of(session.get('console'))
    if size != pkg._mux.get('size') or (pkg.TIER == 'text' and now >= pkg._mux['reprobe']):
        pkg._mux.update(size=size, reprobe=now + pkg.TMUX_REPAINT_S, round=(now, bytearray()))
        session['write'](pkg.PROBE)
    return False


def _size_of(console):
    try:
        return tuple(console.size) if console is not None else None
    except (OSError, ValueError):
        return None


def repaint():
    """Paint every image again: sixel parts on the next after(), kitty tiles on the next before()."""
    pkg._drawn.clear()
    if pkg.TIER == 'kitty':
        for tile in pkg._by_id.values():
            if tile.sent:
                tile.sent = False
                pkg._pending.append(tile)


def _wrap(data):
    """Kitty escapes for the terminal outside tmux: each in a passthrough DCS, its ESCs doubled."""
    return pkg.tmux.wrap(data) if pkg.PASSTHROUGH and data else data


def forget():
    """Drop the cached probe (a new terminal: the next start asks again)."""
    pkg._caps.clear()


def use(tier, cell=pkg.DEFAULT_CELL, budget=None):
    """Set the tier without a terminal (tests)."""
    pkg._reset()
    pkg.GENERATION += 1
    pkg.TIER, pkg.CELL, pkg.BUDGET = tier, cell, budget
    pkg.K = 1 if tier == 'text' else max(1, min(4, round(cell[0] / 3.5)))


def _reset():
    pkg._pool.clear()
    pkg._by_id.clear()
    pkg._free[:] = list(range(pkg.POOL, 0, -1))
    pkg._pending.clear()
    pkg._surfaces.clear()
    pkg._painted[:] = [0.0, 0, None, 0]
    pkg._sizes.clear()
    pkg._live.clear()
    pkg._drawn.clear()
    pkg._sent.clear()
    pkg._size = None
    pkg.STATS.update(upload=0, sixel=0, tiles=0)


def active():
    return pkg.TIER != 'text'


def _meter(size):
    now = pkg.clock()
    pkg._sent.append((now, size))
    while pkg._sent[0][0] < now - pkg.METER_S:      # trimmed here too: rate() runs only under a budget
        pkg._sent.popleft()


def rate(now=None):
    """Image bytes per second written over the last METER_S."""
    now = pkg.clock() if now is None else now
    while pkg._sent and pkg._sent[0][0] < now - pkg.METER_S:
        pkg._sent.popleft()
    return sum(size for _, size in pkg._sent) / pkg.METER_S
