"""The TUI's input layer over mhseals_hardware's KeyReader.

One KeyReader for the whole session (App.attached): it reads in bulk, the session's one probe
(ui/imaging/probe.py) tells it the terminal (`terminal`: kitty events, tmux/cx legacy keys, a
plain shell), and manual mode turns key events on in the same reader (`events`), so its queue
and terminal state carry across. Hold-to-confirm timing per terminal, which actions confirm by a hold
and whether the mouse reports are config/input.yaml. `Keys` wraps the session reader for the mouse.
"""

from contextlib import contextmanager, nullcontext
from pathlib import Path

from mhseals_hardware.keyboard import MOUSE_OFF, MOUSE_ON, Hold, KeyReader, Mouse, Terminal, terminal_write
from mhseals_testing.strict_yaml import load_strict

FILE = Path(__file__).resolve().parents[2] / 'config' / 'input.yaml'   # source tree
FIELDS = ('confirm_hold', 'first_gap', 'gap')
ACTIONS = ('arm', 'release', 'take', 'quit_armed')    # the confirms a hold may replace (``hold:``)


def load_file(path=FILE):
    """The whole input.yaml, checked; ValueError when malformed."""
    data = load_strict(Path(path).read_text(), path) or {}
    if data.get('schema') != 'mhseals.input/1':
        raise ValueError(f'{path}: schema must be mhseals.input/1')
    hold = data.get('hold', {})
    if not isinstance(hold, dict) or not set(hold) <= set(ACTIONS) or not all(
            isinstance(v, bool) for v in hold.values()):
        raise ValueError(f'{path}: hold maps {", ".join(ACTIONS)} to true or false')
    if not isinstance(data.get('mouse', False), bool):
        raise ValueError(f'{path}: mouse must be true or false')
    load_terminals(data, path)
    return data


def load(path=FILE):
    """``{kind: {confirm_hold, first_gap, gap}}`` for every Terminal kind; ValueError when malformed."""
    return load_file(path)['terminals']


def by_hold(action, path=FILE):
    """True when ``action`` (one of ACTIONS) confirms by a hold; an unreadable file keeps its popup."""
    try:
        return load_file(path).get('hold', {}).get(action, False)
    except (OSError, ValueError):
        return False


def mouse_on(path=FILE):
    """True when the session turns mouse reports on (``mouse:``); an unreadable file leaves them off."""
    try:
        return load_file(path).get('mouse', False)
    except (OSError, ValueError):
        return False


def load_terminals(data, path):
    """``terminals:`` checked: every Terminal kind, each timing in (0, 5] s."""
    terminals = data.get('terminals') or {}
    if set(terminals) != set(Terminal.KINDS):
        raise ValueError(f'{path}: terminals must be exactly {", ".join(Terminal.KINDS)}')
    for kind, timing in terminals.items():
        if not isinstance(timing, dict) or set(timing) != set(FIELDS) or not all(
                isinstance(timing[f], (int, float)) and 0 < timing[f] <= 5 for f in FIELDS):
            raise ValueError(f'{path}: terminals.{kind} needs {", ".join(FIELDS)} in (0, 5] s')
    return terminals


def terminal(keys):
    """The reader's Terminal (``unknown`` for a reader without one, as tests pass)."""
    return getattr(keys, 'terminal', None) or Terminal()


def attach(keys, caps, env=None):
    """After the session probe: tell the reader what the terminal reports."""
    if keys is not None and hasattr(keys, 'terminal'):
        keys.terminal = Terminal.detect(caps, env)


def events(keys):
    """Key events (press/repeat/release) for a block: the session reader's own mode when it has
    one, else a reader of its own (no session reader, as before the session starts)."""
    if keys is not None and hasattr(keys, 'events'):
        return keys.events()
    return KeyReader(report_events=True)


def chords_unavailable(keys):
    """Why held keys cannot chord on this terminal (``tmux``), or None: the visible indicator."""
    return terminal(keys).chords_unavailable


def hold(name, keys, timing=None):
    """A `Hold` (hold-to-confirm) for key ``name`` timed for the reader's terminal: its release with
    kitty events on in the reader (and every mouse button), else its key-repeat stream. A kitty
    terminal outside manual mode sends plain keys with no release, so its hold is the repeat stream
    too: a tap never confirms by waiting."""
    term = terminal(keys)
    timing = (timing or load())[term.kind]
    releases = (term.releases and bool(getattr(keys, 'report_events', False))) or name.startswith('mouse-')
    return Hold(name, timing['confirm_hold'], releases, timing['first_gap'], timing['gap'])


def mouse(keys):
    """A block in which ``keys`` returns every mouse report (a hold-to-confirm popup); a reader that is
    not a `Keys` has none to pass."""
    return keys.mouse() if isinstance(keys, Keys) else nullcontext(keys)


class Keys:
    """The session reader with the mouse: wheel up/down are the up/down keys (lists, popups, logs) and
    other reports are dropped (a read returns None, as a timeout), except inside `mouse` (a hold popup,
    which takes a button press-and-hold). With key events on (manual driving) every report is dropped,
    so the wheel never drives. Everything else is the reader's own (terminal, events, fd ...)."""

    _OWN = ('reader', 'passing', 'reporting')

    def __init__(self, reader):
        object.__setattr__(self, 'reader', reader)
        object.__setattr__(self, 'passing', 0)
        object.__setattr__(self, 'reporting', False)

    def __getattr__(self, name):
        return getattr(self.reader, name)

    def __setattr__(self, name, value):
        if name in self._OWN:
            object.__setattr__(self, name, value)
        else:
            setattr(self.reader, name, value)

    def __enter__(self):
        """Mouse reports on, for a real terminal only (a test's reader writes nothing)."""
        if isinstance(self.reader, KeyReader) and self.reader.fd is not None:
            terminal_write(MOUSE_ON)
            self.reporting = True
        return self

    def __exit__(self, *_):
        if self.reporting:
            self.reporting = False
            terminal_write(MOUSE_OFF)

    @contextmanager
    def events(self):
        with self.reader.events():
            yield self

    @contextmanager
    def mouse(self):
        self.passing += 1
        try:
            yield self
        finally:
            self.passing -= 1

    def read(self, timeout=None):
        key = self.reader.read(timeout)
        if not isinstance(key, Mouse) or self.passing:
            return key
        if key.action == 'wheel' and key.button in ('up', 'down') and not getattr(self.reader, 'report_events', False):
            return key.button
        return None
