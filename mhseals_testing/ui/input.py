"""The TUI's input layer over mhseals_hardware's KeyReader.

One KeyReader for the whole session (App.attached): it reads in bulk, the session's one probe
(ui/imaging/probe.py) tells it the terminal (`terminal`: kitty events, tmux/cx legacy keys, a
plain shell), and manual mode turns key events on in the same reader (`events`), so its queue
and terminal state carry across. Hold-to-confirm timing per terminal is config/input.yaml.
"""

from pathlib import Path

from mhseals_hardware.keyboard import Hold, KeyReader, Terminal
from mhseals_testing.strict_yaml import load_strict

FILE = Path(__file__).resolve().parents[2] / 'config' / 'input.yaml'   # source tree
FIELDS = ('confirm_hold', 'first_gap', 'gap')


def load(path=FILE):
    """``{kind: {confirm_hold, first_gap, gap}}`` for every Terminal kind; ValueError when malformed."""
    data = load_strict(Path(path).read_text(), path) or {}
    if data.get('schema') != 'mhseals.input/1':
        raise ValueError(f'{path}: schema must be mhseals.input/1')
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
    kitty events (and every mouse button), else its key-repeat stream."""
    term = terminal(keys)
    timing = (timing or load())[term.kind]
    releases = term.releases or name.startswith('mouse-')
    return Hold(name, timing['confirm_hold'], releases, timing['first_gap'], timing['gap'])
