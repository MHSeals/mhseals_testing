"""One structured ask for a settings change. ROS-free.

Anything that asks an agent or the operator to change a setting builds it here, so the ask always names the key,
its current and target values, where it lives (layer and file or TUI row), why, when it is allowed and the one
command that does it. ``text`` is the same ask as one line for a hint, a notice or a runbook.
"""

import json
import shlex

FIELDS = ('key', 'current', 'target', 'where', 'why', 'when', 'command')


def show(value):
    """A value as typed on the command line: lists and mappings as compact JSON (YAML reads it), else str."""
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, separators=(', ', ': '))
    if isinstance(value, bool):
        return 'on' if value else 'off'
    return 'unset' if value is None else str(value)


def set_command(key, target, claim='ID', why='...', host=None):
    """The exact ``boat_agent set`` line for ``key=target`` (quoted for the shell); ``host`` adds the
    BOAT_AGENT_TARGET that reaches another machine's TUI."""
    word = shlex.quote(f'{key}={show(target)}')
    env = f'BOAT_AGENT_TARGET={shlex.quote(host)} ' if host else ''
    return f'{env}boat_agent set {word} --claim {claim} --why {shlex.quote(why)}'


def setting_ask(key, current, target, where, why, command=None, when=''):
    """The ask as a dict (FIELDS); ``command`` defaults to the ``boat_agent set`` line."""
    return {'key': key, 'current': current, 'target': target, 'where': where, 'why': why,
            'when': when or 'now', 'command': command or set_command(key, target)}


def text(ask):
    """The ask as one line: what to change, from what to what, where, why, when, and the command."""
    when = '' if ask.get('when', 'now') == 'now' else f"; allowed {ask['when']}"
    return (f"set {ask['key']} from {show(ask['current'])} to {show(ask['target'])} in {ask['where']} "
            f"({ask['why']}{when}): {ask['command']}")


def assignments(words, reads=False):
    """(``{key: value}``, None) from ``KEY=VALUE`` words (set_command's inverse), or (None, hint). With ``reads`` a
    bare ``KEY`` is a read (value None)."""
    out = {}
    for word in words:
        key, separator, value = word.partition('=')
        if (not separator and not reads) or not key.strip() or key.strip() in out:
            return None, f'{word!r}: expected KEY=VALUE, each key once'
        out[key.strip()] = value.strip() if separator else None
    return out, None


def reads_only(words):
    """``set KEY [KEY ...]``: every word a bare key, a read."""
    return bool(words) and all('=' not in word for word in words)
