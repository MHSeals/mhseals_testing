"""``boat_trial config``: log, diff, revert, name, observe and set over the config history.

    config log [SUB] [--since 3d] [--session S] [--json]
    config diff [A] [B] [--sub SUB] [--json]       (no refs: previous -> current; one ref: what it did)
    config revert REF [--include-calibration] [--reboot] [--dry-run] [--yes] [--json]
    config name REF NAME [--note TEXT]
    config observe [SUB]
    config set SUB FILE [--reboot] [--yes] [--json]

Exit codes: 0 done, 1 nothing to do, 2 refused (blockers, unknown ref, or no ``--yes``: the preview
is printed), 3 failed and rolled back, 4 the rollback also failed (state re-observed).
Any write (``revert`` or ``set``) to a subsystem with a ``confirm_word`` (fcu) needs a terminal where the
operator types that word; ``set fcu`` also needs ``--reboot``. It is never answerable headless, ``--yes`` or not.

Subsystems come from adapters, ``{name: adapter}``; ROS and file knowledge lives there, not here.
An adapter provides: ``read() -> (value, raw, meta) | None``, ``blockers() -> [str]``,
``apply(version, reboot=False)`` (raises ``ApplyFailed`` after rolling back, ``RollbackFailed`` when
that failed too; returns ``'confirmed'`` or ``'pending'``) and, for ``set``, ``parse(text, current)
-> (value, raw)``. Optional: ``calibration(path) -> bool`` (left out of reverts unless asked),
``needs_reboot(paths) -> bool`` and ``confirm_word`` (a word the operator must type).
"""

import argparse
import datetime
import json
import re
import sys

from mhseals_testing.history import History

DONE, NOTHING, REFUSED, ROLLED_BACK, ROLLBACK_FAILED = 0, 1, 2, 3, 4


class ApplyFailed(Exception):
    """The apply or its verify failed and the previous state was restored."""


class RollbackFailed(Exception):
    """The apply failed and the previous state could not be restored."""


def flatten(value, prefix=''):
    """``{dotted.path: leaf}``; lists and scalars are leaves, so a matrix changes as one row."""
    if not isinstance(value, dict):
        return {prefix: value}
    flat = {}
    for key, node in value.items():
        flat.update(flatten(node, f'{prefix}.{key}' if prefix else str(key)))
    return flat


def diff_rows(before, after):
    """Changed paths as ``{group, label, path, before, after, unit}``, sorted by path."""
    old, new = flatten(before), flatten(after)
    rows = []
    for path in sorted(old.keys() | new.keys()):
        if old.get(path) != new.get(path) or (path in old) != (path in new):
            group, _, label = path.partition('.')
            rows.append({'group': group, 'label': label or group, 'path': path,
                         'before': old.get(path), 'after': new.get(path), 'unit': ''})
    return rows


def parse_since(text, now=None):
    """The cutoff for ``3d`` / ``12h`` / ``90m``."""
    match = re.fullmatch(r'(\d+)([dhm])', text)
    if not match:
        raise ValueError(f'--since {text!r}: expected a number and d, h or m, e.g. 3d')
    unit = {'d': 'days', 'h': 'hours', 'm': 'minutes'}[match[2]]
    return (now or datetime.datetime.now().astimezone()) - datetime.timedelta(**{unit: int(match[1])})


def _show(value):
    return '-' if value is None else json.dumps(value)


class Command:
    """One invocation: the history, the adapters and where output goes."""

    def __init__(self, history, adapters, out, ask, json_out=False):
        self.history, self.adapters, self.out, self.ask, self.json = history, adapters, out, ask, json_out

    def emit(self, doc, lines):
        print(json.dumps(doc, default=str) if self.json else '\n'.join(lines), file=self.out)

    def summary(self, change, rows=()):
        return {k: change.get(k) for k in ('id', 'time', 'name', 'subsystem', 'source', 'applied', 'revert_of')} | {
            'sessions': [change['session']] if change.get('session') else [], 'changes': list(rows)}

    def rows_text(self, rows):
        return [f'  {r["path"]}  {_show(r["before"])} -> {_show(r["after"])}' for r in rows] or ['  no differences']

    def refuse(self, reason, **extra):
        self.emit({'outcome': 'refused', 'reasons': [reason], **extra}, [f'refused: {reason}'])
        return REFUSED

    def resolve(self, ref, sub=None):
        try:
            return self.history.resolve(ref, sub)
        except KeyError as error:
            self.refuse(str(error.args[0]))

    # ---- read-only verbs

    def log(self, args):
        changes = self.history.changes(args.sub)
        if args.since:
            cutoff = parse_since(args.since)
            changes = [c for c in changes if datetime.datetime.fromisoformat(c['time']) >= cutoff]
        if args.session:
            changes = [c for c in changes if c.get('session') == args.session]
        self.emit([self.summary(c) for c in changes],
                  [f'{c["id"]}  {c["time"]}  {c["subsystem"]:<8} {c["source"]:<14} {c.get("name") or ""}  '
                   f'{"; ".join(c.get("summary") or [])}'.rstrip() for c in changes] or ['no config changes'])
        return DONE

    def diff(self, args):
        if args.b:
            first, second = self.resolve(args.a, args.sub), self.resolve(args.b, args.sub)
            if first is None or second is None:
                return REFUSED
        else:
            second = self.resolve(args.a or 'current', args.sub)
            if second is None:
                return REFUSED
            earlier = [c for c in self.history.changes(second['subsystem']) if c['id'] < second['id']]
            first = earlier[-1] if earlier else None
        before = self.history.version(first)['value'] if first else {}
        rows = diff_rows(before, self.history.version(second)['value'])
        self.emit(self.summary(second, rows), [f'{second["id"]} {second["subsystem"]}', *self.rows_text(rows)])
        return DONE

    def name(self, args):
        change = self.resolve(args.ref)
        if change is None:
            return REFUSED
        self.history.label(change['id'], name=args.name, note=args.note)
        self.emit({'id': change['id'], 'name': args.name, 'note': args.note}, [f'{change["id"]} named {args.name!r}'])
        return DONE

    def observe(self, args):
        subs = [args.sub] if args.sub else sorted(self.adapters)
        if args.sub and args.sub not in self.adapters:
            return self.refuse(f'unknown subsystem {args.sub!r}; one of {", ".join(sorted(self.adapters))}')
        recorded = []
        for sub in subs:
            state = self.adapters[sub].read()
            change = self.history.observe(sub, *state) if state else None
            if change:
                recorded.append(change)
        self.emit([self.summary(c) for c in recorded], [f'{c["id"]} {c["subsystem"]} observed' for c in recorded]
                  or ['nothing changed'])
        return DONE if recorded else NOTHING


def build_parser():
    parser = argparse.ArgumentParser(prog='boat_trial config')
    verbs = parser.add_subparsers(dest='verb', required=True)

    def verb(name, *flags, positional=()):
        p = verbs.add_parser(name)
        for arg, kw in positional:
            p.add_argument(arg, **kw)
        for flag in flags:
            p.add_argument(*flag[0], **flag[1])
        return p

    json_flag = (('--json',), dict(action='store_true'))
    sub_flag = (('--sub',), dict(help='restrict refs to one subsystem'))
    verb('log', json_flag, (('--since',), {}), (('--session',), {}), positional=[('sub', dict(nargs='?'))])
    verb('diff', json_flag, sub_flag, positional=[('a', dict(nargs='?')), ('b', dict(nargs='?'))])
    verb('name', (('--note',), {}), positional=[('ref', {}), ('name', {})])
    verb('observe', positional=[('sub', dict(nargs='?'))])
    return parser


def main(argv=None, adapters=None, history=None, out=None, ask=None):
    """Run one config verb; returns the exit code. ``ask`` is None unless the entry point found a terminal."""
    args = build_parser().parse_args(argv)
    args.dry_run = getattr(args, 'dry_run', False)
    command = Command(history or History(), adapters or {}, out or sys.stdout, ask, getattr(args, 'json', False))
    try:
        return getattr(command, args.verb)(args)
    except ValueError as error:
        return command.refuse(str(error))


def entry(argv=None, **kw):
    """Entry wiring: a typed answer is possible only when stdin is a terminal."""
    return main(argv, ask=input if sys.stdin.isatty() else None, **kw)
