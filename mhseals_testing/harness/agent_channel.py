"""The agent channel: one append-only JSONL file between agents and the TUI (spec agent-control section 2). ROS-free.

``bags/agent/channel.jsonl`` under the data root. The CLI and the hook append ``req``, ``renew`` and
``note`` lines; the session owner's desk tails it and appends ``reply`` lines. Every line is one
O_APPEND write plus fsync under 4 KB (``data.append_jsonl``), so any number of writers append
safely. Every line carries ``schema`` (``mhseals.agent/1``), ``t_ns``, ``host``, ``pid``, ``who``
and ``kind``:

=======  ============  =====================================================================
kind     writer        fields
=======  ============  =====================================================================
req      CLI           id (uuid4 hex), op, args (dict), claim (id or None), dry, deadline_ns
renew    blocking CLI  claim, act (the req id)
note     hook / CLI    phase (pre/post/text), tool_id, cmd (redacted, <= 300 chars), exit, dur_s, target
reply    TUI desk      re (req id), ok, error, state (queued/started/progress/done), result, hint
=======  ============  =====================================================================

The file is a same-uid cooperative channel, not a security boundary. ``Tail`` reads new complete
lines from a stored ``{inode, offset}``, bounded per call, and re-opens on rotation.
"""

import json
import os
from pathlib import Path
import re
import socket
import time
import uuid

from mhseals_testing import data

SCHEMA = 'mhseals.agent/1'
KINDS = ('req', 'renew', 'note', 'reply')
STATES = ('queued', 'started', 'progress', 'done')
REQUIRED = {'req': ('id', 'op', 'deadline_ns'), 'renew': ('claim', 'act'), 'note': ('phase',),
            'reply': ('re', 'ok', 'state')}
CMD_MAX = 300
TEXT_ARGS = ('question', 'note', 'reason')   # free text an agent writes: redacted like a cmd
TEXT_MAX = 1000
# Reply errors (``overridden: <who>`` carries its cause) and the CLI exit codes they map to (spec
# section 3): 2 refused/busy/blocked (the hint
# names the human action that unblocks), 3 revoked/overridden/cancelled/expired, 4 no TUI owner,
# 5 agents blocked, 6 timeout. Anything else is 1.
EXIT_CODES = {**dict.fromkeys(('refused', 'busy', 'busy_ui', 'not_holder', 'unknown_claim', 'no_mux',
                               'not_allowed', 'not_armed', 'motion_observe', 'bad_args', 'bad_op'), 2),
              **dict.fromkeys(('revoked', 'overridden', 'cancelled', 'expired', 'released', 'handover'), 3),
              'no_owner': 4, 'blocked': 5, 'timeout': 6}
ROTATE_BYTES = 20 * 1024 * 1024
_VALUE = r'''("[^"]*"|'[^']*'|[^\s'"]+)'''
_SECRETS = (   # (pattern, kept group): the value after the kept text becomes …
    re.compile(r'(?i)(\b(?:token|pass(?:wd|word)?|secret|key)\s*=\s*)' + _VALUE),
    re.compile(r'(?i)(--(?:token|pass(?:wd|word)?|secret|api-?key|key)(?:\s+|=))' + _VALUE),
    re.compile(r'(\bsshpass\b(?:\s+-[^p\s]\S*)*\s+-p\s*)' + _VALUE),
    re.compile(r'(?i)(\bauthorization\s*:\s*(?:(?:bearer|basic|token)\s+)?)' + _VALUE),
    re.compile(r'(?i)(\bbearer\s+)' + _VALUE))


def exit_code(reply):
    """0 for an ok reply, else the CLI exit code of its error."""
    return 0 if reply.get('ok') else EXIT_CODES.get(str(reply.get('error')).split(':')[0], 1)


def channel_path():
    return data.path('agent', 'channel.jsonl')


def redact(cmd, limit=CMD_MAX):
    """``cmd`` with secret values hidden, cut to ``limit`` chars.

    ``token=``/``password=``/``secret=``/``key=``, ``--password x``, ``sshpass -p x`` and
    ``Authorization: Bearer x``; a quoted value is hidden whole.
    """
    text = str(cmd)
    for pattern in _SECRETS:
        text = pattern.sub(r'\1…', text)
    return text[:limit]


def line(kind, who, **fields):
    """One channel record (not yet written)."""
    return {'schema': SCHEMA, 't_ns': time.time_ns(), 'host': socket.gethostname(), 'pid': os.getpid(),
            'who': who, 'kind': kind, **fields}


def append(record, path=None):
    """Append ``record``; returns it."""
    data.append_jsonl(Path(path or channel_path()), record)
    return record


def request(op, who, args=None, claim=None, dry=False, request_s=5.0, path=None):
    """Append a ``req``; its ``id`` is what the reply's ``re`` names. Free text is redacted."""
    deadline = time.time_ns() + int(request_s * 1e9)
    args = {k: redact(v, TEXT_MAX) if k in TEXT_ARGS and isinstance(v, str) else v for k, v in dict(args or {}).items()}
    return append(line('req', who, id=uuid.uuid4().hex, op=op, args=args, claim=claim,
                       dry=bool(dry), deadline_ns=deadline), path)


def renew(who, claim, act, path=None):
    return append(line('renew', who, claim=claim, act=act), path)


def note(who, phase, cmd='', tool_id=None, exit=None, dur_s=None, target=None, path=None):
    return append(line('note', who, phase=phase, tool_id=tool_id, cmd=redact(cmd), exit=exit,
                       dur_s=dur_s, target=target), path)


def reply(re, who, ok=True, state='done', error='', result=None, hint='', path=None):
    """Append a ``reply`` to request ``re``; a result too big for one line keeps its short keys."""
    record = line('reply', who, re=re, ok=bool(ok), error=error, state=state, result=result, hint=hint)
    try:
        return append(record, path)
    except ValueError:
        if not isinstance(result, dict):
            raise
        short = {k: v for k, v in result.items() if len(json.dumps(v, default=str)) <= 200}
        return append(dict(record, result={**short, 'truncated': True}), path)


def invalid(record):
    """Why ``record`` is not a channel line, or None."""
    if not isinstance(record, dict) or record.get('schema') != SCHEMA:
        return 'not an mhseals.agent/1 line'
    kind = record.get('kind')
    if kind not in KINDS:
        return f'unknown kind {kind!r}'
    if not isinstance(record.get('who'), str) or not record['who']:
        return f'{kind} without who'
    if not isinstance(record.get('t_ns'), int):
        return f'{kind} without t_ns'
    missing = [k for k in REQUIRED[kind] if k not in record]
    return f"{kind} without {', '.join(missing)}" if missing else None


class Tail:
    """Reads new complete lines from ``{inode, offset}``; a new inode or a shorter file starts over."""

    def __init__(self, path=None, inode=None, offset=0):
        self.path = Path(path or channel_path())
        self.inode, self.offset = inode, offset

    def position(self):
        return {'inode': self.inode, 'offset': self.offset}

    def seek_end(self):
        """Skip everything already in the file (a CLI waits only for lines after its request)."""
        try:
            st = os.stat(self.path)
        except FileNotFoundError:
            self.inode, self.offset = None, 0
            return self.position()
        self.inode, self.offset = st.st_ino, st.st_size
        return self.position()

    def read(self, max_lines=64, max_s=0.005, clock=time.monotonic):
        """(records, errors): at most ``max_lines`` lines or ``max_s`` seconds of parsing.

        A torn last line (no newline yet) stays unread until it is complete. Malformed lines are
        skipped, each giving one error string.
        """
        try:
            st = os.stat(self.path)
        except FileNotFoundError:
            return [], []
        if st.st_ino != self.inode or st.st_size < self.offset:
            self.inode, self.offset = st.st_ino, 0   # rotated, replaced or truncated
        if st.st_size == self.offset:
            return [], []
        with open(self.path, 'rb') as handle:
            handle.seek(self.offset)
            chunk = handle.read(min(st.st_size - self.offset, max_lines * data.MAX_LINE))
        records, errors, start = [], [], clock()
        lines = chunk.split(b'\n')         # never splitlines: a stray \r must not end a line
        for raw in [part + b'\n' for part in lines[:-1]][:max_lines]:
            self.offset += len(raw)
            if not raw.strip():
                continue
            try:
                record = json.loads(raw)
                error = invalid(record)
            except ValueError:
                error = 'malformed JSON line'
            if error:
                errors.append(error)
            else:
                records.append(record)
            if clock() - start > max_s:
                break
        return records, errors


def replied(position, path=None):
    """Request ids that already have a reply after ``position`` (dedupes a crash between act and persist)."""
    tail = Tail(path, **(position or {}))
    seen = set()
    while True:
        before = tail.offset
        records, errors = tail.read(max_lines=4096, max_s=float('inf'))
        seen.update(r['re'] for r in records if r['kind'] == 'reply')
        if tail.offset == before:
            return seen


def rotate(active, path=None, now=None, limit=ROTATE_BYTES):
    """At TUI start: rename a file over ``limit`` bytes to ``channel.<YYYYMMDD-HHMMSS>[-N].jsonl``.

    Never while a claim is ``active``, and never over an earlier rotation. Returns the new path, or None.
    """
    path = Path(path or channel_path())
    try:
        if active or path.stat().st_size <= limit:
            return None
    except FileNotFoundError:
        return None
    stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(now))
    n = 0
    while (target := path.with_name(f"channel.{stamp}{f'-{n}' if n else ''}.jsonl")).exists():
        n += 1
    os.replace(path, target)
    return target
