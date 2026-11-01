"""The operator-declared boat state: normal, manual, battery swap or power cycle. ROS-free.

The owner TUI sets it (S, or m for manual control) and writes it to ``agent/operator.json`` under the data root,
which survives a reboot. Everyone else reads that file: the header (owner and viewers), status.json, boat_agent
when no TUI runs, and the next TUI. A battery swap or power cycle declared on an earlier boot is the restart
marker: the next owner TUI starts ``normal`` with ``restarted_after`` set for its session.

While the state is not normal the agent desk refuses changing verbs (``paused``, exit 7) and agent claim leases are
suspended, not lapsed (ros/authority.py ``Arbiter.pause``). Holds from the watchdogs and safety are unchanged.
"""

import os
import time

from mhseals_testing import data

SCHEMA = 'mhseals.operator/1'
STATES = ('normal', 'manual', 'battery_swap', 'power_cycle')
DOWN = ('battery_swap', 'power_cycle')       # the boat goes down: the restart marker
LABELS = {'normal': 'normal', 'manual': 'manual', 'battery_swap': 'battery swap', 'power_cycle': 'power cycle'}
CHIPS = {'manual': 'MANUAL', 'battery_swap': 'BATTERY SWAP', 'power_cycle': 'POWER CYCLE'}
BOOT_ID = '/proc/sys/kernel/random/boot_id'
_cache = {}


def path():
    return data.path('agent', 'operator.json')


def boot_id(source=BOOT_ID):
    try:
        with open(source) as handle:
            return handle.read().strip()
    except OSError:
        return ''


def normal():
    return {'state': 'normal', 'since_ns': None, 'eta_ns': None, 'reason': '', 'by': '', 'restarted_after': None}


def load(target=None):
    """The saved state; normal when there is none or it is unreadable."""
    doc = data.read_json(target or path())
    if not isinstance(doc, dict) or doc.get('state') not in STATES:
        return normal()
    return {**normal(), **{k: doc.get(k) for k in (*normal(), 'boot', 'pid')}}


def current(target=None):
    """``load`` cached by the file's mtime: the header asks every frame."""
    target = target or path()
    try:
        mtime = os.stat(target).st_mtime_ns
    except OSError:
        return normal()
    if _cache.get('key') != (str(target), mtime):
        _cache.update(key=(str(target), mtime), doc=load(target))
    return dict(_cache['doc'])


def change(state, eta_s=None, reason='', by='', prev=None, now_ns=None):
    """The record for a new declared ``state``; ``restarted_after`` stays only while the state stays normal."""
    if state not in STATES:
        raise ValueError(f'operator state {state!r}: one of {", ".join(STATES)}')
    now_ns = now_ns or time.time_ns()
    keep = (prev or {}).get('restarted_after') if state == 'normal' else None
    return {'state': state, 'since_ns': now_ns,
            'eta_ns': now_ns + int(eta_s * 1e9) if eta_s and state != 'normal' else None,
            'reason': str(reason or '')[:200], 'by': str(by or ''), 'restarted_after': keep}


def save(record, target=None):
    data.write_json(target or path(), dict(record, boot=boot_id(), pid=os.getpid()), SCHEMA)
    return record


def start(target=None, boot=None, now_ns=None):
    """At an owner TUI's start: a battery swap or power cycle declared on an earlier boot becomes
    ``restarted_after`` and the state goes normal; manual ends with the TUI that ran it; a swap declared on this
    boot (a TUI restart or takeover while the boat stayed up) stays. Saves and returns the record."""
    prev, boot = load(target), boot_id() if boot is None else boot
    if prev['state'] in DOWN and (not boot or prev.get('boot') != boot):
        record = dict(change('normal', reason='boat back', by='tui', now_ns=now_ns),
                      restarted_after={'state': prev['state'], 'since_ns': prev['since_ns'],
                                       'reason': prev['reason'], 'by': prev['by']})
    elif prev['state'] in DOWN:
        record = prev
    else:
        record = change('normal', prev=None, now_ns=now_ns, by='tui', reason='TUI started')
    return save({k: v for k, v in record.items() if k not in ('boot', 'pid')}, target)


def paused(record):
    return bool(record) and record.get('state', 'normal') != 'normal'


def clock(ns):
    return time.strftime('%H:%M', time.localtime(ns / 1e9)) if ns else '?'


def text(record):
    """``operator: battery swap since 11:30 (ETA 11:40)`` (and the reason); ``operator: normal``."""
    record = record or normal()
    if not paused(record):
        after = record.get('restarted_after')
        return (f"operator: normal; restarted after {LABELS.get(after.get('state'), '?')} at "
                f"{clock(after.get('since_ns'))}" if after else 'operator: normal')
    out = f"operator: {LABELS[record['state']]} since {clock(record.get('since_ns'))}"
    if record.get('eta_ns'):
        out += f" (ETA {clock(record['eta_ns'])})"
    return out + (f" — {record['reason']}" if record.get('reason') else '')


def summary(record):
    """The ``operator`` section of status and of paused replies."""
    record = record or normal()
    return {'state': record.get('state', 'normal'), 'since_ns': record.get('since_ns'),
            'eta_ns': record.get('eta_ns'), 'reason': record.get('reason', ''), 'by': record.get('by', ''),
            'restarted_after': record.get('restarted_after'), 'text': text(record)}


def refusal(record):
    """The hint of a changing verb refused while paused."""
    what = {'manual': 'the operator is driving or repositioning',
            'battery_swap': 'the boat goes down for a battery swap', 'power_cycle': 'the boat goes down for a power cycle'}
    return (f"{text(record)} — paused: {what[record['state']]}. Wait once with `follow --until "
            "operator_state:state=normal` (no retries); after a battery swap or power cycle re-check the stack, "
            'FCU params and RTK')
