"""Who controls a boat_test session, for every client alike: the control tracker. Pure and ROS-free.

The session registry (harness/registry.py) holds the one owner seat: the TUI client that controls the boat.
Viewers are its ``watch`` guests: boat_test processes that render the same feeds and send no commands but
Space and E. An agent drives only through a claim on the owner's arbiter, so an active agent claim in the
authority record (harness/authority.py) is in control while it lasts, hosted by the owner's client.
``tracker`` turns the registry entry and that record into one dict; every client computes the same one.

Client identity comes from the environment tools/boat_tui sets: ``BOAT_TUI_CLIENT`` (its tmux session) and
``BOAT_TUI_FROM`` (the ssh client address, else the host it was started on).
"""

import os

VIEW = 'watch'          # the registry guest procedure of a viewer


def identity(environ=None):
    """``client`` and ``from`` for this process's registry record."""
    environ = os.environ if environ is None else environ
    ssh = (environ.get('SSH_CLIENT') or '').split()
    return {'client': environ.get('BOAT_TUI_CLIENT') or '', 'from': environ.get('BOAT_TUI_FROM') or
            (ssh[0] if ssh else '')}


def name(record):
    """``boat@blastoise-odroid``: the client and host of a registry record (``pid N`` without a client)."""
    record = record or {}
    client = record.get('client') or (f"pid {record['pid']}" if record.get('pid') else '?')
    return f"{client}@{record['host']}" if record.get('host') else client


def viewers(entry, alive):
    """The live viewers: {pid: record}. A viewer whose process is gone (killed, lost) is not counted."""
    return {pid: guest for pid, guest in ((entry or {}).get('guests') or {}).items()
            if guest.get('procedure') == VIEW and alive(int(pid))}


def is_viewer(entry, pid, alive):
    return str(pid) in viewers(entry, alive)


def tracker(entry, record, me, alive, now):
    """The tracker for the client ``me`` (its pid) at wall time ``now``.

    ``controller``: kind ``human`` (the owner seat), ``agent`` (an active agent claim, ``via`` the seat),
    ``stale`` (the seat's process is gone: nobody controls, its children wait to be adopted), ``unknown``
    (a live seat but no fresh authority record, ``record`` None: an agent may hold it, so never claim a
    human) or ``none``;
    ``client``, ``host``, ``from``, ``pid``, ``since`` (wall s) and for an agent ``who`` and ``claim``.
    ``viewers`` counts live viewers, ``role`` is ``control``, ``view`` or ``none`` for ``me``.
    """
    entry = entry or {}
    owner = entry.get('owner')
    seen = viewers(entry, alive)
    role = ('control' if owner and owner.get('pid') == me else 'view' if str(me) in seen else 'none')
    out = {'viewers': len(seen), 'role': role, 'handover': bool(entry.get('handover'))}
    if not owner:
        return dict(out, controller={'kind': 'none'})
    seat = {key: owner.get(key) for key in ('client', 'host', 'from', 'pid')}
    seat['name'] = name(owner)
    if not alive(owner['pid']):
        return dict(out, controller=dict(seat, kind='stale', since=owner.get('started')))
    if record is None:          # missing or stale (ViewerFeed pauses it): who drives is not known
        return dict(out, controller=dict(seat, kind='unknown', since=None))
    claim = next((c for c in record.get('claims', ())
                  if c.get('owner') == 'agent' and c.get('state', 'active') == 'active'), None)
    if claim:
        return dict(out, controller=dict(seat, kind='agent', who=claim.get('who'), claim=claim.get('id'),
                                         since=(claim.get('since_ns') or 0) / 1e9 or None,
                                         agent_host=claim.get('host'), via=seat['name']))
    return dict(out, controller=dict(seat, kind='human', since=owner.get('control_since') or owner.get('started')))


def summary(view):
    """One line for a log or ``boat_trial status``: ``agent agent-12 c4 via boat@host · 2 viewing``."""
    ctl = view['controller']
    text = {'none': 'nobody controls',
            'unknown': f"controller unknown (no fresh authority record) via {ctl.get('name')}", 'stale': f"stale seat {ctl.get('name')} (process gone)",
            'human': f"human {ctl.get('name')}" + (f" from {ctl['from']}" if ctl.get('from') else ''),
            'agent': f"agent {ctl.get('who')} {ctl.get('claim')} via {ctl.get('via')}"}[ctl['kind']]
    return f"{text} · {view['viewers']} viewing"


def viewer_who(client):
    """The channel ``who`` of a viewer's request: never an agent id (agents never send ``revoke``)."""
    return f'view:{client}'


def request_revoke(client, path=None):
    """A viewer's E: ask the owner's desk to revoke the active agent claim (the viewer already held the boat).
    Best effort: a missing channel only loses the revoke, never the stop."""
    from mhseals_testing.harness import agent_channel
    try:
        return agent_channel.request('revoke', viewer_who(client), path=path)
    except OSError:
        return None


def main():
    """``python -m mhseals_testing.harness.control`` (tools/boat_tui status): the tracker from the registry and
    the owner's status.json (its authority record, stale after STATUS_STALE_S), then one line per client."""
    import time
    from mhseals_testing import data
    from mhseals_testing.harness.registry import Registry, alive, runtime_dir
    entry = Registry().state()
    status = data.read_json(runtime_dir() / 'status.json', {}) or {}
    fresh = time.time() - (status.get('t_ns') or 0) / 1e9 < STATUS_STALE_S
    record = ((status.get('authority') or {}).get('record') if fresh else None)
    view = tracker(entry, record, None, alive, time.time())
    print(summary(view))
    owner = entry.get('owner')
    if owner:
        print(f"  control  {name(owner)}  pid {owner['pid']}  from {owner.get('from') or '-'}"
              f"{'' if alive(owner['pid']) else '  (gone)'}")
    for pid, guest in viewers(entry, alive).items():
        print(f"  view     {name(dict(guest, pid=pid))}  pid {pid}  from {guest.get('from') or '-'}")


STATUS_STALE_S = 5.0

if __name__ == '__main__':
    main()
