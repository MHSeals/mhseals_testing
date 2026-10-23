"""Count DDS participants per host from endpoint gids; pure, no ROS imports.

An endpoint gid is a 16-byte RTPS GUID: a 12-byte participant prefix plus a
4-byte entity id. Fast DDS puts vendor and host id in prefix bytes 0-3, so
participants of one host share those bytes (verify once on the boat that
squirtle and blastoise differ). A participant with no endpoint is invisible.
"""

from fnmatch import fnmatchcase

PREFIX = 12      # bytes of a gid naming the participant
HOST = 4         # leading prefix bytes shared by every participant of a host


def participants(gids):
    """Unique participant prefixes (bytes) among endpoint ``gids``; short gids are skipped."""
    return {bytes(g)[:PREFIX] for g in gids if len(bytes(g)) >= PREFIX}


def by_host(gids, hosts=None, nodes=()):
    """``{host: participant count}`` for endpoint ``gids``.

    ``nodes`` is [(node_name, gid)]. ``hosts`` maps a node-name glob to a host
    name; a host group takes the label of its first matching node and falls
    back to the hex of the host bytes.
    """
    groups = {}
    for prefix in participants(gids):
        groups.setdefault(prefix[:HOST], set()).add(prefix)
    labels = {}
    for name, gid in nodes:
        key = bytes(gid)[:HOST]
        for pattern, host in (hosts or {}).items():
            if fnmatchcase(name, pattern):
                labels.setdefault(key, host)
                break
    counts = {}
    for key, found in groups.items():
        label = labels.get(key, key.hex())
        counts[label] = counts.get(label, 0) + len(found)
    return counts


def facts(gids, hosts=None, nodes=(), now=0.0):
    """Health Facts: ``dds.participants`` (total) and ``dds.participants_by_host``."""
    from .model import Fact
    counts = by_host(gids, hosts, nodes)
    return {'dds.participants': Fact(sum(counts.values()), time=now),
            'dds.participants_by_host': Fact(counts, time=now, data=dict(counts))}
