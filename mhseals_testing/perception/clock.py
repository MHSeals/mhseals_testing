"""Clock offset of a remote ZED host from DDS source timestamps (no rosapi, no sockets)."""


def offset(source_ts, recv_ts):
    """Seconds the source clock is ahead of ours: source_timestamp - receive time.

    Includes one-way delay (small on the boat LAN), so it is an upper bound on skew.
    Returns None for non-finite or non-positive stamps.
    """
    for ts in (source_ts, recv_ts):
        if not ts > 0 or ts == float('inf'):
            return None
    return source_ts - recv_ts
