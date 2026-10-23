"""Socket liveness for the rosbridge and zenoh transports, read from /proc/net/tcp.

The TUI never SSHes anywhere, so a remote link is judged by the local TCP table:
rosbridge is up when we hold an ESTABLISHED connection to the remote :9090, and
zenoh when peers hold ESTABLISHED connections to our :7447. IPv4 only, as on the boat LAN.
"""

from collections import namedtuple
import socket
import struct

ESTABLISHED = '01'
ROSBRIDGE_PORT = 9090
ZENOH_PORT = 7447

Socket = namedtuple('Socket', 'local_ip local_port remote_ip remote_port state')


def _address(field):
    """'0100007F:1F90' (host-order hex, little-endian on x86/arm) -> ('127.0.0.1', 8080)."""
    ip_hex, port_hex = field.split(':')
    return socket.inet_ntoa(struct.pack('<I', int(ip_hex, 16))), int(port_hex, 16)


def parse_proc_net_tcp(text):
    """Sockets from /proc/net/tcp content; the header and malformed rows are skipped."""
    sockets = []
    for line in text.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 4:
            continue
        try:
            local, remote = _address(fields[1]), _address(fields[2])
        except (ValueError, struct.error):
            continue
        sockets.append(Socket(*local, *remote, fields[3]))
    return sockets


def read_proc_net_tcp(path='/proc/net/tcp'):
    """Sockets from the live table, or [] when it cannot be read."""
    try:
        with open(path) as handle:
            return parse_proc_net_tcp(handle.read())
    except OSError:
        return []


def rosbridge_connected(sockets, host_ip, port=ROSBRIDGE_PORT):
    """True when an ESTABLISHED connection to ``host_ip:port`` exists."""
    return any(s.state == ESTABLISHED and s.remote_ip == host_ip and s.remote_port == port
               for s in sockets)


def zenoh_peers(sockets, port=ZENOH_PORT):
    """Sorted unique peer IPs ESTABLISHED on our local ``port`` (outgoing links to a router
    on the same port are not peers of ours and are ignored)."""
    return sorted({s.remote_ip for s in sockets
                   if s.state == ESTABLISHED and s.local_port == port})
