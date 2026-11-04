"""The backend's serving end: hello and refusal, single instance, silent clients."""

from dataclasses import replace
import socket
import time

import pytest

from mhseals_testing.backend import protocol
from mhseals_testing.backend.link import Busy, Server

pytestmark = pytest.mark.smoke   # tools/smoke

FAST = replace(protocol.load_settings(), heartbeat=0.05, link_timeout=0.3, reconnect=(0.02, 0.1))


def wait_for(test, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if test():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def path(tmp_path):
    return tmp_path / 'backend.sock'


def raw_attach(path, who='station'):
    """The welcome a raw hello gets."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(1.0)
        sock.connect(str(path))
        sock.sendall(protocol.encode(protocol.hello(who), FAST.max_frame))
        return protocol.decode(sock.recv(65536))


def test_an_incompatible_client_is_refused_with_why(path):
    server = Server(FAST, dict, path=path).start()
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(str(path))
        newer = protocol.hello('station', (protocol.VERSION[0] + 1, 0))
        sock.sendall(protocol.encode(newer, FAST.max_frame))
        sock.settimeout(1.0)
        refuse = protocol.decode(sock.recv(4096))
        assert refuse['kind'] == 'refuse' and 'incompatible' in refuse['why']
        assert sock.recv(4096) == b''   # and closed
        assert server.attached() == []
    finally:
        sock.close()
        server.stop()


def test_a_second_backend_never_takes_a_live_socket_but_replaces_a_dead_ones(path):
    first = Server(FAST, dict, path=path).start()
    try:
        with pytest.raises(Busy):
            Server(FAST, dict, path=path).start()
        assert raw_attach(path)['snapshot'] == {}
    finally:
        first.stop()
    stale = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    stale.bind(str(path))   # a crashed backend leaves its socket file behind
    stale.close()
    second = Server(FAST, lambda: {'up': 2}, path=path).start()
    try:
        assert raw_attach(path)['snapshot'] == {'up': 2}
        assert path.stat().st_mode & 0o077 == 0
    finally:
        second.stop()


def test_the_backend_drops_a_client_that_goes_silent(path):
    server = Server(FAST, dict, path=path).start()
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(str(path))
        sock.sendall(protocol.encode(protocol.hello('mute'), FAST.max_frame))
        assert wait_for(lambda: server.attached() == ['mute'])
        assert wait_for(lambda: server.attached() == [], timeout=FAST.link_timeout + 1)
    finally:
        sock.close()
        server.stop()
