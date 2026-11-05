"""The backend's thin client: attach time, deltas and requests, link loss and backend restarts."""

from dataclasses import replace
import socket
import threading
import time

import pytest

from mhseals_testing.backend import protocol
from mhseals_testing.backend.client import Client
from mhseals_testing.backend.link import Lost, RequestError, Server

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


def test_attach_returns_the_full_snapshot_well_inside_a_second(path):
    server = Server(FAST, lambda: {'armed': False, 'holds': ['fence']}, path=path).start()
    try:
        start = time.monotonic()
        snapshot, seq = Client(FAST, 'station', path=path).attach()
        assert time.monotonic() - start < 1.0
        assert snapshot == {'armed': False, 'holds': ['fence']} and seq == 0
    finally:
        server.stop()
    assert not path.exists()


def test_deltas_arrive_in_order_and_requests_get_replies_or_errors(path):
    def handle(op, args, who):
        if op == 'echo':
            return {'who': who, **args}
        raise RequestError(f'unknown op {op!r}')

    server = Server(FAST, dict, handle=handle, path=path).start()
    seen, attached = [], threading.Event()
    client = Client(FAST, 'station', path=path, on_delta=lambda changes, removed, seq: seen.append(seq),
                    on_link=lambda state, why: state == 'attached' and attached.set()).start()
    try:
        assert attached.wait(2)
        assert wait_for(lambda: server.attached() == ['station'])
        for n in range(20):
            server.publish('control', {'n': n})
        assert wait_for(lambda: len(seen) == 20) and seen == list(range(1, 21))
        assert client.model == {'n': 19}
        assert client.request('echo', {'x': 1}) == {'who': 'station', 'x': 1}
        with pytest.raises(RequestError, match='unknown op'):
            client.request('arm')
    finally:
        client.close()
        server.stop()


def test_the_client_rides_out_a_backend_restart_and_resyncs(path):
    states, welcomes = [], []
    server = Server(FAST, lambda: {'generation': 1}, path=path).start()
    client = Client(FAST, 'station', path=path, on_welcome=lambda snapshot, seq: welcomes.append(snapshot),
                    on_link=lambda state, why: states.append(state)).start()
    try:
        assert wait_for(lambda: welcomes == [{'generation': 1}])
        server.stop()
        assert wait_for(lambda: 'lost' in states)
        with pytest.raises(Lost):
            client.request('echo')
        server = Server(FAST, lambda: {'generation': 2}, path=path).start()
        assert wait_for(lambda: welcomes[-1:] == [{'generation': 2}])
        assert states == ['attached', 'lost', 'attached']
    finally:
        client.close()
        server.stop()


def test_a_silent_backend_is_a_lost_link_within_link_timeout(path):
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(path))
    listener.listen()
    held = []

    def frozen():   # welcomes, then never answers a ping: a hung backend or a dead link
        conn, _ = listener.accept()
        held.append(conn)
        conn.recv(4096)
        conn.sendall(protocol.encode({'kind': 'welcome', 'version': list(protocol.VERSION), 'seq': 0,
                                      'snapshot': {}}, FAST.max_frame))

    threading.Thread(target=frozen, daemon=True).start()
    lost = []
    client = Client(FAST, 'station', path=path, on_link=lambda state, why: state == 'lost' and lost.append(
        (time.monotonic(), why))).start()
    start = time.monotonic()
    try:
        assert wait_for(lambda: lost, timeout=2)
        assert lost[0][0] - start < FAST.link_timeout + 0.5 and 'silent' in lost[0][1]
    finally:
        client.close()
        for conn in held:
            conn.close()
        listener.close()
