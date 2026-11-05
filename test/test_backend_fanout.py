"""The backend's fan-out under load: slow clients, latest-wins rate groups, priority stops and resync."""

from dataclasses import replace
import socket
import threading
import time

import pytest

from mhseals_testing.backend import protocol
from mhseals_testing.backend.client import Client
from mhseals_testing.backend.link import Server

pytestmark = pytest.mark.smoke   # tools/smoke

FAST = replace(protocol.load_settings(), heartbeat=0.05, link_timeout=0.5, reconnect=(0.02, 0.1),
               slow_client=0.3, rates={'fast': 20.0, 'status': 4.0})


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


def attached(path, settings=FAST, **hooks):
    up = threading.Event()
    on_link = hooks.pop('on_link', lambda state, why: None)

    def link(state, why):
        on_link(state, why)
        if state == 'attached':
            up.set()
    client = Client(settings, hooks.pop('who', 'station'), path=path, on_link=link, **hooks).start()
    assert up.wait(2)
    return client


def ask(client, op, answers=None):
    """A request from its own thread; a link closed under it at teardown is no answer."""
    try:
        reply = client.request(op, timeout=6)
    except ConnectionError:
        return
    if answers is not None:
        answers.append(reply)


def stalled(path, who='stalled'):
    """A client that says hello and then never reads: a hung station or a link that stopped draining."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    sock.connect(str(path))
    sock.sendall(protocol.encode(protocol.hello(who), FAST.max_frame))

    def ping():   # it keeps pinging, so only its stalled reads can drop it
        try:
            while True:
                sock.sendall(protocol.encode({'kind': 'ping'}, FAST.max_frame))
                time.sleep(FAST.heartbeat)
        except OSError:
            pass
    threading.Thread(target=ping, daemon=True).start()
    return sock


def test_a_stalled_client_never_holds_up_the_publisher_or_the_others(path):
    server = Server(FAST, {'pose': {}}, path=path).start()
    good = attached(path)
    sock = stalled(path)
    try:
        assert wait_for(lambda: server.attached() == ['stalled', 'station'])
        blob, slowest, start = 'x' * 4096, 0.0, time.monotonic()
        for n in range(400):   # ~1.6 MB of status, far past any socket buffer, plus fast pose
            began = time.monotonic()
            server.publish('status', {f'sensors.s{n % 20}': blob + str(n)})
            server.publish('fast', {'pose.x': n})
            slowest = max(slowest, time.monotonic() - began)
            assert all(row['dirty'] <= 21 and row['outbox'] < FAST.outbox for row in server.stats())   # bounded per client
            time.sleep(0.001)
        assert slowest < 0.05, f'a publish waited {slowest:.3f} s on a client'
        assert wait_for(lambda: good.model.get('pose', {}).get('x') == 399, timeout=1)
        assert wait_for(lambda: good.model['sensors']['s19'] == blob + '399', timeout=1)
        assert good.model == server.model
        assert wait_for(lambda: server.attached() == ['station'], timeout=FAST.slow_client + 2)
        assert time.monotonic() - start < 5
    finally:
        sock.close()
        good.close()
        server.stop()


def test_rate_groups_send_only_the_newest_value_and_control_every_change(path):
    server = Server(FAST, path=path).start()
    deltas = []
    client = attached(path, on_delta=lambda changes, removed, seq: deltas.append(dict(changes)))
    try:
        for n in range(200):
            server.publish('fast', {'pose.x': n, 'pose.y': -n})
            time.sleep(0.002)   # ~0.4 s of 500 Hz input
        for n in range(30):
            server.publish('control', {'armed': n % 2 == 0})
        assert wait_for(lambda: client.model == {'pose': {'x': 199, 'y': -199}, 'armed': False})
        poses = [d for d in deltas if 'pose.x' in d]
        assert 2 <= len(poses) <= 0.6 * FAST.rates['fast'] + 3, len(poses)   # coalesced to the rate
        assert all(d['pose.y'] == -d['pose.x'] for d in poses)
        assert [d['armed'] for d in deltas if 'armed' in d] == [n % 2 == 0 for n in range(30)]
    finally:
        client.close()
        server.stop()


def test_removed_paths_and_a_replaced_parent_reach_the_client(path):
    server = Server(FAST, {'panels': {'gnss': {'fix': 'rtk', 'sats': 20}, 'imu': {'ok': True}}}, path=path).start()
    client = attached(path)
    try:
        server.publish('status', {'panels.gnss.sats': 21})
        server.publish('status', removed=['panels.imu'])
        server.publish('status', {'panels.gnss': {'fix': 'none'}})
        assert wait_for(lambda: client.model == {'panels': {'gnss': {'fix': 'none'}}})
        assert client.model == server.model
    finally:
        client.close()
        server.stop()


def test_a_priority_stop_overtakes_slow_requests_on_the_same_link(path):
    release, running = threading.Event(), threading.Event()

    def handle(op, args, who):
        if op == 'slow':
            running.set()
            release.wait(5)
            return 'slow done'
        if op == 'stop':
            return 'stopped'
        raise ValueError(op)

    server = Server(FAST, handle=handle, path=path).start()
    client = attached(path)
    slow = []
    try:
        threading.Thread(target=ask, args=(client, 'slow', slow), daemon=True).start()
        assert running.wait(2)
        for _ in range(2):   # two queued behind the running one
            threading.Thread(target=ask, args=(client, 'slow', slow), daemon=True).start()
        assert wait_for(lambda: server.clients[0].requests.qsize() == 2)
        for n in range(200):   # and the link busy with fast deltas
            server.publish('fast', {'pose.x': n})
        sent = time.monotonic()
        assert client.request('stop', timeout=1) == 'stopped'
        assert time.monotonic() - sent < 0.1 and slow == []
        release.set()
        assert wait_for(lambda: slow == ['slow done'] * 3)
    finally:
        release.set()
        client.close()
        server.stop()


def test_a_full_request_queue_answers_busy_at_once(path):
    release, running = threading.Event(), threading.Event()

    def handle(op, args, who):
        running.set()
        release.wait(5)

    server = Server(replace(FAST, requests=2), handle=handle, path=path).start()
    client = attached(path)
    try:
        threading.Thread(target=ask, args=(client, 'slow'), daemon=True).start()
        assert running.wait(2)
        for _ in range(2):   # two queued behind the running one
            threading.Thread(target=ask, args=(client, 'slow'), daemon=True).start()
        assert wait_for(lambda: server.clients[0].requests.qsize() == 2)
        with pytest.raises(Exception, match='busy'):
            client.request('slow', timeout=1)
    finally:
        release.set()
        client.close()
        server.stop()


def test_a_client_too_slow_for_its_control_changes_is_dropped_and_resyncs(path):
    settings = replace(FAST, outbox=8)
    server = Server(settings, {'generation': 0}, path=path).start()
    stuck, states = threading.Event(), []

    def on_delta(changes, removed, seq):
        if 'big' in changes and not stuck.is_set():
            stuck.set()
            time.sleep(1.0)   # a frontend that hangs in a callback stops draining its socket

    client = attached(path, settings, on_delta=on_delta, on_link=lambda state, why: states.append(state))
    try:
        for n in range(200):
            server.publish('control', {'big': 'x' * 8192, 'generation': n})
        assert wait_for(lambda: server.attached() == [], timeout=settings.slow_client + 2)
        server.publish('control', {'generation': 'after'})
        assert wait_for(lambda: states.count('attached') == 2, timeout=4)
        assert wait_for(lambda: client.model == server.model)
        assert client.model['generation'] == 'after'
    finally:
        client.close()
        server.stop()


def test_a_gap_in_the_delta_chain_resyncs_from_a_fresh_welcome(path):
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(path))
    listener.listen()
    held, welcomes = [], []

    def backend():   # a welcome at seq 5, then a delta on seq 3: the client must not merge it
        for generation in (1, 2):
            conn, _ = listener.accept()
            held.append(conn)
            conn.recv(4096)
            frames = [{'kind': 'welcome', 'version': list(protocol.VERSION), 'seq': 5, 'snapshot': {'g': generation}},
                      {'kind': 'delta', 'seq': 6, 'base': 3, 'set': {'g': 'merged across a gap'}, 'del': []}]
            conn.sendall(b''.join(protocol.encode(f, FAST.max_frame) for f in frames[:3 - generation]))

    threading.Thread(target=backend, daemon=True).start()
    lost = []
    client = Client(FAST, 'station', path=path, on_welcome=lambda snapshot, seq: welcomes.append(snapshot),
                    on_link=lambda state, why: state == 'lost' and lost.append(why)).start()
    try:
        assert wait_for(lambda: len(welcomes) == 2)
        assert lost and lost[0].startswith('gap') and client.model == {'g': 2}
    finally:
        client.close()
        for conn in held:
            conn.close()
        listener.close()
