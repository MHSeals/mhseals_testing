"""The backend's local socket and its serving end. ROS-free.

``Server`` listens on one Unix socket in the runtime directory and owns the model it serves: each client
gets the full snapshot in its welcome, then merge-patch deltas by rate group (protocol.py). Every client
has its own writer thread and bounded buffers, so ``publish`` never waits on a socket and a slow client
never holds up the publisher or the other clients: non-control groups keep only the newest value per
path (latest wins), the outbox (control changes, replies, pongs) is capped, and a client that takes no
bytes for ``slow_client`` or overflows its outbox is dropped and resyncs on reconnect. Requests run on
a per-client worker; the ``priority`` ops (stop, e-stop, cancel arm) run on the reader at once, ahead of
anything queued. A second backend never takes over a live socket (``Busy``); a dead backend's socket
file is replaced. ``Conn`` is one framed stream, shared with ``backend.client``.
"""

from collections import deque
import json
import math
import os
from pathlib import Path
import queue
import select
import socket
import threading
import time

from mhseals_testing.backend import protocol
from mhseals_testing.harness.registry import runtime_dir


class Lost(ConnectionError):
    """The link closed, stalled, went silent, or was never up."""


class Refused(ConnectionError):
    """The backend refused the hello; the message says why."""


class Busy(RuntimeError):
    """A live backend already serves this socket."""


class RequestError(RuntimeError):
    """The backend answered a request with an error."""


def socket_path(settings):
    return runtime_dir() / settings.socket


def _wait(sock, events, timeout):
    """True once ``sock`` is ready for ``events`` within ``timeout``; ``Lost`` when it is closed."""
    poller = select.poll()
    try:
        poller.register(sock, events)
        ready = poller.poll(max(0.0, timeout) * 1000)
    except (OSError, ValueError) as error:   # closed under us
        raise Lost(f'socket closed: {error}') from None
    if any(flags & select.POLLNVAL for _, flags in ready):
        raise Lost('socket closed')
    return bool(ready)


class Conn:
    """One framed, non-blocking stream: whole-frame sends under a lock (``Lost`` once the other end takes no
    bytes for ``stall`` seconds) and buffered receives with a deadline. Safe to read and write from two threads."""

    def __init__(self, sock, max_frame, stall):
        sock.setblocking(False)
        self.sock, self.max_frame, self.stall, self.buffer = sock, max_frame, stall, b''
        self.sending, self.sent = threading.Lock(), 0

    def send(self, frame):
        """Send one frame (a dict, or the bytes ``protocol.encode`` made)."""
        data = memoryview(frame if isinstance(frame, bytes) else protocol.encode(frame, self.max_frame))
        with self.sending:
            while data:
                try:
                    taken = self.sock.send(data)
                except BlockingIOError:
                    taken = 0
                except OSError as error:
                    raise Lost(f'send failed: {error}') from None
                if taken:
                    data, self.sent = data[taken:], self.sent + taken
                elif not _wait(self.sock, select.POLLOUT, self.stall):
                    raise Lost(f'stalled: no bytes taken for {self.stall:g} s')

    def recv(self, timeout):
        """The next frame, or None when ``timeout`` passes first; ``Lost`` when the stream ends."""
        deadline = time.monotonic() + timeout
        while b'\n' not in self.buffer:
            if len(self.buffer) > self.max_frame:
                raise Lost('frame too long')
            try:
                chunk = self.sock.recv(65536)
            except BlockingIOError:
                if not _wait(self.sock, select.POLLIN, deadline - time.monotonic()):
                    return None
                continue
            except OSError as error:
                raise Lost(f'receive failed: {error}') from None
            if not chunk:
                raise Lost('closed by the other end')
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b'\n', 1)
        try:
            return protocol.decode(line)
        except protocol.FrameError as error:
            raise Lost(str(error)) from None

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


def _refuses_nothing(op, args, who):
    raise RequestError(f'unknown op {op!r}')


class _Client:
    """One attached client's buffers; every field but ``conn`` is guarded by the server's lock."""

    def __init__(self, server, conn, who, seq):
        self.conn, self.who, self.last = conn, who, seq
        self.outbox = deque()   # frames in send order: the welcome, control deltas, replies, pongs
        self.dirty = {}         # path -> group: prefix-free, values read from the model when sent
        self.sent_at = {}       # group -> monotonic time of its last send
        self.wake = threading.Condition(server.lock)
        self.requests = queue.Queue(server.settings.requests)
        self.closed = False


class Server:
    """``model`` is the initial snapshot (a JSON-able dict, or a callable returning one);
    ``handle(op, args, who)`` answers a request; ``publish`` changes the model and fans it out."""

    def __init__(self, settings, model=None, handle=_refuses_nothing, path=None):
        self.settings, self.handle = settings, handle
        self.model = _copy((model() if callable(model) else model) or {})
        self.path = Path(path) if path else socket_path(settings)
        self.clients, self.seq, self.lock = [], 0, threading.Lock()
        self.listener, self.running = None, False
        groups = sorted(settings.rates, key=settings.period)
        self.rank = {protocol.CONTROL: -1, **{group: n for n, group in enumerate(groups)}}

    def start(self):
        if self.path.exists():
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            probe.settimeout(self.settings.attach_timeout)
            try:
                probe.connect(str(self.path))
                raise Busy(f'a backend already listens on {self.path}')
            except OSError:
                self.path.unlink(missing_ok=True)   # a dead backend's socket
            finally:
                probe.close()
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        old = os.umask(0o177)
        try:
            listener.bind(str(self.path))
        finally:
            os.umask(old)
        listener.listen()
        self.listener, self.running = listener, True
        threading.Thread(target=self._accept, name='backend-accept', daemon=True).start()
        return self

    def publish(self, group, changes=None, removed=()):
        """Change the model (``changes``: path -> value, ``removed``: paths) in rate group ``group``.
        Never waits on a client: it queues a control delta, or marks the paths for each writer."""
        self.settings.period(group)   # KeyError for an unknown group
        changes, removed = _copy(changes or {}), list(removed)
        with self.lock:
            self.seq += 1
            protocol.apply(self.model, changes, removed)
            for client in list(self.clients):   # _queue may drop one
                if group == protocol.CONTROL:
                    for path in [*removed, *changes]:
                        _forget(client.dirty, path)
                    self._queue(client, {'kind': 'delta', 'seq': self.seq, 't': time.time(),
                                         'set': changes, 'del': removed})
                else:
                    for path in [*removed, *changes]:
                        self._mark(client, path, group)
                    client.wake.notify()

    def attached(self):
        with self.lock:
            return sorted(client.who for client in self.clients)

    def stats(self):
        """Per attached client: who, bytes sent, frames waiting in its outbox, paths waiting to be sent."""
        with self.lock:
            return [{'who': c.who, 'sent': c.conn.sent, 'outbox': len(c.outbox), 'dirty': len(c.dirty)}
                    for c in self.clients]

    def stop(self):
        self.running = False
        if self.listener:
            self.listener.close()
        with self.lock:
            for client in list(self.clients):
                self._drop(client)
        self.path.unlink(missing_ok=True)

    # serving

    def _accept(self):
        while self.running:
            try:
                sock, _ = self.listener.accept()
            except OSError:
                return
            conn = Conn(sock, self.settings.max_frame, self.settings.slow_client)
            threading.Thread(target=self._serve, args=(conn,), name='backend-client', daemon=True).start()

    def _serve(self, conn):
        """The client's reader: hello, then pings and requests until it goes silent or is dropped."""
        try:
            first = conn.recv(self.settings.attach_timeout)
            why = 'no hello in time' if first is None else protocol.check_hello(first)
            if why:
                conn.send({'kind': 'refuse', 'why': why, 'version': list(protocol.VERSION)})
                conn.close()
                return
        except Lost:
            conn.close()
            return
        with self.lock:   # the welcome holds the model as of seq; every later change reaches this client
            try:
                welcome = protocol.encode({'kind': 'welcome', 'version': list(protocol.VERSION), 'seq': self.seq,
                                           't': time.time(), 'snapshot': self.model}, self.settings.max_frame)
            except protocol.FrameError:   # a model past max_frame: the client sees a closed link and retries
                conn.close()
                return
            client = _Client(self, conn, first['who'], self.seq)
            client.outbox.append(welcome)
            self.clients.append(client)
        threading.Thread(target=self._write, args=(client,), name='backend-writer', daemon=True).start()
        threading.Thread(target=self._work, args=(client,), name='backend-requests', daemon=True).start()
        try:
            while self.running and not client.closed:
                frame = conn.recv(self.settings.link_timeout)
                if frame is None:
                    break   # silent past link_timeout: the client is gone
                if frame['kind'] == 'ping':
                    self._reply(client, {'kind': 'pong'})
                elif frame['kind'] == 'request':
                    self._request(client, frame)
        except Lost:
            pass
        with self.lock:
            self._drop(client)

    def _request(self, client, frame):
        if frame.get('op') in self.settings.priority:   # a stop never waits behind a slow request
            self._reply(client, self._answer(frame, client.who), first=True)
            return
        try:
            client.requests.put_nowait(frame)
        except queue.Full:
            self._reply(client, {'kind': 'reply', 'id': frame.get('id'),
                                 'error': f'busy: {self.settings.requests} requests already queued'})

    def _work(self, client):
        """The client's request worker: one request at a time, in order."""
        while True:
            frame = client.requests.get()
            if frame is None:
                return
            self._reply(client, self._answer(frame, client.who))

    def _answer(self, frame, who):
        reply = {'kind': 'reply', 'id': frame.get('id')}
        try:
            reply['data'] = self.handle(frame.get('op'), frame.get('args') or {}, who)
        except Exception as error:   # any handler failure is the client's answer, never the backend's end
            reply['error'] = str(error) or type(error).__name__
        return reply

    def _reply(self, client, frame, first=False):
        with self.lock:
            self._queue(client, frame, first)

    def _write(self, client):
        """The client's writer: sends the outbox, then each due group's newest values, as the socket drains."""
        while True:
            with self.lock:
                frames = self._next(client)
                if frames is None:
                    return
            try:
                for frame in frames:
                    client.conn.send(frame)
            except (Lost, protocol.FrameError):
                with self.lock:
                    self._drop(client)
                return

    def _next(self, client):
        """Under the lock: the frames to send next (waiting until there are some), or None once dropped."""
        while not client.closed:
            frames = [self._chain(client, frame) for frame in client.outbox]
            client.outbox.clear()
            now = time.monotonic()
            due = {group: client.sent_at.get(group, -math.inf) + self.settings.period(group)
                   for group in set(client.dirty.values())}
            ready = {group for group, at in due.items() if at <= now}
            if ready:
                frames.append(self._chain(client, self._coalesce(client, ready, now)))
            if frames:
                return frames
            client.wake.wait(min(due.values()) - now if due else None)
        return None

    def _coalesce(self, client, groups, now):
        """One delta with the model's current value of every dirty path in ``groups``."""
        changes, removed = {}, []
        for path, group in list(client.dirty.items()):
            if group in groups:
                del client.dirty[path]
                found, value = protocol.lookup(self.model, path)
                if found:
                    changes[path] = value
                else:
                    removed.append(path)
        for group in groups:
            client.sent_at[group] = now
        return {'kind': 'delta', 'seq': self.seq, 't': time.time(), 'set': _copy(changes), 'del': removed}

    def _chain(self, client, frame):
        """Give a delta the client's previous seq as its base."""
        if isinstance(frame, dict) and frame['kind'] == 'delta':
            frame = {**frame, 'base': client.last}
            client.last = frame['seq']
        return frame

    def _mark(self, client, path, group):
        """Mark ``path`` dirty in ``group``; the dirty set stays prefix-free and the faster group wins."""
        parts = protocol.keys(path)
        for n in range(1, len(parts)):
            parent = '.'.join(parts[:n])
            if parent in client.dirty:   # a dirty parent carries this path's newest value
                client.dirty[parent] = self._faster(client.dirty[parent], group)
                return
        client.dirty[path] = self._faster(group, *_forget(client.dirty, path))

    def _faster(self, *groups):
        return min(groups, key=self.rank.__getitem__)

    def _queue(self, client, frame, first=False):
        if client.closed:
            return
        if len(client.outbox) >= self.settings.outbox:
            self._drop(client)   # too far behind for its control changes: it resyncs on reconnect
            return
        client.outbox.appendleft(frame) if first else client.outbox.append(frame)
        client.wake.notify()

    def _drop(self, client):
        """Under the lock: detach ``client`` and wake its threads so they end."""
        if client in self.clients:
            self.clients.remove(client)
        if not client.closed:
            client.closed = True
            client.wake.notify()
            try:
                client.requests.put_nowait(None)
            except queue.Full:
                _drain(client.requests)
            client.conn.close()


def _forget(dirty, path):
    """Remove ``path`` and every path under it from ``dirty``; the groups they were in."""
    prefix = path + '.'
    return [dirty.pop(other) for other in [p for p in dirty if p == path or p.startswith(prefix)]]


def _drain(requests):
    while True:
        try:
            requests.get_nowait()
        except queue.Empty:
            break
    requests.put_nowait(None)


def _copy(value):
    """A deep, JSON-only copy: a publisher mutating its value later never changes what was queued."""
    return json.loads(json.dumps(value, allow_nan=False))
