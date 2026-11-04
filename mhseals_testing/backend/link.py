"""The backend's local socket and its serving end. ROS-free.

``Server`` listens on one Unix socket in the runtime directory, welcomes each client with the full
snapshot, pushes deltas to every attached client and answers requests through one handler. A
second backend never takes over a live socket (``Busy``); a dead backend's socket file is replaced.
``Conn`` is one framed stream, shared with ``backend.client``.
"""

import os
from pathlib import Path
import socket
import threading
import time

from mhseals_testing.backend import protocol
from mhseals_testing.harness.registry import runtime_dir


class Lost(ConnectionError):
    """The link closed, went silent, or was never up."""


class Refused(ConnectionError):
    """The backend refused the hello; the message says why."""


class Busy(RuntimeError):
    """A live backend already serves this socket."""


class RequestError(RuntimeError):
    """The backend answered a request with an error."""


def socket_path(settings):
    return runtime_dir() / settings.socket


class Conn:
    """One framed stream: whole-frame sends under a lock, buffered receives with a deadline."""

    def __init__(self, sock, max_frame):
        self.sock, self.max_frame, self.buffer = sock, max_frame, b''
        self.sending = threading.Lock()

    def send(self, frame):
        data = protocol.encode(frame, self.max_frame)
        with self.sending:
            try:
                self.sock.sendall(data)
            except OSError as error:
                raise Lost(f'send failed: {error}') from None

    def recv(self, timeout):
        """The next frame, or None when ``timeout`` passes first; ``Lost`` when the stream ends."""
        deadline = time.monotonic() + timeout
        while b'\n' not in self.buffer:
            if len(self.buffer) > self.max_frame:
                raise Lost('frame too long')
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            self.sock.settimeout(left)
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                return None
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
            self.sock.close()
        except OSError:
            pass


def _refuses_nothing(op, args, who):
    raise RequestError(f'unknown op {op!r}')


class Server:
    """``snapshot()`` returns the full state (JSON-able); ``handle(op, args, who)`` answers a request."""

    def __init__(self, settings, snapshot, handle=_refuses_nothing, path=None):
        self.settings, self.snapshot, self.handle = settings, snapshot, handle
        self.path = Path(path) if path else socket_path(settings)
        self.clients, self.seq, self.lock = {}, 0, threading.Lock()
        self.listener, self.running = None, False

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

    def publish(self, data):
        """Push one delta to every attached client, in order; a client that cannot take it is dropped."""
        with self.lock:
            self.seq += 1
            frame = {'kind': 'delta', 'seq': self.seq, 'data': data}
            for conn in list(self.clients):
                try:
                    conn.send(frame)
                except Lost:
                    self._drop(conn)

    def attached(self):
        with self.lock:
            return sorted(self.clients.values())

    def stop(self):
        self.running = False
        if self.listener:
            self.listener.close()
        with self.lock:
            for conn in list(self.clients):
                self._drop(conn)
        self.path.unlink(missing_ok=True)

    def _drop(self, conn):
        self.clients.pop(conn, None)
        conn.close()

    def _accept(self):
        while self.running:
            try:
                sock, _ = self.listener.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(Conn(sock, self.settings.max_frame),),
                             name='backend-client', daemon=True).start()

    def _serve(self, conn):
        try:
            first = conn.recv(self.settings.attach_timeout)
            why = 'no hello in time' if first is None else protocol.check_hello(first)
            if why:
                conn.send({'kind': 'refuse', 'why': why, 'version': list(protocol.VERSION)})
                conn.close()
                return
            with self.lock:   # the welcome and later deltas stay in seq order
                conn.send({'kind': 'welcome', 'version': list(protocol.VERSION), 'seq': self.seq,
                           'snapshot': self.snapshot()})
                self.clients[conn] = first['who']
            while self.running:
                frame = conn.recv(self.settings.link_timeout)
                if frame is None:
                    break   # silent past link_timeout: the client is gone
                if frame['kind'] == 'ping':
                    conn.send({'kind': 'pong'})
                elif frame['kind'] == 'request':
                    conn.send(self._answer(frame, first['who']))
        except Lost:
            pass
        with self.lock:
            self._drop(conn)

    def _answer(self, frame, who):
        reply = {'kind': 'reply', 'id': frame.get('id')}
        try:
            reply['data'] = self.handle(frame.get('op'), frame.get('args') or {}, who)
        except Exception as error:   # any handler failure is the client's answer, never the backend's end
            reply['error'] = str(error) or type(error).__name__
        return reply
