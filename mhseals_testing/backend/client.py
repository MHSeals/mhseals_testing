"""The thin client of the backend: attaches in one bounded attempt, or stays attached through link loss. ROS-free.

``attach`` is one attempt within ``attach_timeout``. ``start`` keeps the client attached in the
background: a lost link (closed, or silent past ``link_timeout``) is reported once, then the client
reattaches with doubling waits and resyncs from the fresh welcome, so it never holds a partial state.
"""

import itertools
from pathlib import Path
import socket
import threading
import time

from mhseals_testing.backend import protocol
from mhseals_testing.backend.link import Conn, Lost, Refused, RequestError, socket_path


class Client:
    """``on_welcome(snapshot, seq)``, ``on_delta(data, seq)`` and ``on_link(state, why)`` run on the reader thread;
    ``state`` is 'attached', 'lost' or 'refused'."""

    def __init__(self, settings, who, path=None, on_welcome=None, on_delta=None, on_link=None):
        self.settings, self.who = settings, who
        self.path = Path(path) if path else socket_path(settings)
        self.on_welcome = on_welcome or (lambda snapshot, seq: None)
        self.on_delta = on_delta or (lambda data, seq: None)
        self.on_link = on_link or (lambda state, why: None)
        self.conn, self.closed, self.ids = None, threading.Event(), itertools.count(1)
        self.pending, self.lock, self.thread = {}, threading.Lock(), None

    def attach(self):
        """One attempt within ``attach_timeout``: (snapshot, seq), then detach. ``Lost`` when unreachable,
        ``Refused`` when refused."""
        conn, snapshot, seq = self._open()
        conn.close()
        return snapshot, seq

    def _open(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.settings.attach_timeout)
        try:
            sock.connect(str(self.path))
        except OSError as error:
            sock.close()
            raise Lost(f'no backend at {self.path}: {error.strerror or error}') from None
        conn = Conn(sock, self.settings.max_frame)
        try:
            conn.send(protocol.hello(self.who))
            frame = conn.recv(self.settings.attach_timeout)
        except Lost:
            conn.close()
            raise
        if frame is None or frame['kind'] not in ('welcome', 'refuse'):
            conn.close()
            raise Lost('no welcome in time' if frame is None else f'expected welcome, got {frame["kind"]}')
        if frame['kind'] == 'refuse':
            conn.close()
            raise Refused(frame.get('why') or 'refused')
        return conn, frame['snapshot'], frame['seq']

    def start(self):
        """Stay attached in the background until ``close``."""
        self.thread = threading.Thread(target=self._keep, name='backend-link', daemon=True)
        self.thread.start()
        return self

    def request(self, op, args=None, timeout=None):
        """The reply data of one request; ``Lost`` when not attached or no reply in time."""
        conn = self.conn
        if conn is None:
            raise Lost('not attached')
        ident, done = next(self.ids), threading.Event()
        with self.lock:
            self.pending[ident] = [done, None]
        try:
            conn.send({'kind': 'request', 'id': ident, 'op': op, 'args': args or {}})
            if not done.wait(self.settings.link_timeout if timeout is None else timeout):
                raise Lost(f'no reply to {op} in time')
        finally:
            with self.lock:
                reply = self.pending.pop(ident)[1]
        if reply is None:
            raise Lost(f'link lost before the reply to {op}')
        if 'error' in reply:
            raise RequestError(reply['error'])
        return reply.get('data')

    def close(self):
        self.closed.set()
        if self.conn:
            self.conn.close()
        if self.thread:
            self.thread.join(self.settings.link_timeout)

    def _keep(self):
        wait = self.settings.reconnect[0]
        while not self.closed.is_set():
            try:
                conn, snapshot, seq = self._open()
            except Refused as error:
                self.on_link('refused', str(error))
            except Lost:
                pass
            else:
                wait = self.settings.reconnect[0]
                self.on_welcome(snapshot, seq)
                self.conn = conn
                self.on_link('attached', None)
                why = self._read(conn)
                self.conn = None
                conn.close()
                self._fail_pending()
                if self.closed.is_set():
                    return
                self.on_link('lost', why)
            if self.closed.wait(wait):
                return
            wait = min(wait * 2, self.settings.reconnect[1])

    def _read(self, conn):
        """Read until the link is lost; returns why."""
        heartbeat, silent = self.settings.heartbeat, self.settings.link_timeout
        heard = sent = time.monotonic()
        while not self.closed.is_set():
            now = time.monotonic()
            if now - heard > silent:
                return f'silent for {silent:g} s'
            try:
                if now - sent >= heartbeat:
                    conn.send({'kind': 'ping'})
                    sent = now
                frame = conn.recv(max(0.0, min(heartbeat - (time.monotonic() - sent), silent)))
            except Lost as error:
                return str(error)
            if frame is None:
                continue
            heard = time.monotonic()
            if frame['kind'] == 'delta':
                self.on_delta(frame.get('data'), frame.get('seq'))
            elif frame['kind'] == 'reply':
                with self.lock:
                    slot = self.pending.get(frame.get('id'))
                    if slot:
                        slot[1] = frame
                        slot[0].set()
        return 'closed'

    def _fail_pending(self):
        with self.lock:
            for done, _ in self.pending.values():
                done.set()
