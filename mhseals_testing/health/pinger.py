"""Unprivileged ICMP echo to the boat peers: rtt and loss over a sliding window (health.md D2).

Uses SOCK_DGRAM/IPPROTO_ICMP, which needs no root when net.ipv4.ping_group_range
covers our gid. If the socket cannot be opened, facts are empty and checks read
``unknown``. One daemon thread; the tick only copies results under a lock.
"""

import socket
import struct
import threading
import time
from collections import deque

from mhseals_testing.health.model import Fact

WINDOW = 20             # samples kept per peer
TIMEOUT = 0.8           # seconds before a reply counts as lost


def checksum(data):
    """Internet checksum of ``data`` (the kernel rewrites it on ping sockets, but keep it valid)."""
    if len(data) % 2:
        data += b'\0'
    total = sum(struct.unpack(f'!{len(data) // 2}H', data))
    total = (total & 0xFFFF) + (total >> 16)
    total += total >> 16
    return ~total & 0xFFFF


def echo_request(ident, seq):
    """An ICMP echo request packet with a valid checksum."""
    head = struct.pack('!BBHHH', 8, 0, 0, ident, seq)
    return struct.pack('!BBHHH', 8, 0, checksum(head), ident, seq)


def reply_seq(packet):
    """Sequence number of an echo reply (type 0), else None. DGRAM sockets omit the IP header."""
    if len(packet) < 8 or packet[0] != 0:
        return None
    return struct.unpack('!H', packet[6:8])[0]


class Window:
    """Last ``size`` ping outcomes for one peer; a lost ping is stored as None."""

    def __init__(self, size=WINDOW):
        self.samples = deque(maxlen=size)

    def add(self, rtt_s):
        self.samples.append(rtt_s)

    def loss_pct(self):
        """Percent lost over the window, or None before the first sample."""
        if not self.samples:
            return None
        return 100.0 * sum(s is None for s in self.samples) / len(self.samples)

    def rtt_ms(self):
        """Mean rtt of the answered pings in ms, or None when none were answered."""
        got = [s for s in self.samples if s is not None]
        return 1000.0 * sum(got) / len(got) if got else None


def open_icmp():
    """An unprivileged ICMP socket; raises OSError when ping_group_range forbids it."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_ICMP)
    sock.settimeout(TIMEOUT)
    return sock


class Pinger:
    """Pings each peer once per ``period`` from a daemon thread and exposes facts."""

    def __init__(self, peers, open_socket=open_icmp, clock=time.monotonic,
                 period=1.0, size=WINDOW):
        self.peers = dict(peers)                     # name -> IPv4 address
        self.open_socket = open_socket
        self.clock = clock
        self.period = period
        self.windows = {name: Window(size) for name in self.peers}
        self.available = True
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.seq = 0
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run, name='pinger', daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()

    def run(self):
        try:
            sock = self.open_socket()
        except OSError:
            self.available = False
            return
        with sock:
            while not self.stop_event.is_set():
                self.round(sock)
                self.stop_event.wait(self.period)

    def round(self, sock):
        """Ping every peer once and record the outcomes."""
        for name, address in self.peers.items():
            rtt = self.ping(sock, address)
            with self.lock:
                self.windows[name].add(rtt)

    def ping(self, sock, address):
        """rtt in seconds, or None on timeout, error or a reply for another request."""
        self.seq = (self.seq + 1) & 0xFFFF
        seq = self.seq
        start = self.clock()
        try:
            sock.sendto(echo_request(0, seq), (address, 0))
            while self.clock() - start < TIMEOUT:
                if reply_seq(sock.recv(1024)) == seq:
                    return self.clock() - start
        except OSError:                              # includes socket.timeout
            pass
        return None

    def facts(self):
        """``net.rtt_ms.<peer>`` and ``net.loss_pct.<peer>``; empty if sockets are forbidden."""
        out = {}
        if not self.available:
            return out
        now = self.clock()
        with self.lock:
            for name, window in self.windows.items():
                loss, rtt = window.loss_pct(), window.rtt_ms()
                if loss is not None:
                    out[f'net.loss_pct.{name}'] = Fact(loss, label=name, time=now)
                if rtt is not None:
                    out[f'net.rtt_ms.{name}'] = Fact(rtt, label=name, time=now)
        return out
