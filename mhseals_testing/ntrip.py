"""NTRIP corrections for rtk_base: caster sourcetables, the mountpoint picker, a client with a
GGA uplink, and failover between mountpoints and casters. No ROS imports.

The casters, their order and the picker's weights are config/ntrip.yaml. Credentials are never
in a repo: each caster names environment variables, filled from a secrets file (``.env`` style).

Picking (``rank``): every RTCM3 mountpoint of every reachable caster gets a cost in kilometres,
its distance from the rough position plus penalties for an older message set (MSM7 best, then
MSM4), a single band, fewer constellations and the caster's own bias. A network (VRS-style)
mountpoint counts half the distance to the network's nearest station. Mountpoints beyond
``max_km`` stay as last-resort fallbacks. A mountpoint that went stale, refused or dropped rests
for ``mount_cooldown`` (doubling to 10 min); a caster that refused the login rests for
``auth_cooldown``.
"""

import base64
from dataclasses import dataclass, field
import datetime
import json
import math
import os
from pathlib import Path
import re
import socket
import ssl
import threading
import time

AGENT = 'NTRIP mhseals_testing/1.0'
MSM7 = {1077, 1087, 1097, 1117, 1127}
MSM5 = {1075, 1085, 1095, 1115, 1125}
MSM4 = {1074, 1084, 1094, 1114, 1124}
LEGACY = {1001, 1002, 1003, 1004, 1009, 1010, 1011, 1012}
# MSM message id // 10 -> constellation
SYSTEMS = {107: 'GPS', 108: 'GLO', 109: 'GAL', 111: 'QZS', 112: 'BDS'}
NAMES = {'GPS': 'GPS', 'GLO': 'GLO', 'GLONASS': 'GLO', 'GAL': 'GAL', 'GALILEO': 'GAL',
         'BDS': 'BDS', 'BEIDOU': 'BDS', 'BEI': 'BDS', 'CMP': 'BDS', 'QZS': 'QZS', 'QZSS': 'QZS'}

DEFAULTS = dict(enabled=True, max_km=30.0, near_km=15.0, refresh_s=600.0, stale_s=5.0,
                connect_grace_s=10.0, gga_every_s=10.0, repick_s=60.0, repick_margin_km=5.0,
                mount_cooldown_s=60.0, auth_cooldown_s=600.0, timeout_s=5.0,
                penalties=dict(msm4=3.0, msm5=2.0, legacy=15.0, unknown=8.0, single_band=40.0,
                               per_missing_system=3.0, beyond_max=200.0),
                secrets=['~/.config/astro/secrets.env', '~/.config/mhseals_testing/secrets.env'],
                position_cache='~/.cache/mhseals_testing/ntrip_position.json', fallback=None, casters=[])


SHIPPED = Path(__file__).resolve().parent.parent / 'config' / 'ntrip.yaml'


def shipped_path():
    """config/ntrip.yaml in the source tree, else the installed share directory."""
    if SHIPPED.is_file():
        return SHIPPED
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'ntrip.yaml'
    except (ImportError, KeyError):
        return SHIPPED


def load_settings(rtk_config=None, path=None):
    """The shipped NTRIP settings with rtk.yaml's ``ntrip:`` mapping over them.

    Overlay keys replace shipped ones, except ``casters`` entries named in the overlay, which
    merge into the shipped caster of that name (e.g. ``{name: FPRN, enabled: false}``); an
    unknown name adds a caster.
    """
    import yaml
    path = Path(path or shipped_path())
    shipped = (yaml.safe_load(path.read_text()) or {}) if path.is_file() else {}
    overlay = dict((rtk_config or {}).get('ntrip') or {})
    settings = {**DEFAULTS, **shipped}
    casters = [dict(c) for c in settings.get('casters') or []]
    for change in overlay.pop('casters', None) or []:
        match = next((c for c in casters if c['name'] == change.get('name')), None)
        if match is None:
            casters.append(dict(change))
        else:
            match.update(change)
    settings.update(overlay)
    settings['casters'] = casters
    for caster in casters:
        missing = [k for k in ('name', 'host', 'port') if not caster.get(k)]
        if missing:
            raise ValueError(f"NTRIP caster {caster.get('name', '?')}: missing {', '.join(missing)}")
    return settings


class NtripError(Exception):
    """A caster refused or broke the stream; ``kind`` is auth, mount, stale or link."""

    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


# --- secrets ---------------------------------------------------------------------------------

ASTRO_SECRETS = '~/.config/astro/secrets.env'     # astro_dock's secrets file; $ASTRO_SECRETS overrides it


def secrets_file(setting):
    """The secrets file ``secrets`` names: a path, or a list whose first existing file wins (the last
    when none exists). ``$ASTRO_SECRETS``, when set, stands in for ~/.config/astro/secrets.env."""
    candidates = [setting] if isinstance(setting, (str, os.PathLike)) else list(setting or ())
    override = os.environ.get('ASTRO_SECRETS')
    astro = os.path.expanduser(ASTRO_SECRETS)
    paths = []
    for candidate in candidates:
        path = Path(os.path.expanduser(str(candidate)))
        paths.append(Path(override).expanduser() if override and str(path) == astro else path)
    return next((p for p in paths if p.is_file()), paths[-1] if paths else None)


def read_env_file(path):
    """KEY=VALUE lines of a .env file (quotes stripped, # comments); {} when it does not exist."""
    values = {}
    path = Path(os.path.expanduser(str(path)))
    if not path.is_file():
        return values
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, value = line.split('=', 1)
        key = key.strip().removeprefix('export ').strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in '"\'':
            value = value[1:-1]
        values[key] = value
    return values


def credentials(caster, env):
    """(user, password) from the caster's user_env/password_env names; (None, None) for an open caster.

    A caster that names variables which are unset is not usable for streams (its sourcetable is).
    """
    user_key, password_key = caster.get('user_env'), caster.get('password_env')
    if not user_key:
        return None, None
    return env.get(user_key) or None, env.get(password_key, '') if password_key else ''


def needs_login(caster):
    return bool(caster.get('user_env'))


# --- sourcetable and ranking ----------------------------------------------------------------

@dataclass
class Stream:
    """One STR record of a sourcetable (NTRIP 1 and 2 share the layout)."""
    caster: str
    mount: str
    identifier: str = ''
    format: str = ''
    details: str = ''
    carrier: int = 0            # 0 none, 1 L1, 2 L1+L2 (and more)
    nav: str = ''
    network: str = ''
    country: str = ''
    lat: float = None
    lon: float = None
    nmea: int = 0               # 1: the caster wants GGA from the client
    solution: int = 0           # 0 single base, 1 network
    auth: str = ''
    bitrate: int = 0
    messages: set = field(default_factory=set)

    @property
    def vrs(self):
        return self.solution == 1 or (self.nmea == 1 and bool(re.search(r'VRS|NEAR|MAX|NRT|NET', self.mount + self.identifier, re.I)))


def _number(text, kind=float, default=None):
    try:
        value = kind(text)
    except (TypeError, ValueError):
        return default
    return value if not isinstance(value, float) or math.isfinite(value) else default


def parse_sourcetable(text, caster=''):
    """STR records of a sourcetable body (CAS/NET lines and the ENDSOURCETABLE marker are skipped)."""
    streams = []
    for line in text.splitlines():
        if not line.startswith('STR;'):
            continue
        f = line.split(';') + [''] * 19
        lat, lon = _number(f[9]), _number(f[10])
        if lon is not None and lon > 180:
            lon -= 360          # some tables give east longitudes 0..360
        messages = {int(m) for m in re.findall(r'(?<!\d)(\d{4})(?!\d)', f[4])}
        streams.append(Stream(caster=caster, mount=f[1], identifier=f[2], format=f[3], details=f[4],
                              carrier=_number(f[5], int, 0), nav=f[6], network=f[7], country=f[8],
                              lat=lat, lon=lon, nmea=_number(f[11], int, 0), solution=_number(f[12], int, 0),
                              auth=f[15], bitrate=_number(f[17], int, 0), messages=messages))
    return streams


def distance_km(a, b):
    """Great-circle distance between (lat, lon) pairs in degrees."""
    lat1, lon1, lat2, lon2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
    return 2 * 6371.0 * math.asin(min(1.0, math.sqrt(h)))


def msm_level(stream):
    """7, 5, 4, 3 (legacy 1004/1012), or 0 when the table does not list messages."""
    ids = stream.messages
    for level, group in ((7, MSM7), (5, MSM5), (4, MSM4)):
        if ids & group:
            return level
    if ids & LEGACY:
        return 3
    match = re.search(r'MSM\s*([4-7])', stream.format + ' ' + stream.details, re.I)
    return int(match.group(1)) if match else 0


def systems(stream):
    """Constellations from the MSM ids, else from the nav-system field."""
    found = {SYSTEMS[i // 10] for i in stream.messages if i // 10 in SYSTEMS}
    if not found:
        found = {NAMES[word] for word in re.split(r'[+,/ ]+', stream.nav.upper()) if word in NAMES}
    return found


def usable(stream):
    """RTCM 3 streams only: RTCM 2, CMR, RAW and the like cannot feed the F9P rover."""
    return bool(re.match(r'\s*RTCM\s*3', stream.format, re.I))


@dataclass
class Candidate:
    stream: Stream
    km: float                   # station distance (VRS: the nearest network station)
    cost: float                 # km plus penalties; lower is better
    level: int
    systems: tuple
    in_range: bool

    @property
    def key(self):
        return self.stream.caster, self.stream.mount

    def summary(self):
        return dict(caster=self.stream.caster, mount=self.stream.mount, km=round(self.km, 1),
                    cost=round(self.cost, 1), msm=self.level, systems='+'.join(self.systems),
                    vrs=self.stream.vrs, gga=bool(self.stream.nmea))


def rank(streams, here, settings=None, biases=None, resting=(), groups=None):
    """Candidates for position ``here`` (lat, lon), best first; resting keys (caster, mount) are left out.

    ``groups`` maps a caster to the network it belongs to (its stations may be in another
    caster's table, e.g. FPRN-NET -> FPRN); a network mountpoint is as near as its group's
    nearest single station (VRS/IMAX: half that, as the virtual base sits at the rover).
    """
    settings = {**DEFAULTS, **(settings or {})}
    weights = {**DEFAULTS['penalties'], **settings.get('penalties', {})}
    biases, groups = biases or {}, groups or {}
    single = {}
    for s in streams:            # nearest single-base station per network group
        if usable(s) and not s.vrs and s.lat is not None and s.lon is not None:
            km, group = distance_km(here, (s.lat, s.lon)), groups.get(s.caster, s.caster)
            single[group] = min(single.get(group, km), km)
    out = []
    for s in streams:
        if not usable(s) or (s.caster, s.mount) in resting:
            continue
        if s.lat is None or s.lon is None or (s.lat == 0 and s.lon == 0 and not s.vrs):
            continue
        km = distance_km(here, (s.lat, s.lon))
        if s.vrs:
            km = single.get(groups.get(s.caster, s.caster), km) * (1.0 if 'NEAR' in s.mount.upper() else 0.5)
        level, found = msm_level(s), systems(s)
        cost = km + biases.get(s.caster, 0.0)
        cost += {7: 0.0, 5: weights['msm5'], 4: weights['msm4'], 3: weights['legacy']}.get(level, weights['unknown'])
        if s.carrier < 2:
            cost += weights['single_band']
        cost += weights['per_missing_system'] * max(0, 4 - len(found & {'GPS', 'GLO', 'GAL', 'BDS'}))
        in_range = km <= settings['max_km']
        if not in_range:
            cost += weights['beyond_max']
        out.append(Candidate(s, km, cost, level, tuple(sorted(found)), in_range))
    out.sort(key=lambda c: (c.cost, c.km, c.stream.mount))
    return out


# --- NMEA GGA uplink ------------------------------------------------------------------------

def gga(lat, lon, alt=0.0, quality=1, satellites=12, hdop=1.0, when=None):
    """A $GPGGA sentence with CRLF for the caster; VRS networks build the virtual base here."""
    when = when or datetime.datetime.now(datetime.timezone.utc)

    def dm(value, width):
        value = abs(value)
        degrees = int(value)
        return f'{degrees:0{width}d}{(value - degrees) * 60:08.5f}'

    body = (f'GPGGA,{when:%H%M%S}.{when.microsecond // 10000:02d},{dm(lat, 2)},{"N" if lat >= 0 else "S"},'
            f'{dm(lon, 3)},{"E" if lon >= 0 else "W"},{quality},{satellites:02d},{hdop:.1f},'
            f'{alt:.1f},M,0.0,M,,')
    check = 0
    for char in body.encode():
        check ^= char
    return f'${body}*{check:02X}\r\n'


# --- RTCM3 framing --------------------------------------------------------------------------

def _crc_table():
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        table.append(crc & 0xFFFFFF)
    return table


_CRC = _crc_table()


def crc24q(data):
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC[((crc >> 16) ^ byte) & 0xFF]
    return crc


class Framer:
    """Byte stream -> CRC-checked RTCM3 frames; resyncs on 0xD3 after junk or a bad CRC."""

    def __init__(self):
        self.buffer, self.bad = bytearray(), 0

    def feed(self, data):
        self.buffer += data
        frames = []
        while True:
            start = self.buffer.find(0xD3)
            if start < 0:
                self.buffer.clear()
                return frames
            del self.buffer[:start]
            if len(self.buffer) < 3:
                return frames
            if self.buffer[1] & 0xFC:       # the 6 reserved bits are zero in every RTCM3 frame
                del self.buffer[:1]
                continue
            length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
            if len(self.buffer) < length + 6:
                return frames
            frame = bytes(self.buffer[:length + 6])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], 'big'):
                frames.append(frame)
                del self.buffer[:length + 6]
            else:
                self.bad += 1
                del self.buffer[:1]


def frame_id(frame):
    return int.from_bytes(frame[3:5], 'big') >> 4 if len(frame) >= 5 else None


# --- HTTP / NTRIP client --------------------------------------------------------------------

class Dechunker:
    """HTTP/1.1 chunked transfer decoding, fed in arbitrary pieces (NTRIP 2 data streams)."""

    def __init__(self):
        self.buffer, self.left, self.done = bytearray(), 0, False

    def feed(self, data):
        self.buffer += data
        out = bytearray()
        while self.buffer and not self.done:
            if self.left > 0:
                take = self.buffer[:self.left]
                out += take
                del self.buffer[:len(take)]
                self.left -= len(take)
                if self.left == 0:
                    self.left = -2          # the CRLF after the chunk
                continue
            if self.left < 0:
                if len(self.buffer) < -self.left:
                    break
                del self.buffer[:-self.left]
                self.left = 0
                continue
            end = self.buffer.find(b'\r\n')
            if end < 0:
                break
            size = int(bytes(self.buffer[:end]).split(b';')[0] or b'0', 16)
            del self.buffer[:end + 2]
            if size == 0:
                self.done = True
            self.left = size
        return bytes(out)


def _connect(host, port, tls, timeout):
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    if tls:
        sock = ssl.create_default_context().wrap_socket(sock, server_hostname=host)
    return sock


def request(mount, host, port, user=None, password=None, version=2, gga_line=None):
    """The GET for a mountpoint ('' for the sourcetable)."""
    lines = [f'GET /{mount} HTTP/1.{1 if version == 2 else 0}', f'User-Agent: {AGENT}']
    if version == 2:
        lines += [f'Host: {host}:{port}', 'Ntrip-Version: Ntrip/2.0', 'Connection: close']
        if gga_line:
            lines.append('Ntrip-GGA: ' + gga_line.strip())
    if user:
        token = base64.b64encode(f'{user}:{password or ""}'.encode()).decode()
        lines.append('Authorization: Basic ' + token)
    return ('\r\n'.join(lines) + '\r\n\r\n').encode()


class Client:
    """One connection to a caster: the sourcetable, or a mountpoint's RTCM stream with GGA uplink."""

    def __init__(self, caster, env=None, connect=_connect):
        self.caster, self.env, self.connect = caster, env or {}, connect
        self.sock, self.chunks, self.rest = None, None, b''

    def _open(self, mount, gga_line=None, timeout=5.0):
        c = self.caster
        user, password = credentials(c, self.env) if mount else (None, None)
        if mount and needs_login(c) and not user:
            raise NtripError('auth', f"{c['name']}: no credentials ({c.get('user_env')} unset)")
        version = int(c.get('version', 2))
        sock = self.connect(c['host'], int(c['port']), bool(c.get('tls')), timeout)
        sock.sendall(request(mount, c['host'], c['port'], user, password, version, gga_line))
        head = b''
        while b'\r\n\r\n' not in head and len(head) < 8192:
            if head.startswith(b'ICY 200') and b'\r\n' in head:
                break       # NTRIP 1 data: one status line, maybe a blank line, then RTCM
            data = sock.recv(4096)
            if not data:
                break
            head += data
        status, _, rest = head.partition(b'\r\n')
        headers, _, body = rest.partition(b'\r\n\r\n') if b'\r\n\r\n' in head else (b'', b'', rest)
        if head.startswith(b'ICY 200'):
            body = rest.removeprefix(b'\r\n')
            headers = b''
        self.sock, self.rest = sock, body
        self.chunks = Dechunker() if b'chunked' in headers.lower() else None
        text = status.decode('latin-1', 'replace')
        code = _number(text.split()[1], int) if len(text.split()) > 1 else None
        if code == 401 or code == 403:
            self.close()
            raise NtripError('auth', f"{c['name']}: login refused ({text.strip()})")
        if mount and (text.startswith('SOURCETABLE') or code == 404):
            self.close()
            raise NtripError('mount', f"{c['name']}/{mount}: no such mountpoint")
        if code != 200:
            self.close()
            raise NtripError('link', f"{c['name']}: {text.strip() or 'no reply'}")
        return sock

    def sourcetable(self, timeout=5.0):
        """The caster's STR records (no login: tables are public on every caster we use)."""
        sock = self._open('', timeout=timeout)
        data = self.rest
        try:
            while b'ENDSOURCETABLE' not in data:
                piece = sock.recv(65536)
                if not piece:
                    break
                data += piece
        finally:
            self.close()
        if self.chunks:
            data = self.chunks.feed(data)
        return parse_sourcetable(data.decode('latin-1', 'replace'), self.caster['name'])

    def open(self, mount, gga_line=None, timeout=5.0):
        sock = self._open(mount, gga_line, timeout)
        if gga_line and int(self.caster.get('version', 2)) != 2:
            sock.sendall(gga_line.encode())        # NTRIP 1 has no header for it: send after the reply
        sock.settimeout(0.5)
        return self

    def read(self):
        """Stream bytes, b'' on a read timeout; NtripError('link') when the caster closes."""
        if self.rest:
            data, self.rest = self.rest, b''
        else:
            try:
                data = self.sock.recv(4096)
            except (socket.timeout, TimeoutError):
                return b''
            if not data:
                raise NtripError('link', f"{self.caster['name']}: caster closed the stream")
        return self.chunks.feed(data) if self.chunks else data

    def send(self, text):
        self.sock.sendall(text.encode())

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None


# --- the failover source --------------------------------------------------------------------

class NtripSource:
    """Streams the best mountpoint to ``emit(frame, stamp)``; fails over and re-picks by itself.

    ``position()`` returns (lat, lon, alt) or None. ``wanted`` gates connecting: the corrections
    mux sets it while NTRIP is (or may become) the active source, so a caster is not held idle.
    """

    def __init__(self, settings, emit=lambda frame, stamp: None, position=lambda: None, env=None,
                 connect=_connect, clock=time.monotonic):
        self.settings = {**DEFAULTS, **settings}
        self.casters = [c for c in self.settings['casters'] if c.get('enabled', True)]
        self.biases = {c['name']: float(c.get('bias_km', 0.0)) for c in self.casters}
        self.groups = {c['name']: c.get('group', c['name']) for c in self.casters}
        self.emit, self.position, self.connect, self.clock = emit, position, connect, clock
        self.env = env if env is not None else {**read_env_file(secrets_file(self.settings['secrets']) or ''), **os.environ}
        self.tables, self.fetched = {}, {}          # caster -> streams, caster -> clock stamp
        self.resting, self.strikes = {}, {}         # key or caster name -> until; key -> count
        self.lock = threading.RLock()
        self.wanted, self.stopped = threading.Event(), threading.Event()
        self.thread = None
        self.state = dict(phase='off', detail='', mount='', caster='', km=None, vrs=False, station=None,
                          candidates=[], failovers=0, last_error='', received=0, last=None, gga_sent=0)

    # status
    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.state))

    def best_km(self):
        """Distance of the best usable candidate, None before a table and a position."""
        ranked = self.ranked(peek=True)
        return ranked[0].km if ranked else None

    @property
    def available(self):
        return bool(self.ranked(peek=True))

    # picking
    def _resting(self):
        now = self.clock()
        return {k for k, until in self.resting.items() if until > now}

    def ranked(self, peek=False):
        here = self.position()
        if here is None:
            return []
        resting = self._resting()
        streams, closed = [], set()
        for caster in self.casters:
            streams += self.tables.get(caster['name'], [])
            if caster['name'] in resting or (needs_login(caster) and not credentials(caster, self.env)[0]):
                closed.add(caster['name'])      # its table still locates network stations
        ranked = [c for c in rank(streams, here[:2], self.settings, self.biases, resting, self.groups)
                  if c.stream.caster not in closed]
        if not peek:
            self.update(candidates=[c.summary() for c in ranked[:5]])
        return ranked

    def refresh(self, force=False):
        """Fetch each caster's sourcetable when older than refresh_s; a failure rests that caster."""
        now = self.clock()
        for caster in self.casters:
            name = caster['name']
            if not force and now - self.fetched.get(name, -1e9) < self.settings['refresh_s']:
                continue
            if name in self._resting():
                continue
            try:
                self.tables[name] = Client(caster, self.env, self.connect).sourcetable(self.settings['timeout_s'])
                self.fetched[name] = now
            except (OSError, NtripError, ValueError) as error:
                self.fetched[name] = now - self.settings['refresh_s'] + self.settings['mount_cooldown_s']
                self.update(last_error=f'{name} sourcetable: {error}')

    def rest(self, error, candidate):
        """Cool down the mountpoint, or the whole caster on a refused login."""
        now = self.clock()
        if error.kind == 'auth':
            self.resting[candidate.stream.caster] = now + self.settings['auth_cooldown_s']
        else:
            strikes = self.strikes.get(candidate.key, 0) + 1
            self.strikes[candidate.key] = strikes
            wait = min(self.settings['mount_cooldown_s'] * 2 ** (strikes - 1), 600.0)
            self.resting[candidate.key] = now + wait
        with self.lock:
            self.state['failovers'] += 1
            self.state['last_error'] = str(error)

    # running
    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self):
        self.stopped.clear()
        self.update(phase='connecting', detail='reading sourcetables')
        self.thread = threading.Thread(target=self.run, name='ntrip', daemon=True)
        self.thread.start()

    def close(self):
        self.stopped.set()
        self.wanted.set()
        if self.thread:
            self.thread.join(timeout=4)
        self.update(phase='off', detail='stopped')

    def run(self):
        while not self.stopped.is_set():
            self.step()

    def step(self):
        """One pick-connect-stream cycle; returns when the stream ends, fails or is re-picked."""
        self.refresh()
        if not self.wanted.is_set():
            self.update(phase='standby', detail='standby', mount='')
            self.wanted.wait(1)         # close() sets it too
            return
        ranked = self.ranked()
        if not ranked:
            here = self.position()
            self.update(phase='connecting', mount='', detail='waiting for a rough position' if here is None
                        else 'no usable mountpoint (credentials, cooldowns or tables)')
            self.stopped.wait(2)
            return
        best = ranked[0]
        try:
            self.stream(best)
        except NtripError as error:
            self.rest(error, best)
            self.update(phase='connecting', detail='failover: ' + str(error))

    def _gga(self):
        here = self.position()
        return gga(*here) if here else None

    def stream(self, candidate):
        s, settings = candidate.stream, self.settings
        caster = next(c for c in self.casters if c['name'] == s.caster)
        self.update(phase='connecting', detail=f'connecting {s.caster}/{s.mount}', mount=s.mount, caster=s.caster,
                    km=round(candidate.km, 1), vrs=s.vrs,
                    station=None if s.vrs else dict(latitude=s.lat, longitude=s.lon))
        line = self._gga() if s.nmea else None
        client = Client(caster, self.env, self.connect).open(s.mount, line, settings['timeout_s'])
        framer = Framer()
        start = last_frame = picked = self.clock()
        last_gga = start if line else -1e9
        got = False         # the first frame may take the connect grace; later gaps only stale_s
        try:
            while not self.stopped.is_set() and self.wanted.is_set():
                data = client.read()
                now = self.clock()
                for frame in framer.feed(data):
                    # stamped as read, like the base's frames, so the sinks' 2 s stale drop applies
                    self.emit(frame, now)
                    last_frame, got = now, True
                    with self.lock:
                        self.state['received'] += 1
                        self.state['last'] = now
                        if self.state['phase'] != 'streaming':
                            self.state.update(phase='streaming', detail=f'{s.caster}/{s.mount}')
                limit = settings['stale_s'] if got else max(settings['stale_s'], settings['connect_grace_s'])
                if now - last_frame > limit:
                    raise NtripError('stale', f'{s.caster}/{s.mount}: no corrections for {now - last_frame:.0f} s')
                if got and now - start > 60:
                    self.strikes.pop(candidate.key, None)       # a minute of stream forgives earlier failures
                if s.nmea and now - last_gga >= settings['gga_every_s']:
                    line = self._gga()
                    if line:
                        client.send(line)
                        last_gga = now
                        with self.lock:
                            self.state['gga_sent'] += 1
                if now - picked >= settings['repick_s']:
                    picked = now
                    self.refresh()
                    better = self.ranked()
                    if better and better[0].key != candidate.key and \
                            better[0].cost + settings['repick_margin_km'] < self._cost_now(candidate, better):
                        self.update(detail=f're-picked {better[0].stream.caster}/{better[0].stream.mount}')
                        return
        except OSError as error:
            raise NtripError('link', f'{s.caster}/{s.mount}: {error}') from None
        finally:
            client.close()

    @staticmethod
    def _cost_now(candidate, ranked):
        """The current mountpoint's cost at the new position (it moves with the boat)."""
        for c in ranked:
            if c.key == candidate.key:
                return c.cost
        return float('inf')


# --- rough position ---------------------------------------------------------------------------

class Position:
    """The rough position for picking and GGA: the freshest of the rover fix, the base's own fix,
    then the last one saved (survives reboots), then the configured fallback."""

    def __init__(self, cache=None, fallback=None, max_age=30.0, clock=time.monotonic):
        self.cache = Path(os.path.expanduser(str(cache))) if cache else None
        self.fallback, self.max_age, self.clock = fallback, max_age, clock
        self.sources, self.saved = {}, None
        if self.cache and self.cache.is_file():
            try:
                saved = json.loads(self.cache.read_text())
                self.saved = tuple(float(saved[k]) for k in ('lat', 'lon', 'alt'))
            except (OSError, ValueError, KeyError, TypeError):
                self.saved = None
        self.written = -1e9

    def offer(self, name, lat, lon, alt=0.0):
        """A fix from ``name`` (rover or base); ignored when not finite."""
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (lat, lon, alt)):
            return
        if lat == 0 and lon == 0:
            return
        self.sources[name] = ((lat, lon, alt), self.clock())
        self.saved = (lat, lon, alt)
        if self.cache and self.clock() - self.written > 60:
            self.written = self.clock()
            try:
                self.cache.parent.mkdir(parents=True, exist_ok=True)
                self.cache.write_text(json.dumps(dict(lat=lat, lon=lon, alt=alt)))
            except OSError:
                pass

    def __call__(self):
        now = self.clock()
        for name in ('rover', 'base'):
            fix = self.sources.get(name)
            if fix and now - fix[1] <= self.max_age:
                return fix[0]
        if self.saved:
            return self.saved
        if self.fallback:
            return (float(self.fallback['latitude']), float(self.fallback['longitude']),
                    float(self.fallback.get('height', 0.0)))
        return None
