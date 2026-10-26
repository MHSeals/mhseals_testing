"""NTRIP picker, client, GGA uplink and failover against mocked sourcetables and a local mock caster.

No internet: the casters are threads on 127.0.0.1. Positions are real TX/FL test sites.
"""

import base64
import socket
import socketserver
import threading
import time

import pytest

from mhseals_testing import ntrip
from mhseals_testing.ntrip import (Candidate, Client, Dechunker, Framer, NtripError, NtripSource, Position,
                                   crc24q, distance_km, gga, load_settings, parse_sourcetable, rank)

BENDERSON = (27.3721, -82.4469)      # RoboBoat course, Nathan Benderson Park, Sarasota FL
ARLINGTON = (32.7300, -97.1100)      # UT Arlington, TX (the 10-22 audit site is 32.612, -97.120)


def STR(mount, msgs, lat, lon, carrier=2, nav='GPS+GLO+GAL+BDS', nmea=0, solution=0, fmt='RTCM 3.2'):
    return (f'STR;{mount};{mount};{fmt};{msgs};{carrier};{nav};NET;USA;{lat:.4f};{lon:.4f};'
            f'{nmea};{solution};gen;none;B;N;9600;')


MSM7 = '1005(10),1077(1),1087(1),1097(1),1127(1),1230(10)'
MSM4 = '1005(10),1074(1),1084(1),1094(1),1124(1),1230(10)'
TABLES = {
    'FPRN': [STR('SRSA', MSM4, 27.3800, -82.5600), STR('TAMP', MSM4, 27.9500, -82.4600),
             STR('OLD1', '1004(1),1012(1),1005(10)', 27.4000, -82.5000, nav='GPS+GLO', fmt='RTCM 3.0'),
             STR('RAW1', '', 27.3725, -82.4470, fmt='RAW')],
    'FPRN-NET': [STR('NAD_MSM4_VRS', MSM4, 28.0, -82.0, nmea=1, solution=1),
                 STR('NAD_MSM4_NEAR', MSM4, 28.0, -82.0, nmea=1, solution=1),
                 STR('NAD_RTCM2_VRS', '18,19', 28.0, -82.0, nmea=1, solution=1, fmt='RTCM 2.3')],
    'RTK2go': [STR('BendersonBase', MSM7, 27.3740, -82.4440),
               STR('DFW_L1', '1005(10),1077(1)', 32.7310, -97.1210, carrier=1, nav='GPS')],
    'EarthScope': [STR('TXAR_RTCM3P3', MSM7, 32.7500, -97.0800), STR('KVTX_RTCM3P3', MSM7, 27.5458, -97.8836)],
}
CASTERS = [dict(name='FPRN', host='h', port=1, user_env='FPRN_USER', password_env='FPRN_PASSWORD'),
           dict(name='FPRN-NET', group='FPRN', host='h', port=2, user_env='FPRN_USER', password_env='FPRN_PASSWORD', bias_km=2),
           dict(name='EarthScope', host='h', port=3, user_env='ES_USER', password_env='ES_PASSWORD', bias_km=2),
           dict(name='RTK2go', host='h', port=4, user_env='RTK2GO_USER', bias_km=5)]
BIAS = {c['name']: c.get('bias_km', 0) for c in CASTERS}
GROUPS = {c['name']: c.get('group', c['name']) for c in CASTERS}


def streams():
    return [s for name, lines in TABLES.items() for s in parse_sourcetable('\n'.join(lines), name)]


# --- sourcetable and picking ---------------------------------------------------------------

def test_sourcetable_parses_v1_and_v2_bodies_and_skips_cas_net_lines():
    body = '\r\n'.join(['CAS;rtk2go.com;2101;RTK2go;SNIP;0;USA;0;0;0.0.0.0;0;none',
                        'NET;SNIP;RTK2go;B;N;none;none;none;none'] + TABLES['RTK2go'] + ['ENDSOURCETABLE'])
    for text in ('SOURCETABLE 200 OK\r\n\r\n' + body, body):
        found = parse_sourcetable(text, 'RTK2go')
        assert [s.mount for s in found] == ['BendersonBase', 'DFW_L1']
    base = found[0]
    assert (base.carrier, ntrip.msm_level(base), ntrip.systems(base)) == (2, 7, {'GPS', 'GLO', 'GAL', 'BDS'})
    assert found[1].carrier == 1 and ntrip.systems(found[1]) == {'GPS'}


def test_florida_picks_the_nearby_msm7_base_then_the_vrs_then_single_msm4():
    ranked = rank(streams(), BENDERSON, biases=BIAS, groups=GROUPS)
    order = [(c.stream.caster, c.stream.mount) for c in ranked[:4]]
    assert order == [('RTK2go', 'BendersonBase'), ('FPRN-NET', 'NAD_MSM4_VRS'),
                     ('FPRN-NET', 'NAD_MSM4_NEAR'), ('FPRN', 'SRSA')]
    mounts = [c.stream.mount for c in ranked]
    assert 'RAW1' not in mounts and 'NAD_RTCM2_VRS' not in mounts      # not RTCM 3
    vrs = next(c for c in ranked if c.stream.mount == 'NAD_MSM4_VRS')
    near = next(c for c in ranked if c.stream.mount == 'NAD_MSM4_NEAR')
    # the network is as near as FPRN's nearest RTCM 3 station (OLD1, 6.1 km; RAW1 does not count)
    assert near.km == pytest.approx(distance_km(BENDERSON, (27.40, -82.50))) and vrs.km == pytest.approx(near.km / 2)
    legacy = next(c for c in ranked if c.stream.mount == 'OLD1')
    srsa = next(c for c in ranked if c.stream.mount == 'SRSA')
    assert legacy.km < srsa.km and legacy.cost > srsa.cost       # nearer, but the old message set costs more
    assert not next(c for c in ranked if c.stream.mount == 'KVTX_RTCM3P3').in_range


def test_texas_prefers_a_dual_band_station_over_a_nearer_l1_gps_only_one():
    ranked = rank(streams(), ARLINGTON, biases=BIAS, groups=GROUPS)
    assert ranked[0].stream.mount == 'TXAR_RTCM3P3' and ranked[0].in_range
    l1 = next(c for c in ranked if c.stream.mount == 'DFW_L1')
    assert l1.km < ranked[0].km and l1.cost > ranked[0].cost
    assert all(not c.in_range for c in ranked if c.stream.caster.startswith('FPRN'))


def test_beyond_max_km_is_a_last_resort_and_resting_mounts_are_skipped():
    far = (30.27, -97.74)         # Austin: nothing within 30 km in these tables
    ranked = rank(streams(), far, biases=BIAS, groups=GROUPS)
    assert ranked and not any(c.in_range for c in ranked)
    assert ranked[0].stream.mount == 'TXAR_RTCM3P3' and ranked[0].km == pytest.approx(280, abs=10)
    skipped = rank(streams(), BENDERSON, biases=BIAS, groups=GROUPS, resting={('RTK2go', 'BendersonBase')})
    assert skipped[0].stream.mount == 'NAD_MSM4_VRS'


def test_shipped_casters_load_and_overlay_merges_by_name(tmp_path):
    settings = load_settings({})
    names = [c['name'] for c in settings['casters']]
    assert names == ['FPRN', 'FPRN-NET', 'EarthScope', 'RTK2go']
    assert all('password' not in c and 'user' not in c for c in settings['casters'])     # names of env vars only
    merged = load_settings({'ntrip': {'max_km': 20, 'casters': [{'name': 'RTK2go', 'enabled': False},
                                                                  {'name': 'Mine', 'host': 'x', 'port': 2101}]}})
    assert merged['max_km'] == 20
    rtk2go = next(c for c in merged['casters'] if c['name'] == 'RTK2go')
    assert rtk2go['enabled'] is False and rtk2go['host']
    assert merged['casters'][-1]['name'] == 'Mine'
    with pytest.raises(ValueError, match='missing host'):
        load_settings({'ntrip': {'casters': [{'name': 'Bad', 'port': 1}]}})


def test_secrets_file_feeds_credentials_and_an_unset_login_is_not_streamed(tmp_path):
    env = tmp_path / 'secrets.env'
    env.write_text('# boat secrets\nFPRN_USER=team\nexport FPRN_PASSWORD="p w"\nRTK2GO_USER=\n')
    values = ntrip.read_env_file(env)
    assert ntrip.credentials(CASTERS[0], values) == ('team', 'p w')
    assert ntrip.credentials(CASTERS[3], values) == (None, '')
    source = NtripSource(dict(casters=CASTERS), position=lambda: BENDERSON + (5.0,), env=values)
    source.tables = {name: parse_sourcetable('\n'.join(lines), name) for name, lines in TABLES.items()}
    ranked = source.ranked()
    assert {c.stream.caster for c in ranked} == {'FPRN', 'FPRN-NET'}       # only FPRN has a login
    assert ranked[0].stream.mount == 'NAD_MSM4_VRS'
    assert source.snapshot()['candidates'][0]['mount'] == 'NAD_MSM4_VRS'


def test_the_shipped_secrets_merge_astros_file_over_the_package_one(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('ASTRO_SECRETS', raising=False)
    shipped = load_settings({})['secrets']
    assert shipped == ['~/.config/astro/secrets.env', '~/.config/mhseals_testing/secrets.env'] == ntrip.DEFAULTS['secrets']
    astro, package = tmp_path / '.config' / 'astro' / 'secrets.env', tmp_path / '.config' / 'mhseals_testing' / 'secrets.env'

    def write(path, text, mode=0o600):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        path.chmod(mode)

    assert ntrip.read_secrets(shipped) == {}                      # neither exists
    write(package, 'FPRN_USER=old\nFPRN_PASSWORD=pkg\n')
    assert ntrip.read_secrets(shipped) == {'FPRN_USER': 'old', 'FPRN_PASSWORD': 'pkg'}
    write(astro, 'FPRN_USER=new\nRTK2GO_USER=a\n')
    assert ntrip.read_secrets(shipped) == {'FPRN_USER': 'new', 'FPRN_PASSWORD': 'pkg', 'RTK2GO_USER': 'a'}
    for path in (astro, package):                                 # either one readable by others: refused
        path.chmod(0o644)
        with pytest.raises(PermissionError, match='is mode 644; secrets must be private: chmod 600'):
            ntrip.read_secrets(shipped)
        path.chmod(0o600)
    override = tmp_path / 'elsewhere.env'
    monkeypatch.setenv('ASTRO_SECRETS', str(override))
    assert ntrip.read_secrets(shipped)['FPRN_USER'] == 'old'      # $ASTRO_SECRETS replaces astro's path
    write(override, 'FPRN_USER=env\n')
    assert ntrip.read_secrets(shipped)['FPRN_USER'] == 'env'
    assert ntrip.secrets_files(str(package)) == [package]         # an explicit path (rtk.yaml) stays as is
    monkeypatch.setenv('FPRN_USER', 'shell')
    monkeypatch.delenv('FPRN_PASSWORD', raising=False)
    source = NtripSource(dict(casters=CASTERS, secrets=shipped))
    assert source.env['FPRN_USER'] == 'shell'                     # the environment wins over the file
    monkeypatch.delenv('FPRN_USER')
    assert NtripSource(dict(casters=CASTERS, secrets=shipped)).env['FPRN_USER'] == 'env'


def test_distance_is_great_circle():
    assert distance_km(BENDERSON, ARLINGTON) == pytest.approx(1520, rel=0.02)


# --- GGA, framing, chunking ------------------------------------------------------------------

def test_gga_is_valid_nmea_with_checksum():
    import datetime
    when = datetime.datetime(2026, 10, 25, 18, 7, 58, 250000, tzinfo=datetime.timezone.utc)
    line = gga(27.3721, -82.4469, 3.2, when=when)
    assert line.endswith('\r\n')
    body, check = line[1:-2].split('*')
    fields = body.split(',')
    assert fields[:6] == ['GPGGA', '180758.25', '2722.32600', 'N', '08226.81400', 'W']
    value = 0
    for char in body.encode():
        value ^= char
    assert int(check, 16) == value


def frame(number, size=19, fill=0):
    payload = ((number << (size * 8 - 12)) | fill).to_bytes(size, 'big')
    head = b'\xd3' + bytes([size >> 8, size & 0xFF]) + payload
    return head + crc24q(head).to_bytes(3, 'big')


def test_framer_resyncs_after_junk_and_a_bad_crc_and_handles_split_reads():
    good, other = frame(1005), frame(1077, 400, fill=7)
    bad = bytearray(frame(1087))
    bad[-1] ^= 0xFF
    stream = b'junk\xd3' + bytes(bad) + good + other
    framer, out = Framer(), []
    for i in range(0, len(stream), 7):
        out += framer.feed(stream[i:i + 7])
    assert out == [good, other] and framer.bad >= 1
    assert ntrip.frame_id(other) == 1077


def test_crc24q_matches_pyrtcm():
    pyrtcm = pytest.importorskip('pyrtcm')
    data = frame(1005)[:-3]
    assert crc24q(data) == pyrtcm.calc_crc24q(data)


def test_dechunker_joins_chunks_fed_in_pieces():
    raw = b'5\r\nhello\r\n6;ext=1\r\n world\r\n0\r\n\r\n'
    d, out = Dechunker(), b''
    for i in range(len(raw)):
        out += d.feed(raw[i:i + 1])
    assert out == b'hello world' and d.done


# --- a mock caster ---------------------------------------------------------------------------

class MockCaster:
    """NTRIP 1/2 caster on 127.0.0.1. mounts: name -> dict(frames, interval, silent, close_after, nmea)."""

    def __init__(self, table, mounts, users=None, version=2, chunked=True):
        self.table, self.mounts, self.users = table, mounts, users
        self.version, self.chunked = version, chunked
        self.requests, self.ggas, self.sent = [], [], []       # sent: (mount, index, monotonic time)
        self.connections = {}
        caster = self

        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                caster.serve(self.request)

        socketserver.ThreadingTCPServer.allow_reuse_address = True
        self.server = socketserver.ThreadingTCPServer(('127.0.0.1', 0), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()

    def reply(self, sock, status, headers, body=b''):
        sock.sendall((status + '\r\n' + ''.join(h + '\r\n' for h in headers) + '\r\n').encode() + body)

    def wrap(self, data):
        return f'{len(data):X}\r\n'.encode() + data + b'\r\n' if self.chunked and self.version == 2 else data

    def serve(self, sock):
        head = b''
        while b'\r\n\r\n' not in head:
            data = sock.recv(4096)
            if not data:
                return
            head += data
        text, _, rest = head.partition(b'\r\n\r\n')
        lines = text.decode().split('\r\n')
        mount = lines[0].split()[1].lstrip('/')
        headers = dict(line.split(': ', 1) for line in lines[1:] if ': ' in line)
        self.requests.append((mount, headers))
        if not mount:
            body = ('\r\n'.join(self.table) + '\r\nENDSOURCETABLE\r\n').encode()
            if self.version == 2:
                self.reply(sock, 'HTTP/1.1 200 OK', ['Content-Type: gnss/sourcetable', f'Content-Length: {len(body)}'], body)
            else:
                self.reply(sock, 'SOURCETABLE 200 OK', ['Content-Type: text/plain'], body)
            return
        if mount not in self.mounts:
            self.reply(sock, 'SOURCETABLE 200 OK' if self.version == 1 else 'HTTP/1.1 404 Not Found', [])
            return
        if self.users is not None:
            auth = headers.get('Authorization', '').removeprefix('Basic ')
            if base64.b64decode(auth or b'').decode() not in self.users:
                self.reply(sock, 'HTTP/1.1 401 Unauthorized', ['WWW-Authenticate: Basic realm="/"'])
                return
        if 'Ntrip-GGA' in headers:
            self.ggas.append(headers['Ntrip-GGA'])
        spec = self.mounts[mount]
        self.connections[mount] = self.connections.get(mount, 0) + 1
        if self.version == 2:
            self.reply(sock, 'HTTP/1.1 200 OK', ['Ntrip-Version: Ntrip/2.0', 'Content-Type: gnss/data']
                       + (['Transfer-Encoding: chunked'] if self.chunked else []))
        else:
            sock.sendall(b'ICY 200 OK\r\n')
        sock.settimeout(0.01)
        index, pending = 0, rest
        try:
            while True:
                try:
                    pending += sock.recv(4096)
                except socket.timeout:
                    pass
                while b'\r\n' in pending:
                    line, _, pending = pending.partition(b'\r\n')
                    if line.startswith(b'$'):
                        self.ggas.append(line.decode())
                if spec.get('silent'):
                    time.sleep(spec.get('interval', 0.05))
                    continue
                if spec.get('close_after') is not None and index >= spec['close_after']:
                    return
                data = spec.get('frame', lambda i: frame(1077, 120, fill=i))(index)
                self.sent.append((mount, index, time.monotonic()))
                sock.sendall(self.wrap(data))
                index += 1
                time.sleep(spec.get('interval', 0.05))
        except OSError:
            return


@pytest.fixture
def casters():
    made = []

    def make(*args, **kwargs):
        caster = MockCaster(*args, **kwargs)
        made.append(caster)
        return caster
    yield make
    for caster in made:
        caster.close()


FAST = dict(stale_s=0.3, connect_grace_s=0.6, mount_cooldown_s=30.0, auth_cooldown_s=30.0, timeout_s=2.0,
            refresh_s=600.0, repick_s=600.0, gga_every_s=10.0)


def run_source(source, until, timeout=6.0):
    """Start the source wanted, wait for ``until(source)`` and stop it; returns the snapshot."""
    source.wanted.set()
    source.start()
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline and not until(source):
            time.sleep(0.02)
        return source.snapshot()
    finally:
        source.close()


def entry(name, caster, **extra):
    return dict(name=name, host='127.0.0.1', port=caster.port, version=caster.version, **extra)


@pytest.mark.parametrize('version,chunked', [(2, True), (2, False), (1, False)])
def test_client_reads_the_table_and_streams_frames_in_ntrip_1_and_2(casters, version, chunked):
    caster = casters([STR('A', MSM7, *BENDERSON)], {'A': {}}, version=version, chunked=chunked)
    client = Client(entry('C', caster))
    assert [s.mount for s in client.sourcetable()] == ['A']
    client.open('A')
    framer, frames = Framer(), []
    deadline = time.monotonic() + 3
    while len(frames) < 3 and time.monotonic() < deadline:
        frames += framer.feed(client.read())
    client.close()
    assert frames[:3] == [frame(1077, 120, fill=i) for i in range(3)]


def test_wrong_login_unknown_mount_and_missing_credentials_raise_kinds(casters):
    caster = casters([STR('A', MSM7, *BENDERSON)], {'A': {}}, users={'team:good'})
    c = entry('C', caster, user_env='U', password_env='P')
    with pytest.raises(NtripError) as error:
        Client(c, {'U': 'team', 'P': 'bad'}).open('A')
    assert error.value.kind == 'auth'
    with pytest.raises(NtripError) as error:
        Client(c, {'U': 'team', 'P': 'good'}).open('NOPE')
    assert error.value.kind == 'mount'
    with pytest.raises(NtripError) as error:
        Client(c, {}).open('A')
    assert error.value.kind == 'auth' and caster.connections == {}       # never sent an empty login
    Client(c, {'U': 'team', 'P': 'good'}).open('A').close()
    assert caster.connections == {'A': 1}


# --- failover ----------------------------------------------------------------------------

def test_stale_mount_fails_over_to_the_next_and_rests(casters):
    caster = casters([STR('NEAR', MSM7, 27.373, -82.447), STR('NEXT', MSM7, 27.40, -82.47)],
                     {'NEAR': {'silent': True}, 'NEXT': {}})
    got = []
    source = NtripSource(dict(FAST, casters=[entry('C', caster)]), lambda f, t: got.append(f),
                         position=lambda: BENDERSON + (0.0,), env={})
    state = run_source(source, lambda s: s.snapshot()['mount'] == 'NEXT' and len(got) >= 3)
    assert state['mount'] == 'NEXT' and state['phase'] == 'streaming' and len(got) >= 3
    assert state['failovers'] == 1 and 'no corrections' in state['last_error']
    assert ('C', 'NEAR') in source._resting()


def test_auth_failure_rests_the_whole_caster_and_uses_the_next_caster(casters):
    locked = casters([STR('BEST', MSM7, 27.373, -82.447)], {'BEST': {}}, users={'team:right'})
    open_ = casters([STR('OTHER', MSM7, 27.45, -82.50)], {'OTHER': {}})
    got = []
    source = NtripSource(dict(FAST, casters=[entry('Locked', locked, user_env='U', password_env='P'),
                                             entry('Open', open_)]),
                         lambda f, t: got.append(f), position=lambda: BENDERSON + (0.0,), env={'U': 'team', 'P': 'wrong'})
    state = run_source(source, lambda s: len(got) >= 2)
    assert (state['caster'], state['mount']) == ('Open', 'OTHER')
    assert 'login refused' in state['last_error'] and 'Locked' in source._resting()
    assert locked.connections == {}


def test_disconnect_fails_over_and_the_dropped_mount_returns_after_its_cooldown(casters):
    caster = casters([STR('A', MSM7, 27.373, -82.447), STR('B', MSM7, 27.42, -82.49)],
                     {'A': {'close_after': 3}, 'B': {}})
    got = []
    # A closes after 3 frames: B takes over at once; once A has rested, the re-pick goes back to it
    settings = dict(FAST, casters=[entry('C', caster)], mount_cooldown_s=0.5, repick_s=0.3)
    source = NtripSource(settings, lambda f, t: got.append(f), position=lambda: BENDERSON + (0.0,), env={})
    state = run_source(source, lambda s: caster.connections.get('A', 0) >= 2, timeout=8)
    assert caster.connections['A'] >= 2 and caster.connections.get('B', 0) >= 1
    assert state['failovers'] >= 1 and 'closed' in state['last_error']


def test_vrs_mount_gets_gga_at_connect_and_on_its_cadence(casters):
    caster = casters([STR('NAD_MSM4_VRS', MSM4, 28.0, -82.0, nmea=1, solution=1), STR('SRSA', MSM4, 27.38, -82.56)],
                     {'NAD_MSM4_VRS': {}, 'SRSA': {}})
    source = NtripSource(dict(FAST, casters=[entry('C', caster)], gga_every_s=0.2), position=lambda: BENDERSON + (4.0,),
                         env={})
    state = run_source(source, lambda s: len(caster.ggas) >= 3)
    assert state['mount'] == 'NAD_MSM4_VRS' and state['vrs'] and state['station'] is None
    assert len(caster.ggas) >= 3 and state['gga_sent'] >= 2
    for line in caster.ggas:
        assert line.startswith('$GPGGA,') and ',2722.32' in line and ',08226.81' in line
    plain = casters([STR('SRSA', MSM4, 27.38, -82.56)], {'SRSA': {}})
    run_source(NtripSource(dict(FAST, casters=[entry('P', plain)], gga_every_s=0.1), position=lambda: BENDERSON + (4.0,),
                           env={}), lambda s: len(plain.sent) >= 5)
    assert plain.ggas == []           # a single base that does not ask gets no NMEA (RTK2go bans chatty clients)


def test_moving_boat_repicks_the_nearer_station(casters):
    caster = casters([STR('SOUTH', MSM7, 27.37, -82.45), STR('NORTH', MSM7, 27.95, -82.46)], {'SOUTH': {}, 'NORTH': {}})
    here = [BENDERSON + (0.0,)]
    source = NtripSource(dict(FAST, casters=[entry('C', caster)], repick_s=0.2), position=lambda: here[0], env={})

    def moved(s):
        if s.snapshot()['mount'] == 'SOUTH' and s.snapshot()['received'] >= 2:
            here[0] = (27.94, -82.45, 0.0)      # trailered to Tampa
        return s.snapshot()['mount'] == 'NORTH'
    state = run_source(source, moved)
    assert state['mount'] == 'NORTH'
    assert caster.connections == {'SOUTH': 1, 'NORTH': 1}
    assert state['failovers'] == 0              # a re-pick is not a failure


def test_unwanted_source_stays_off_the_caster(casters):
    caster = casters([STR('A', MSM7, *BENDERSON)], {'A': {}})
    source = NtripSource(dict(FAST, casters=[entry('C', caster)]), position=lambda: BENDERSON + (0.0,), env={})
    source.start()
    time.sleep(0.4)
    assert source.available and source.best_km() < 1
    source.close()
    assert caster.connections == {} and [m for m, _ in caster.requests] == ['']      # only the table


def test_position_prefers_a_fresh_rover_fix_then_base_then_the_saved_one(tmp_path):
    now = [0.0]
    cache = tmp_path / 'pos.json'
    p = Position(cache, fallback=dict(latitude=1, longitude=2), clock=lambda: now[0])
    assert p() == (1.0, 2.0, 0.0)
    p.offer('base', 27.0, -82.0, 1.0)
    p.offer('rover', 27.1, -82.1, 2.0)
    assert p()[:2] == (27.1, -82.1)
    now[0] = 31.0
    assert p() == (27.1, -82.1, 2.0)            # both stale: the last one seen
    assert Position(cache)() == (27.0, -82.0, 1.0)      # after a reboot: the last one saved (once a minute)
    p.offer('rover', float('nan'), 0.0)
    assert 'rover' in p.sources and p.sources['rover'][0][0] == 27.1


# --- end to end: correction age through the real radio route -----------------------------------

def test_end_to_end_correction_age_mock_caster_to_gps_rtcm_data(casters):
    """Caster send -> NtripSource -> Corrections -> Radio (pymavlink udpout) -> GPS_RTCM_DATA reassembled."""
    pytest.importorskip('pymavlink')
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
    from mhseals_testing.corrections import Corrections
    from mhseals_testing.rtk import Radio
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(('127.0.0.1', 0))
    listener.settimeout(0.05)
    sizes = [25, 140, 380, 560]          # 1005, small MSM, and MSM7 frames that need 3-4 fragments

    def frames(i):
        return frame(1077, sizes[i % len(sizes)], fill=i)
    caster = casters([STR('A', MSM7, *BENDERSON)], {'A': {'frame': frames, 'interval': 0.02}})
    radio = Radio(f'udpout:127.0.0.1:{listener.getsockname()[1]}')
    mux = Corrections([radio], priority='ntrip')
    source = NtripSource(dict(FAST, casters=[entry('C', caster)]), mux.gate('ntrip'),
                         position=lambda: BENDERSON + (0.0,), env={})
    mux.attach(None, source)
    parser, parts, ages = mavlink.MAVLink(None), {}, []
    source.start()
    try:
        ready = time.monotonic() + 3
        while not source.available and time.monotonic() < ready:
            time.sleep(0.01)            # the sourcetable, read while not yet wanted
        mux.evaluate()
        assert source.wanted.is_set() and mux.active == 'ntrip'
        deadline = time.monotonic() + 6
        while len(ages) < 40 and time.monotonic() < deadline:
            try:
                data = listener.recv(4096)
            except socket.timeout:
                continue
            arrived = time.monotonic()
            for msg in parser.parse_buffer(data) or []:
                if msg.get_type() != 'GPS_RTCM_DATA':
                    continue
                payload = bytes(msg.data[:msg.len])
                if not msg.flags & 1:
                    whole = payload
                else:       # as the FCU does: fragments by id until a short one ends the block
                    block = parts.setdefault(msg.flags >> 3, {})
                    block[(msg.flags >> 1) & 3] = payload
                    if msg.len == 180 and len(block) < 4:
                        continue
                    whole = b''.join(block[k] for k in sorted(parts.pop(msg.flags >> 3)))
                assert crc24q(whole[:-3]) == int.from_bytes(whole[-3:], 'big')
                index = int.from_bytes(whole[3:-3], 'big') & 0xFFFF
                sent = next(t for m, i, t in caster.sent if i == index)
                ages.append(arrived - sent)
    finally:
        source.close()
        radio.close()
        listener.close()
    assert len(ages) >= 10
    ages.sort()
    median, worst = ages[len(ages) // 2], ages[-1]
    print(f'correction age caster->GPS_RTCM_DATA: median {median * 1000:.2f} ms, max {worst * 1000:.2f} ms, n={len(ages)}')
    assert median < 0.02 and worst < 0.1
