"""Base receiver wire formats, frame forwarding and resume-without-reconfigure; no ROS or hardware."""

from copy import deepcopy
from types import SimpleNamespace
import io
import pathlib
import pathlib
import time

import pytest
# Optional GNSS libraries: skip this module instead of aborting collection when the host lacks them
pyubx2 = pytest.importorskip('pyubx2')
pyrtcm = pytest.importorskip('pyrtcm')
GET, UBXMessage, UBXReader = pyubx2.GET, pyubx2.UBXMessage, pyubx2.UBXReader
calc_crc24q = pyrtcm.calc_crc24q

from mhseals_testing.rtk import (BaseSession, DEFAULTS, Radio, base_commands, coordinates, fragments,
                                 load_config, rtcm_reader, save_config, split_position)


def rtcm_frame(number=1005):
    payload = ((number << 140) | (1 << 128)).to_bytes(19, 'big')
    frame = b'\xd3\0' + bytes([len(payload)]) + payload
    return frame + calc_crc24q(frame).to_bytes(3, 'big')


class FakeReceiver(io.BytesIO):
    """Serial port that answers polls like a u-blox base and records every command."""

    def __init__(self, model='M8P', mode=1, valid=True, active=False):
        super().__init__()
        self.model, self.mode, self.valid, self.active = model, mode, valid, active
        self.sent, self.pending = [], b''

    def write(self, data):
        try:
            message = UBXReader.parse(data, msgmode=2)
        except Exception:  # pyubx2 only parses a VALGET request in POLL mode
            message = UBXReader.parse(data, msgmode=1)
        self.sent.append(message.identity)
        poll = len(data) == 8 or message.identity == 'CFG-VALGET'  # an empty payload, or the VALGET request
        reply = poll and {'MON-VER': self.version, 'NAV-SVIN': self.survey,
                 'CFG-TMODE3': self.tmode, 'CFG-VALGET': self.tmode}.get(message.identity)
        if reply:
            self.pending += reply().serialize()
        else:
            self.pending += UBXMessage('ACK', 'ACK-ACK', GET, clsID=message.msg_cls[0],
                                       msgID=message.msg_id[0]).serialize()
        return len(data)

    def read(self, count=-1):
        data, self.pending = self.pending[:count], self.pending[count:]
        if not data:
            time.sleep(.002)
        return data

    def version(self):
        payload = b'HPG 1.40'.ljust(30, b'\0') + b'00080000'.ljust(10, b'\0')
        model = b'MOD=NEO-M8P-2' if self.model == 'M8P' else b'MOD=ZED-F9P'
        return UBXMessage('MON', 'MON-VER', GET, payload=payload + model.ljust(30, b'\0'))

    def survey(self):
        return UBXMessage('NAV', 'NAV-SVIN', GET, dur=200, meanAcc=5000, meanX=637813700,
                          meanXHP=25, obs=100, valid=int(self.valid), active=int(self.active))

    def tmode(self):
        if self.model == 'M8P':
            return UBXMessage('CFG', 'CFG-TMODE3', GET, rcvrMode=self.mode)
        # CFG-VALGET reply: version, layer, position, then key (CFG-TMODE-MODE) and value
        payload = bytes([1, 0, 0, 0]) + (0x20030001).to_bytes(4, 'little') + bytes([self.mode])
        return UBXMessage('CFG', 'CFG-VALGET', GET, payload=payload)


def run_session(monkeypatch, port, extra=(), **start):
    """Run one session on the fake receiver until two pushed frames are read; return its state and the frames."""
    import serial
    monkeypatch.setattr(serial, 'Serial', lambda *a, **k: port)
    frames = []
    session = BaseSession([lambda frame, stamp: frames.append(frame), *extra], dict(DEFAULTS, device='/dev/test'))
    session.start(**start)
    deadline = time.monotonic() + 5
    while session.snapshot()['phase'] in ('off', 'connecting', 'configuring') and time.monotonic() < deadline:
        time.sleep(.005)
    port.pending += rtcm_frame() + rtcm_frame(1230)
    while session.snapshot()['received'] < 2 and time.monotonic() < deadline:
        time.sleep(.005)
    state = session.snapshot()
    session.close()
    return state, frames


@pytest.mark.parametrize('model', ['M8P', 'F9P'])
@pytest.mark.parametrize('position', [None, dict(latitude=-33.234567899, longitude=151.123456789, height=-12.3456)])
def test_commands_are_valid_ubx_and_ram_only(model, position):
    commands = base_commands(model, DEFAULTS, position)
    decoded = [UBXReader.parse(c.serialize(), msgmode=1) for c in commands]
    if model == 'M8P':
        assert decoded[0].rcvrMode == 0
        assert decoded[-1].rcvrMode == (2 if position else 1)
        assert not any(c.identity == 'CFG-CFG' for c in commands)
        # 1005, 1077 (MSM7), 1087 (MSM7), 1230 enabled on USB
        assert [d.msgID for d in decoded if d.identity == 'CFG-MSG'][2:] == [5, 0x4D, 0x57, 0xE6]
        if position:
            assert decoded[-1].ecefXOrLat == -332345678
            assert decoded[-1].ecefXOrLatHP == -99
    else:
        assert all(c.identity == 'CFG-VALSET' for c in decoded)
        assert all(c.ram == 1 and c.bbr == 0 and c.flash == 0 for c in decoded)
        assert decoded[-1].CFG_TMODE_MODE == (2 if position else 1)
        assert decoded[-1].CFG_MSGOUT_RTCM_3X_TYPE1074_USB == 1
        assert not hasattr(decoded[-1], 'CFG_MSGOUT_RTCM_3X_TYPE1077_USB')


def test_unknown_model_is_refused():
    with pytest.raises(ValueError, match='unsupported base receiver'):
        base_commands('M10', DEFAULTS)


def test_reader_drops_a_corrupt_frame_and_keeps_the_good_one():
    frame = rtcm_frame()
    errors = []
    bad = frame[:-1] + bytes([frame[-1] ^ 1])
    reader = rtcm_reader(io.BytesIO(bad + frame), protfilter=7, quitonerror=1, errorhandler=errors.append)
    raw, message = reader.read()
    assert raw == frame and message.identity == '1005'
    assert len(errors) == 1


def test_reader_forwards_a_crc_valid_frame_pyrtcm_cannot_decode():
    # An MSM4 header with a truncated body: valid CRC24Q, but pyrtcm cannot walk it.
    frame = rtcm_frame(1074)
    payload = (1074 << 12 | 1).to_bytes(3, 'big')
    body = b'\xd3\0' + bytes([len(payload)]) + payload
    body += calc_crc24q(body).to_bytes(3, 'big')
    with pytest.raises(Exception):
        UBXReader(io.BytesIO(body), protfilter=7, quitonerror=2).read()
    errors = []
    raw, message = rtcm_reader(io.BytesIO(body + frame), protfilter=7, quitonerror=1,
                               errorhandler=errors.append).read()
    assert raw == body and message.identity == '1074' and not errors


def test_pyubx2_floor_has_the_metadata_only_parse_mode_the_reader_relies_on():
    # Older pyubx2 treats _parsing as a bool, so the override would silently decode (and drop) again.
    assert getattr(pyubx2, 'PARSE_META', None) == 2
    assert 'pyubx2>=1.3.9' in (pathlib.Path(__file__).parent.parent / 'setup.py').read_text()


def test_pyubx2_floor_has_the_metadata_only_parse_mode_the_reader_relies_on():
    # Older pyubx2 treats _parsing as a bool, so the override would silently decode (and drop) again.
    assert getattr(pyubx2, 'PARSE_META', None) == 2
    assert 'pyubx2>=1.3.9' in (pathlib.Path(__file__).parent.parent / 'setup.py').read_text()


def test_config_and_negative_high_precision_coordinates_round_trip(tmp_path):
    p = dict(latitude=-33.234567899, longitude=-80.123456789, height=-12.3456, accuracy=.04)
    lat, lat_hp, lon, lon_hp, alt, alt_hp = split_position(p)
    assert (lat * 100 + lat_hp) / 1e9 == p['latitude']
    assert (lon * 100 + lon_hp) / 1e9 == p['longitude']
    assert (alt * 100 + alt_hp) / 1e4 == p['height']
    config = deepcopy(DEFAULTS)
    config['positions']['dock'] = p
    save_config(config, tmp_path / 'rtk.yaml')
    assert load_config(tmp_path / 'rtk.yaml') == config
    assert not list(tmp_path.glob('*.tmp'))
    with pytest.raises(ValueError):
        coordinates(dict(p, height=float('nan')))
    with pytest.raises(ValueError):
        coordinates(dict(p, accuracy=-1))


def test_missing_or_malformed_config_shares_nothing(tmp_path):
    first = load_config(tmp_path / 'missing.yaml')
    first['positions']['dock'] = dict(latitude=0, longitude=0, height=0)
    assert not load_config(tmp_path / 'missing.yaml')['positions']
    (tmp_path / 'bad.yaml').write_text('- not a mapping\n')
    with pytest.raises(ValueError, match='YAML mapping'):
        load_config(tmp_path / 'bad.yaml')


@pytest.mark.parametrize('model', ['M8P', 'F9P'])
def test_survey_request_configures_then_forwards_frames(monkeypatch, model):
    port = FakeReceiver(model)
    state, frames = run_session(monkeypatch, port, survey=True)
    assert state['model'] == model, state
    assert state['phase'] == 'ready', state['detail']
    assert port.sent.count('CFG-TMODE3' if model == 'M8P' else 'CFG-VALSET') >= 2
    assert frames == [rtcm_frame(), rtcm_frame(1230)]
    assert state['survey']['seconds'] == 200 and state['survey']['accuracy'] == .5
    assert state['base_position'] == pytest.approx(dict(latitude=0, longitude=0, height=.0025))


@pytest.mark.parametrize('model,mode,valid,phase,forwarded', [
    ('M8P', 1, True, 'ready', 2),
    ('M8P', 2, False, 'fixed', 2),
    ('F9P', 2, False, 'fixed', 2),
    ('M8P', 1, False, 'surveying', 0),
    ('M8P', 0, False, 'idle', 0),
])
def test_resume_never_configures_or_resets_the_survey(monkeypatch, model, mode, valid, phase, forwarded):
    port = FakeReceiver(model, mode=mode, valid=valid, active=mode == 1 and not valid)
    state, frames = run_session(monkeypatch, port)
    assert not any(name in port.sent for name in ('CFG-VALSET', 'CFG-MSG', 'CFG-RATE'))
    assert state['phase'] == phase
    assert len(frames) == forwarded


def test_fragments_follow_the_gps_rtcm_data_layout():
    assert fragments(b'a' * 10, 3) == [(3 << 3, 10, b'a' * 10 + bytes(170))]
    flags = [(f, n) for f, n, _ in fragments(b'b' * 400, 33)]
    assert flags == [(1 | 1 << 3, 180), (1 | 2 | 1 << 3, 180), (1 | 4 | 1 << 3, 40)]


def test_exact_multiple_gets_an_empty_terminator_and_oversize_is_refused():
    assert [n for _, n, _ in fragments(b'c' * 360, 0)] == [180, 180, 0]
    assert [n for _, n, _ in fragments(b'c' * 720, 0)] == [180] * 4
    with pytest.raises(ValueError):
        fragments(b'c' * 721, 0)


def radio_link(sent, fail=False, dead=False):
    def send(*args):
        if fail:
            raise OSError('radio unplugged')
        sent.append(args)
    return SimpleNamespace(mav=SimpleNamespace(gps_rtcm_data_send=send), close=lambda: None, portdead=dead)


def test_session_gives_radio_and_second_sink_each_frame_with_rising_seq(monkeypatch):
    sent, other = [], []
    radio = Radio('', link=radio_link(sent))
    state, frames = run_session(monkeypatch, FakeReceiver('M8P', mode=1, valid=True),
                                extra=[lambda f, s: other.append(f), radio])
    assert len(frames) == len(other) == 2
    assert [a[0] >> 3 for a in sent] == [0, 1]


def test_failing_radio_does_not_stop_the_other_sink(monkeypatch):
    other = []
    radio = Radio('', link=radio_link([], fail=True))
    state, frames = run_session(monkeypatch, FakeReceiver('M8P', mode=1, valid=True),
                                extra=[radio, lambda f, s: other.append(f)])
    assert len(other) == 2
    assert state['errors'] == 2


def test_dead_radio_link_counts_as_an_error_even_though_pymavlink_does_not_raise(monkeypatch):
    radio = Radio('', link=radio_link([], dead=True))
    state, frames = run_session(monkeypatch, FakeReceiver('M8P', mode=1, valid=True), extra=[radio])
    assert state['errors'] == 2


def test_radio_connects_as_a_ground_station(monkeypatch):
    from pymavlink import mavutil
    seen = {}
    monkeypatch.setattr(mavutil, 'mavlink_connection', lambda device, **kw: seen.update(kw) or radio_link([]))
    Radio('/dev/radio')
    assert (seen['source_system'], seen['source_component']) == (255, 190)
    assert seen['autoreconnect'] is True


def test_slow_identify_after_a_good_session_keeps_retrying(monkeypatch):
    import serial

    class Silent(FakeReceiver):
        def write(self, data):  # swallows the MON-VER poll
            return len(data)

    class Unplugged(FakeReceiver):
        def read(self, count=-1):  # works until the first survey poll, then the cable goes
            if 'NAV-SVIN' in self.sent:
                raise OSError('unplugged')
            return super().read(count)

    ports = [Unplugged(), Silent(), FakeReceiver()]
    opened = []
    monkeypatch.setattr(serial, 'Serial', lambda *a, **k: opened.append(1) or ports[len(opened) - 1])
    # A fast clock shortens the 4 s MON-VER deadline.
    session = BaseSession([], dict(DEFAULTS, device='/dev/test'), clock=lambda: time.monotonic() * 100)
    session.start(survey=False)
    deadline = time.monotonic() + 15
    while not (session.snapshot().get('model') and len(opened) >= 3) and time.monotonic() < deadline:
        time.sleep(.01)
    time.sleep(.3)
    state = session.snapshot()
    session.close()
    assert state['phase'] != 'error', state


def test_routes_count_per_sink_and_recovery_clears_the_dead_detail():
    link = radio_link([], dead=True)
    session = BaseSession([Radio('', link=link)], dict(DEFAULTS, device='x'), clock=lambda: 7.0)
    session.state['phase'] = 'ready'
    message = SimpleNamespace(identity='1005')
    raw = rtcm_frame()

    session._handle(raw, message)
    state = session.snapshot()
    assert (state['delivered'], state['errors'], state['detail']) == (0, 1, 'radio link is dead')
    assert state['routes'] == [dict(sent=0, failed=1, error='radio link is dead', error_at=7.0)]

    link.portdead = False
    session._handle(raw, message)
    state = session.snapshot()
    assert (state['delivered'], state['detail']) == (1, 'sending corrections')
    assert state['routes'][0]['sent'] == 1 and state['routes'][0]['failed'] == 1
