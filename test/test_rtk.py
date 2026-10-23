"""Base receiver wire formats, frame forwarding and resume-without-reconfigure; no ROS or hardware."""

from copy import deepcopy
import io
import time

import pytest
from pyrtcm import calc_crc24q
from pyubx2 import GET, UBXMessage, UBXReader

from mhseals_testing.rtk import (BaseSession, DEFAULTS, base_commands, coordinates, load_config,
                                 save_config, split_position)


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


def run_session(monkeypatch, port, **start):
    """Run one session on the fake receiver until two pushed frames are read; return its state and the frames."""
    import serial
    monkeypatch.setattr(serial, 'Serial', lambda *a, **k: port)
    frames = []
    session = BaseSession([lambda frame, stamp: frames.append(frame)], dict(DEFAULTS, device='/dev/test'))
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
    reader = UBXReader(io.BytesIO(bad + frame), protfilter=7, quitonerror=1, errorhandler=errors.append)
    raw, message = reader.read()
    assert raw == frame and message.identity == '1005'
    assert len(errors) == 1


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
