"""Base receiver wire formats, frame forwarding and resume-without-reconfigure; no ROS or hardware."""

from copy import deepcopy
import io

import pytest
from pyrtcm import calc_crc24q
from pyubx2 import UBXReader

from mhseals_testing.rtk import DEFAULTS, base_commands, coordinates, load_config, save_config, split_position


def rtcm_frame(number=1005):
    payload = ((number << 140) | (1 << 128)).to_bytes(19, 'big')
    frame = b'\xd3\0' + bytes([len(payload)]) + payload
    return frame + calc_crc24q(frame).to_bytes(3, 'big')


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
