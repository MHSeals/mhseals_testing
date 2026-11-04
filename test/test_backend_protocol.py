"""The backend protocol: shipped settings, frames and hello versions."""

import pytest

from mhseals_testing.backend import protocol

pytestmark = pytest.mark.smoke   # tools/smoke


def test_shipped_settings_load_and_bad_ones_are_refused(tmp_path):
    settings = protocol.load_settings()
    assert settings.attach_timeout <= 1.0 and settings.link_timeout > settings.heartbeat
    bad = tmp_path / 'backend.yaml'
    bad.write_text(protocol.CONFIG.read_text().replace('heartbeat: 0.5', 'heartbeat: 5.0'))
    with pytest.raises(ValueError, match='link_timeout must exceed heartbeat'):
        protocol.load_settings(bad)
    bad.write_text(protocol.CONFIG.read_text().replace('max_frame:', 'maxframe:'))
    with pytest.raises(ValueError, match='missing'):
        protocol.load_settings(bad)


def test_frames_round_trip_and_bad_ones_are_errors():
    frame = {'kind': 'delta', 'seq': 3, 'data': {'mode': 'hold'}}
    assert protocol.decode(protocol.encode(frame, 1024)) == frame
    with pytest.raises(protocol.FrameError):
        protocol.encode({'kind': 'nope'}, 1024)
    with pytest.raises(protocol.FrameError, match='exceeds'):
        protocol.encode({'kind': 'delta', 'data': 'x' * 2048}, 1024)
    for line in (b'[1]', b'{"kind": "spoof"}', b'not json'):
        with pytest.raises(protocol.FrameError):
            protocol.decode(line)


def test_hello_versions_must_share_a_major():
    assert protocol.check_hello(protocol.hello('station')) is None
    assert protocol.check_hello(protocol.hello('station', (protocol.VERSION[0], 99))) is None
    assert 'incompatible' in protocol.check_hello(protocol.hello('station', (protocol.VERSION[0] + 1, 0)))
    assert 'who' in protocol.check_hello({'kind': 'hello', 'version': list(protocol.VERSION)})
    assert 'expected hello' in protocol.check_hello({'kind': 'ping'})
