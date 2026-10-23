"""USB RTK base reader for a Here+ (M8P) or Here4 (F9P); no ROS imports.

One worker owns the serial receiver and hands each CRC-checked RTCM3 frame to
its sinks. The receiver is configured only when the operator asks for a survey
or a known position; a reconnect polls the receiver and resumes instead.
"""

from copy import deepcopy
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import yaml

DEFAULTS = dict(device='', radio='', radio_baud=57600, survey_seconds=120,
                survey_accuracy=2.0, positions={})

GCS_SYSTEM, GCS_COMPONENT = 255, 190


def config_path():
    if os.environ.get('MHSEALS_RTK_CONFIG'):
        return Path(os.environ['MHSEALS_RTK_CONFIG']).expanduser()
    return Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'mhseals_testing/rtk.yaml'


def load_config(path=None):
    path = Path(path or config_path())
    config = deepcopy(DEFAULTS)
    if path.exists():
        saved = yaml.safe_load(path.read_text()) or {}
        if not isinstance(saved, dict):
            raise ValueError('RTK settings must be a YAML mapping')
        config.update(saved)
    if not 1 <= int(config['survey_seconds']) <= 86400:
        raise ValueError('survey duration must be 1..86400 seconds')
    if not math.isfinite(float(config['survey_accuracy'])) or not .001 <= float(config['survey_accuracy']) <= 100:
        raise ValueError('survey accuracy must be .001..100 metres')
    if not 1200 <= int(config['radio_baud']) <= 921600:
        raise ValueError('radio baud must be 1200..921600')
    if not isinstance(config['positions'], dict):
        raise ValueError('positions must be a mapping of names to latitude, longitude, height')
    for position in config['positions'].values():
        coordinates(position)
    return config


def save_config(config, path=None):
    """Write beside the target and rename, so a crash never leaves half a file."""
    path = Path(path or config_path())
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(yaml.safe_dump(config, sort_keys=False))
    temporary.replace(path)


def coordinates(position):
    """Validated (latitude, longitude, ellipsoidal height) of a WGS84 position."""
    values = tuple(float(position[key]) for key in ('latitude', 'longitude', 'height'))
    if (not all(math.isfinite(v) for v in values) or not -90 <= values[0] <= 90
            or not -180 <= values[1] <= 180 or not -1000 <= values[2] <= 100000):
        raise ValueError('invalid WGS84 latitude/longitude/ellipsoidal height')
    accuracy = float(position.get('accuracy', .02))
    if not math.isfinite(accuracy) or not .001 <= accuracy <= 100:
        raise ValueError('known-position accuracy must be .001..100 metres')
    return values


def split_position(position):
    """u-blox LLA coarse/HP units: 1e-7 deg/1e-9 deg, cm/0.1 mm."""
    result = []
    for value, scale in zip(coordinates(position), (1e9, 1e9, 1e4)):
        whole = round(value * scale)
        coarse = math.trunc(whole / 100)
        result.extend((coarse, whole - coarse * 100))
    return result


def base_commands(model, config, position=None):
    """Receiver-specific commands that start a survey, or fix a known position."""
    from pyubx2 import UBXMessage, SET
    seconds, accuracy = int(config['survey_seconds']), round(float(config['survey_accuracy']) * 10000)
    if model == 'M8P':
        def mode(value):
            fields = dict(rcvrMode=value, svinMinDur=seconds, svinAccLimit=accuracy)
            if position:
                lat, lat_hp, lon, lon_hp, alt, alt_hp = split_position(position)
                fields.update(lla=1, ecefXOrLat=lat, ecefXOrLatHP=lat_hp,
                              ecefYOrLon=lon, ecefYOrLonHP=lon_hp,
                              ecefZOrAlt=alt, ecefZOrAltHP=alt_hp,
                              fixedPosAcc=round(float(position.get('accuracy', .02)) * 10000))
            return UBXMessage('CFG', 'CFG-TMODE3', SET, **fields)
        commands = [mode(0), UBXMessage('CFG', 'CFG-RATE', SET, measRate=1000, navRate=1, timeRef=1)]
        # NAV-SVIN, NAV-PVT, then RTCM 1005 / 1077 / 1087 (MSM7) / 1230; rates are per epoch.
        for cls, msg, rate in ((1, 0x3B, 1), (1, 7, 1), (0xF5, 5, 5),
                               (0xF5, 0x4D, 1), (0xF5, 0x57, 1), (0xF5, 0xE6, 5)):
            commands.append(UBXMessage('CFG', 'CFG-MSG', SET, msgClass=cls, msgID=msg, rateUSB=rate))
        return commands + [mode(2 if position else 1)]
    if model != 'F9P':
        raise ValueError(f'unsupported base receiver {model}; expected M8P or F9P')
    commands = [UBXMessage.config_set(1, 0, [('CFG_TMODE_MODE', 0)])]
    data = [('CFG_USBOUTPROT_UBX', 1), ('CFG_USBOUTPROT_RTCM3X', 1),
            ('CFG_RATE_MEAS', 1000), ('CFG_RATE_NAV', 1),
            ('CFG_MSGOUT_UBX_NAV_SVIN_USB', 1), ('CFG_MSGOUT_UBX_NAV_PVT_USB', 1)]
    # MSM4 keeps one epoch under the 2400 B DroneCAN buffer.
    for msg, rate in ((1005, 5), (1074, 1), (1084, 1), (1094, 1), (1124, 1), (1230, 5)):
        data.append((f'CFG_MSGOUT_RTCM_3X_TYPE{msg}_USB', rate))
    if position:
        lat, lat_hp, lon, lon_hp, alt, alt_hp = split_position(position)
        data += list(zip(('CFG_TMODE_LAT', 'CFG_TMODE_LAT_HP', 'CFG_TMODE_LON',
                          'CFG_TMODE_LON_HP', 'CFG_TMODE_HEIGHT', 'CFG_TMODE_HEIGHT_HP'),
                         (lat, lat_hp, lon, lon_hp, alt, alt_hp)))
        data += [('CFG_TMODE_POS_TYPE', 1),
                 ('CFG_TMODE_FIXED_POS_ACC', round(float(position.get('accuracy', .02)) * 10000))]
    data += [('CFG_TMODE_SVIN_MIN_DUR', seconds), ('CFG_TMODE_SVIN_ACC_LIMIT', accuracy),
             ('CFG_TMODE_MODE', 2 if position else 1)]
    return commands + [UBXMessage.config_set(1, 0, data)]


def mode_poll(model):
    """Request for the receiver's time mode (survey-in or fixed), answered in RAM."""
    from pyubx2 import UBXMessage, POLL
    if model == 'M8P':
        return UBXMessage('CFG', 'CFG-TMODE3', POLL), 'CFG-TMODE3'
    return UBXMessage.config_poll(0, 0, ['CFG_TMODE_MODE']), 'CFG-VALGET'


# GPS_RTCM_DATA carries 180 B; a longer frame goes out as up to four fragments.
CHUNK, MAX_FRAME = 180, 720


def fragments(frame, seq):
    """Split one RTCM3 frame into (flags, length, padded data) GPS_RTCM_DATA payloads.

    flags = fragmented | fragment_id << 1 | seq << 3, with a 5-bit sequence shared by a
    frame's fragments. A frame that fills its last fragment exactly is closed with an
    empty one, as the FCU only completes a block on a short fragment.
    """
    if len(frame) > MAX_FRAME:
        raise ValueError(f'RTCM frame of {len(frame)} B exceeds {MAX_FRAME} B')
    seq &= 0x1F
    if len(frame) <= CHUNK:
        return [(seq << 3, len(frame), frame.ljust(CHUNK, b'\0'))]
    parts = [frame[i:i + CHUNK] for i in range(0, len(frame), CHUNK)]
    if len(frame) % CHUNK == 0 and len(parts) < 4:
        parts.append(b'')
    return [(1 | n << 1 | seq << 3, len(part), part.ljust(CHUNK, b'\0'))
            for n, part in enumerate(parts)]


class Radio:
    """BaseSession sink for a telemetry radio: sink(frame, stamp) sends the frame as GPS_RTCM_DATA.

    Runs beside the MAVROS sink; the FCU de-duplicates the same corrections arriving on two links.
    Sends as a ground station (system 255, component 190) so the FCU never learns a route for its own ID.
    """

    def __init__(self, device, baud=57600, link=None):
        if link is None:
            from pymavlink import mavutil
            link = mavutil.mavlink_connection(device, baud=baud, source_system=GCS_SYSTEM, source_component=GCS_COMPONENT,
                                              autoreconnect=True)
        self.link, self.seq = link, 0

    def __call__(self, frame, stamp):
        for flags, length, data in fragments(frame, self.seq):
            self.link.mav.gps_rtcm_data_send(flags, length, data)
            # pymavlink swallows write errors on a vanished device and only sets portdead
            if getattr(self.link, 'portdead', False):
                raise OSError('radio link is dead')
        self.seq = (self.seq + 1) & 0x1F

    def close(self):
        self.link.close()


def rtcm_reader(port, **options):
    """UBXReader that gates RTCM3 on its CRC24Q alone, so a body pyrtcm cannot decode is still forwarded."""
    from pyubx2 import UBXReader
    from pyrtcm import calc_crc24q
    from pyrtcm.exceptions import RTCMParseError

    class Reader(UBXReader):
        def _parse_rtcm3(self, header):
            # Metadata-only parsing skips pyrtcm's decode (and its CRC check); UBX stays fully parsed.
            full, self._parsing = self._parsing, 2
            try:
                raw, _ = super()._parse_rtcm3(header)
            finally:
                self._parsing = full
            if calc_crc24q(raw[:-3]) != int.from_bytes(raw[-3:], 'big'):
                raise RTCMParseError('RTCM3 CRC24Q mismatch')
            return raw, SimpleNamespace(identity=str(int.from_bytes(raw[3:5], 'big') >> 4))

    return Reader(port, **options)


class BaseSession:
    """Reads one base receiver and calls each sink as sink(frame, stamp) per RTCM3 frame.

    `stamp` is the monotonic time the frame was read, so a sink can drop old ones.
    """

    def __init__(self, sinks=(), config=None, clock=time.monotonic):
        self.sinks, self.clock = list(sinks), clock
        self.config = load_config() if config is None else deepcopy(config)
        self.lock = threading.RLock()
        self.state = dict(phase='off', detail='Enter to configure', received=0, delivered=0,
                          errors=0, model='', survey=None, base_position=None, base_satellites=None)
        self.stopped = threading.Event()
        self.thread = None
        self.position = None
        self.configure = False
        self.fixed = False

    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def count(self, key):
        with self.lock:
            self.state[key] += 1

    def snapshot(self):
        with self.lock:
            state = deepcopy(self.state)
        if not self.running and state['phase'] not in ('off', 'error'):
            state['phase'] = 'off'
        return state

    def start(self, survey=False, position=None):
        """Open the receiver; configure it only for a survey or a known position."""
        if self.running:
            raise ValueError('stop the current base session first')
        if not self.config['device']:
            raise ValueError('select a USB base receiver first')
        if position:
            coordinates(position)
        self.position = deepcopy(position)
        self.configure = bool(survey or position)
        self.fixed = bool(position)
        self.stopped.clear()
        self.update(phase='connecting', detail='identifying receiver', survey=None, received=0,
                    delivered=0, errors=0, base_position=None)
        self.thread = threading.Thread(target=self._run, name='rtk-base', daemon=True)
        self.thread.start()

    def close(self):
        self.stopped.set()
        if self.thread:
            self.thread.join(timeout=4)
        if self.running:
            raise RuntimeError('RTK worker did not stop')
        self.update(phase='off', detail='stopped')

    def _run(self):
        # Imports are lazy so the UI still works without the USB extras.
        serial_port = None
        identified = False
        try:
            import serial
            while not self.stopped.is_set():
                try:
                    if serial_port is None:
                        self.update(phase='connecting', detail='opening ' + self.config['device'], survey=None)
                        serial_port = serial.Serial(self.config['device'], 115200, timeout=.2,
                                                    write_timeout=1, exclusive=True)
                        reader = rtcm_reader(serial_port, protfilter=7, quitonerror=1,
                                             errorhandler=lambda _: self.count('errors'))
                        model = self._identify(serial_port, reader)
                        identified = True
                        if self.configure:
                            self._configure(serial_port, reader, model)
                            # Only the operator's request configures; a reconnect never does.
                            self.configure = False
                        else:
                            self._resume(serial_port, reader, model)
                        polled = float('-inf')
                    if self.clock() - polled >= 2:
                        serial_port.write(self._poll('NAV-SVIN').serialize())
                        polled = self.clock()
                    self._handle(*reader.read())
                except (serial.SerialException, OSError, RuntimeError, ValueError) as error:
                    # A slow or odd receiver is only fatal before the first success (wrong device).
                    if not identified and not isinstance(error, (serial.SerialException, OSError)):
                        raise
                    if serial_port:
                        serial_port.close()
                    serial_port = None
                    self.update(phase='connecting', detail='receiver disconnected: ' + str(error))
                    self.stopped.wait(1)
        except Exception as error:
            self.update(phase='error', detail=str(error))
        finally:
            if serial_port:
                serial_port.close()

    @staticmethod
    def _poll(identity):
        from pyubx2 import UBXMessage, POLL
        return UBXMessage(identity.split('-')[0], identity, POLL)

    def _handle(self, raw, message):
        """Route one parsed receiver message: survey progress, satellites or an RTCM3 frame."""
        if message is None:
            return
        identity = message.identity
        if identity == 'NAV-SVIN':
            self._survey(message)
        elif identity == 'NAV-PVT':
            self.update(base_satellites=message.numSV)
        elif raw[:1] == b'\xd3':
            # pyubx2 has already checked the CRC24Q of an RTCM3 frame by here.
            self.count('received')
            if self.state['phase'] not in ('ready', 'fixed'):
                return
            stamp = self.clock()
            for sink in self.sinks:
                try:
                    sink(raw, stamp)
                except (OSError, RuntimeError, ValueError) as error:
                    self.update(detail=str(error))
                    self.count('errors')
            self.count('delivered')

    def _survey(self, message):
        from pyubx2 import ecef2llh
        done = bool(message.valid) and not message.active
        self.update(survey=dict(seconds=message.dur, accuracy=message.meanAcc / 10000,
                                observations=message.obs, valid=bool(message.valid),
                                active=bool(message.active)))
        if done:
            lla = ecef2llh(*(getattr(message, 'mean' + axis) / 100
                             + getattr(message, 'mean' + axis + 'HP') / 10000 for axis in 'XYZ'))
            self.update(base_position=dict(zip(('latitude', 'longitude', 'height'), lla)))
        if not self.fixed:
            phase = 'ready' if done else 'surveying' if message.active else 'idle'
            self.update(phase=phase, detail='survey valid' if done else 'waiting for survey accuracy'
                        if message.active else 'no survey running; start one to send corrections')

    def _identify(self, port, reader):
        port.write(self._poll('MON-VER').serialize())
        deadline = self.clock() + 4
        while not self.stopped.is_set() and self.clock() < deadline:
            _, message = reader.read()
            if message is not None and message.identity == 'MON-VER':
                description = str(message)
                for model in ('M8P', 'F9P'):
                    if model in description:
                        self.update(model=model)
                        return model
                raise ValueError('unsupported receiver: ' + description)
        raise RuntimeError('receiver did not answer MON-VER; check selected USB device')

    def _configure(self, port, reader, model):
        self.update(phase='configuring', detail='configuring ' + model)
        commands = base_commands(model, self.config, self.position)
        for index, command in enumerate(commands):
            if index == len(commands) - 1:
                # Let the 1 Hz engine see time mode off before re-enabling, so survey-in restarts.
                self.stopped.wait(1.2)
            self._command(port, reader, command)
        self.update(phase='fixed' if self.position else 'surveying',
                    detail='known position' if self.position else 'waiting for survey observations')

    def _resume(self, port, reader, model):
        """Adopt whatever the receiver is already doing: fixed, surveyed, surveying or idle."""
        request, identity = mode_poll(model)
        port.write(request.serialize())
        port.write(self._poll('NAV-SVIN').serialize())
        deadline, mode, svin = self.clock() + 2, None, None
        while not self.stopped.is_set() and self.clock() < deadline and (mode is None or svin is None):
            _, message = reader.read()
            if message is not None and message.identity == identity:
                mode = getattr(message, 'rcvrMode', getattr(message, 'CFG_TMODE_MODE', None))
            elif message is not None and message.identity == 'NAV-SVIN':
                svin = message
        self.fixed = mode == 2
        if self.fixed:
            self.update(phase='fixed', detail='resumed known position')
        elif svin is not None and (svin.valid or svin.active):
            self._survey(svin)
        elif mode == 1:
            self.update(phase='surveying', detail='waiting for survey observations')
        else:
            self.update(phase='idle', detail='no survey running; start one to send corrections')

    def _command(self, port, reader, command):
        if self.stopped.is_set():
            return
        port.write(command.serialize())
        deadline = self.clock() + 3
        while not self.stopped.is_set() and self.clock() < deadline:
            _, message = reader.read()
            if message is not None and message.identity in ('ACK-ACK', 'ACK-NAK'):
                if (message.clsID, message.msgID) != (command.msg_cls[0], command.msg_id[0]):
                    continue
                if message.identity == 'ACK-NAK':
                    raise ValueError(f'receiver rejected {command.identity}')
                return
        if not self.stopped.is_set():
            raise RuntimeError(f'no receiver acknowledgement for {command.identity}')
