"""USB RTK base reader for a Here+ (M8P) or Here4 (F9P); no ROS imports.

The base settings, the operator's saved positions and the UBX commands that
configure a receiver for a survey or a known position, in RAM only.
"""

from copy import deepcopy
import math
import os
from pathlib import Path

import yaml

DEFAULTS = dict(device='', radio='', radio_baud=57600, survey_seconds=120,
                survey_accuracy=2.0, system_id=1, positions={})


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
    if not 1 <= int(config['system_id']) <= 255:
        raise ValueError('vehicle system ID must be 1..255')
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
