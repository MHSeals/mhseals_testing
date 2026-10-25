"""preflight probe: read-only facts the pre-water checks need that no other probe publishes.

Pure: ros/preflight.py feeds it plain values from the ROS callbacks and the graph, and the
camera host's ros/host_diagnostic.py fills the ``host:`` diagnostic it reads. Nothing here writes
a pin, a parameter or a command.

Facts:
  driver.outputs    'match', 'differs' (label: the keys), 'pending' or 'error', from the
                    driver's latched ~/status against the local hardware config
  driver.mosfet     the driver's MOSFET level: 'on', 'off' or 'none' (no MOSFET configured)
  driver.estop      'armed', 'latched' or 'disabled' (only when the status has an estop block)
  rc.link           'valid' or 'invalid' (label: why), the command mux's own RC rule
  rc.age_s          seconds since the last /mavros/rc/in frame
  bag.active        the session's recorder child is running
  orin.temp_max     hottest thermal zone on the camera host, deg C
  orin.power_mode   nvpmodel mode name on the camera host
  orin.disk_free_gb free disk on the camera host's data directory
"""

from pathlib import Path
import shutil

from mhseals_testing.health.model import Fact
from mhseals_testing.health.probes import THERMAL_ZONES, hottest

OUTPUT_KEYS = ('pwm_topic', 'pwm_chips', 'pwm_channels', 'mosfet_chip', 'mosfet_line',
               'mosfet_active_high')    # the pins; output_backend is a launch argument
HOST_PREFIX = 'host:'                   # DiagnosticStatus.hardware_id of ros/host_diagnostic.py
NVPMODEL_STATUS = '/var/lib/nvpmodel/status'
NVPMODEL_CONF = '/etc/nvpmodel.conf'


def driver_facts(status, config):
    """Facts from the driver's ~/status dict and the hardware ``config`` it should run."""
    if not isinstance(status, dict):
        return {}
    facts = {'driver.mosfet': Fact(str(status.get('mosfet', 'none')))}
    outputs = status.get('outputs') if isinstance(status.get('outputs'), dict) else {}
    differs = [k for k in OUTPUT_KEYS if config and k in config and outputs.get(k) != config[k]]
    if status.get('error'):
        facts['driver.outputs'] = Fact('error', data={'reason': str(status['error'])})
    elif status.get('pending_restart_keys'):
        keys = ', '.join(map(str, status['pending_restart_keys']))
        facts['driver.outputs'] = Fact('pending', data={'reason': f'restart pending for {keys}'})
    elif differs:
        facts['driver.outputs'] = Fact('differs', data={'reason': 'driver runs other ' + ', '.join(differs)})
    elif config:
        facts['driver.outputs'] = Fact('match', data={'reason': ' '.join(
            f'{k}={outputs.get(k)}' for k in ('pwm_chips', 'pwm_channels', 'mosfet_line') if k in outputs)})
    estop = status.get('estop')
    if isinstance(estop, dict):
        state = ('latched' if estop.get('latched') else 'armed' if estop.get('enabled') else 'disabled')
        reason = f"cut by {estop.get('cause') or '?'}" if state == 'latched' else ''
        facts['driver.estop'] = Fact(state, data={'reason': reason})
    return facts


class RcLink:
    """/mavros/rc/in frames judged by the command mux's rule (Arbitration.valid), so a frame the
    mux would refuse never reads valid here. ``section`` is the hardware config's command_mux."""

    def __init__(self, section):
        from mhseals_hardware.command_arbitration import Arbitration
        self.rule = Arbitration.from_config(section)
        self.time = None

    def frame(self, channels, now):
        self.rule.update_rc(list(channels), now)
        self.time = now

    def facts(self, now):
        if self.time is None:
            return {}                   # no frame yet: unknown, never valid
        age = now - self.time
        if self.rule.valid(now):
            return {'rc.link': Fact('valid'), 'rc.age_s': Fact(age)}
        channels = self.rule.channels
        why = ('no receiver (no channels)' if not channels else
               f'last frame {age:.1f} s old' if age > self.rule.rc_max_age else
               f'sticks or mode channel out of 900-2100 ({len(channels)} channels)')
        return {'rc.link': Fact('invalid', data={'reason': why}), 'rc.age_s': Fact(age)}


def bag_facts(children, alive):
    """``bag.active`` from the session registry's children and ``alive(pid)``."""
    record = (children or {}).get('bag')
    pid = record.get('pid') if isinstance(record, dict) else None
    return {'bag.active': Fact(bool(pid) and alive(pid), label='' if pid else 'no recorder child')}


def host_values(zones=THERMAL_ZONES, status=NVPMODEL_STATUS, conf=NVPMODEL_CONF, data='/'):
    """The camera host's own readings as strings, for its ``host:`` diagnostic. Unreadable
    sources are left out, so the boat reads them as unknown."""
    values = {}
    temp = hottest(zones)
    if temp is not None:
        values['temp_max_c'] = f'{temp:.1f}'
    try:
        mode = Path(status).read_text().split('pmode:', 1)[1].split()[0].lstrip('0') or '0'
        names = {}
        for line in Path(conf).read_text().splitlines():
            if 'POWER_MODEL' in line and 'ID=' in line and 'NAME=' in line:
                fields = dict(part.split('=', 1) for part in line.strip('<> ').split() if '=' in part)
                names[fields['ID']] = fields['NAME']
        values['power_mode'] = names.get(mode, f'mode {mode}')
    except (OSError, IndexError, KeyError):
        pass
    try:
        values['disk_free_gb'] = f'{shutil.disk_usage(data).free / 1e9:.1f}'
    except OSError:
        pass
    return values


def orin_facts(values, host):
    """Facts from a ``host:`` diagnostic's {key: value} strings (``host`` labels them)."""
    facts = {}
    for key, name in (('temp_max_c', 'orin.temp_max'), ('disk_free_gb', 'orin.disk_free_gb')):
        try:
            facts[name] = Fact(float(values[key]), label=host)
        except (KeyError, ValueError):
            pass
    if values.get('power_mode'):
        facts['orin.power_mode'] = Fact(values['power_mode'], data={'reason': f"{host}: {values['power_mode']}"})
    return facts
