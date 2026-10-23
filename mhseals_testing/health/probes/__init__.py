"""Probes: the only health code that reads the outside world; each publishes named facts.

The functions here read the world once and publish named facts for the threshold kind;
submodules (``graph``) hold the larger probes.

Each function returns ``{name: Fact}`` and takes its inputs as arguments, so
tests need no ROS and no hardware. ``Facts`` is the thread-safe store the ROS
callbacks and the App tick publish into; the engine reads a copy of it.

Fact catalogue (this module):
  snapshot  clock.stamp_offset (worst live sensor stamp offset, label = sensor)
  host      host.load (load1 per CPU), host.temp_max, host.disk_free_gb
  fcu       fcu.connected, fcu.armed, fcu.mode
  battery   battery.voltage (pack, label = cell count), battery.cell_v, battery.plausible,
            battery.percent
"""

import glob
import math
import os
from pathlib import Path
import shutil
import threading
import time

from mhseals_testing.health.model import Fact

PACK_CELLS = 4                      # assumed when the profile gives no `battery: {cells: N}`
CELL_PLAUSIBLE = (2.5, 5.0)         # volts per cell outside which the reading is wrong, not low
BRICK_ABSENT = 1.0                  # volts; below this the power brick is not feeding the FCU
CURRENT_MIN = 0.05                  # amps; below this the coulomb-counted percentage is stale
VOLTAGE_ABSENT = 65.5               # volts; MAVLink UINT16_MAX reads 65.535
THERMAL_ZONES = '/sys/class/thermal/thermal_zone*/temp'


class Facts:
    """Facts and the time each probe last published, shared between threads."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._lock = threading.Lock()
        self._facts, self._probes, self._owned = {}, {}, {}

    def publish(self, probe, facts, error=None):
        """Replace the probe's facts. A failed probe passes ``error`` and keeps its old facts."""
        now = self.clock()
        with self._lock:
            if not error:
                for name in self._owned.get(probe, ()):
                    self._facts.pop(name, None)
                self._facts.update({k: Fact(v.value, v.label, v.data, now) for k, v in facts.items()})
                self._owned[probe] = set(facts)
            self._probes[probe] = (now, error)

    def read(self):
        """(facts, probes) copies for ``Engine.tick``."""
        with self._lock:
            return dict(self._facts), dict(self._probes)


def snapshot_facts(snapshot):
    """The largest stamp offset among live sensors: big values mean an unsynced computer."""
    ages = {name: row['stamp_age'] for name, row in snapshot['sensors'].items()
            if row['status'] != 'STALE' and row['stamp_age'] is not None}
    if not ages:
        return {}
    name = max(ages, key=lambda key: abs(ages[key]))
    return {'clock.stamp_offset': Fact(ages[name], label=name)}


def host_facts(bag_directory, load=os.getloadavg, cpus=os.cpu_count, zones=THERMAL_ZONES):
    """Load per CPU, hottest thermal zone and free disk (GB) where the bag is written."""
    count, load1 = cpus() or 1, load()[0]
    facts = {'host.load': Fact(load1 / count, label=f'{load1:.1f}/{count}')}
    temperatures = []
    for zone in glob.glob(zones):
        try:
            temperatures.append(int(Path(zone).read_text()) / 1000)
        except (OSError, ValueError):
            pass
    if temperatures:
        facts['host.temp_max'] = Fact(max(temperatures))
    directory = Path(bag_directory)
    while not directory.exists():
        directory = directory.parent
    facts['host.disk_free_gb'] = Fact(shutil.disk_usage(directory).free / 1e9)
    return facts


def fcu_facts(connected, armed, mode):
    """MAVROS state: an armed FCU or a mode other than MANUAL could also drive the thrusters."""
    if not connected:
        return {'fcu.connected': Fact(False, label='not connected')}
    return {'fcu.connected': Fact(True),
            'fcu.armed': Fact(bool(armed)),
            'fcu.mode': Fact(mode, label=f"{'ARMED' if armed else 'disarmed'}, {mode}")}


def battery_facts(voltage, percentage, current=math.nan, cells=None):
    """Pack and cell voltage, whether the reading is believable, and the charge estimate.

    ``percentage`` is 0..1; the FCU's "unknown" arrives as NaN or a small negative.
    MAVLink's UINT16_MAX "no voltage" (65.535 V) counts as absent. The FCU
    percentage is a coulomb count, so with no measured current it never drops
    and is left out. Volts per cell outside CELL_PLAUSIBLE is a mis-scaled
    monitor or a wrong cell count, not a low pack, so no cell voltage is
    published then; under BRICK_ABSENT the power brick is not connected.
    """
    cells = cells or PACK_CELLS
    facts = {}
    if math.isfinite(voltage) and voltage < VOLTAGE_ABSENT:
        per_cell = voltage / cells
        if voltage < BRICK_ABSENT:
            facts['battery.plausible'] = Fact(False, label='power brick not connected?')
        elif not CELL_PLAUSIBLE[0] <= per_cell <= CELL_PLAUSIBLE[1]:
            facts['battery.plausible'] = Fact(
                False, label=f'implausible for {cells}S ({per_cell:.1f} V/cell)')
        else:
            facts['battery.plausible'] = Fact(True)
            facts['battery.voltage'] = Fact(voltage, label=f'{cells}S')
            facts['battery.cell_v'] = Fact(per_cell)
    measured = not (math.isfinite(current) and abs(current) < CURRENT_MIN)
    if measured and math.isfinite(percentage) and percentage >= 0:
        facts['battery.percent'] = Fact(percentage * 100)
    return facts
