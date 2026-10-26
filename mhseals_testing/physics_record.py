"""Physics characterization records: one YAML per step run, with the provenance the crane gate reads. No ROS.

Each step (procedures/physics.py) writes ``<data root>/characterization/<fit id>-<step>.yaml``
(schema ``mhseals.physics/1``). Bulky per-sample data goes beside it as ``.samples.json``, named in
the YAML with its sha256. Every record carries, at fit time: the config-history snapshot of the
hardware config (id and digest), the hash and values of the keys the physics mapping reads, what
the thruster driver reported running, the water conditions, the battery, the software sha, sample
counts and the units of every number. Nothing here is written into hardware.yaml: a record is
evidence for the crane import, never a setting the driver runs.
"""

import hashlib
import json
from pathlib import Path
import subprocess

import yaml

from mhseals_hardware.configuration import config_sha, load_config
from mhseals_testing import calibration, data

SCHEMA = 'mhseals.physics/1'
# The hardware keys the physics mapping reads (crane-physics-design 3 and 9): mixer signs and arms,
# which thruster is flipped, the reverse ratio and C+/- the driver applies, the output routing.
KEYS = ('thruster_matrix', 'reversed', 'reverse_thrust_ratio', 'decoupling', 'decoupling_negative',
        'channel_map', 'hardware_epoch')
THRUSTERS = ('FL', 'FR', 'RR', 'RL')     # the hardware mixer's row order


def software():
    """{mhseals_testing: commit sha or None, dirty: bool or None}; None outside a git checkout."""
    root = Path(__file__).resolve().parent

    def git(*args):
        return subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True,
                              timeout=2, check=True).stdout.strip()
    try:
        return dict(mhseals_testing=git('rev-parse', 'HEAD'), dirty=bool(git('status', '--porcelain', '--', '.')))
    except (OSError, subprocess.SubprocessError):
        return dict(mhseals_testing=None, dirty=None)


def driver_check(status, hardware_path, needs_driver):
    """(driver block, refusal or ''): the driver must run the hardware file this record names.

    ``status`` is the driver's latched ~/status. A pending restart key, an error, or an accepted
    file other than the one on disk means the mixer and C the record would name are not the ones
    the thrusters run, so a motion step refuses. Without a status, only ``needs_driver`` refuses.
    """
    if not status:
        return None, ('no thruster driver status: the running mixer and C are unconfirmed' if needs_driver else '')
    block = {key: status.get(key) for key in ('config_sha', 'effective_config_sha', 'pending_restart_keys')}
    if status.get('pending_restart_keys'):
        return block, 'the driver needs a restart for ' + ', '.join(map(str, status['pending_restart_keys']))
    if status.get('error'):
        return block, 'the driver reports: ' + str(status['error'])
    if hardware_path and status.get('config_sha') != config_sha(load_config(hardware_path)):
        return block, 'the driver runs another hardware file than the one on disk: wait for it to load or restart it'
    return block, ''


def hardware_block(hardware_path, history=None, driver=None):
    """What the record was made under: the config-history snapshot, the mapping keys and their hashes."""
    config = load_config(hardware_path) if hardware_path else {}
    head = history.head('hardware') if history else None
    source = str(Path(hardware_path).resolve()) if hardware_path else None
    return dict(
        source=source,
        sim_derived=bool(source and '/config/sim/' in source),
        history=dict(id=head['id'], sha=head['after']) if head else None,
        hash=calibration.dependency_hash(config, calibration.depends_on()['invalid']),
        physics_hash=calibration.dependency_hash(config, KEYS),
        keys={key: config.get(key) for key in KEYS}, thrusters=list(THRUSTERS), driver=driver)


def provenance(ctx, step, fit=None, driver=None):
    """The header every record starts with; missing pieces stay None so the gate can say which."""
    services = ctx.services
    fit = fit or calibration.new_fit()
    context = dict((services.get('fit_context') or dict)())
    conditions = services.get('conditions')
    try:
        water = conditions.summary(ctx.monitor.clock()) if conditions else None
    except Exception:   # an unwired feed is CONDITIONS_UNKNOWN for the gate, never a lost record
        water = None
    return dict(
        step=step, fit=dict(fit, session=context.get('session'), run=context.get('run')),
        software=software(),
        hardware=hardware_block(services.get('user_config'), services.get('history'), driver),
        conditions=water or dict(verdict='unknown'),
        battery_v=context.get('battery_v'), gate_muted=bool(context.get('muted')))


def sha256(text):
    return 'sha256:' + hashlib.sha256(text.encode()).hexdigest()


def write(header, body, samples=None, root=None):
    """Write the record (and its samples sidecar); returns the YAML path.

    ``body`` must name its ``units`` and ``counts``; ``samples`` is any JSON-able per-sample data.
    """
    if 'units' not in body or 'counts' not in body:
        raise ValueError('a physics record needs units and counts')
    name = f"{header['fit']['id']}-{header['step']}"
    folder = Path(root) if root else data.path('characterization')
    record = dict(header, **body)
    if samples is not None:
        text = json.dumps(samples, separators=(',', ':'), allow_nan=False)
        data.atomic_write(folder / f'{name}.samples.json', text + '\n')
        record['samples_file'] = dict(path=f'{name}.samples.json', sha256=sha256(text + '\n'))
    target = folder / f'{name}.yaml'
    data.atomic_write(target, yaml.safe_dump(data.stamp(record, SCHEMA), sort_keys=False))
    return target
