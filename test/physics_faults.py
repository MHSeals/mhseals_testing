"""Faults for the crane quality gate, sim and test only: nothing here is installed or imported by the package.

Two layers, mirroring the old 10-18 characterization fault suite on characterization v2:

* ``FaultBoat``: the survey's fake boat with the old CASES physical faults (a weak or angled thruster,
  hull drag), so a real step writes a record that is honestly different.
* ``corrupt``: a seeded corruption of a *written* ``mhseals.physics/1`` record and its samples sidecar
  (the files the gate adapter reads), at severity 1 to 3, into a NEW folder. Records are copied, never
  edited in place, and are labelled ``fault_injection`` so they cannot pass for evidence.

Sim only: ``corrupt`` refuses a record whose hardware block is not ``sim_derived``, and refuses a folder
that is not under a temp or ``sim`` path unless called with ``allow_path``. Nothing reaches the boat's
data root: the destination must not exist.
"""

import hashlib
import json
import math
from pathlib import Path
import random
import shutil

import yaml

from test_physics_procedure import Boat

# The old suite's fault table: thruster strength, angle (deg) and hull drag, per FL FR RR RL / surge sway yaw.
CASES = {
    'reference': {},
    'weak_rr': dict(strengths=[1, 1, 0.6, 1]),
    'angled_fr': dict(angles=[0, 15, 0, 0]),
    'hull_drag': dict(drag=[3, 3, 2]),
    'combined': dict(strengths=[0.65, 1, 1.2, 0.85], angles=[8, -10, 0, 5], drag=[2, 3, 1.5]),
}
YAW = (-0.43, 0.43, 0.43, -0.43)          # the Boat's torque per unit force, outputs FL FR RR RL
BEARING = (45, -45, 45, -45)               # the Boat's force direction per output (sway row +.7 -.7 +.7 -.7)


def allocation(angles=(0, 0, 0, 0), strengths=(1, 1, 1, 1)):
    """3x4 wrench-per-effort matrix: surge, sway, yaw. A fault angle turns the thrust line, not the lever."""
    rows = [[], [], []]
    for out in range(4):
        a = math.radians(BEARING[out] + angles[out])
        rows[0].append(math.cos(a) * strengths[out])
        rows[1].append(math.sin(a) * strengths[out])
        rows[2].append(YAW[out] * strengths[out])
    return rows


class FaultBoat(Boat):
    """``Boat`` whose effort holds go through a faulted thruster set and whose hull drag is scaled.

    A hold's command is allocated with the *nominal* allocation (least norm) to four efforts, then the
    faulted allocation turns those into forces, so a weak or angled thruster shows up as cross-coupling
    and a short axis, as on the water. ``drag`` scales the hull d1 and d2 per axis.
    """

    def __init__(self, case='reference', faults=None, d1=(5.0, 9.0, 1.0), d2=(12.0, 25.0, 1.5), **kwargs):
        faults = dict(CASES[case] if faults is None else faults)
        drag = faults.get('drag', (1, 1, 1))
        super().__init__(d1=tuple(v * s for v, s in zip(d1, drag)), d2=tuple(v * s for v, s in zip(d2, drag)),
                         **kwargs)
        self.faults = faults
        nominal = allocation()
        self.faulted = allocation(faults.get('angles', (0,) * 4), faults.get('strengths', (1,) * 4))
        # least-norm efforts: e = N^T (N N^T)^-1 w, N the nominal 3x4
        gram = [[sum(a * b for a, b in zip(r, s)) for s in nominal] for r in nominal]
        self.inverse = _inverse3(gram)
        self.nominal = nominal

    def wrench(self, command):
        if command is None:
            return super().wrench(command)
        want = [k * e * abs(e) for k, e in zip(self.k, command)]
        weights = [sum(self.inverse[i][j] * want[j] for j in range(3)) for i in range(3)]
        efforts = [sum(self.nominal[i][o] * weights[i] for i in range(3)) for o in range(4)]
        return [sum(self.faulted[i][o] * efforts[o] for o in range(4)) for i in range(3)]


def _inverse3(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    return [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]


# --- record corruption ----------------------------------------------------------------------------------

def _nan(samples, rng, level):
    """Null a share of odometry values (a dropped or garbled message); the adapter reads null as NaN."""
    for row in samples['odometry']:
        for col in (1, 2, 3):
            if rng.random() < (0.02, 0.1, 0.3)[level]:
                row[col] = None


def _dropout(samples, rng, level):
    """Drop a share of rows in whole bursts (a stalled bridge), not singly."""
    rows, keep, i = samples['odometry'], [], 0
    while i < len(rows):
        burst = rng.randint(5, 20)
        if rng.random() < (0.05, 0.2, 0.45)[level]:
            i += burst
            continue
        keep.extend(rows[i:i + burst])
        i += burst
    samples['odometry'] = keep


def _skew(samples, rng, level):
    """Shift odometry against the segment clock by 0.2, 0.8 or 2 s (unsynced logs)."""
    shift = (0.2, 0.8, 2.0)[level] * rng.choice((-1, 1))
    for row in samples['odometry']:
        row[0] += shift


def _spikes(samples, rng, level):
    for row in samples['odometry']:
        if rng.random() < (0.005, 0.02, 0.06)[level]:
            row[rng.choice((1, 2, 3))] += rng.choice((-1, 1)) * (1.0, 3.0, 10.0)[level]


def _scale(samples, rng, level):
    """A consistent sensor scale error on speed (a wrong velocity gain), 5, 15 or 40 percent."""
    gain = 1 + rng.choice((-1, 1)) * (0.05, 0.15, 0.4)[level]
    for row in samples['odometry']:
        row[1] *= gain
        row[2] *= gain


def _stuck(samples, rng, level):
    """A frozen estimator: repeat one row's speeds over a stretch covering 10, 30 or 60 percent."""
    rows = samples['odometry']
    span = int(len(rows) * (0.1, 0.3, 0.6)[level])
    start = rng.randrange(0, max(1, len(rows) - span))
    for k in range(start, start + span):
        rows[k][1:5] = rows[start][1:5]


def _segments(samples, rng, level):
    """Mislabel overrides: hide takeovers (segments say clean) or invent them."""
    for seg in samples['segments']:
        if 'axis' in seg and rng.random() < (0.1, 0.3, 0.6)[level]:
            seg['overridden'] = not seg.get('overridden')


SAMPLE_KINDS = dict(nan=_nan, dropout=_dropout, skew=_skew, spikes=_spikes, scale=_scale, stuck=_stuck,
                    overrides=_segments)


def _record_stale_config(record, rng, level):
    record['hardware']['hash'] = 'sha256:' + hashlib.sha256(f'stale{rng.random()}'.encode()).hexdigest()
    if level >= 1 and record['hardware'].get('history'):
        record['hardware']['history']['sha'] = 'deadbeef' + record['hardware']['history']['sha'][8:]
    if level >= 2:
        record['hardware']['keys']['hardware_epoch'] = (record['hardware']['keys'].get('hardware_epoch') or 0) + 1


def _record_conditions(record, rng, level):
    record['conditions'] = dict(verdict=('marginal', 'rough', 'unknown')[level])
    if level == 2:
        record['battery_v'] = None


def _record_muted(record, rng, level):
    record['gate_muted'] = True


RECORD_KINDS = dict(stale_config=_record_stale_config, conditions=_record_conditions, muted=_record_muted)
# Edits that break the file instead of the data: the sidecar bytes change and the sha is left alone.
BYTE_KINDS = ('sha_mismatch', 'truncated')
KINDS = tuple(SAMPLE_KINDS) + tuple(RECORD_KINDS) + BYTE_KINDS


def _is_sim(record):
    return bool((record.get('hardware') or {}).get('sim_derived'))


def corrupt(source, destination, name, kind, severity=2, seed=0, allow_path=False):
    """Copy ``source``'s ``<name>.yaml`` (+ sidecar) to ``destination`` corrupted; returns the new yaml path.

    ``severity`` is 1..3. The result carries ``fault_injection`` (kind, severity, seed). Sample edits
    re-hash the sidecar, so they pass the integrity check and test the data checks; ``sha_mismatch`` and
    ``truncated`` leave the recorded sha stale on purpose.
    """
    if kind not in KINDS:
        raise ValueError(f'unknown fault {kind!r}: {", ".join(KINDS)}')
    if severity not in (1, 2, 3):
        raise ValueError('severity is 1, 2 or 3')
    source, destination = Path(source), Path(destination)
    record = yaml.safe_load((source / f'{name}.yaml').read_text())
    if not _is_sim(record):
        raise PermissionError('fault injection is sim only: this record is not sim_derived')
    if not allow_path and not any(p in ('sim', 'tmp') or p.startswith(('pytest-', 'sim')) for p in destination.resolve().parts):
        raise PermissionError('fault injection writes only under a sim or temp folder')
    if destination.resolve() == source.resolve() or destination.exists():
        raise FileExistsError('fault injection writes a new folder, never over a record')
    destination.mkdir(parents=True)
    rng, level = random.Random(f'{kind}/{severity}/{seed}'), severity - 1
    side = record.get('samples_file')
    text = (source / side['path']).read_text() if side else None
    if kind in SAMPLE_KINDS:
        samples = json.loads(text)
        SAMPLE_KINDS[kind](samples, rng, level)
        text = json.dumps(samples, separators=(',', ':'), allow_nan=False) + '\n'
        side['sha256'] = 'sha256:' + hashlib.sha256(text.encode()).hexdigest()
    elif kind == 'sha_mismatch':
        samples = json.loads(text)
        row = samples['odometry'][rng.randrange(len(samples['odometry']))]
        row[1] += 1e-3 * 10 ** level
        text = json.dumps(samples, separators=(',', ':'), allow_nan=False) + '\n'
    elif kind == 'truncated':
        text = text[:int(len(text) * (0.9, 0.6, 0.2)[level])]
    else:
        RECORD_KINDS[kind](record, rng, level)
    if side:
        (destination / side['path']).write_text(text)
    record['fault_injection'] = dict(kind=kind, severity=severity, seed=seed)
    target = destination / f'{name}.yaml'
    target.write_text(yaml.safe_dump(record, sort_keys=False))
    for extra in ('config-history',):        # the gate reads the history beside the record's folder
        if (source.parent / extra).is_dir():
            shutil.copytree(source.parent / extra, destination.parent / extra, dirs_exist_ok=True)
    return target
