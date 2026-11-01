"""boat_agent fcu: agent FCU parameter reads, allowlisted sets, revert, restart and a rates check.

Owner decision 2026-10-31: agents may change FCU rate parameters and restart the FCU to reach the test goals.
Everything goes through the TUI's MAVROS link (ros/fcu.py MavrosLink, the one MAVROS path) under the agent's
claim; this module is the ROS-free part: which parameters an agent may write, the snapshot taken before a
session's first write, the agent change ledger (what ``revert`` restores) and the markdown change log.

- Set: a name must match agent.yaml ``fcu.set`` and never ``NEVER`` (yaw source, compass, servos, RC, arming,
  failsafes, fences, safety switch), whatever agent.yaml lists. ``fcu.values`` pins a parameter to listed values;
  SCHED_LOOP_RATE 400 also needs the last ``fcu rates`` check at ``fcu.loop_400.after`` with the FCU load below
  ``max_load_pct``.
- Before the first set of a TUI session every parameter is snapshotted to ``fcu.store``
  (``fcu_<vessel>_<stamp>.param``, ``NAME,value``); each set is read back, diffed and appended to
  ``fcu_changes.md`` (old -> new, who, claim) and to ``fcu_agent_changes.jsonl``.
- Revert ``--to SNAPSHOT`` writes back, from that snapshot, only the parameters agents changed since it.
"""

import datetime
import fnmatch
import json
import math
import re
from pathlib import Path

from mhseals_testing.history.seed import read_param_file
from mhseals_testing.ros.fcu import same_float32

# Never an agent's to write, listed or not: the yaw source and compass (10-19: EK3_SRC1_YAW 8 + COMPASS_USE 0 froze
# yaw), servo outputs (SERVO1-4 function 0 broke RC passthrough), RC and its mapping, arming, every failsafe, fences,
# the safety switch, the EKF/AHRS selection and the GCS id the failsafe watches.
NEVER = re.compile(r'^(EK\d_SRC|EK\d_ENABLE|AHRS_EKF_TYPE|COMPASS_|SERVO|RC\d|RCMAP_|ARMING_|FS_|FENCE_|BRD_SAFETY'
                   r'|SYSID_MYGCS|MOT_SAFE)|(^|_)(FS|FAILSAFE|SAFE\w*)(_|\d|$)|FAILSAFE')
LOOP = 'SCHED_LOOP_RATE'
RESTART = 246                    # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1 1: reboot the autopilot
CHANGES = 'fcu_changes.md'
LEDGER = 'fcu_agent_changes.jsonl'
DEFAULTS = {'store': '~/.claude/plans/boat', 'set': [], 'values': {}, 'loop_400': {'after': 200, 'max_load_pct': 60},
            'rates': {'window_s': 10, 'topics': {}}}


def settings(cfg):
    """agent.yaml ``fcu`` over the defaults (absent: agents set nothing)."""
    fcu = dict(DEFAULTS, **(cfg.get('fcu') or {}))
    fcu['rates'] = dict(DEFAULTS['rates'], **fcu['rates'])
    fcu['loop_400'] = dict(DEFAULTS['loop_400'], **fcu['loop_400'])
    return fcu


def validate(fcu):
    """agent.yaml ``fcu`` checked; ValueError names the key."""
    if not isinstance(fcu, dict) or set(fcu) - set(DEFAULTS):
        raise ValueError(f"agent.yaml: fcu takes {', '.join(DEFAULTS)}")
    if not all(isinstance(p, str) and p for p in fcu.get('set', [])):
        raise ValueError('agent.yaml: fcu.set must be a list of parameter names or globs')
    for name, values in fcu.get('values', {}).items():
        if not isinstance(values, list) or not all(_number(v) for v in values):
            raise ValueError(f'agent.yaml: fcu.values.{name} must be a list of numbers')
    if not all(_number(v) for v in fcu.get('loop_400', {}).values()):
        raise ValueError('agent.yaml: fcu.loop_400 needs numbers')
    return fcu


def _number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def never(name):
    return bool(NEVER.search(name))


def refusal(name, value, fcu, last_rates=None):
    """``(error, hint)`` when an agent may not set ``name`` to ``value``, else None."""
    if never(name):
        return 'not_allowed', (f"{name} is never an agent's (yaw source, compass, servos, RC, arming, failsafe or "
                               'safety): the owner sets it')
    if not any(fnmatch.fnmatchcase(name, pattern) for pattern in fcu['set']):
        return 'not_allowed', f"agent.yaml fcu.set lists {', '.join(fcu['set']) or 'nothing'}"
    if not _number(value):
        return 'bad_args', f'{name}: {value!r} is not a finite number'
    allowed = fcu['values'].get(name)
    if allowed is not None and not any(same_float32(value, v) for v in allowed):
        return 'not_allowed', f"{name} may be {', '.join(f'{v:g}' for v in allowed)} (agent.yaml fcu.values)"
    if name == LOOP and value > fcu['loop_400']['after']:
        gate, rates = fcu['loop_400'], last_rates or {}
        load, loop = rates.get('load_pct'), rates.get('loop_rate')
        if loop is None or not same_float32(loop, gate['after']) or load is None or load >= gate['max_load_pct']:
            seen = f'last check: loop {loop}, load {load}%' if rates else 'no fcu rates check yet'
            return 'refused', (f"{LOOP} {value:g} needs a fcu rates check at {gate['after']:g} with FCU load below "
                               f"{gate['max_load_pct']:g}% ({seen}): set {gate['after']:g}, restart, run fcu rates")
    return None


def typed(value, current):
    """``value`` as the FCU stores the parameter: an int when its current value is one."""
    return int(value) if isinstance(current, int) and float(value).is_integer() else float(value)


def changed(before, after):
    """``{name: [old, new]}`` for every parameter that differs (float32 equality), appeared or vanished."""
    names = sorted(set(before) | set(after))
    return {n: [before.get(n), after.get(n)] for n in names
            if n not in before or n not in after or not same_float32(before[n], after[n])}


def stats(times):
    """``{n, hz, gap_p99_ms, gap_max_ms}`` of receive times in seconds (None fields with under two)."""
    times = sorted(times)
    gaps = sorted(b - a for a, b in zip(times, times[1:]))
    if not gaps or times[-1] <= times[0]:
        return {'n': len(times), 'hz': None, 'gap_p99_ms': None, 'gap_max_ms': None}
    p99 = gaps[min(len(gaps) - 1, math.ceil(0.99 * len(gaps)) - 1)]
    return {'n': len(times), 'hz': round(len(gaps) / (times[-1] - times[0]), 1),
            'gap_p99_ms': round(p99 * 1000, 1), 'gap_max_ms': round(gaps[-1] * 1000, 1)}


class Store:
    """Snapshots, the agent change ledger and the change log in ``directory`` (on the TUI's computer)."""

    def __init__(self, directory, vessel='fcu', now=datetime.datetime.now):
        self.dir, self.vessel, self.now = Path(directory).expanduser(), vessel, now

    def snapshot(self, values, who, claim):
        self.dir.mkdir(parents=True, exist_ok=True)
        stamp = self.now().strftime('%Y%m%d-%H%M%S')
        path = self.dir / f'fcu_{self.vessel}_{stamp}.param'
        path.write_text(f'# {len(values)} parameters before agent {who} {claim} changed any ({stamp})\n'
                        + ''.join(f'{n},{values[n]}\n' for n in sorted(values)))
        self.log(f'snapshot {path.name}', [f'- {len(values)} parameters, taken before the first agent write of '
                                           f'this TUI session (agent {who}, claim {claim}).'])
        return path

    def resolve(self, name):
        """The snapshot file ``name`` names (a path or a file in the store); FileNotFoundError when there is none."""
        path = Path(name).expanduser()
        path = path if path.is_absolute() or path.exists() else self.dir / name
        if not path.is_file():
            raise FileNotFoundError(f'no snapshot {name} (in {self.dir})')
        return path

    def read_snapshot(self, path):
        return read_param_file(path, ignore=())

    def record(self, entry):
        self.dir.mkdir(parents=True, exist_ok=True)
        with open(self.dir / LEDGER, 'a') as out:
            out.write(json.dumps(dict(entry, t=self.now().isoformat(timespec='seconds'))) + '\n')

    def entries(self):
        try:
            lines = (self.dir / LEDGER).read_text().splitlines()
        except FileNotFoundError:
            return []
        return [json.loads(line) for line in lines if line.strip()]

    def since(self, snapshot):
        """The parameters agents set after ``snapshot`` was taken (sets recorded against it, and every later set)."""
        names, on = [], False
        for entry in self.entries():
            on = on or entry.get('snapshot') == snapshot
            if on and entry.get('kind') == 'set':
                names += [n for n in entry['changes'] if n not in names]
        return names

    def last_rates(self):
        return next((e for e in reversed(self.entries()) if e.get('kind') == 'rates'), None)

    def log(self, title, lines):
        self.dir.mkdir(parents=True, exist_ok=True)
        stamp = self.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(self.dir / CHANGES, 'a') as out:
            out.write(f'\n## {stamp} {title}\n' + ''.join(line + '\n' for line in lines))


class AgentFcu:
    """The FCU work behind the desk's fcu ops, off the key loop; ``params()`` is a ros.fcu.FcuParams over the
    TUI's MavrosLink, ``measure(window_s)`` the rates sampler. Methods return ``(ok, error, hint, result)``."""

    def __init__(self, params, store, config, measure=None):
        self.params, self.store, self.config, self.measure = params, store, config, measure
        self.snapshot = None             # this TUI session's snapshot, taken before its first agent write

    def fcu(self):
        return settings(self.config())

    def get(self, names):
        values = self.params().read(names)
        missing = [n for n in names if n not in values]
        return True, '', f"unknown: {', '.join(missing)}" if missing else '', {'values': values}

    def set(self, changes, who, claim, revert_of=None):
        """Write ``changes`` ({name: number}) after the refusals, a snapshot (first write) and a read; read back,
        diff, log. ``revert_of`` (a snapshot name) skips the allowlist and value checks, never ``NEVER``."""
        fcu, params = self.fcu(), self.params()
        last = self.store.last_rates()
        for name, value in changes.items():
            found = (('not_allowed', f'{name} is never an agent\'s') if never(name) else None) if revert_of \
                else refusal(name, value, fcu, last)
            if found:
                return (False, *found, None)
        if self.snapshot is None:
            everything = params.read_all()
            if not everything:
                return False, 'refused', 'could not read every FCU parameter for the snapshot (still streaming?)', None
            self.snapshot = self.store.snapshot(everything, who, claim)
            before_all = everything
        else:
            before_all = params.read(list(changes))
        missing = [n for n in changes if n not in before_all]
        if missing:
            return False, 'bad_args', f"not FCU parameters: {', '.join(missing)}", None
        problems = params.check({n: float(v) for n, v in changes.items()})
        if problems:
            return False, 'not_allowed', '; '.join(problems), None
        before = {n: before_all[n] for n in changes}
        written, failed = {}, []
        for name, value in changes.items():
            value = typed(value, before[name])
            if params.link.set(name, value):
                written[name] = value
            else:
                failed.append(name)
        after = params.read(list(changes))
        diff = changed(before, after)
        wrong = [n for n in written if n not in after or not same_float32(after[n], written[n])]
        self.store.record({'kind': 'set', 'who': who, 'claim': claim, 'snapshot': self.snapshot.name,
                           'revert_of': revert_of, 'changes': diff, 'failed': failed + wrong})
        title = f"{'revert to ' + revert_of if revert_of else 'set'} by agent {who} (claim {claim})"
        self.store.log(title, [f'- {n}: {old} -> {new}' for n, (old, new) in diff.items()]
                       + [f'- {n}: write not acknowledged or read back differs ({after.get(n)})'
                          for n in failed + wrong] + [f'- snapshot: {self.snapshot.name}'])
        reboot = [n for n in written if n == LOOP or n.startswith('INS_')]
        result = {'changes': diff, 'read_back': after, 'snapshot': self.snapshot.name,
                  'restart_needed': reboot}
        if failed or wrong:
            return False, 'refused', f"not applied: {', '.join(failed + wrong)} (see read_back)", result
        return True, '', (f"restart the FCU for {', '.join(reboot)} (fcu restart --claim ID)" if reboot else ''), result

    def revert(self, snapshot, who, claim):
        try:
            path = self.store.resolve(snapshot)
        except FileNotFoundError as error:
            return False, 'bad_args', str(error), None
        names = self.store.since(path.name)
        if not names:
            return False, 'refused', f'no agent change recorded since {path.name}', None
        saved = self.store.read_snapshot(path)
        changes = {n: saved[n] for n in names if n in saved}
        current = self.params().read(list(changes))
        changes = {n: v for n, v in changes.items() if n not in current or not same_float32(current[n], v)}
        if not changes:
            return True, '', f'already at {path.name}', {'changes': {}}
        return self.set(changes, who, claim, revert_of=path.name)

    def restart(self, who, claim):
        link = self.params().link
        if link.armed() is not False:
            return False, 'refused', 'the FCU is armed or its state is unknown: restart only disarmed', None
        if not link.command(RESTART, [1, 0, 0, 0, 0, 0, 0]):
            return False, 'refused', 'the FCU did not accept the restart command', None
        self.store.record({'kind': 'restart', 'who': who, 'claim': claim})
        self.store.log(f'restart by agent {who} (claim {claim})', ['- MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN accepted.'])
        return True, '', 'MAVROS reconnects in about 20 s; fcu get SCHED_LOOP_RATE confirms', {'restarted': True}

    def rates(self, window_s, who):
        fcu = self.fcu()
        window_s = min(max(float(window_s or fcu['rates']['window_s']), 2.0), 60.0)
        result = self.measure(window_s)
        loop = self.params().read([LOOP]).get(LOOP)
        result = dict(result, loop_rate=loop, window_s=window_s)
        self.store.record(dict(result, kind='rates', who=who))
        return True, '', '', result
