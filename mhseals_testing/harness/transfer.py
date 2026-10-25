"""Resumable rsync of one session to a target host (bags.md section 7); ROS-free.

Two passes: everything except ``bag/`` first so reports arrive before the
large bags, then the closed ``bag/NN/`` segments. ``--partial-dir`` makes a retry resume.
The job is complete only when rsync exits 0 and a dry run finds nothing left
(a size/mtime check), never from the last progress line. ``transfer.json`` in
the session records the last send. Cancel is a flag plus SIGINT, so the
state never depends on rsync's stderr wording.
"""

import json
import os
import re
import shlex
import signal
import subprocess
import time
from glob import escape as glob_escape
from pathlib import Path


def ssh_command():
    """rsync's ``-e``: the same key, known hosts and connection sharing as remote unit calls."""
    from mhseals_testing.harness import remote
    return 'ssh ' + shlex.join([*remote._options(), '-o', 'ServerAliveInterval=5',
                                '-o', 'ServerAliveCountMax=2'])


# 3.2 prints "1,234  45%  3.10MB/s  0:00:12", 3.5 adds a B suffix to the byte count.
PROGRESS = re.compile(r'^\s*([\d,]+)B?\s+(\d+)%\s+([\d.]+)([kKMGT]?)B/s\s+(\d+):(\d\d):(\d\d)')
# rsync filters per pass: reports first, then only the bags (data_layout: bag/NN/ segments).
PASSES = {'reports': ['--exclude=/bag/'],
          'bags': ['--include=/bag/', '--include=/bag/**', '--exclude=*']}
# Legacy flat sessions (no ``bag/``): segments are ``<session>/NN/`` and the report, journal,
# session file and runs sit beside the folder as ``<name>.*`` and the logs in ``<root>/logs/<name>/``
# (SessionPaths.from_output); each gets its own pass. The markers are not rsync flags.
SIBLINGS, LOGS = '--siblings', '--logs'
LEGACY_PASSES = {'siblings': [SIBLINGS],
                 'logs': [LOGS],
                 'reports': ['--exclude=/[0-9][0-9]/'],
                 'bags': ['--include=/[0-9][0-9]/', '--include=/[0-9][0-9]/**', '--exclude=*']}
UNITS = {'': 1, 'k': 1e3, 'K': 1e3, 'M': 1e6, 'G': 1e9, 'T': 1e12}


def parse_progress(text):
    """Return one dict (bytes, pct, rate, eta_s) per progress line in ``text``; rsync ends them with \\r."""
    found = []
    for line in re.split(r'[\r\n]+', text):
        m = PROGRESS.match(line)
        if m:
            h, mi, s = int(m[5]), int(m[6]), int(m[7])
            found.append({'bytes': int(m[1].replace(',', '')), 'pct': int(m[2]),
                          'rate': float(m[3]) * UNITS[m[4]], 'eta_s': h * 3600 + mi * 60 + s})
    return found


def parse_itemize(text):
    """Return [(flags, size)] for file items of ``--out-format='%i %l'``; directories and deletions are skipped."""
    items = []
    for line in text.splitlines():
        flags, _, size = line.strip().partition(' ')
        if len(flags) == 11 and flags[1] == 'f' and size.isdigit():
            items.append((flags, int(size)))
    return items


def fraction(total, pending, reported):
    """Resume-correct progress: bytes already there plus bytes this run reported, over the total."""
    if total <= 0:
        return 1.0
    return min(1.0, max(0.0, (total - pending + reported) / total))


def locate(role):
    """(devices.Host of ``role``, True when it is another computer), from the device registry."""
    from mhseals_testing import devices
    registry = devices.load()
    host = registry.host_for(role)
    return host, host.name != devices.whoami(registry, devices.local_addresses()).name


def destination(target, session_id, where=None):
    """Where one session goes. A target's ``role`` names the computer (the device registry gives its
    user, address and workspace): ``user@address:<workspace>/<path>/<id>/``, or that path here when
    the role is this machine's. Without a role, ``<path>/<id>/`` on this machine (tests).
    ``where`` is ``locate(role)``'s answer, looked up when not given."""
    path = target['path']
    if target.get('role'):
        host, remote = where or locate(target['role'])
        path = f"{host.workspace.rstrip('/')}/{path}"
        if remote:
            user = f'{host.user}@' if host.user else ''
            return f'{user}{host.address}:{path}/{session_id}/'
        path = os.path.expanduser(path)
    return f'{path}/{session_id}/'


def _write_json(path, record):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(record))
    os.replace(tmp, path)


class Transfer:
    """Send one session directory. ``run`` blocks; ``cancel`` may be called from another thread."""

    def __init__(self, session, target, open_segments=(), bwlimit_kbps=0, clock=time.time):
        self.session = Path(session)
        self.target = target
        self.where = locate(target['role']) if target.get('role') else None
        self.dest = destination(target, self.session.name, self.where)
        self.open_segments = list(open_segments)
        self.bwlimit_kbps = bwlimit_kbps
        self.clock = clock
        self.record = {'dest': self.dest, 'status': 'idle', 'bytes': 0, 'total': 0,
                       'updated': None, 'error': None, 'finished': None}
        self._proc = None
        self._cancelled = False

    @property
    def legacy(self):
        """True for a flat session: no ``bag/`` folder, so segments are top-level ``NN/``."""
        return not (self.session / 'bag').exists()

    @property
    def passes(self):
        return LEGACY_PASSES if self.legacy else PASSES

    def siblings(self):
        """Entries beside a legacy session folder named ``<name>.*`` (report, journal, session, runs, logs)."""
        return sorted(self.session.parent.glob(glob_escape(self.session.name) + '.*'))

    @property
    def logs_dir(self):
        """``<root>/logs/<name>/`` of a flat session (outside the session folder), or None when absent."""
        path = self.session.parent / 'logs' / self.session.name
        return path if self.legacy and path.is_dir() else None

    def command(self, extra=(), dry=False):
        """The rsync argv for one pass; ``extra`` is its filter list or a SIBLINGS / LOGS marker."""
        extra = list(extra)
        siblings, logs = SIBLINGS in extra, LOGS in extra
        extra = [e for e in extra if e not in (SIBLINGS, LOGS)]
        argv = ['rsync', '-a', '--partial-dir=.rsync-partial', '--mkpath', '--exclude=transfer.json*']
        if dry:
            argv += ['-n', '-i', '--out-format=%i %l']
        else:
            argv += ['--info=progress2', '--no-inc-recursive', '--outbuf=L']
        if self.bwlimit_kbps:
            argv.append(f'--bwlimit={self.bwlimit_kbps}')
        prefix = '' if self.legacy else 'bag/'
        argv += [f'--exclude=/{prefix}{s}/' for s in self.open_segments]
        if self.where is not None and self.where[1]:
            argv += ['-e', ssh_command()]
        parent = self.dest.rstrip('/').rpartition('/')[0]
        if siblings:
            return argv + [str(p) for p in self.siblings()] + [parent + '/']
        if logs:
            return argv + [f'{self.logs_dir}/', f'{parent}/logs/{self.session.name}/']
        return argv + extra + [f'{self.session}/', self.dest]

    def pending(self, extra=None):
        """File items still to send for a pass (every pass when None), via dry run; raises on rsync failure."""
        if extra is None:
            return [item for e in self.passes.values() for item in self.pending(e)]
        if (SIBLINGS in extra and not self.siblings()) or (LOGS in extra and not self.logs_dir):
            return []
        out = subprocess.run(self.command(extra, dry=True), capture_output=True, text=True)
        if out.returncode:
            raise RuntimeError(out.stderr.strip().splitlines()[-1] if out.stderr.strip() else f'rsync {out.returncode}')
        return parse_itemize(out.stdout)

    def total(self, kind=None):
        """Bytes of the files a pass (``reports``, ``bags`` or all when None) covers, summed from disk."""
        prefix = '' if self.legacy else 'bag/'
        opened = tuple(f'{prefix}{s}/' for s in self.open_segments)
        size = 0
        if kind in (None, 'siblings') and self.legacy:
            size += sum(f.stat().st_size for p in self.siblings() for f in (p.rglob('*') if p.is_dir() else [p])
                        if f.is_file())
        if kind in (None, 'logs') and self.logs_dir:
            size += sum(f.stat().st_size for f in self.logs_dir.rglob('*') if f.is_file())
        if kind in ('siblings', 'logs'):
            return size
        for path in self.session.rglob('*'):
            rel = path.relative_to(self.session).as_posix()
            if not path.is_file() or rel.startswith(opened) or path.name.startswith('transfer.json'):
                continue
            is_bag = re.match(r'[0-9][0-9]/', rel) is not None if self.legacy else rel.startswith('bag/')
            if kind and is_bag != (kind == 'bags'):
                continue
            size += path.stat().st_size
        return size

    def cancel(self):
        self._cancelled = True
        if self._proc and self._proc.poll() is None:
            self._proc.send_signal(signal.SIGINT)

    def _save(self, **fields):
        self.record.update(fields, updated=self.clock())
        _write_json(self.session / 'transfer.json', self.record)

    def run(self, on_progress=None):
        """Run both passes and the final size check; returns the final status."""
        base = 0
        self._save(status='running', error=None, finished=None, bytes=0)
        try:
            totals = [self.total(kind) for kind in self.passes]
            self._save(total=sum(totals))
            for extra, total in zip(self.passes.values(), totals):
                if self._cancelled:
                    break
                if (SIBLINGS in extra and not self.siblings()) or (LOGS in extra and not self.logs_dir):
                    continue  # nothing to send for this pass
                left = sum(size for _, size in self.pending(extra))
                rc, tail = self._rsync(extra, total, left, base, on_progress)
                if self._cancelled:
                    break
                if rc:
                    return self._finish('failed', tail or f'rsync exit {rc}')
                base += total
                self._save(bytes=base)
            if self._cancelled:
                return self._finish('cancelled')
            if self.pending():
                return self._finish('failed', 'remote sizes differ after transfer')
            return self._finish('complete', bytes=self.record['total'])
        except (RuntimeError, OSError) as exc:
            return self._finish('cancelled' if self._cancelled else 'failed', str(exc))

    def _rsync(self, extra, total, left, base, on_progress):
        self._proc = subprocess.Popen(self.command(extra), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if self._cancelled:
            self._proc.send_signal(signal.SIGINT)
        tail = ''
        while chunk := os.read(self._proc.stdout.fileno(), 4096):
            text = chunk.decode('utf-8', 'replace')
            for prog in parse_progress(text):
                done = int(fraction(total, left, prog['bytes']) * total)
                self._save(bytes=base + done)
                if on_progress:
                    on_progress(self.record, prog)
            lines = [l for l in re.split(r'[\r\n]+', text) if l.strip() and not PROGRESS.match(l)]
            tail = lines[-1] if lines else tail
        return self._proc.wait(), tail

    def _finish(self, status, error=None, **fields):
        self._save(status=status, error=error, finished=self.clock(), **fields)
        return status
