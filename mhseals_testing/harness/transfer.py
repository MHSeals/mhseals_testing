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
import signal
import subprocess
import time
from pathlib import Path

SSH = 'ssh -o BatchMode=yes -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2'
# 3.2 prints "1,234  45%  3.10MB/s  0:00:12", 3.5 adds a B suffix to the byte count.
PROGRESS = re.compile(r'^\s*([\d,]+)B?\s+(\d+)%\s+([\d.]+)([kKMGT]?)B/s\s+(\d+):(\d\d):(\d\d)')
# rsync filters per pass: reports first, then only the bags (data_layout: bag/NN/ segments).
PASSES = {'reports': ['--exclude=/bag/'],
          'bags': ['--include=/bag/', '--include=/bag/**', '--exclude=*']}
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


def destination(target, session_id):
    """``user@host:path/id/`` for a remote target, a plain path when ``host`` is empty."""
    path = f"{target['path']}/{session_id}/"
    if not target.get('host'):
        return path
    user = f"{target['user']}@" if target.get('user') else ''
    return f"{user}{target['host']}:{path}"


def _write_json(path, record):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(record))
    os.replace(tmp, path)


class Transfer:
    """Send one session directory. ``run`` blocks; ``cancel`` may be called from another thread."""

    def __init__(self, session, target, open_segments=(), bwlimit_kbps=0, clock=time.time):
        self.session = Path(session)
        self.target = target
        self.dest = destination(target, self.session.name)
        self.open_segments = list(open_segments)
        self.bwlimit_kbps = bwlimit_kbps
        self.clock = clock
        self.record = {'dest': self.dest, 'status': 'idle', 'bytes': 0, 'total': 0,
                       'updated': None, 'error': None, 'finished': None}
        self._proc = None
        self._cancelled = False

    def command(self, extra=(), dry=False):
        """The rsync argv for one pass; ``extra`` holds that pass's filters."""
        argv = ['rsync', '-a', '--partial-dir=.rsync-partial', '--mkpath', '--exclude=transfer.json*']
        if dry:
            argv += ['-n', '-i', '--out-format=%i %l']
        else:
            argv += ['--info=progress2', '--no-inc-recursive', '--outbuf=L']
        if self.bwlimit_kbps:
            argv.append(f'--bwlimit={self.bwlimit_kbps}')
        argv += [f'--exclude=/bag/{s}/' for s in self.open_segments]
        if self.target.get('host'):
            argv += ['-e', SSH]
        return argv + list(extra) + [f'{self.session}/', self.dest]

    def pending(self, extra=()):
        """File items still to send for a pass, via dry run; raises on rsync failure."""
        out = subprocess.run(self.command(extra, dry=True), capture_output=True, text=True)
        if out.returncode:
            raise RuntimeError(out.stderr.strip().splitlines()[-1] if out.stderr.strip() else f'rsync {out.returncode}')
        return parse_itemize(out.stdout)

    def total(self, kind=None):
        """Bytes of the files a pass (``reports``, ``bags`` or all when None) covers, summed from disk."""
        opened = tuple(f'bag/{s}/' for s in self.open_segments)
        size = 0
        for path in self.session.rglob('*'):
            rel = path.relative_to(self.session).as_posix()
            if not path.is_file() or rel.startswith(opened) or path.name.startswith('transfer.json'):
                continue
            if kind and rel.startswith('bag/') != (kind == 'bags'):
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
            totals = [self.total(kind) for kind in PASSES]
            self._save(total=sum(totals))
            for extra, total in zip(PASSES.values(), totals):
                if self._cancelled:
                    break
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
