"""The one place that decides where records live and how they are written.

Everything the harness produces (sessions, runs, health, config history) is
JSON or JSONL under a single root, so people, ``jq`` and agents find it the
same way. Nothing else builds these paths.

Root: ``$MHSEALS_DATA``, else ``$ROS_WS/bags``, else ``~/astro_dock/bags``; never the
working directory. ``path(*parts)`` joins under the root and rejects absolute parts and ``..``.
Every JSON file carries a first ``schema`` field such as ``mhseals.run/1``.

* ``write_json(path, record, schema)``: whole-file records, atomic temp-file rename.
* ``atomic_write(path, text)``: other text files (YAML, param snapshots).
* ``append_jsonl(path, record, schema=None)``: streams; one O_APPEND write plus fsync, lines under 4 KB.
* ``read_json``, ``read_jsonl``: readers; ``read_jsonl`` skips a torn last line.

``boat_test`` writes the flat session layout under the root: ``<profile>_<time>/`` (the MCAP bag),
``<session>.report.json``, ``.hardware.yaml`` (the config the driver ran with), ``.nav2.yaml``
(limits from a characterization), ``.session.yaml`` (profile, clock domain ``wall`` or ``sim``),
``.course_truth.json`` (simulator truth, copied at start) and ``logs/<session>/``.
"""

import json
import os
from pathlib import Path
import stat
import threading

MAX_LINE = 4096     # an O_APPEND write below PIPE_BUF-sized lines stays whole under concurrent appenders


def _absolute_env(name):
    """The variable as an expanded absolute path, or None; a relative value is ignored, not resolved against the cwd."""
    value = os.environ.get(name)
    if not value:
        return None
    candidate = Path(value).expanduser()
    return candidate if candidate.is_absolute() else None


def workspace():
    """``$ROS_WS``, else ``~/astro_dock``."""
    explicit = _absolute_env('ROS_WS')
    return explicit.resolve() if explicit else Path.home() / 'astro_dock'


def root():
    """``$MHSEALS_DATA``, else ``$ROS_WS/bags``, else ``~/astro_dock/bags``; never the working directory."""
    if (explicit := _absolute_env('MHSEALS_DATA')):
        return explicit.resolve()
    return workspace() / 'bags'


def data_root():
    """Alias of ``root()`` for existing callers."""
    return root()


def path(*parts):
    """A location under the root; refuses absolute parts and ``..`` so callers cannot escape it."""
    base = root()
    target = Path(os.path.normpath(base.joinpath(*parts)))
    if not target.is_relative_to(base):
        raise ValueError(f'{Path(*parts)} is outside the data root')
    return target


def stamp(record, schema):
    """Copy of ``record`` with ``schema`` (e.g. ``mhseals.run/1``) as its first key."""
    return {'schema': schema, **{k: v for k, v in record.items() if k != 'schema'}}


def atomic_write(target, text):
    """Write ``text`` to a temp file beside ``target`` and rename it over, so readers never see half a file.

    A new file gets the mode a plain open() would (the kernel applies the umask, as in append_jsonl);
    an existing one keeps its mode.
    """
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.parent / f'.{target.name}.{os.getpid()}.{threading.get_ident()}.tmp'
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o666)
    try:
        with os.fdopen(fd, 'w') as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        try:
            os.chmod(tmp, stat.S_IMODE(os.stat(target).st_mode))
        except FileNotFoundError:
            pass
        os.replace(tmp, target)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def write_json(target, record, schema):
    """Atomically replace ``target`` with the stamped, indented record; NaN/Infinity are refused (ValueError), as they are not JSON."""
    atomic_write(target, json.dumps(stamp(record, schema), indent=2, allow_nan=False) + '\n')


def read_json(target, default=None):
    """The parsed file, or ``default`` when it is missing or unreadable."""
    try:
        return json.loads(Path(target).read_text())
    except (OSError, ValueError):
        return default


def append_jsonl(target, record, schema=None):
    """Append one line with a single O_APPEND write and fsync; a torn last line from a crash is fenced off by a newline."""
    line = json.dumps(stamp(record, schema) if schema else record, separators=(',', ':'), allow_nan=False) + '\n'
    if len(line.encode()) > MAX_LINE:
        raise ValueError(f'record is over {MAX_LINE} bytes; store it as a JSON file instead')
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(target, os.O_RDWR | os.O_APPEND | os.O_CREAT, 0o666)
    try:
        size = os.fstat(fd).st_size
        if size and os.pread(fd, 1, size - 1) != b'\n':
            line = '\n' + line     # a crash left a torn last line; keep it from swallowing this record
        os.write(fd, line.encode())
        os.fsync(fd)
    finally:
        os.close(fd)


def read_jsonl(target):
    """Every complete record; torn or non-object lines (a crash mid-append) are skipped."""
    try:
        lines = Path(target).read_text(errors='replace').splitlines()    # a torn multi-byte tail must not raise
    except OSError:
        return []
    records = []
    for line in lines:
        try:
            record = json.loads(line)
        except ValueError:
            continue
        if isinstance(record, dict):
            records.append(record)
    return records
