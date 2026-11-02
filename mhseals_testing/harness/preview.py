"""boat_tui preview's settings and sanity checklist (tools/boat_preview). ROS-free.

    python3 -m mhseals_testing.harness.preview env [--test]        # PV_KEY=... lines for the script to eval
    python3 -m mhseals_testing.harness.preview slug REF             # REF as a cache directory name
    python3 -m mhseals_testing.harness.preview checklist WS OUT [REF]  # the checklist of the snapshot workspace WS as JSON

config/preview.yaml over config/sim/stack.yaml; ``test:`` over both with --test; ``BOAT_PREVIEW_<KEY>`` wins.
The checklist: ``checks``, the runbook's "before a test" items, and the commits landed since ``since`` (a date or a
ref) in each repo of the snapshot, grouped by feature (``feature``).
The TUI's checklist panel (ui/panels/checklist.py) reads it and appends each mark with ``result_line``.
"""

import datetime
import json
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path

import yaml

from mhseals_testing.harness import sim_stack

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'preview.yaml'
PATHS = ('src', 'cache', 'runbook', 'results')
SHARED = ('player', 'image', 'base', 'profile', 'command_mode', 'course_family', 'seed', 'ready_s', 'max_s')
DATE = re.compile(r'\d{4}-\d{2}-\d{2}$')
MARKS = ('pass', 'fail', 'note')
BOAT_DOMAINS = (42, 142)   # the fleet's (harness/mission.py), never a preview's


def settings(test=False, path=CONFIG, environ=None):
    """The preview's settings: the sim's shared keys, preview.yaml, its ``test:`` with ``test``, then the env."""
    environ = os.environ if environ is None else environ
    sim = sim_stack.settings(environ={})
    found = yaml.safe_load(Path(path).read_text()) or {}
    overlay = found.pop('test', {}) or {}
    out = {key: sim[key] for key in SHARED if key in sim}
    out.update(found)
    if test:
        out.update(overlay)
    for key, value in list(out.items()):
        value = environ.get(f'BOAT_PREVIEW_{key.upper()}', value)
        out[key] = os.path.expanduser(str(value)) if key in PATHS + sim_stack.PATHS and value else value
    return out


def refusal(values, other):
    """Why the settings ``values`` may not run (None when they may): never the boat's domain, the boat's or the sim's
    container or tmux server, nor anything of ``other`` (the human preview's settings for a test, and back)."""
    sim = sim_stack.settings(environ={})
    if int(values['domain']) in BOAT_DOMAINS:
        return f"domain {values['domain']} is the boat's"
    if values['container'] in ('roboboat_dev', 'roboboat_test', sim.get('container')):
        return f"container {values['container']} is not the preview's"
    if values['socket'] == 'boat':
        return 'tmux -L boat is the boat TUI'
    shared = [key for key in ('cache', 'container', 'socket', 'domain', 'port') if str(values[key]) == str(other[key])]
    return f"{', '.join(shared)} shared with the other preview" if shared else None


def slug(ref):
    """``ref`` as one directory name: ``origin/main`` is ``origin-main``."""
    return re.sub(r'[^\w.-]+', '-', ref).strip('-.') or 'ref'


def feature(subject, other='other'):
    """The feature a commit subject names: its ``name:`` prefix (up to three words), else the first code-like word
    (``fence_watchdog``, ``hw.sh``, ``tools/boat_tui``) of its first eight, else ``other``. Lower case, ``_`` for spaces."""
    head, colon, _ = subject.partition(':')
    if colon and 0 < len(head.split()) <= 3 and len(head) <= 40:
        name = head
    else:
        name = next((w for w in re.findall(r"[\w./-]+", subject)[:8] if re.search(r'[a-z][_./][a-z]', w)), other)
    name = re.sub(r'\s+', '_', name.strip().rstrip('.,;').lower())
    return name.removesuffix('_tab').rsplit('/', 1)[-1]


def commits(repo, since, ref='HEAD'):
    """``[(short sha, subject)]`` landed in ``repo`` since a date or a ref, newest first; [] when git cannot say."""
    span = [f'--since={since} 00:00', ref] if DATE.match(str(since)) else [f'{since}..{ref}']
    try:
        out = subprocess.run(['git', '-C', str(repo), 'log', '--no-merges', '--format=%h%x09%s', *span],
                             capture_output=True, text=True, timeout=20, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [tuple(line.split('\t', 1)) for line in out.splitlines() if '\t' in line]


def grouped(found):
    """``{repo: [(sha, subject)]}`` as checklist items, one per feature, in first-seen order."""
    groups = {}
    for repo, rows in found.items():
        for sha, subject in rows:
            groups.setdefault(feature(subject, repo.removeprefix('mhseals_')), []).append(f'{repo.removeprefix("mhseals_")} {sha} {subject}')
    return [dict(id=f'change:{name}', group='Changes', text=f'{name} ({len(lines)})', detail=' · '.join(lines))
            for name, lines in groups.items()]


def runbook_items(text, heading=r'before a (boat )?test'):
    """The numbered or bulleted items under the runbook's ``heading`` (a ``##`` section), one line each."""
    items, inside = [], False
    for line in text.splitlines():
        if line.startswith('#'):
            inside = bool(re.search(heading, line, re.I))
            continue
        if not inside:
            continue
        item = re.match(r'\s*(\d+\.|[-*])\s+(.*)', line)
        if item:
            items.append(item.group(2).strip())
        elif line.startswith('   ') and items:
            items[-1] += ' ' + line.strip()
    return [dict(id=f'runbook:{n}', group='Before a test', text=text) for n, text in enumerate(items, 1)]


def checklist(values, ws, ref):
    """The checklist of the snapshot workspace ``ws`` (src/<repo> worktrees) for ``ref``."""
    found = {repo: commits(Path(ws) / 'src' / repo, values['since']) for repo in values['repos']}
    try:
        runbook = runbook_items(Path(values['runbook']).read_text())
    except OSError:
        runbook = []
    checks = [dict(id=c['id'], group='Checks', text=c['text']) for c in values.get('checks') or []]
    return dict(ref=ref, since=str(values['since']), items=checks + runbook + grouped(found))


def results_path(directory, day=None):
    """``directory/preview-<date>.md``."""
    return Path(directory) / f'preview-{day or datetime.date.today().isoformat()}.md'


def result_line(item, mark, note='', at=None):
    """One appended result: ``- HH:MM PASS group: text — note``."""
    if mark not in MARKS:
        raise ValueError(f'mark is one of {MARKS}')
    at = at or datetime.datetime.now()
    tail = f' — {note}' if note else ''
    return f"- {at:%H:%M} {mark.upper()} {item['group']}: {item['text']}{tail}\n"


def header_line(ref, at=None):
    at = at or datetime.datetime.now()
    return f'\n## {at:%H:%M} preview {ref}\n'


def shell(values):
    """``PV_KEY='value'`` lines; a list is space-joined; nested values (checks) are left out."""
    return '\n'.join(f"PV_{key.upper()}={shlex.quote(' '.join(map(str, v)) if isinstance(v, list) else str(v))}"
                     for key, v in values.items() if not (isinstance(v, list) and v and isinstance(v[0], dict)))


def main(argv):
    test = '--test' in argv
    argv = [a for a in argv if a != '--test']
    values = settings(test)
    if argv[:1] == ['env']:
        refused = refusal(values, settings(not test, environ={}))
        if refused:
            print(f'boat_preview: refused: {refused}', file=sys.stderr)
            return 2
        print(shell(values))
        return 0
    if argv[:1] == ['slug'] and len(argv) == 2:
        print(slug(argv[1]))
        return 0
    if argv[:1] == ['checklist'] and len(argv) in (3, 4):
        made = checklist(values, argv[1], argv[3] if len(argv) == 4 else values['ref'])
        Path(argv[2]).write_text(json.dumps(made, indent=1))
        print(f"{len(made['items'])} checklist items", file=sys.stderr)
        return 0
    print(__doc__.split('\n\n')[1], file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
