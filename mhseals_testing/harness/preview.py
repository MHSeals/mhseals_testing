"""boat_tui preview's settings and sanity checklist (tools/boat_preview). ROS-free.

    python3 -m mhseals_testing.harness.preview env [--test] [--resolve [--pull]]  # PV_KEY=... lines for the script
    python3 -m mhseals_testing.harness.preview slug REF             # REF as a cache directory name
    python3 -m mhseals_testing.harness.preview plan [REF]           # "repo sha" per repo of the snapshot
    python3 -m mhseals_testing.harness.preview checklist WS OUT [REF]  # the checklist of the snapshot workspace WS as JSON

config/preview.yaml over config/sim/stack.yaml; ``test:`` over both with --test; ``BOAT_PREVIEW_<KEY>`` wins.
What the workspace knows is never written here (docs/preview.md): ``src`` is the workspace's src/ (sim_stack.src_dir),
``--resolve`` adds its dev image and its manifest's repos (sim_stack.resolve) and this machine's vessel registry and
network env (sim_stack.vessel), and the build is ``hw.sh build``.
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

from mhseals_testing.harness import sim_stack
from mhseals_testing.hook import registry
from mhseals_testing.strict_yaml import load_strict

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'preview.yaml'
PATHS = ('src', 'cache', 'runbook', 'results')
MANIFEST = 'manifest'      # the slug of no REF: each repo at the workspace manifest's ref
SHARED = ('player', 'image', 'roles', 'profile', 'command_mode', 'course_family', 'seed', 'ready_s', 'max_s')
DATE = re.compile(r'\d{4}-\d{2}-\d{2}$')
MARKS = ('pass', 'fail', 'note')


def settings(test=False, path=CONFIG, environ=None):
    """The preview's settings: the sim's shared keys, preview.yaml, its ``test:`` with ``test``, then the env."""
    environ = os.environ if environ is None else environ
    sim = sim_stack.settings(environ={})
    found = load_strict(Path(path).read_text(), path) or {}
    overlay = found.pop('test', {}) or {}
    out = {key: sim[key] for key in SHARED if key in sim}
    out.update(found)
    if test:
        out.update(overlay)
    src = environ.get('BOAT_PREVIEW_SRC') or str(sim_stack.src_dir(environ))
    out['src'] = src
    for key, value in list(out.items()):
        value = environ.get(f'BOAT_PREVIEW_{key.upper()}', value)
        if key == 'roles' and isinstance(value, str):
            value = value.replace(',', ' ').split()
        if isinstance(value, str) and key in PATHS:
            value = value.replace('{src}', src)
        out[key] = os.path.expanduser(str(value)) if key in PATHS + sim_stack.PATHS and value else value
    out['port'] = sim_stack.endpoint_port(out['domain'])
    out['heavy'] = sim_stack.heavy('BOAT_PREVIEW_HEAVY', environ)
    return out


def plan(ref, repos, src, run=subprocess.run):
    """``[(repo, sha)]`` of the snapshot: each repo at ``ref`` where it has it (an explicit REF), else at its
    manifest ref (``origin/<branch>`` or astro_dock's pin, ``sim_stack.manifest``). A pinned repo takes REF only
    when REF descends from its pin (an upstream's own main, e.g. ROS-TCP-Endpoint's ROS 1 one, never does). Missing
    names the fix."""
    out = []
    for repo, pinned in repos:
        git = ['git', '-C', str(Path(src) / repo)]
        sha = None
        for want in ([ref] if ref else []) + [pinned]:
            done = run([*git, 'rev-parse', '-q', '--verify', f'{want}^{{commit}}'], capture_output=True, text=True,
                       timeout=20)
            found = done.stdout.strip() if done.returncode == 0 else ''
            if found and want != pinned and re.fullmatch(r'[0-9a-f]{40}', pinned) and run(
                    [*git, 'merge-base', '--is-ancestor', pinned, found], capture_output=True, timeout=20).returncode:
                found = ''
            if found:
                sha = found
                break
        if sha is None:
            raise sim_stack.Missing(f'{repo} has no {pinned} here; git -C {Path(src) / repo} fetch origin')
        out.append((repo, sha))
    return out


def refusal(values, other):
    """Why the settings ``values`` may not run (None when they may): never a vessel's domain (the profile set's), the
    dev or the sim's container or the boat TUI's tmux server, nor anything of ``other`` (the human preview's settings
    for a test, and back). The port follows the domain."""
    sim = sim_stack.settings(environ={})
    if int(values['domain']) in registry.boat_domains():
        return f"domain {values['domain']} is the boat's"
    if values['container'] in (registry.dev_container(), sim.get('container')):
        return f"container {values['container']} is not the preview's"
    if values['socket'] == 'boat':
        return 'tmux -L boat is the boat TUI'
    shared = [key for key in ('cache', 'container', 'socket', 'domain') if str(values[key]) == str(other[key])]
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
    repos = sorted(p.name for p in (Path(ws) / 'src').iterdir() if (p / '.git').exists())   # the snapshot's
    found = {repo: commits(Path(ws) / 'src' / repo, values['since']) for repo in repos}
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
    """``PV_KEY='value'`` lines; a list is space-joined (repos as ``name=ref``); nested values (checks) are left out."""
    return sim_stack.shell(values, prefix='PV')


def main(argv):
    test = '--test' in argv
    argv = [a for a in argv if a != '--test']
    values = settings(test)
    if argv[:1] == ['env']:
        refused = refusal(values, settings(not test, environ={}))
        if refused:
            print(f'boat_preview: refused: {refused}', file=sys.stderr)
            return 2
        try:
            root = sim_stack.workspace(values['src'])
            if '--resolve' in argv:
                values = dict(sim_stack.resolve(values, root, pull='--pull' in argv), **sim_stack.vessel(root))
        except sim_stack.Missing as error:
            print(f'boat_preview: {error}', file=sys.stderr)
            return 2
        print(shell(dict(values, workspace=str(root))))
        return 0
    if argv[:1] == ['slug'] and len(argv) == 2:
        print(slug(argv[1]) if argv[1] else MANIFEST)
        return 0
    if argv[:1] == ['plan'] and len(argv) in (1, 2):
        try:
            repos = sim_stack.manifest(sim_stack.workspace(values['src']), values.get('roles') or [])
            found = plan(argv[1] if len(argv) == 2 else '', repos, values['src'])
        except sim_stack.Missing as error:
            print(f'boat_preview: {error}', file=sys.stderr)
            return 2
        print('\n'.join(f'{repo} {sha}' for repo, sha in found))
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
