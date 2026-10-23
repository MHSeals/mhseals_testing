"""git probe: per repo, branch, ahead/behind origin, dirty and untracked counts.

One ``git status --porcelain=v2 --branch -uall`` and one ``git log -1`` per
repo. A fetch is opt-in (``fetch`` seconds, 0 = never), runs with
``GIT_TERMINAL_PROMPT=0`` and a 10 s timeout, and never fails the sample.
"""

import os
import subprocess
import threading
import time

from mhseals_testing.health.model import Fact

PERIOD = 30.0            # seconds between samples when run as a worker
FETCH_TIMEOUT = 10.0
RUN_TIMEOUT = 20.0


def parse_status(text):
    """Counts and branch info from ``git status --porcelain=v2 --branch`` output."""
    state = dict(branch='', upstream='', ahead=0, behind=0, dirty=0, untracked=0, hash='')
    for line in text.splitlines():
        if line.startswith('# branch.head '):
            state['branch'] = line.split(' ', 2)[2]
        elif line.startswith('# branch.oid '):
            state['hash'] = line.split(' ', 2)[2][:7]
        elif line.startswith('# branch.upstream '):
            state['upstream'] = line.split(' ', 2)[2]
        elif line.startswith('# branch.ab '):
            ahead, behind = line.split()[2:4]
            state['ahead'], state['behind'] = int(ahead[1:]), int(behind[1:])
        elif line[:1] in '12u':
            state['dirty'] += 1
        elif line.startswith('? '):
            state['untracked'] += 1
    return state


class GitProbe:
    """Samples ``repos`` ({name: path}); ``fetch`` is the default fetch period, per-repo overridable."""

    def __init__(self, repos, fetch=0.0, fetches=None, run=subprocess.run, clock=time.time):
        self.repos = dict(repos)
        self.fetch = fetch
        self.fetches = fetches or {}         # repo -> fetch period overriding the default
        self.run = run
        self.clock = clock
        self.latest = {}
        self._stop = threading.Event()

    def _git(self, path, *args, timeout=RUN_TIMEOUT):
        env = {**os.environ, 'GIT_TERMINAL_PROMPT': '0'}
        done = self.run(['git', '-C', str(path), *args], capture_output=True, text=True,
                        timeout=timeout, env=env)
        if done.returncode:
            raise RuntimeError(done.stderr.strip() or f'git {args[0]} exited {done.returncode}')
        return done.stdout

    def _fetch_age(self, path):
        try:
            return self.clock() - os.stat(os.path.join(path, '.git', 'FETCH_HEAD')).st_mtime
        except OSError:
            return None

    def _repo(self, name, path):
        period = self.fetches.get(name, self.fetch)
        age = self._fetch_age(path)
        if period > 0 and (age is None or age >= period):
            try:
                self._git(path, 'fetch', '--quiet', timeout=FETCH_TIMEOUT)
                age = self._fetch_age(path)
            except (RuntimeError, OSError, subprocess.TimeoutExpired):
                pass                          # offline is normal at the dock; the age shows it
        state = parse_status(self._git(path, 'status', '--porcelain=v2', '--branch', '-uall'))
        state['commit_time'] = int(self._git(path, 'log', '-1', '--format=%ct').strip() or 0)
        state['fetch_age'] = age
        return state

    def sample(self):
        """``{'git.<repo>': Fact}``; a repo that cannot be read has ``{'error': ...}``."""
        now, facts = self.clock(), {}
        for name, path in self.repos.items():
            try:
                value = self._repo(name, path)
            except (RuntimeError, OSError, ValueError, subprocess.TimeoutExpired) as error:
                value = {'error': str(error) or type(error).__name__}
            facts[f'git.{name}'] = Fact(value, label=name, time=now)
        return facts

    def start(self, period=PERIOD):
        """Sample on a daemon thread, publishing into ``latest``; returns the thread."""
        def loop():
            while not self._stop.is_set():
                self.latest = self.sample()
                self._stop.wait(period)
        thread = threading.Thread(target=loop, name='git-probe', daemon=True)
        thread.start()
        return thread

    def stop(self):
        self._stop.set()


def from_specs(specs, settings=None):
    """A GitProbe for every git_state check in ``specs``; ``settings`` is ``probes.git`` from the config."""
    repos, fetches = {}, {}
    for spec in specs:
        if spec.kind == 'git_state' and spec.enabled:
            repos[spec.params.repo] = spec.params.path
            if spec.params.fetch is not None:
                fetches[spec.params.repo] = spec.params.fetch
    return GitProbe(repos, (settings or {}).get('fetch', 0.0), fetches)
