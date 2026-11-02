"""crane-worker backends for RoboBoat: the fast sim and bag replay (replay.py).

    backends:
      fastsim_roboboat:
        type: "mhseals_testing.fastsim_adapter.cluster:FastsimBackend"
        path: <an mhseals_testing checkout>      # only for this import; jobs use their own refs
        python: <a python with numpy, pyyaml, py_trees>
        repos: {mhseals_testing: URL, mhseals_nav: URL, crane_ml: URL}
        workers: 4
        wrap: [<a host guard>, sim]                # optional prefix for the batch command

Stdlib only: it runs inside the worker. A job's code is exact commits: the hypothesis repo at the
job's ref (``ctx.repos``) and the others at ``spec.refs[name]`` (default ``main``), checked out
once per commit under the worker's data folder (crane_ml sparse: ``SPARSE``). The batch runs as a
child process (``fastsim_adapter.batch``) with that code on its path, so two jobs never share
imported modules. Spec keys: ``tune`` (config tune overrides: the sweep's parameter set),
``config`` (a YAML in mhseals_testing, default the shipped one), ``refs``, ``urls``, ``workers``.
Metrics are the batch's ``metrics.json`` (core and referee numbers). coursegen needs dotnet and
Unity's managed assemblies (crane Tools/RoboBoat/cli/README.md).
"""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

NEEDED = ('mhseals_testing', 'mhseals_nav', 'crane_ml')
# crane_ml: its fast-sim core and course tools, and what coursegen compiles and hashes (CourseIdentity)
SPARSE = {'crane_ml': ['fastsim', 'Tools/RoboBoat', 'ProjectSettings', 'Assets/Scripts/Course/Core',
                       'Assets/Scripts/Course/RoboBoat/Tasks', 'Assets/Config/Course/Resources']}
SUMMARY = ('metrics.json', 'run.json', 'fastsim.json', 'session.json')
RAW = ('track.jsonl', 'task_events.jsonl', 'task_status.jsonl', 'events.jsonl', 'sim_events.jsonl')


def merge(base, over):
    out = dict(base)
    for key, value in (over or {}).items():
        out[key] = merge(out[key], value) if isinstance(out.get(key), dict) and isinstance(value, dict) else value
    return out


def _git(*args, cwd=None):
    done = subprocess.run(['git', *args], cwd=cwd, capture_output=True, text=True, timeout=1800,
                          env={**os.environ, 'GIT_TERMINAL_PROMPT': '0'})
    if done.returncode:
        raise RuntimeError(f"git {' '.join(args)}: {done.stderr.strip()[-500:]}")
    return done.stdout.strip()


class Sources:
    """Exact-commit checkouts kept per commit (read-only, shared by jobs) beside the worker's mirrors."""

    def __init__(self, data_dir):
        from crane_cluster.gitcache import GitCache
        self.cache = GitCache(Path(data_dir) / 'git')
        self.root = Path(data_dir) / 'roboboat' / 'src'

    def checkout(self, name, url, ref):
        sha = self.cache.resolve(name, url, ref)
        dest = self.root / f'{name}-{sha[:12]}'
        if not dest.is_dir():
            tmp = dest.with_name(dest.name + f'.tmp{os.getpid()}')
            shutil.rmtree(tmp, ignore_errors=True)
            tmp.parent.mkdir(parents=True, exist_ok=True)
            _git('clone', '--quiet', '--shared', '--no-checkout', str(self.cache.mirror_path(name)), str(tmp))
            if name in SPARSE:
                _git('sparse-checkout', 'set', '--cone', *SPARSE[name], cwd=tmp)
            _git('checkout', '--quiet', '--detach', sha, cwd=tmp)
            try:
                tmp.rename(dest)
            except OSError:  # another job made it first
                shutil.rmtree(tmp, ignore_errors=True)
        return dest, sha


def coursegen(crane, python=None):
    """crane's course generator, built once per crane commit (dotnet)."""
    dll = crane / 'Tools/RoboBoat/cli/bin/Release/net10.0/coursegen.dll'
    if not dll.is_file():
        done = subprocess.run(['dotnet', 'build', '-c', 'Release', '--nologo', '-v', 'q',
                               str(crane / 'Tools/RoboBoat/cli')], capture_output=True, text=True, timeout=1800)
        if done.returncode or not dll.is_file():
            raise RuntimeError(f'coursegen build failed: {(done.stdout + done.stderr)[-800:]}')
    return dll


class FastsimBackend:
    """The RoboBoat fast sim on the crane cluster (module docstring)."""

    def __init__(self, name='fastsim_roboboat', options=None):
        self.name, self.options, self.ctx, self.proc = name, dict(options or {}), None, None

    def update(self):
        pass

    def wrapped(self, cmd):
        """``cmd`` behind the backend's ``wrap`` prefix (e.g. a host's heavy-job guard), if any."""
        return [os.path.expanduser(str(w)) for w in self.options.get('wrap') or []] + cmd

    def code(self, ctx):
        """{repo: (checkout, sha)}: the hypothesis repo at the job's ref, the rest at spec refs."""
        data_dir = Path(ctx.work_dir).parents[1]
        urls = {**self.options.get('repos', {}), **(ctx.spec.get('urls') or {})}
        refs = ctx.spec.get('refs') or {}
        sources, out = None, {}
        for name in NEEDED:
            if name in ctx.repos:
                out[name] = (Path(ctx.repos[name]), ctx.resolved.get(name))
                continue
            if name not in urls:
                raise RuntimeError(f'no url for {name}: set it in the worker backend repos or the spec urls')
            sources = sources or Sources(data_dir)
            out[name] = sources.checkout(name, urls[name], refs.get(name, 'main'))
        return out

    def prepare(self, ctx):
        self.ctx = ctx
        spec = ctx.spec
        code = self.code(ctx)
        self.shas = {k: v[1] for k, v in code.items()}
        testing, nav, crane = (code[k][0] for k in NEEDED)
        coursegen(crane)
        tune = merge(self.options.get('tune') or {}, spec.get('tune') or {})
        truths = Path(ctx.work_dir).parents[1] / 'roboboat' / 'truths' / \
            f"{tune.get('family', 'roboboat_2026')}-{tune.get('mode', 'iid')}-{(self.shas['crane_ml'] or 'local')[:12]}"
        self.out = Path(ctx.work_dir) / 'runs'
        self.variant = str(spec.get('arm') or 'base')
        python = os.path.expanduser(self.options.get('python') or sys.executable)
        cmd = [python, '-m', 'mhseals_testing.fastsim_adapter.batch',
               '--seeds', ','.join(str(int(s)) for s in ctx.job['seeds']), '--out', str(self.out),
               '--variant', self.variant, '--workers', str(int(spec.get('workers') or self.options.get('workers', 4)))]
        if spec.get('config'):
            cmd += ['--config', str(testing / spec['config'])]
        if tune:
            cmd += ['--tune', json.dumps(tune)]
        env = {**os.environ, 'PYTHONPATH': os.pathsep.join(str(p) for p in (testing, nav, crane)),
               'CRANE_DIR': str(crane), 'FASTSIM_TRUTHS': str(truths), **ctx.env}
        log = Path(ctx.work_dir) / 'batch.log'
        with open(log, 'w') as f:
            self.proc = subprocess.Popen(self.wrapped(cmd), stdout=f, stderr=subprocess.STDOUT, env=env,
                                         cwd=str(testing))
            code_ = self.proc.wait()
        self.log = log
        if code_:
            raise RuntimeError(f"batch exit {code_}: {log.read_text()[-1500:]}")

    def run(self, seed):
        run_dir = self.out / self.variant / f's{int(seed)}'
        if not (run_dir / 'metrics.json').is_file():
            raise RuntimeError(f'seed {seed}: no metrics ({self.log.read_text()[-800:]})')
        return run_dir

    def metrics(self, run_dir):
        data = json.loads((Path(run_dir) / 'metrics.json').read_text())
        return {k: float(v) for k, v in data.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}

    def artifacts(self, run_dir):
        run_dir = Path(run_dir)
        out = [(run_dir / n, n, 'summary') for n in SUMMARY if (run_dir / n).is_file()]
        out += [(p, f'analysis/{p.name}', 'summary') for p in sorted((run_dir / 'analysis').glob('score.*.json'))]
        out += [(run_dir / n, n, 'raw') for n in RAW if (run_dir / n).is_file()]
        prov = run_dir / 'code.json'
        prov.write_text(json.dumps({'shas': self.shas, 'spec': self.ctx.spec}, indent=1))
        return out + [(prov, 'code.json', 'summary')]

    def cancel(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()

    def finish(self):
        self.cancel()


class ReplayBackend(FastsimBackend):
    """Bag replay on the crane cluster: seed k replays ``spec.sessions[k - 1]`` (a folder under the
    backend's ``sessions`` option, the worker's data root), so arms pair on sessions."""

    def prepare(self, ctx):
        self.ctx = ctx
        code = self.code(ctx)
        self.shas = {k: v[1] for k, v in code.items()}
        self.paths = [code[k][0] for k in ('mhseals_testing', 'mhseals_nav', 'crane_ml')]
        self.out = Path(ctx.work_dir) / 'runs'
        self.log = Path(ctx.work_dir) / 'replay.log'

    def run(self, seed):
        sessions = self.ctx.spec.get('sessions') or []
        if not 1 <= int(seed) <= len(sessions):
            raise RuntimeError(f'seed {seed}: the spec names {len(sessions)} sessions')
        if not self.options.get('sessions'):
            raise RuntimeError('set the replay backend option sessions (the folder holding session folders)')
        root = Path(os.path.expanduser(self.options['sessions']))
        out = self.out / f's{int(seed)}'
        python = os.path.expanduser(self.options.get('python') or sys.executable)
        cmd = [python, '-m', 'mhseals_testing.fastsim_adapter.replay',
               str(root / sessions[int(seed) - 1]), str(out), '--tune', json.dumps(self.ctx.spec.get('tune') or {})]
        env = {**os.environ, 'PYTHONPATH': os.pathsep.join(map(str, self.paths)), **self.ctx.env}
        with open(self.log, 'a') as f:
            self.proc = subprocess.Popen(self.wrapped(cmd), stdout=f, stderr=subprocess.STDOUT, env=env,
                                         cwd=str(self.paths[0]))
            if self.proc.wait():
                raise RuntimeError(f'replay exit {self.proc.returncode}: {self.log.read_text()[-1200:]}')
        return out

    def artifacts(self, run_dir):
        return [(Path(run_dir) / n, n, 'summary') for n in ('diff.json', 'metrics.json')]
