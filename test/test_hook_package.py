"""The agent hook's hook/ package: loaded by path as the dispatcher does, and guarded like agent_hook.py itself."""

from pathlib import Path
import subprocess
import sys

from mhseals_testing import agent_hook as hook

PACKAGE = Path(hook.__file__).resolve().parent / 'hook'


def test_the_entry_is_thin_and_every_part_is_stdlib_only():
    assert len(Path(hook.__file__).read_text().splitlines()) < 150
    for path in PACKAGE.glob('*.py'):
        text = path.read_text()
        assert 'import mhseals' not in text and 'from mhseals' not in text, path     # relative imports only


def test_loaded_twice_by_path_under_other_names_it_answers_the_same(tmp_path):
    """The dispatcher and the canonical hook each load agent_hook.py by path under their own module name."""
    probe = ('import importlib.util, json, sys\n'
             'out = []\n'
             'for name in ("agents_hook_mhseals_testing", "agents_hook_canonical_x"):\n'
             '    spec = importlib.util.spec_from_file_location(name, sys.argv[1])\n'
             '    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)\n'
             '    out.append(m.agent_event({"agent": "codex", "event": "pre", "session_id": "ab12",\n'
             '                              "command": "ros2 topic pub /cmd_vel x y"}, send=lambda l: None,\n'
             '                             state_dir=sys.argv[2]))\n'
             'print(json.dumps(out))')
    env = {'PATH': '/usr/bin:/bin', 'XDG_STATE_HOME': str(tmp_path), 'HOME': str(tmp_path), 'TMPDIR': str(tmp_path)}
    done = subprocess.run([sys.executable, '-I', '-c', probe, hook.__file__, str(tmp_path)],
                          capture_output=True, text=True, env=env, cwd=tmp_path, timeout=10)
    assert done.returncode == 0, done.stderr
    first, second = __import__('json').loads(done.stdout)
    assert first == second and 'raw motion' in first['deny']


def test_the_live_hook_package_is_guarded_like_the_entry(tmp_path):
    """A live copy (outside a linked worktree) of hook/*.py: no edit tool, no shell write."""
    live = tmp_path / 'astro_dock' / 'src' / 'mhseals_testing' / 'mhseals_testing' / 'hook'
    live.mkdir(parents=True)
    (live / 'rules.py').write_text('')
    event = {'agent': 'claude', 'event': 'pre', 'session_id': 's', 'tool_id': 't', 'command': '',
             'edits': [['write', str(live / 'rules.py')]]}
    reason = hook.agent_event(event, config=dict(hook.DEFAULTS), send=lambda line: None)['deny']
    assert 'the live agent hook' in reason
    for command in (f'sed -i s/a/b/ {live}/rules.py', f'rm {live}/scan.py', f'echo x > {live}/new.py'):
        assert 'the live agent hook' in hook.deny_reason(*hook.denied(command, hook.DEFAULTS, cwd='/')), command
    assert hook.denied(f'cat {live}/rules.py', hook.DEFAULTS) is None                  # reading stays allowed
