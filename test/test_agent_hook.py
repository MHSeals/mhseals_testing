"""The Claude Code hook (spec agent-control section 6): deny list, notes, rate cap, -I stdlib run."""

import json
from pathlib import Path
import subprocess
import sys

import pytest

from mhseals_testing.harness import agent_channel, agent_config
from mhseals_testing.harness import agent_hook as hook

SID = '3f2a9c1e-0000-4000-8000-000000000000'


@pytest.fixture
def sent(tmp_path, monkeypatch):
    monkeypatch.delenv('BOAT_AGENT', raising=False)
    lines = []

    def run(phase, command, now=100.0, tool='t1', **extra):
        event = {'session_id': SID, 'tool_use_id': tool, 'tool_input': {'command': command}, **extra}
        return hook.handle(phase, event, send=lines.append, now=now, state_dir=tmp_path)
    run.lines = lines
    return run


def test_defaults_are_the_shipped_hook_config():
    shipped = agent_config.load()['hook']
    assert hook.hook_config() == {k: shipped[k] for k in hook.DEFAULTS}
    assert hook.hook_config('/nonexistent') == hook.DEFAULTS
    assert {k: v for k, v in shipped.items() if k in hook.DEFAULTS} == hook.DEFAULTS


@pytest.mark.parametrize('command', [
    'ssh blastoise ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"',
    'ros2 topic pub --once /command_mux/request std_msgs/msg/String "{data: release}"',
    'ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"',
    'ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}"',
    'ssh blastoise pkill -f boat_test'])
def test_raw_motion_is_denied_with_the_verb_to_use(sent, command):
    decision = json.loads(sent('pre', command))['hookSpecificOutput']
    assert decision['permissionDecision'] == 'deny' and decision['hookEventName'] == 'PreToolUse'
    assert 'boat_agent run|goal|stop' in decision['permissionDecisionReason']
    assert 'claude-3f2a' in decision['permissionDecisionReason'] and not sent.lines


def test_boat_commands_become_redacted_notes_and_others_are_ignored(sent):
    assert sent('pre', 'ssh blastoise ros2 topic hz /odom TOKEN=abc', now=100.0) == ''
    assert sent('post', 'ssh blastoise ros2 topic hz /odom TOKEN=abc', now=102.5,
                tool_response={'exit_code': 0}) == ''
    for ignored in ('git status', 'ssh blastoise boat_agent status --who claude-3f2a', 'ls ~/ros2_ws'):
        sent('pre', ignored)
    pre, post = sent.lines
    for line in (pre, post):
        assert agent_channel.invalid(line) is None and line['who'] == 'claude-3f2a'
        assert line['cmd'] == agent_channel.redact('ssh blastoise ros2 topic hz /odom TOKEN=abc')
    assert pre['phase'] == 'pre' and pre['tool_id'] == 't1' and pre['target'] == 'blastoise'
    assert post['exit'] == 0 and post['dur_s'] == 2.5


def test_rate_cap_folds_the_excess_into_the_next_line(sent):
    for i in range(9):
        sent('pre', 'ros2 node list', now=100.0 + i * 0.1, tool=f't{i}')
    assert len(sent.lines) == 6 and 'folded' not in sent.lines[-1]
    sent('pre', 'ros2 node list', now=111.0, tool='late')
    assert sent.lines[-1]['folded'] == 3 and sent.lines[-1]['tool_id'] == 'late'


def test_start_names_the_agent_and_boat_agent_env_wins(sent, monkeypatch):
    context = json.loads(sent('start', ''))['hookSpecificOutput']
    assert context['hookEventName'] == 'SessionStart'
    assert 'claude-3f2a' in context['additionalContext'] and 'boat_agent' in context['additionalContext']
    monkeypatch.setenv('BOAT_AGENT', 'mapper')
    assert 'mapper' in json.loads(sent('start', ''))['hookSpecificOutput']['additionalContext']


def test_runs_isolated_from_the_checkout_and_always_exits_0(tmp_path):
    script = Path(hook.__file__)
    deny = {'session_id': SID, 'tool_input': {'command': 'ros2 topic pub /cmd_vel x y'}}
    env = {'PATH': '/usr/bin:/bin', 'XDG_STATE_HOME': str(tmp_path), 'HOME': str(tmp_path)}
    done = subprocess.run([sys.executable, '-I', str(script), 'pre'], input=json.dumps(deny),
                          capture_output=True, text=True, env=env, cwd=tmp_path, timeout=10)
    assert done.returncode == 0
    assert json.loads(done.stdout)['hookSpecificOutput']['permissionDecision'] == 'deny'
    bad = subprocess.run([sys.executable, '-I', str(script), 'pre'], input='not json',
                         capture_output=True, text=True, env=env, cwd=tmp_path, timeout=10)
    assert bad.returncode == 0 and bad.stdout == ''
    assert (tmp_path / 'boat_agent_hook.log').read_text()


def test_delivery_appends_locally_on_the_target_and_logs_failures(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path))
    channel = tmp_path / 'channel.jsonl'
    monkeypatch.setattr(hook.socket, 'gethostname', lambda: 'blastoise')
    hook.deliver({'kind': 'note'}, dict(hook.DEFAULTS, path=str(channel)))
    assert json.loads(channel.read_text()) == {'kind': 'note'}
    hook.deliver({'kind': 'note'}, dict(hook.DEFAULTS, path=str(tmp_path / 'no' / 'dir.jsonl')))
    assert 'deliver to blastoise' in (tmp_path / 'boat_agent_hook.log').read_text()
