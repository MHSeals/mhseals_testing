"""One event in, one answer out (``agent_event``): the session-start text, a deny, or a note to the TUI. Stdlib only."""

import os
from pathlib import Path
import subprocess
import time

from .hosts import hostname, note_match, target_of
from .notes import SCHEMA, State, deliver, redact
from .playbook import brief_text, deny_reason, start_text, use_of, who_of
from .rules import hook_config
from .scan import claimed_ids, denied, edit_denied
from .shell import matches


def deps_notice():
    """The workspace's missing host tools (devices/deps.sh check: declared in the repos' deps.yaml), '' when complete."""
    script = Path(__file__).resolve().parents[4] / 'devices' / 'deps.sh'
    try:
        done = subprocess.run([str(script), 'check'], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return ''
    return done.stderr.strip()


def agent_event(event, config=None, send=None, now=None, state_dir=None):
    """The answer to one neutral event (None, ``{'context'}`` or ``{'deny'}``); sends at most one note."""
    phase, agent = event.get('event'), event.get('agent') or 'agent'
    who = who_of(event)
    if phase not in ('start', 'pre', 'post'):
        return None
    config = config or hook_config()
    if phase == 'start':
        full = event.get('source') == 'playbook' or target_of(config) != 'local'
        text = start_text(who, agent, config) if full else brief_text(who, config)
        notice = deps_notice()
        return {'context': text + ('\n\n' + notice + '\nTell the owner; do not run sudo yourself.' if notice else '')}
    send = send or (lambda line: deliver(line, config))
    now = time.time() if now is None else now
    command = str(event.get('command') or '')
    answer = blocked = None
    if phase == 'pre' and 'edits' in event:     # an edit tool, or a shell apply_patch: its files first
        reason = edit_denied(event, config)
        if reason:
            return {'deny': reason}
    if phase == 'pre':
        blocked = denied(command, config, event.get('cwd'), who)
    named = claimed_ids(command) if phase == 'pre' else []
    if named or blocked:
        with State(str(event.get('session_id') or 'none'), state_dir) as state:
            ids = state.data.setdefault('who', {})
            key = str(event.get('agent_id') or '')
            if named:
                ids[key] = named[-1]
            who = ids.get(key, who)
    if blocked:
        answer, phase = {'deny': deny_reason(*blocked, who=who, config=config)}, 'deny'
    elif not matches(note_match(config), command) or matches(config['skip'], command):
        return None
    tool_id = event.get('tool_id')
    note = {'phase': phase, 'tool_id': tool_id, 'cmd': redact(command), 'exit': None, 'dur_s': None,
            'target': target_of(config), 'agent': agent}
    if phase == 'deny':
        note['use'] = use_of(blocked[0])
    with State(str(event.get('session_id') or 'none'), state_dir) as state:
        if phase == 'pre':
            state.data['started'] = {**dict(list(state.data['started'].items())[-31:]), str(tool_id): now}
        elif phase == 'post':
            note['exit'] = event.get('exit')            # None: interrupted or not reported
            began = state.data['started'].pop(str(tool_id), None)
            took = event.get('duration_ms')
            note['dur_s'] = (round(took / 1000, 2) if isinstance(took, (int, float))
                             else None if began is None else round(now - began, 2))
        admitted, folded = state.admit(now, config['max_per_10s'])
    if admitted:
        send({'schema': SCHEMA, 't_ns': int(now * 1e9), 'host': hostname(), 'pid': os.getpid(),
              'who': who, 'kind': 'note', **note, **({'folded': folded} if folded else {})})
    return answer
