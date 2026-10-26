"""boat_trial agent verbs: claim, release, status, follow, run, goal, stop, mark, note, ask.

Spec agent-control section 3. ROS-free except ``stop``'s backup path. A verb appends a ``req``
line to the agent channel and waits for the owner TUI's ``reply`` lines; the TUI runs the act
(an owner with no agent desk, a review, is exit 4 once the request goes unanswered). Output is one JSON document, or
JSON lines for ``run``, ``goal`` and ``follow``. Exit codes: 0 ok, 2 refused/busy/blocked (the
``hint`` names the human action that unblocks), 3 revoked/overridden/cancelled/expired, 4 no TUI
owner, 5 agents blocked, 6 timeout. No verb releases a hold, and none arms a boat: only ``run --sim-arm``
arms, and only a stack every harness/sim_arm.py check proves a simulator.
"""

import json
import math
import os
import socket
import sys
import time
from pathlib import Path

from mhseals_testing import data
from mhseals_testing.harness import agent_channel as channel
from mhseals_testing.harness.registry import Registry, runtime_dir

VERBS = ('claim', 'release', 'status', 'follow', 'run', 'goal', 'stop', 'mark', 'note', 'ask')
STREAMS = ('run', 'goal')
SECTIONS = ('authority', 'run', 'health', 'odometry', 'perception', 'nav2', 'agent')
ALWAYS = ('schema', 't_ns', 'host', 'profile', 'sim', 'session', 'owner', 'stale', 'age_s')
DEFAULTS = {'request_s': 5.0, 'ask_s': 120.0, 'lease': {'idle_s': 120.0, 'motion_s': 10.0}}
STATUS_STALE_S = 3.0
STOP_WAIT_S = 1.0
POLL_S = 0.1
SLACK_S = 2.0          # past a request's deadline with no reply at all: the desk is not reading


def add_parsers(sub):
    """Register the agent verbs on the boat_trial subparsers."""
    def verb(name, help, claim=None):
        parser = sub.add_parser(name, help=help)
        parser.add_argument('--who', default=os.environ.get('BOAT_AGENT'),
                            help='your own agent id (default $BOAT_AGENT; required: no shared default)')
        if claim is not None:
            parser.add_argument('--claim', required=claim, metavar='ID')
        return parser
    verb('claim', 'ask the TUI for driving authority').add_argument('--reason', default='')
    verb('release', 'end a claim (the TUI cancels its act first)', claim=True)
    verb('status', 'one snapshot of what the TUI shows').add_argument(
        '--section', type=lambda s: s.split(','), default=[], metavar=','.join(SECTIONS))
    follow = verb('follow', 'stream session events and agent replies', claim=False)
    follow.add_argument('--until', metavar='PHASE[:FIELD=VAL]|owner:none')
    follow.add_argument('--timeout', type=float, metavar='S')
    run = verb('run', 'run a procedure through the TUI', claim=True)
    run.add_argument('procedure', metavar='KEY|NAME')
    run.add_argument('--dry-run', action='store_true', help='evaluate the gates only')
    run.add_argument('--task', help='mission only: the task to rehearse (else the operator picks it)')
    run.add_argument('--sim-arm', action='store_true',
                     help='sim only: arm a verified sim for this run, disarm after (harness/sim_arm.py)')
    run.add_argument('--mission-profile', help="mission only: mhseals_nav's mission profile (qualifying, finals; "
                     "else the nav YAML's own); it sets the run budget")
    goal = verb('goal', 'send a map-frame goal through the TUI', claim=True)
    for name in ('x', 'y'):
        goal.add_argument(name, type=float)
    goal.add_argument('yaw', type=float, nargs='?')
    goal.add_argument('--dry-run', action='store_true', help='evaluate the gates only')
    verb('stop', 'Space: hold, neutral and disarm (mux hold over ROS if the TUI is silent)')
    verb('mark', 'put an agent marker in the session').add_argument('note', nargs='?', default='')
    verb('note', 'a text line in the TUI agent log').add_argument('text')
    ask = verb('ask', 'ask the operator a question')
    ask.add_argument('question')
    ask.add_argument('--options', type=lambda s: s.split(','), default=['yes', 'no'])


def config():
    from mhseals_testing.harness import agent_config
    try:
        return agent_config.load()
    except Exception:          # an uninstalled or broken agent.yaml must not stop stop or status
        return DEFAULTS


class Agent:
    """One verb call; the registry, channel path, clock and stop backup are injectable for tests."""

    def __init__(self, args, registry=None, path=None, out=sys.stdout, clock=time.monotonic,
                 sleep=time.sleep, backup=None, settings=None):
        self.args, self.who = args, args.who
        self.registry = registry or Registry()
        self.path = Path(path or channel.channel_path())
        self.out, self.clock, self.sleep = out, clock, sleep
        self.backup, self.cfg = backup, settings or config()

    def emit(self, doc):
        print(json.dumps(doc, default=str), file=self.out, flush=True)

    def owner(self):
        found = self.registry.owner()
        return found if found and self.registry.alive(found['pid']) else None

    def has_desk(self):
        """Whether the owner runs an agent desk: only a desk writes status.json, with its pid."""
        doc, owner = data.read_json(runtime_dir() / 'status.json'), self.owner()
        return bool(owner) and isinstance(doc, dict) and doc.get('pid') == owner['pid']

    def local(self, req, error, hint='', **result):
        """Answer ``req`` here (no desk), journal the reply on the channel and print it."""
        reply = channel.reply(req['id'], self.who, ok=not error, error=error, hint=hint,
                              result=result or None, path=self.path)
        self.emit(reply)
        return channel.exit_code(reply)

    def send(self, op, args=None, claim=None, dry=False, wait=None):
        """Append a req and wait for its final reply; returns the exit code."""
        tail = channel.Tail(self.path)
        tail.seek_end()
        req = channel.request(op, self.who, args, claim, dry, self.cfg['request_s'], path=self.path)
        if self.owner() is None:
            return self.local(req, 'no_owner', 'start boat_test on the boat; agents act only through it')
        return self.wait(req, tail, wait)

    def wait(self, req, tail, limit=None):
        inode = tail.inode or os.stat(self.path).st_ino
        motion = req['op'] in STREAMS and not req['dry']
        every = self.cfg['lease']['motion_s'] / 3
        start, renewed, heard = self.clock(), self.clock(), False
        while True:
            records, _ = tail.read()
            for reply in (r for r in records if r['kind'] == 'reply' and r['re'] == req['id']):
                heard = True
                if req['op'] in STREAMS or reply['state'] == 'done' or not reply['ok']:
                    self.emit(reply)
                if reply['state'] == 'done' or not reply['ok']:
                    return channel.exit_code(reply)
            now = self.clock()
            if motion and now - renewed >= every:
                channel.renew(self.who, req['claim'], req['id'], path=self.path)
                renewed = now
            if self.owner() is None:
                return self.local(req, 'no_owner', 'the TUI quit; the act ended with it')
            try:
                moved = os.stat(self.path).st_ino != inode
            except FileNotFoundError:
                moved = True
            late = not heard and time.time_ns() > req['deadline_ns'] + SLACK_S * 1e9
            if late and not moved and not self.has_desk():
                return self.local(req, 'no_owner', 'the owner TUI runs no agent desk (review or guest); '
                                  'start a live boat_test')
            if moved or late or (limit is not None and now - start > limit):
                hint = 'channel file rotated or removed' if moved else 'no reply from the TUI'
                self.emit({'re': req['id'], 'ok': False, 'error': 'timeout', 'hint': hint})
                return 6     # nothing assumed: the motion lease lapses if the act is still running
            self.sleep(POLL_S)

    # ---- verbs

    def claim(self):
        return self.send('claim', {'reason': self.args.reason})

    def release(self):
        return self.send('release', claim=self.args.claim)

    def run(self):
        args = {'key': self.args.procedure}
        if getattr(self.args, 'task', None):
            args['task'] = self.args.task
        if getattr(self.args, 'sim_arm', False):
            args['sim_arm'] = True
        if getattr(self.args, 'mission_profile', None):
            args['profile'] = self.args.mission_profile
        return self.send('run', args, self.args.claim, self.args.dry_run)

    def goal(self):
        args = {'x': self.args.x, 'y': self.args.y, 'yaw': self.args.yaw}
        if not all(math.isfinite(v) for v in args.values() if v is not None):   # JSON has no nan/inf
            self.emit({'ok': False, 'error': 'bad_args', 'hint': 'goal needs finite x, y (and yaw)'})
            return channel.EXIT_CODES['bad_args']
        return self.send('goal', args, self.args.claim, self.args.dry_run)

    def ask(self):
        args = {'question': self.args.question, 'options': self.args.options}
        return self.send('ask', args, wait=self.cfg['ask_s'] + self.cfg['request_s'] + SLACK_S)

    def note(self):
        self.emit(channel.note(self.who, 'text', self.args.text, path=self.path))
        return 0

    def mark(self):
        if self.owner() is not None:
            return self.send('mark', {'note': self.args.note})
        session = last_session(self.registry)
        if session is None:
            self.emit({'ok': False, 'error': 'no_session', 'hint': 'no session recorded yet'})
            return 2
        events = data.read_jsonl(session / 'events.jsonl')
        record = {'t_ns': time.time_ns(), 'phase': 'marker', 'kind': 'agent', 'who': self.who,
                  'id': max((e['id'] for e in events if e.get('phase') == 'marker'
                             and isinstance(e.get('id'), int)), default=0) + 1,
                  'note': self.args.note, 'rec': False}
        data.append_jsonl(session / 'events.jsonl', record)
        self.emit({'ok': True, 'session': str(session), 'marker': record})
        return 0

    def stop(self):
        """Ask the TUI (Space); with no reply in STOP_WAIT_S, publish the mux hold over ROS too."""
        if self.owner() is not None:
            tail = channel.Tail(self.path)
            tail.seek_end()
            req = channel.request('stop', self.who, path=self.path, request_s=self.cfg['request_s'])
            end = self.clock() + STOP_WAIT_S
            while self.clock() < end:
                replies = [r for r in tail.read()[0] if r['kind'] == 'reply' and r['re'] == req['id']]
                if replies:
                    self.emit(replies[-1])
                    return 0 if replies[-1]['ok'] else channel.exit_code(replies[-1])
                self.sleep(POLL_S)
        else:
            req = channel.request('stop', self.who, path=self.path, request_s=self.cfg['request_s'])
        backup = self.backup
        if backup is None:
            from mhseals_testing.ros.agent_stop import hold as backup
        try:
            held = backup(self.who)
        except Exception as error:      # no rclpy, no domain: say so, never raise
            held, why = False, repr(error)
        else:
            why = '' if held else 'no subscriber on /command_mux/request'
        if held:
            return self.local(req, '', via='ros', hold='agent_stop')
        hint = 'press Space or use the RC/kill switch'
        return self.local(req, f'timeout: {why}' if self.owner() else 'no_owner', hint, via='ros')

    def status(self):
        self.emit(status(self.registry, self.args.section))
        return 0

    def follow(self):
        return follow(self, self.args.until, self.args.timeout, self.args.claim)


def last_session(registry):
    """The registry's session, else the newest under sessions/, else None."""
    named = (registry.state() or {}).get('session')
    if named and Path(named).is_dir():
        return Path(named)
    found = sorted(p for p in data.path('sessions').glob('*/') if p.is_dir())
    return found[-1] if found else None


def status(registry, sections=(), now_ns=None):
    """status.json while its TUI is alive and it is fresh; else what is known headless."""
    owner = registry.owner()
    alive = owner and registry.alive(owner['pid'])
    doc = data.read_json(runtime_dir() / 'status.json')
    now_ns = now_ns or time.time_ns()
    if alive and isinstance(doc, dict):
        age = (now_ns - int(doc.get('t_ns') or 0)) / 1e9
        doc = dict(doc, age_s=round(age, 3), stale=age > STATUS_STALE_S)
    else:
        from mhseals_testing.health import probes
        session = last_session(registry)
        try:
            host = {k: f.value for k, f in probes.host_facts(data.root()).items()}
        except OSError as error:
            host = {'error': repr(error)}
        doc = {'schema': 'mhseals.status/1', 't_ns': now_ns, 'owner': owner if alive else None,
               'stale': True, 'registry': registry.state(), 'host_facts': host,
               'session': data.read_json(session / 'session.json') if session else None}
    if sections:
        doc = {k: v for k, v in doc.items() if k in ALWAYS or k in sections or k == 'registry'}
    return doc


def matches(event, until):
    phase, _, test = until.partition(':')
    if event.get('phase') != phase:
        return False
    if not test:
        return True
    field, _, want = test.partition('=')
    value = event.get(field, (event.get('rec') or {}).get(field) if isinstance(event.get('rec'), dict) else None)
    return str(value) == want


class Lines:
    """New complete JSON lines of one file, from its end at construction."""

    def __init__(self, path):
        self.path = Path(path)
        try:
            self.offset = self.path.stat().st_size
        except FileNotFoundError:
            self.offset = 0

    def read(self):
        try:
            with open(self.path, 'rb') as handle:
                handle.seek(self.offset)
                chunk = handle.read(1 << 20)
        except FileNotFoundError:
            return []
        out = []
        for raw in chunk.splitlines(keepends=True):
            if not raw.endswith(b'\n'):
                break
            self.offset += len(raw)
            try:
                record = json.loads(raw)
            except ValueError:
                continue
            if isinstance(record, dict):
                out.append(record)
        return out


def follow(agent, until=None, timeout=None, claim=None):
    """Stream the owner session's events and agent replies until ``until``, timeout or no owner."""
    if agent.owner() is None:
        if until == 'owner:none':
            agent.emit({'ok': True, 'until': until})
            return 0
        session = last_session(agent.registry)
        for event in data.read_jsonl(session / 'events.jsonl')[-50:] if session else []:
            agent.emit(event)
        return 4 if until else 0
    session = last_session(agent.registry)
    events = Lines(session / 'events.jsonl') if session else None
    replies = channel.Tail(agent.path)
    replies.seek_end()
    every = agent.cfg['lease']['idle_s'] / 3
    start = renewed = agent.clock()
    if claim:
        channel.renew(agent.who, claim, None, path=agent.path)
    while True:
        for event in events.read() if events else []:
            agent.emit(event)
            if until and until != 'owner:none' and matches(event, until):
                return 0
        for record in replies.read()[0]:
            if record['kind'] == 'reply':
                agent.emit(record)
        now = agent.clock()
        if agent.owner() is None:
            agent.emit({'ok': until == 'owner:none', 'error': '' if until == 'owner:none' else 'no_owner'})
            return 0 if until == 'owner:none' else 4
        if timeout is not None and now - start > timeout:
            agent.emit({'ok': False, 'error': 'timeout', 'until': until})
            return 6
        if claim and now - renewed >= every:
            channel.renew(agent.who, claim, None, path=agent.path)
            renewed = now
        agent.sleep(POLL_S)


SHARED_WHO = {'', 'agent', 'agent-0000'}     # the old defaults: every agent that copied them was one principal


def who_of(args):
    """``(who, None)``, or ``(None, hint)`` when the verb needs an id of its own and has none. ``status`` reads only
    and needs none; ``stop`` always works: without an id it is ``agent@HOST:PPID``, so the operator still sees
    that an agent (not they) stopped."""
    who = (args.who or '').strip()
    if who not in SHARED_WHO:
        return who, None
    if args.command == 'stop':
        return f'agent@{socket.gethostname().split(".")[0]}:{os.getppid()}', None
    if args.command == 'status':
        return who or 'agent', None
    return None, ('pass --who ID (or export BOAT_AGENT=ID) with an id of your own, such as the one '
                  '`python3 .agents/hook.py --playbook` prints; "agent" and "agent-0000" are shared, so refused')


def main(args, **seams):
    """Run verb ``args.command``; returns its exit code."""
    args.who, hint = who_of(args)
    if hint:
        print(json.dumps({'ok': False, 'error': 'bad_args', 'hint': hint}),
              file=seams.get('out') or sys.stdout, flush=True)
        return channel.EXIT_CODES['bad_args']
    return getattr(Agent(args, **seams), args.command)()
