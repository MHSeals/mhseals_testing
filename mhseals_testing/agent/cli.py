"""boat_trial agent verbs: claim, release, status, follow, run (and run --list), goal, start, stop, arm, reload, set, fcu, fence, mark,
note, ask.

Spec agent-control section 3. ROS-free except ``stop``'s backup path. A verb appends a ``req``
line to the agent channel and waits for the owner TUI's ``reply`` lines; the TUI runs the act
(an owner with no agent desk, a review, is exit 4 once the request goes unanswered). Output is one JSON document, or
JSON lines for ``run``, ``goal`` and ``follow``. Exit codes: 0 ok, 2 refused/busy/blocked (the
``hint`` names the human action that unblocks), 3 revoked/overridden/cancelled/expired, 4 no TUI
owner (``unreachable``: the boat host did not answer; tools/boat_agent says so with the last operator state it
saw), 5 agents blocked, 6 timeout, 7 paused (the operator declared manual, battery swap or power cycle; the hint
says since when and the ETA: wait for normal). ``release HOLD --why`` releases a hold agent/grants.py
allows (never the e-stop). Acting verbs need ``--why``. ``arm --claim ID`` arms through the TUI
after a visible countdown the operator cancels (the e-stop always refuses); ``run --sim-arm`` arms a verified sim
for one run. ``start COMPONENT`` and ``stop COMPONENT`` (claimed) start and stop a stack part agent.yaml ``start:``
lists, as its Status row does; ``reload config [NAME]`` re-reads the TUI's YAML, ``reload COMPONENT`` respawns one.
``run KEY --set K=V`` and ``set K=V`` (claimed) change a setting agent.yaml ``settings:`` lists, as the operator does.
``fence corners LAT,LON ... | on_exit alert|return | anchor --why`` sets fence_watchdog's course area through the TUI
(no claim: agents may at any time, D-20261101-23).
"""

import json
import math
import os
import socket
import sys
import time
from pathlib import Path

from mhseals_testing import data
from mhseals_testing.harness import operator_state
from mhseals_testing.agent import channel
from mhseals_testing.harness.registry import Registry, runtime_dir

FCU_ACTIONS = ('get', 'set', 'revert', 'restart', 'rates')
# acting verbs: each needs --why (owner decision 2026-11-01); stop takes one but never needs it, fcu only to write
WHY = ('claim', 'release', 'run', 'goal', 'route', 'fetch', 'start', 'arm', 'reload', 'set', 'fcu', 'fence')
VERBS = ('claim', 'release', 'status', 'follow', 'run', 'goal', 'route', 'start', 'stop', 'arm', 'reload', 'set', 'fcu',
         'fence', 'mark', 'note', 'ask', 'quality', 'fetch')
FENCE_ACTIONS = ('corners', 'on_exit', 'anchor')
STREAMS = ('run', 'goal')      # route sends goals
SECTIONS = ('authority', 'run', 'health', 'odometry', 'perception', 'nav2', 'agent')
ALWAYS = ('schema', 't_ns', 'host', 'profile', 'sim', 'session', 'owner', 'stale', 'age_s', 'operator')
DEFAULTS = {'request_s': 5.0, 'ask_s': 120.0, 'lease': {'idle_s': 600.0, 'motion_s': 10.0}}
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
        if name in WHY or name in ('stop', 'mark', 'quality'):
            parser.add_argument('--why', metavar='TEXT', help='why you act: the TUI agent panel shows it'
                                + ('' if name in ('stop', 'mark', 'quality') else ' (required)'))
        return parser
    verb('claim', 'ask the TUI for driving authority').add_argument('--reason', default='')
    verb('release', 'end a claim (--claim ID, the TUI cancels its act first), or release HOLD: a lease or watchdog '
         'hold once its fault cleared, an operator hold with a grant (agent_grants.yaml); never the e-stop',
         claim=False).add_argument('hold', nargs='?', metavar='HOLD')
    verb('status', 'one snapshot of what the TUI shows').add_argument(
        '--section', type=lambda s: s.split(','), default=[], metavar=','.join(SECTIONS))
    follow = verb('follow', 'stream session events and agent replies', claim=False)
    follow.add_argument('--until', metavar='PHASE[:FIELD=VAL]|owner:none')
    follow.add_argument('--timeout', type=float, metavar='S')
    run = verb('run', 'run a procedure through the TUI; run --list: every procedure, whether agents may run it, '
               'its settings and its gate (read-only, no claim)', claim=False)
    run.add_argument('procedure', nargs='?', metavar='KEY|NAME')
    run.add_argument('--list', action='store_true', help='list the procedures and their gate state; runs nothing')
    run.add_argument('--dry-run', action='store_true', help='evaluate the gates only')
    run.add_argument('--task', help='mission only: the task to rehearse (else the operator picks it)')
    run.add_argument('--sim-arm', action='store_true',
                     help='sim only: arm a verified sim for this run, disarm after (harness/sim_arm.py)')
    run.add_argument('--mission-profile', help="mission only: mhseals_nav's mission profile (qualifying, finals; "
                     "else the nav YAML's own); it sets the run budget")
    run.add_argument('--set', action='append', default=[], metavar='KEY=VALUE', dest='settings',
                     help="a setting agent.yaml settings: lists for this procedure (e.g. thrust_step=survey), "
                          'applied before the run starts; repeatable')
    setting = verb('set', "change settings agent.yaml settings: lists, as the operator's Settings change "
                   '(disarmed and idle)', claim=True)
    setting.add_argument('settings', nargs='+', metavar='KEY=VALUE')
    goal = verb('goal', 'send a map-frame goal through the TUI', claim=True)
    for name in ('x', 'y'):
        goal.add_argument(name, type=float)
    goal.add_argument('yaw', type=float, nargs='?')
    goal.add_argument('--dry-run', action='store_true', help='evaluate the gates only')
    route = verb('route', 'map-frame goals in order, each one goal through the TUI (its gates, Allow and lease); '
                 'the first that does not succeed ends the route', claim=True)
    route.add_argument('waypoints', nargs='+', metavar='X,Y[,YAW]')
    route.add_argument('--dry-run', action='store_true', help='evaluate the first goal\'s gates only')
    start = verb('start', 'start a stack component (agent.yaml start:) as its Status row does', claim=True)
    start.add_argument('component', metavar='COMPONENT', help='a component key: odometry, ekf, sensors, ...')
    fcu = verb('fcu', 'FCU parameters through the TUI: get PARAM..., rates [--window S] (read-only); '
               'set PARAM VALUE..., revert --to SNAPSHOT, restart (--claim, disarmed)', claim=False)
    fcu.add_argument('action', choices=FCU_ACTIONS)
    fcu.add_argument('params', nargs='*', metavar='PARAM [VALUE]')
    fcu.add_argument('--to', metavar='SNAPSHOT', help='revert: the snapshot file (fcu_<vessel>_<stamp>.param)')
    fcu.add_argument('--window', type=float, metavar='S', help='rates: seconds to measure (default agent.yaml)')
    fence = verb('fence', "fence_watchdog's course area through the TUI: corners LAT,LON LAT,LON ... (3 or more), "
                 'on_exit alert|return, anchor; validated, read back and kept in ~/.config/mhseals/fence.yaml')
    fence.add_argument('action', choices=FENCE_ACTIONS)
    fence.add_argument('values', nargs='*', metavar='LAT,LON|MODE')
    stop = verb('stop', 'Space: hold, neutral and disarm (mux hold over ROS if the TUI is silent); '
                'with COMPONENT and --claim, stop that stack component instead', claim=False)
    stop.add_argument('component', nargs='?', metavar='COMPONENT',
                      help='stop this stack component (needs --claim), not the boat')
    arm = verb('arm', 'arm through the TUI: a countdown banner the operator cancels; refused on e-stop', claim=True)
    arm.add_argument('--countdown', type=float, metavar='S',
                     help='seconds of banner (at least agent.yaml arm.countdown, the default)')
    reload = verb('reload', 'reload config [NAME]: re-read the TUI YAML; reload COMPONENT --claim ID: respawn one '
                  'stack part', claim=False)
    reload.add_argument('target', metavar='config|COMPONENT')
    reload.add_argument('name', nargs='?', metavar='NAME', help='config: one watched file (hardware, settings, ...)')
    mark = verb('mark', 'put an agent marker in the session (Labels tab, manifest); --tag one of config/annotate.yaml')
    mark.add_argument('note', nargs='?', default='')
    mark.add_argument('--tag', metavar='LABEL', help='wake, gust, contact, pushed, sensor_bump, fouling, good, ...')
    note = verb('note', 'a text line in the TUI agent log; --doing TEXT sets your "doing:" line there')
    note.add_argument('text', nargs='?', default='')
    note.add_argument('--doing', metavar='TEXT')
    quality = verb('quality', "a session's data quality, read-only: rates, gaps, commanded vs measured coverage of the "
                   "profile's command, odometry and sensor topics (default: the newest session here; run it on the "
                   'station after fetch: BOAT_AGENT_TARGET=local)')
    quality.add_argument('session', nargs='?', metavar='SESSION|DIR', help='a session id under sessions/ or a path')
    quality.add_argument('--last', type=float, metavar='S', help='only the last S seconds of the bags')
    quality.add_argument('--topics', type=lambda s: s.split(','), default=[], metavar='T1,T2',
                         help='more topics to rate')
    fetch = verb('fetch', 'on the station (BOAT_AGENT_TARGET names the boat): pull one session (default the newest) '
                 'over rsync into this data root, resumable and throttled (agent.yaml fetch.bwlimit_kbps); the open '
                 'bag segment is left out; read-only on the boat')
    fetch.add_argument('session', nargs='?', metavar='SESSION', help='a session id under sessions/')
    fetch.add_argument('--bwlimit', type=int, metavar='KBPS', help='rsync --bwlimit for this call')
    fetch.add_argument('--no-bags', action='store_true', help='only the reports, events and manifest, no bag/')
    fetch.add_argument('--to', metavar='DIR', help='station directory to fetch into (default <data root>/sessions)')
    fetch.add_argument('--where', action='store_true', help=argparse_suppress())
    ask = verb('ask', 'ask the operator a question')
    ask.add_argument('question')
    ask.add_argument('--options', type=lambda s: s.split(','), default=['yes', 'no'])


def argparse_suppress():
    import argparse
    return argparse.SUPPRESS


def config():
    from mhseals_testing.agent import config as agent_config
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

    def send(self, op, args=None, claim=None, dry=False, wait=None, why=None):
        """Append a req and wait for its final reply; returns the exit code."""
        tail = channel.Tail(self.path)
        tail.seek_end()
        req = channel.request(op, self.who, args, claim, dry, self.cfg['request_s'], path=self.path,
                              why=why or getattr(self.args, 'why', None))
        if self.owner() is None:
            return self.no_tui(req, 'start boat_test on the boat; agents act only through it')
        return self.wait(req, tail, wait)

    def no_tui(self, req, hint):
        """``no_owner`` with the last operator state: a declared battery swap or power cycle explains it."""
        record = operator_state.load()
        if operator_state.paused(record):
            hint = f'TUI not running: last {operator_state.text(record)}; wait for the operator (no retries)'
        return self.local(req, 'no_owner', hint, operator=operator_state.summary(record))

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
                return self.no_tui(req, 'the TUI quit; the act ended with it')
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
        hold = getattr(self.args, 'hold', None)
        if hold:
            return self.send('release_hold', {'hold': hold}, self.args.claim)
        if not self.args.claim:
            return self.bad('release --claim ID ends your claim; release HOLD --why "..." releases a hold')
        return self.send('release', claim=self.args.claim)

    def run(self):
        if getattr(self.args, 'list', False):
            self.emit(procedure_list(self.cfg, self.registry))
            return 0
        if not self.args.procedure or not self.args.claim:
            return self.bad('run KEY --claim ID (run --list names the procedures)')
        args = {'key': self.args.procedure}
        if getattr(self.args, 'task', None):
            args['task'] = self.args.task
        if getattr(self.args, 'sim_arm', False):
            args['sim_arm'] = True
        if getattr(self.args, 'mission_profile', None):
            args['profile'] = self.args.mission_profile
        if getattr(self.args, 'settings', None):
            args['settings'], hint = assignments(self.args.settings)
            if hint:
                return self.bad(hint)
        return self.send('run', args, self.args.claim, self.args.dry_run)

    def set(self):
        settings, hint = assignments(self.args.settings)
        return self.bad(hint) if hint else self.send('set', {'settings': settings}, self.args.claim)

    def bad(self, hint):
        self.emit({'ok': False, 'error': 'bad_args', 'hint': hint})
        return channel.EXIT_CODES['bad_args']

    def goal(self):
        args = {'x': self.args.x, 'y': self.args.y, 'yaw': self.args.yaw}
        if not all(math.isfinite(v) for v in args.values() if v is not None):   # JSON has no nan/inf
            self.emit({'ok': False, 'error': 'bad_args', 'hint': 'goal needs finite x, y (and yaw)'})
            return channel.EXIT_CODES['bad_args']
        return self.send('goal', args, self.args.claim, self.args.dry_run)

    def route(self):
        """Each waypoint as its own ``goal`` act, in order: nothing new on the TUI side, so every goal keeps its
        gates, the operator's Allow, the motion lease and Space; a goal that fails, is refused or is cancelled ends
        the route with its exit code. ``--dry-run`` checks the first goal's gates."""
        legs = []
        for word in self.args.waypoints:
            try:
                values = [float(v) for v in word.split(',')]
            except ValueError:
                values = []
            if len(values) not in (2, 3) or not all(math.isfinite(v) for v in values):
                return self.bad(f'route X,Y[,YAW] ...: {word!r} is not 2 or 3 finite numbers')
            legs.append(dict(zip(('x', 'y', 'yaw'), values + [None])))
        if self.args.dry_run:
            return self.send('goal', legs[0], self.args.claim, True)
        for index, leg in enumerate(legs, 1):
            self.emit({'route': index, 'of': len(legs), 'goal': leg})
            code = self.send('goal', leg, self.args.claim, why=f'{self.args.why} (route {index}/{len(legs)})')
            if code:
                self.emit({'ok': False, 'route': index, 'of': len(legs), 'error': 'route_ended', 'exit': code})
                return code
        self.emit({'ok': True, 'route': len(legs), 'of': len(legs), 'state': 'done'})
        return 0

    def start(self):
        return self.send('start', {'component': self.args.component}, self.args.claim)

    def fcu(self):
        """boat_agent fcu ACTION: the TUI does it over its MAVROS link (ui/feeds/agent.py do_fcu)."""
        action, words, claim = self.args.action, self.args.params, getattr(self.args, 'claim', None)

        def bad(hint):
            self.emit({'ok': False, 'error': 'bad_args', 'hint': hint})
            return channel.EXIT_CODES['bad_args']
        if action in ('set', 'revert', 'restart') and not claim:
            return bad(f'fcu {action} needs --claim ID (claim --reason "..." first)')
        if action == 'get':
            if not words:
                return bad('fcu get PARAM [PARAM ...]')
            return self.send('fcu_get', {'params': words}) if self.owner() is not None else self.fcu_saved(words)
        if action == 'rates':
            return self.send('fcu_rates', {'window_s': self.args.window})
        if action == 'restart':
            return self.send('fcu_restart', {}, claim)
        if action == 'revert':
            return self.send('fcu_revert', {'to': self.args.to}, claim) if self.args.to else bad('fcu revert --to SNAPSHOT')
        if not words or len(words) % 2:
            return bad('fcu set PARAM VALUE [PARAM VALUE ...]')
        try:
            changes = {name: float(value) for name, value in zip(words[::2], words[1::2])}
        except ValueError:
            return bad('fcu set: every VALUE must be a number')
        return self.send('fcu_set', {'changes': changes}, claim)

    def fcu_saved(self, names):
        """fcu get with no TUI controller: the newest snapshot in agent.yaml ``fcu.store``, marked as such (owner
        decision 2026-11-01: reads work without a controller)."""
        from mhseals_testing.agent.fcu import Store
        store = Store((self.cfg.get('fcu') or {}).get('store') or '~/.claude/plans/boat')
        found = sorted(store.dir.glob('fcu_*.param'), key=lambda p: p.stat().st_mtime) if store.dir.is_dir() else []
        if not found:
            self.emit({'ok': False, 'error': 'no_owner', 'hint': 'no TUI and no FCU snapshot yet: boat_tui up'})
            return channel.EXIT_CODES['no_owner']
        values = store.read_snapshot(found[-1])
        self.emit({'ok': True, 'source': 'snapshot', 'snapshot': found[-1].name, 'stale': True,
                   'params': {n: values.get(n) for n in names},
                   'hint': 'no TUI: values from the newest snapshot, not the FCU; boat_tui up for live values'})
        return 0

    def fence(self):
        from mhseals_testing.harness import course_area
        action, words = self.args.action, self.args.values
        if action == 'corners':
            corners, hint = course_area.parse(words)
            hint = hint or course_area.problem(corners)
            if hint:
                return self.bad(f'fence corners LAT,LON LAT,LON LAT,LON ...: {hint}')
            return self.send('fence', {'action': action, 'corners': [f'{a!r},{b!r}' for a, b in corners]})
        if action == 'on_exit':
            if len(words) != 1 or words[0] not in course_area.ON_EXIT:
                return self.bad('fence on_exit alert|return')
            return self.send('fence', {'action': action, 'mode': words[0]})
        return self.bad('fence anchor takes no values') if words else self.send('fence', {'action': action})

    def arm(self):
        countdown = getattr(self.args, 'countdown', None)
        if countdown is not None and not (math.isfinite(countdown) and 0 <= countdown <= 60):
            return self.bad('arm --countdown S: 0 to 60 seconds')
        return self.send('arm', {'countdown': countdown}, self.args.claim,
                         wait=(countdown or 60.0) + self.cfg['request_s'] + 60.0)

    def reload(self):
        target, claim = self.args.target, getattr(self.args, 'claim', None)
        if target == 'config':
            return self.send('reload', {'config': self.args.name or ''})
        if not claim:
            return self.bad(f'reload {target} respawns a stack part: it needs --claim ID')
        return self.send('reload', {'component': target}, claim, wait=60.0)

    def stop_part(self):
        if not getattr(self.args, 'claim', None):
            self.emit({'ok': False, 'error': 'bad_args',
                       'hint': f'stop {self.args.component} needs --claim ID '
                               '(plain stop, no COMPONENT, stops the boat)'})
            return channel.EXIT_CODES['bad_args']
        return self.send('stop_part', {'component': self.args.component}, self.args.claim)

    def ask(self):
        args = {'question': self.args.question, 'options': self.args.options}
        return self.send('ask', args, wait=self.cfg['ask_s'] + self.cfg['request_s'] + SLACK_S)

    def note(self):
        doing = getattr(self.args, 'doing', None)
        if not (doing or self.args.text):
            return self.bad('note "TEXT" or note --doing "TEXT"')
        self.emit(channel.note(self.who, 'doing' if doing else 'text', doing or self.args.text, path=self.path))
        return 0

    def mark(self):
        tag, why = getattr(self.args, 'tag', None) or '', getattr(self.args, 'why', None) or ''
        if self.owner() is not None:
            return self.send('mark', {'note': self.args.note, 'tag': tag})
        session = last_session(self.registry)
        if session is None:
            self.emit({'ok': False, 'error': 'no_session', 'hint': 'no session recorded yet'})
            return 2
        events = data.read_jsonl(session / 'events.jsonl')
        record = {'t_ns': time.time_ns(), 'phase': 'marker', 'kind': 'agent', 'who': self.who,
                  'id': max((e['id'] for e in events if e.get('phase') == 'marker'
                             and isinstance(e.get('id'), int)), default=0) + 1,
                  'note': self.args.note, 'tag': tag, 'why': why, 'rec': False}
        data.append_jsonl(session / 'events.jsonl', record)
        self.emit({'ok': True, 'session': str(session), 'marker': record})
        return 0

    def stop(self):
        """Ask the TUI (Space); with no reply in STOP_WAIT_S, publish the mux hold over ROS too.
        With COMPONENT, stop that stack component instead (``stop_part``): never the ROS backup."""
        if getattr(self.args, 'component', None):
            return self.stop_part()
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
            from mhseals_testing.agent.ros_stop import hold as backup
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

    def quality(self):
        from mhseals_testing.harness import quality
        name = self.args.session
        try:
            session = (Path(name) if name and Path(name).is_dir() else data.path('sessions', name) if name
                       else last_session(self.registry))
        except ValueError as error:
            return self.bad(f'quality SESSION: {error}')
        if session is None or not Path(session).is_dir():
            self.emit({'ok': False, 'error': 'no_session', 'hint': f'no session {name or "recorded yet"} here'})
            return 2
        last = self.args.last
        if last is not None and not (math.isfinite(last) and last > 0):
            return self.bad('quality --last S: a positive number of seconds')
        try:
            self.emit(quality.summarize(session, self.args.topics, last))
        except ImportError as error:        # no rosbag2_py: the dev container has it
            self.emit({'ok': False, 'error': 'no_reader', 'hint': f'{error}: run it in the dev container'})
            return 2
        return 0

    def fetch(self):
        """The boat's half of ``fetch`` (``--where``, relay.py's first hop): which session, where it lies below
        the workspace, its open segment and the rate, with a note in the TUI agent log. Called on the station
        itself it says where fetch runs."""
        if not self.args.where:
            return self.bad('fetch runs on the station with BOAT_AGENT_TARGET naming the boat (tools/boat_agent '
                            'relays it); here the session is already local: quality SESSION')
        name = self.args.session
        try:
            session = data.path('sessions', name) if name else last_session(self.registry)
        except ValueError as error:
            return self.bad(f'fetch SESSION: {error}')
        if session is None or not session.is_dir():
            self.emit({'ok': False, 'error': 'no_session', 'hint': f'no session {name or "recorded yet"} here'})
            return 2
        session = session.resolve()
        home = data.workspace()
        if not session.is_relative_to(home):
            self.emit({'ok': False, 'error': 'refused', 'hint': f'{session} is outside the workspace {home}'})
            return 2
        live = (self.owner() is not None and str(session) == str(Path((self.registry.state() or {})
                                                                        .get('session') or '/').resolve()))
        segments = sorted(p.name for p in (session / 'bag').glob('[0-9][0-9]*') if p.is_dir())
        rate = self.args.bwlimit or int((self.cfg.get('fetch') or {}).get('bwlimit_kbps') or 0)
        if not rate:
            from mhseals_testing.agent.config import FETCH_KBPS
            rate = FETCH_KBPS
        if rate <= 0:
            return self.bad('fetch --bwlimit KBPS: a positive rate')
        doc = {'ok': True, 'id': session.name, 'rel': session.relative_to(home).as_posix(),
               'open_segments': segments[-1:] if live else [], 'live': live, 'bwlimit_kbps': rate}
        try:
            channel.note(self.who, 'text', f"fetch {session.name} to the station: {self.args.why}", path=self.path)
        except OSError:
            pass                    # the log line is advisory; the pull reads only
        self.emit(doc)
        return 0

    def status(self):
        self.emit(status(self.registry, self.args.section))
        return 0

    def follow(self):
        return follow(self, self.args.until, self.args.timeout, self.args.claim)


def procedure_list(cfg, registry):
    """``run --list``: each procedure (key, agent name, title, needs), whether agent.yaml ``run:`` lets agents run
    it, the settings ``run --set`` takes, and its gate from a fresh TUI status (``None``: clear; text: why not;
    ``unknown``: no fresh TUI). Read-only."""
    from mhseals_testing.agent import config as agent_config
    try:
        from mhseals_testing.procedures import PROCEDURES
        from mhseals_testing.ui.agent_acts import NAMES
    except ImportError as error:        # no mhseals_hardware here: the agent names alone
        PROCEDURES, NAMES, why = (), {}, repr(error)
    else:
        why = ''
    allowed = list(cfg.get('run') or [])
    doc = status(registry, ('health',))
    fresh = doc.get('stale') is False
    gates = ((doc.get('health') or {}).get('gates') or {}) if fresh else {}
    names = {key: name for name, key in NAMES.items()}
    rows = []
    for p in PROCEDURES:
        name = names.get(p.key)
        agent = name in allowed
        rows.append({'key': p.key, 'name': name, 'title': p.title, 'group': p.group, 'needs': sorted(p.needs),
                     'agent': agent, 'settings': agent_config.settable(cfg, name) if agent else [],
                     'gate': (gates.get(name) if name in gates else 'unknown') if agent else 'operator only'})
    out = {'ok': True, 'procedures': rows, 'agent_run': allowed, 'tui': 'fresh' if fresh else 'not running or stale',
           'hint': 'gate null is clear; run KEY --dry-run --claim ID re-checks one'}
    if why:
        out['warning'] = f'procedures not importable here ({why}): agent_run only'
    return out


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
               'session': data.read_json(session / 'session.json') if session else None,
               'tui': 'not running', 'operator': operator_state.summary(operator_state.load())}
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
        agent.emit({'tui': 'not running', 'operator': operator_state.summary(operator_state.load())})
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
            agent.emit({'ok': until == 'owner:none', 'error': '' if until == 'owner:none' else 'no_owner',
                        'operator': operator_state.summary(operator_state.load())})
            return 0 if until == 'owner:none' else 4
        if timeout is not None and now - start > timeout:
            agent.emit({'ok': False, 'error': 'timeout', 'until': until})
            return 6
        if claim and now - renewed >= every:
            channel.renew(agent.who, claim, None, path=agent.path)
            renewed = now
        agent.sleep(POLL_S)


def assignments(words):
    """(``{key: value}``, None) from ``KEY=VALUE`` words, or (None, hint)."""
    out = {}
    for word in words:
        key, separator, value = word.partition('=')
        if not separator or not key.strip() or key.strip() in out:
            return None, f'{word!r}: expected KEY=VALUE, each key once'
        out[key.strip()] = value.strip()
    return out, None


def needs_why(args):
    """An acting verb with no ``--why``: fcu only to write, stop never."""
    if args.command not in WHY or (getattr(args, 'why', None) or '').strip():
        return False
    if args.command == 'run' and getattr(args, 'list', False):      # run --list reads only
        return False
    return args.command != 'fcu' or getattr(args, 'action', None) in ('set', 'revert', 'restart')


SHARED_WHO = {'', 'agent', 'agent-0000'}     # the old defaults: every agent that copied them was one principal


def who_of(args):
    """``(who, None)``, or ``(None, hint)`` when the verb needs an id of its own and has none. ``status`` reads only
    and needs none; ``stop`` always works: without an id it is ``agent@HOST:PPID``, so the operator still sees
    that an agent (not they) stopped."""
    who = (args.who or '').strip()
    if who not in SHARED_WHO:
        return who, None
    if args.command == 'stop' and not getattr(args, 'component', None):
        return f'agent@{socket.gethostname().split(".")[0]}:{os.getppid()}', None
    if args.command == 'status':
        return who or 'agent', None
    return None, ('pass --who ID (or export BOAT_AGENT=ID) with an id of your own, such as the one '
                  '`python3 .agents/hook.py --playbook` prints; "agent" and "agent-0000" are shared, so refused')


def main(args, **seams):
    """Run verb ``args.command``; returns its exit code."""
    args.who, hint = who_of(args)
    if not hint and needs_why(args):
        hint = f'{args.command} acts: say why with --why "..." (the TUI agent panel shows it)'
    if hint:
        print(json.dumps({'ok': False, 'error': 'bad_args', 'hint': hint}),
              file=seams.get('out') or sys.stdout, flush=True)
        return channel.EXIT_CODES['bad_args']
    return getattr(Agent(args, **seams), args.command)()
