"""The desk's channel: it reads the agent channel (at most 64 lines a tick, the read position persisted), routes each
request to its handler, answers it, and keeps the notes, the notices and the status file."""

import time

from mhseals_testing.agent import channel as agent_channel
from mhseals_testing.harness import operator_state
from mhseals_testing.ui import agent_acts
from mhseals_testing.ui.feeds import agent_ops
from mhseals_testing.ui.feeds.agent_desk.common import CHANGING, FCU_OPS, MOTION, STATUS_S, URGENT


class ChannelMixin:
    """Read and answer: poll, the on_<kind> handlers, reply, the persisted position, notes and status."""

    def poll(self, urgent=False):
        records, errors = self.tail.read()
        for error in errors:
            if error not in self.errors:     # one notice per distinct error
                self.errors.add(error)
                self.app.notice = f'agent channel: {error}'
        records, self.deferred = self.deferred + records, []
        dry = False
        for record in records:
            if urgent and not (record['kind'] in ('renew', 'reply')
                               or (record['kind'] == 'req' and record.get('op') in URGENT)):
                self.deferred.append(record)
                continue
            if self.working and record['kind'] == 'req' and record.get('op') not in URGENT:
                self.deferred.append(record)     # a handler still works (its popup ticks): after it, if in time
                continue
            if record['kind'] == 'req' and record.get('dry') and record.get('op') in MOTION \
                    and record.get('id') not in self.answered:
                if dry or self.dry is not None:   # one at a time: a goal's waits on Nav2 run off the tick
                    self.deferred.append(record)
                    continue
                dry = True
            try:
                getattr(self, 'on_' + record['kind'])(record)
            except (KeyError, TypeError, ValueError, AttributeError, ArithmeticError) as error:   # bad values
                if record['kind'] == 'req':
                    self.reply(record, False, 'bad_args', hint=str(error))
                else:
                    self.app.notice = f"agent channel: bad {record['kind']}: {error}"

    def persist(self):
        position = self.tail.position()
        if self.registry is not None and position != self.saved:
            self.saved = dict(position)
            self.registry.update(lambda entry: entry.update(agent=position))

    def reply(self, req, ok=True, error='', state='done', result=None, hint='', label=None):
        hint = agent_acts.agent_text(hint)
        agent_channel.reply(req['id'], req['who'], ok=ok, state=state, error=error, result=result,
                            hint=hint, path=self.path)
        if state == 'done':
            label = label or (self.act['label'] if self.act and self.act['id'] == req['id'] else req['op'])
            self.recent.append({'id': req['id'], 'op': req['op'], 'who': req['who'], 'label': label,
                                't_ns': time.time_ns(), 'ok': bool(ok), 'error': error, 'hint': hint,
                                'why': str(req.get('why') or '')})
        if state != 'progress':
            self.app.emit('agent_act', id=req['id'], who=req['who'], op=req['op'], state=state, error=error)

    def on_reply(self, record):
        pass

    def on_note(self, record):
        self.touch(record['who'])
        if record['phase'] == 'doing':        # note --doing: the agent's one "doing:" line in the panel
            self.doing[record['who']] = {'text': str(record.get('cmd') or ''), 't_ns': record.get('t_ns')}
            return
        row = {k: record.get(k) for k in ('t_ns', 'who', 'phase', 'tool_id', 'cmd', 'exit', 'dur_s', 'target',
                                          'agent', 'use')}
        self.notes.append(row)
        if record['phase'] != 'pre':          # one journal line per tool call
            self.app.emit('agent_note', who=row['who'], stage=row['phase'], cmd=row['cmd'], exit=row['exit'],
                          dur_s=row['dur_s'])     # 'phase' is the journal's own key

    def on_renew(self, record):
        self.touch(record['who'], record['claim'])
        act = self.act
        if act and act['id'] == record['act'] and act['claim'] == record['claim'] and act['who'] == record['who']:
            act['renewed'] = self.clock()
        for req in self.queue:
            if req['id'] == record['act'] and req.get('claim') == record['claim'] and req['who'] == record['who']:
                req['renewed'] = self.clock()

    def on_req(self, req):
        if req['id'] in self.answered:
            return
        self.answered.add(req['id'])
        args = req.get('args') if isinstance(req.get('args'), dict) else {}
        args = {k: agent_channel.redact(v, agent_channel.TEXT_MAX)       # any writer: the journal stays clean
                if k in agent_channel.TEXT_ARGS and isinstance(v, str) else v for k, v in args.items()}
        # never later than its own stamp allows: a replay keeps no deadline it was not sent with
        deadline = min(req['deadline_ns'], req['t_ns'] + int(self.config.get()['request_s'] * 1e9))
        req = dict(req, args=args, deadline_ns=deadline)
        self.app.emit('agent_req', id=req['id'], who=req['who'], op=req['op'], args=args, dry=bool(req.get('dry')),
                      **({'why': req['why']} if req.get('why') else {}))
        self.touch(req['who'], req.get('claim'))
        if req['op'] in MOTION and req['t_ns'] < self.started_ns:
            return self.reply(req, False, 'expired', hint='sent before this TUI started; nothing was done')
        if time.time_ns() > req['deadline_ns']:
            return self.reply(req, False, 'expired', hint='request deadline passed; nothing was done')
        state = operator_state.current()
        if req['op'] in CHANGING and operator_state.paused(state):
            return self.reply(req, False, 'paused', hint=operator_state.refusal(state),
                              result={'operator': operator_state.summary(state)})
        handler = {'claim': self.do_claim, 'release': self.do_release, 'stop': self.do_stop, 'revoke': self.do_revoke,
                   'mark': self.do_mark, 'run': self.enqueue, 'goal': self.enqueue,
                   'ask': self.enqueue, 'start': self.do_part, 'stop_part': self.do_part, 'set': self.do_set,
                   **dict.fromkeys(FCU_OPS, self.do_fcu),
                   **{op: (lambda r, f=f: f(self, r)) for op, f in agent_ops.HANDLERS.items()}}.get(req['op'])
        if handler is None:
            return self.reply(req, False, 'bad_op', hint=f"unknown op {req['op']!r}")
        self.working = req['op']
        try:
            handler(req)
        finally:
            self.working = None

    def do_mark(self, req):
        number = self.app.next_id('marker')
        self.app.emit('marker', kind='agent', id=number, who=req['who'], note=str(req['args'].get('note', '')),
                      tag=str(req['args'].get('tag') or ''), why=req.get('why') or '', t_ns=time.time_ns())
        self.reply(req, True, result={'marker': number})

    def follow(self):
        """Activity switches to the agent view whenever an agent claim turns active."""
        holder = self.arbiter.table.holder() if self.arbiter is not None else None
        claim = holder['id'] if holder and holder.get('owner') == 'agent' else None
        if claim and claim != self.active_claim:
            self.app.activity = 'agent'
        self.active_claim = claim
        notice = str(self.app.notice or '')
        if notice and (not self.notices or self.notices[-1] != notice):
            self.notices.append(notice)

    def write_status(self):
        if self.status is None or (self.status_at is not None and self.clock() - self.status_at < STATUS_S):
            return
        self.status_at = self.clock()
        self.status(self.app, self)
