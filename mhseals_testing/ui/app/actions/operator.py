"""Operator stations: who drives, handovers and the operator menu."""

import os
import socket

from mhseals_hardware.menu import Cancelled
from mhseals_testing.harness import control, operator_state
from mhseals_testing.ui import feeds as feeds_module, theme
from mhseals_testing.ui import input as key_input


class Operator:
    """Operator stations, mixed into App."""

    def client_name(self):
        """This client as the tracker names it (``boat-v1@host``)."""
        return control.name(dict(control.identity(), host=socket.gethostname(), pid=os.getpid()))

    def operator_start(self):
        """The owner TUI's start: a battery swap or power cycle declared on an earlier boot is reported once."""
        record = operator_state.start()
        after = record.get('restarted_after')
        if after:
            self.notice = (f"Restarted after {operator_state.LABELS.get(after.get('state'), '?')} at "
                           f"{operator_state.clock(after.get('since_ns'))}: agents re-check the stack, FCU and RTK")
            self.emit('operator_restart', restarted_after=after, operator=operator_state.summary(record))
        self.operator_sync(record)

    def set_operator_state(self, state, eta_s=None, reason='', by='operator', notice=True):
        """Declare ``state``: saved (it survives a reboot), journaled, agents paused or resumed."""
        record = operator_state.save(operator_state.change(state, eta_s, reason, by, prev=operator_state.load()))
        self.emit('operator_state', state=state, operator=operator_state.summary(record))
        if notice:
            self.notice = 'O' + operator_state.text(record)[1:]
        self.operator_sync(record)
        return record

    def operator_sync(self, record):
        """Tell the arbiter (leases suspend or resume, never a hold) and the desk (an act in flight ends)."""
        arbiter, desk = getattr(self, 'arbiter', None), getattr(self, 'desk', None)
        if operator_state.paused(record):
            if arbiter is not None:
                arbiter.pause(record)
            if desk is not None:
                desk.operator_paused()
        elif arbiter is not None:
            for claim in arbiter.resume():
                self.notice = (f"Agent {claim['who']} {claim['id']} ended: not heard from while paused "
                               f"(no hold){theme.SEPARATOR}it claims again")

    def operator_menu(self):
        """S: declare manual, battery swap, power cycle (optional ETA) or normal."""
        if self.review or self.watching:
            self.notice = 'Review: no operator state' if self.review else 'viewer: O takes control'
            return
        now = operator_state.load()
        options = ('Normal', 'Manual', 'Battery swap', 'Power cycle', 'Cancel')
        rows = [('Now', operator_state.text(now)),
                ('Normal', 'agents resume; a claim kept alive resumes, others end cleanly'),
                ('Manual', 'you drive or reposition: agent verbs paused, leases suspended'),
                ('Battery swap', 'the boat goes down; the next TUI reports the restart'),
                ('Power cycle', 'the same, for a restart')]
        try:
            picked = self.choose('Operator state', options, {'n': 0, 'm': 1, 'b': 2, 'p': 3, 'c': 4}, rows=rows,
                                 default=0 if operator_state.paused(now) else 1, width=72)
            if picked in (None, 'Cancel'):
                return
            state = picked.lower().replace(' ', '_')
            eta_s = None
            if state in operator_state.DOWN:
                etas = ('No ETA', '5 min', '10 min', '15 min', '30 min')
                eta = self.choose('ETA', etas, {'0': 0, '5': 1, '1': 2, '2': 3, '3': 4}, default=2)
                eta_s = None if eta in (None, 'No ETA') else float(eta.split()[0]) * 60
        except Cancelled:
            return
        self.set_operator_state(state, eta_s)

    def take_control(self):
        """O: a viewer takes control after a confirm; it restarts as the owner (``--take``, same pid and
        terminal). The owner stops at neutral, disarms and hands its children over; its agent claim ends."""
        if not self.watching:
            self.notice = 'You control this session' if not self.review else 'Review: nothing to control'
            return
        tracked = feeds_module.merged(self.feeds).get('control_tracker') or {}
        ctl = tracked.get('controller') or {}
        holder = (f"agent {ctl.get('who')} {ctl.get('claim')} via {ctl.get('name')}" if ctl.get('kind') == 'agent'
                  else f"{ctl.get('name')} (agent claim unknown)" if ctl.get('kind') == 'unknown'
                  else ctl.get('name') or f'pid {self.watching}')
        rows = [('Controller', holder), ('Boat', 'stops at neutral and disarms; you arm it yourself'),
                ('Agent claim', 'revoked (the agent claims again only if you allow it)'),
                ('Children', 'keep running; you adopt them'), ('Old client', 'becomes a viewer, told who took over')]
        try:
            if key_input.by_hold('take'):
                if not self.hold_confirm('Take control', 'take control', rows=rows, key='O', width=72):
                    return
            elif self.choose('Take control', ('Cancel', 'Take control'), {'n': 0, 'y': 1}, rows=rows,
                             danger=True, width=72) != 'Take control':
                return
        except Cancelled:
            return
        self.restart_as = 'take'
        self.quit = True

    def close_handover(self):
        """Leave the children running, recorded for the successor, and give up the seat."""
        children = self.processes.records()
        handover = (self.registry.state().get('handover') or {}) if self.registry else {}
        self.handed_by = handover.get('by')
        if handover.get('revoke'):       # a human takeover: no agent claim passes to the next controller
            arbiter = getattr(self, 'arbiter', None)
            holder = arbiter.table.holder() if arbiter is not None else None
            if holder is not None and holder.get('owner') == 'agent':
                arbiter.revoke(holder['id'])
        self.ros.emit('handover', to=handover.get('to'))
        self.export()
        if self.registry is not None:
            self.registry.set_children(children)
        self.processes.release(children)
        if self.registry is not None:
            self.registry.release(os.getpid())
        self.handed_to = handover.get('to')
        return f"{', '.join(children) or 'no children'} left running for pid {self.handed_to}"
