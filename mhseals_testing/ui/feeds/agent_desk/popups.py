"""The desk's popups: the Allow/Refuse confirm before a motion act, an agent's ask, and the operator popup a run
waits on (only the operator answers it; no verb answers a popup)."""

from mhseals_testing.ui import agent_acts


def operator_popup(app):
    """Return the popup a run waits on (e.g. the procedure's "Clear the area and press Enter"), or None.

    Only the operator answers it; an agent reports it and waits (no verb answers a popup).
    """
    modal = getattr(app, 'modal', None)
    if not isinstance(modal, dict):
        return None
    title, message = modal.get('title') or '', modal.get('message') or ''
    return f'{title}: {message}' if title and message else (title or message or None)


class PopupsMixin:
    """The confirm and the ask (agent_acts.choose): each keeps Space and E and ends on the act's cancel cause."""

    def confirm(self, act, title):
        """Under agent_motion confirm, Allow from the human (Refuse is focused); returns True to go on."""
        if self.app.settings.agent_motion != 'confirm':
            return True
        self.set_state(act, 'asking')
        choice, why = agent_acts.choose(self.app, title, ('Allow', 'Refuse'), self.config.get()['confirm_s'],
                                        self.watch, clock=self.clock,
                                        message=f"Agent {act['who']} · claim {act['claim']}")
        if choice == 'Allow':
            return True
        cause = act['cause'] or why or 'refused'      # the Refuse button names no cause
        self.finish(act, False, self.error_of(cause) if act['cause'] else 'refused',
                    hint=self.hint_of(cause, f'operator: {cause}'))
        return False

    def act_ask(self, act):
        args = act['req']['args']
        options = [str(o) for o in (args.get('options') or ('yes', 'no'))][:6]
        self.set_state(act, 'asking')
        choice, why = agent_acts.choose(self.app, f"Agent {act['who']} asks", options,
                                        self.config.get()['ask_s'], self.watch, clock=self.clock,
                                        message=str(args.get('question', ''))[:300], danger=False)
        if choice is not None:
            return self.finish(act, True, result={'choice': choice})
        self.finish(act, False, 'timeout' if why == 'timeout' else 'cancelled', result={'cause': why})
        return False
