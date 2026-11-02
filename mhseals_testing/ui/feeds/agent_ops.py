"""The desk's quick agent verbs: arm and reload (owner decision 2026-11-01).

Each runs on the owner's key loop as ``boat_agent fcu`` does, shows in the footer and the agent log, and
journals an event naming the agent. ``arm`` raises a countdown banner the operator cancels (Esc, Enter on
Cancel, Space stops); the e-stop always refuses. ``reload config [NAME]`` re-reads the TUI's watched YAML
through the same path a hand edit takes; ``reload COMPONENT`` stops and starts one stack part under the
``start``/``stop`` rules. ``set KEY=VALUE`` is the desk's own (ui/feeds/agent.py ``do_set``).
"""

import time

from mhseals_testing.harness import operator_state
from mhseals_testing.harness.components import ComponentError
from mhseals_testing.ui import agent_acts, parts, theme

COUNTDOWN = 5.0           # arm banner seconds when agent.yaml has no arm.countdown
RESPAWN_WAIT = 15.0       # reload COMPONENT: how long its stop may take before the start


def claimed(desk, req):
    """(error, hint) unless ``req`` carries its agent's active claim and agents are allowed, else None."""
    if desk.app.settings.agents == 'block':
        return 'blocked', 'operator must set Agents to allow'
    if not req.get('claim'):
        return 'unknown_claim', f'{req["op"]} needs your claim: claim --reason "..." --who {req["who"]}, then --claim ID'
    found = desk.claim_state(req['claim'], req['who'])
    if found:
        return found, 'claim again' if found != 'not_holder' else 'use your own claim'
    return None


def estop(app):
    """Why the e-stop forbids arming ('' when it does not): the driver's latched cut."""
    status = getattr(getattr(app, 'ros', None), 'driver_status', None)
    cut = status.get('estop') if isinstance(status, dict) else None
    if isinstance(cut, dict) and cut.get('latched'):
        return f"e-stop engaged (cut by {cut.get('cause') or '?'}): clear it on the boat first"
    return ''


def countdown_of(desk, req):
    """The banner's seconds: agent.yaml arm.countdown (the operator's), which an agent may lengthen, never shorten."""
    floor = float((desk.config.get().get('arm') or {}).get('countdown', COUNTDOWN))
    value = req['args'].get('countdown')
    value = floor if value is None else float(value)
    if not 0 <= value <= 60:
        raise ValueError('countdown is 0 to 60 seconds')
    return max(value, floor)


def do_arm(desk, req):
    """boat_agent arm --claim ID: countdown banner, then the arm panel's path without its popup."""
    app, who, claim = desk.app, req['who'], req.get('claim')
    found = claimed(desk, req)
    if found:
        return desk.reply(req, False, found[0], hint=found[1])
    if app.armed:
        return desk.reply(req, True, result={'armed': True, 'already': True}, label='arm (already armed)')
    if app.modal is not None or app.running or getattr(app, 'manual', None) or desk.act is not None or desk.busy:
        return desk.reply(req, False, 'busy_ui', hint='a popup is open or something runs; arm after it')
    why = estop(app)
    if why:
        return desk.reply(req, False, 'refused', hint=why)
    try:
        seconds = countdown_of(desk, req)
    except (TypeError, ValueError) as error:
        return desk.reply(req, False, 'bad_args', hint=f'arm --countdown S: {error}')
    app.emit('agent_arm', who=who, claim=claim, state='countdown', countdown=seconds)
    desk.reply(req, True, state='started', result={'countdown': seconds})

    def cause():
        state = operator_state.current()
        return (desk.claim_state(claim, who) or estop(app) or desk.operator_setting()
                or (operator_state.refusal(state) if operator_state.paused(state) else None))
    if seconds > 0:
        app.notice = f'Agent {who} arms in {seconds:g} s{theme.SEPARATOR}Esc cancels'
        choice, why = agent_acts.choose(
            app, f'Agent {who} arms {app.profile.name}', ('Cancel',), seconds, cause,
            message=f'Arming in {seconds:g} s for agent {who} · claim {claim}. Esc or Enter cancels; Space stops.',
            stop_refuses=True)
        if why != 'timeout':
            reason = 'operator cancelled' if choice or why == 'refused' else why
            app.notice = f'Agent {who} arm cancelled: {reason}'
            app.emit('agent_arm', who=who, claim=claim, state='cancelled', reason=reason)
            return desk.reply(req, False, 'cancelled', hint=reason, label='arm')
    why = cause()
    if why:
        return desk.reply(req, False, 'refused', hint=why, label='arm')
    try:
        why = app.agent_arm(who, [], sim=False)
    except BaseException as error:      # whatever escapes arming: what may be armed is disarmed
        why = f'arming failed: {error!r}'
        if app.armed:
            app.disarm(f'agent arm undone: {why}', by='agent')
        if not isinstance(error, Exception):
            raise
    ok = not why and app.armed
    app.emit('agent_arm', who=who, claim=claim, state='armed' if ok else 'refused', reason=why or '')
    desk.reply(req, ok, '' if ok else 'refused', result={'armed': bool(app.armed)}, hint=why or '', label='arm')


def do_reload(desk, req):
    """boat_agent reload config [NAME] (no claim) or reload COMPONENT --claim ID."""
    app, args = desk.app, req['args']
    if 'config' in args:
        watches = app.watcher.watches
        name = str(args.get('config') or '')
        if name and name not in watches:
            return desk.reply(req, False, 'bad_args', hint=f"watched files: {', '.join(watches)}")
        done = []
        for each in [name] if name else list(watches):
            change = app.announce(app.watcher.reload(each))
            done.append({'name': each, 'ok': change.ok, 'error': change.error or ''})
        failed = [d for d in done if not d['ok']]
        app.notice = (f"Agent {req['who']} reloaded {name or 'config'}"
                      + (f": {failed[0]['name']} rejected ({failed[0]['error']})" if failed else ''))
        app.emit('agent_reload', who=req['who'], target='config', files=done)
        return desk.reply(req, not failed, 'refused' if failed else '', result={'files': done},
                          hint=failed[0]['error'] if failed else '', label=f"reload {name or 'config'}")
    key = args.get('component')
    for verb in ('stop', 'start'):
        found = desk.part_refusal(dict(req, args={'component': key}), verb)
        if found:
            return desk.reply(req, False, found[0], hint=found[1])
    stack, who = app.stack, req['who']
    # A respawn starts only ``key`` again: a running part that needs it would stay down, so it is refused.
    dependants = [stack.items[name].title for name in stack._dependants(key) if stack.running(name)]
    if dependants:
        return desk.reply(req, False, 'refused', label=f'reload {key}',
                          hint=f"{', '.join(dependants)} need {stack.items[key].title}: stop them first, or ask the "
                               'operator to restart the stack')
    desk.reply(req, True, state='started')
    try:
        stopped = parts.stop(app, key, who=who) if stack.running(key) else []

        def gone():
            end = time.monotonic() + RESPAWN_WAIT
            while stack.running(key) and time.monotonic() < end:
                time.sleep(0.1)
            return not stack.running(key)
        if stopped and not agent_acts.in_background(app, f'Agent {who}: respawning {key}', gone):
            return desk.reply(req, False, 'refused', hint=f'{key} did not stop in {RESPAWN_WAIT:g} s',
                              label=f'reload {key}')
        parts.start(app, key, who=who)
    except ComponentError as error:
        return desk.reply(req, False, 'refused', hint=str(error), label=f'reload {key}')
    app.emit('agent_reload', who=who, target='component', component=key, stopped=stopped)
    desk.reply(req, True, result={'component': key, 'stopped': stopped}, label=f'reload {key}')


HANDLERS = {'arm': do_arm, 'reload': do_reload}
