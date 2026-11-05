"""The desk's quick agent verbs: arm, reload, release HOLD and fence (owner decisions 2026-11-01).

Each runs on the owner's key loop as ``boat_agent fcu`` does, shows in the footer and the agent log, and
journals an event naming the agent. ``arm`` raises a countdown banner the operator cancels (Esc, Enter on
Cancel, Space stops); the e-stop always refuses. ``reload config [NAME]`` re-reads the TUI's watched YAML
through the same path a hand edit takes; ``reload COMPONENT`` stops and starts one stack part under the
``start``/``stop`` rules. ``set KEY=VALUE`` is the desk's own (ui/feeds/agent_desk/ ``do_set``).
"""

import time

from mhseals_hardware import hold_store

from mhseals_testing.harness import operator_state
from mhseals_testing.agent import config as agent_config, grants as agent_grants
from mhseals_testing.harness.components import ComponentError
from mhseals_testing.ros.arming import muxed
from mhseals_testing.runner import STALE_MUX_S
from mhseals_testing.ui import agent_acts, parts, theme
from mhseals_testing.ui.feeds import authority as authority_feed

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
    floor = float(desk.config.get()['arm']['countdown'])
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
    if app.modal is not None or app.running or app.services.manual.active or desk.act is not None or desk.busy:
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

    stops = desk.stops

    def cause():
        state = operator_state.current()
        stopped = desk.stops != stops         # a stop during the countdown is a stop: never arm after it
        return ((f'stopped by agent {desk.stopped_by}' if stopped else None)
                or desk.claim_state(claim, who) or estop(app) or desk.operator_setting()
                or (operator_state.refusal(state) if operator_state.paused(state) else None))
    if seconds > 0:
        app.say(f'Agent {who} arms in {seconds:g} s{theme.SEPARATOR}Esc cancels')
        choice, why = agent_acts.choose(
            app, f'Agent {who} arms {app.profile.name}', ('Cancel',), seconds, cause,
            message=f'Arming in {seconds:g} s for agent {who} · claim {claim}. Esc or Enter cancels; Space stops.',
            stop_refuses=True)
        if why != 'timeout':
            reason = 'operator cancelled' if choice or why == 'refused' else why
            app.say(f'Agent {who} arm cancelled: {reason}')
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
        app.say(f"Agent {req['who']} reloaded {name or 'config'}"
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


def cleared(app, hold):
    """``(True, '')`` once a lease or watchdog hold's fault has cleared, else ``(False, why)``: a lease is no fault;
    the TUI's own watchdog hold needs every required watchdog alive; a watchdog node's needs its ~/status ok."""
    which = agent_grants.kind(hold)
    if which == 'lease':
        return True, ''
    if which == 'watchdog':
        why = '; '.join(filter(None, app.watchdog_reasons()))
        return not why, why
    ros, by = app.ros, str(hold.get('by'))
    nodes = [n for n in ros.nodes() if n.rsplit('/', 1)[-1] == by]
    if not nodes:
        return False, f'{by} is not running'
    new = getattr(ros, 'watch_beats', lambda nodes: None)(nodes)

    def beat():
        status = getattr(ros, 'beat_status', dict)().get(nodes[0])
        at = ros.beats().get(nodes[0]) if hasattr(ros, 'beats') else None
        return status, at is not None and time.monotonic() - at <= BEAT_FRESH_S
    status, fresh = beat()
    end = time.monotonic() + BEAT_WAIT_S
    while new and not (status is not None and fresh) and time.monotonic() < end:   # just subscribed: a beat or two
        time.sleep(0.05)
        status, fresh = beat()
    if status is None or not fresh:
        return False, f'no fresh ~/status from {by} yet: ask again in a few seconds'
    return bool(status.get('ok')), str(status.get('reason') or '')


def stopped_mux_file(app, arbiter):
    """The stopped mux's saved hold file when the boat is disarmed and the mux state is stale or unknown (the mux
    runs only while armed, so its holds live in that file until the next arm), else None. A remote arm's mux saves
    on the boat, not here: a local file of that name is some other session's, so it is never judged or cleared."""
    if app.armed or not muxed(app.profile) or getattr(getattr(app, 'arming', None), 'remote', None) is not None:
        return None
    with arbiter.lock:
        mux, at = arbiter.mux, arbiter.mux_at
    if mux is not None and at is not None and 0 <= arbiter.clock() - at <= STALE_MUX_S:
        return None
    return authority_feed.hold_file(app)


def release_saved(stored, holds=None):
    """Drop ``holds`` (every one when None, U's tokenless release) from the stopped mux's saved file; their causes."""
    saved = hold_store.load(stored)
    judged = {(h['cause'], h['since']) for h in (saved.values() if holds is None else holds)}
    hold_store.save(stored, {cause: h for cause, h in saved.items() if (cause, h['since']) not in judged})
    return sorted(cause for cause, since in judged)


def do_release_hold(desk, req):
    """boat_agent release HOLD --why: the mux's release (U's path) when every hold is the agent's to release now
    (agent/grants.py); no claim needed. The footer, the agent log and the journal name the agent and why."""
    app, who, name = desk.app, req['who'], str(req['args'].get('hold') or '')
    why = str(req.get('why') or '').strip()
    if app.settings.agents == 'block':
        return desk.reply(req, False, 'blocked', hint='operator must set Agents to allow')
    if not name or not why:
        return desk.reply(req, False, 'bad_args', hint='release HOLD --why "..." (hold causes: status .authority.record.holds)')
    arbiter = desk.arbiter
    if arbiter is None:
        return desk.reply(req, False, 'no_owner')
    stored = stopped_mux_file(app, arbiter)
    if stored is not None:      # disarmed: the stopped mux's saved holds, which the next arm restores
        holds = [{'cause': h['cause'], 'by': h['by'], 'since': h['since']} for h in hold_store.load(stored).values()]
    else:
        with arbiter.lock:
            holds = [dict(h) for h in (arbiter.mux or {}).get('holds') or ([arbiter.mux['hold']] if (arbiter.mux or {})
                                                                            .get('hold') else [])]
    grants, error = agent_grants.load(desk.config.get().get('grants', agent_config.GRANTS))
    refusal = agent_grants.verdict(holds, name, who, grants, lambda hold: cleared(app, hold), time.time())
    if refusal is None and stored is None:
        why_not = app.release_refusal()
        refusal = ('refused', why_not) if why_not else None
    if refusal:
        return desk.reply(req, False, refusal[0], hint=refusal[1] + (f' (grant file: {error})' if error else ''))
    if stored is not None:      # U's tokenless release clears every hold: drop exactly the ones judged above
        release_saved(stored, holds)
    else:
        release = getattr(app.plant, 'release_hold', None)
        if release is None or not release(by=f'agent {who}'):
            return desk.reply(req, False, 'refused', hint='this profile has no mux to release')
    causes = [str(h.get('cause')) for h in holds]      # the panel row and the header's attention show it
    app.emit('agent_release', who=who, holds=causes, why=why, **({'saved': str(stored)} if stored is not None else {}))
    desk.reply(req, True, result={'released': causes, 'why': why}, label=f"release {', '.join(causes)}")


def do_fence(desk, req):
    """boat_agent fence corners|on_exit|anchor --why (owner decision D-20261101-23: any time, no claim): the course
    area popup's own path, validated with fence_watchdog.problem, read back and kept in the fence file."""
    from mhseals_testing.harness import course_area
    from mhseals_testing.ui.popups import course
    app, args, who = desk.app, req['args'], req['who']
    why, action = str(req.get('why') or '').strip(), args.get('action')
    if app.settings.agents == 'block':
        return desk.reply(req, False, 'blocked', hint='operator must set Agents to allow')
    if not why:
        return desk.reply(req, False, 'bad_args', hint='fence ... --why "..."')
    client = app.fence_client()
    if client is None:
        return desk.reply(req, False, 'refused', hint='no fence client: this TUI watches or reviews')
    by, label = {'who': who, 'why': why}, f'fence {action}'
    if action == 'corners':
        corners, bad = course_area.parse(args.get('corners') or [])
        bad = bad or course_area.problem(corners)
        if bad:
            return desk.reply(req, False, 'bad_args', hint=f'course area: {bad}', label=label)
        ok, bad, path = course.set_corners(app, client, corners, by)
        result = {'corners': [list(c) for c in corners], 'read_back': ok, 'file': str(path) if path else None}
        text = f'set the course area ({len(corners)} corners)'
    elif action == 'on_exit':
        mode = args.get('mode')
        if mode not in course_area.ON_EXIT:
            return desk.reply(req, False, 'bad_args', hint='fence on_exit alert|return', label=label)
        ok, bad = course.set_on_exit(app, client, mode, by)
        result, text, label = {'on_exit': mode, 'read_back': ok}, f'set on_exit {mode}', f'fence on_exit {mode}'
    elif action == 'anchor':
        try:
            ok, bad = agent_acts.in_background(app, 'Fence anchor', client.anchor)
        except Exception as error:      # noqa: BLE001 - no fence_watchdog: refused, said why
            ok, bad = False, f'{type(error).__name__}: {error}'
        app.emit('fence_anchor', ok=ok, message=bad, **by)
        result, text = {'anchored': ok, 'message': bad}, f'anchored the fence ({bad})'
    else:
        return desk.reply(req, False, 'bad_args', hint='fence corners LAT,LON ... | on_exit alert|return | anchor')
    app.say(f'Agent {who} {text}: {why}' if ok else f'Agent {who} fence {action} refused: {bad}')
    desk.reply(req, ok, '' if ok else 'refused', result=result, hint='' if ok else bad, label=label)


BEAT_FRESH_S = 2.0       # a watchdog's ~/status older than this does not prove its fault cleared
BEAT_WAIT_S = 1.5        # a ~/status subscribed at the release itself: its first beat (watchdogs send at 2 Hz)
HANDLERS = {'arm': do_arm, 'reload': do_reload, 'release_hold': do_release_hold, 'fence': do_fence}
