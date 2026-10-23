"""What a starting instance does when the registry says the session is already taken.

A live owner can be asked to detach (it hands over its children) or stopped; a
stale owner (its pid is gone, or it is stuck headless after a dead `docker exec -it`
client) leaves children behind that can be adopted or stopped. Plain console menu:
it runs before the page and ROS exist. ROS-free.
"""

from dataclasses import dataclass, field
import signal

from mhseals_testing.harness.registry import Busy

STOP_WAIT = 40.0   # s: a stopping owner flushes the bag before it exits


@dataclass
class Choice:
    mode: str                                   # own | adopt | watch | quit
    children: dict = field(default_factory=dict)
    session: str = ''
    watching: int = 0


def describe(state, alive, now):
    owner = state['owner']
    since = max(0, now - owner.get('started', now)) / 60
    children = ' / '.join(f"{name} {child['pid']}" for name, child in state.get('children', {}).items()
                          if alive(child['pid'])) or 'none'
    return [f"pid {owner['pid']} ({'running' if alive(owner['pid']) else 'gone'}) on "
            f"{owner.get('host', '?')} owns this session, {since:.0f} min.",
            f'  Children  {children}', f"  Session   {state.get('session') or '-'}"]


def decide(registry, console, me, ask=None):
    """Claim the seat or let the operator choose; returns a Choice. ``me`` is the owner record."""
    ask = ask or console.input
    while True:
        state = registry.state()
        owner = state.get('owner')
        if owner is None or owner['pid'] == me['pid']:
            return claimed(registry, me, state, 'adopt' if state.get('children') else 'own')
        stale = not registry.alive(owner['pid'])
        console.print('Session in use' if not stale else 'Stale session (owner gone)')
        for line in describe(state, registry.alive, registry.clock()):
            console.print(line)
        options = (['Adopt its children', 'Stop its children and start fresh'] if stale else
                   ['Take over: it detaches at neutral, you adopt its children',
                    'Stop it: it ends the session and flushes the bag'])
        options += ['Watch only: no arming, no commands', 'Quit']
        for number, text in enumerate(options, 1):
            console.print(f'  {number}. {text}')
        answer = ask('Choose 1-4: ').strip()
        if answer == '1' and stale:
            return claimed(registry, me, state, 'adopt')
        if answer == '2' and stale:
            return claimed(registry, me, state, 'own')    # the caller stops the recorded children
        if answer == '1':
            if registry.request_handover(me['pid']):
                continue
            console.print('It did not let go within 5 s; choose Stop it to end it.')
        elif answer == '2':
            stop_owner(registry, owner['pid'])
            if not registry.alive(owner['pid']):
                registry.release(owner['pid'], keep_children=True)
                continue
            console.print(f"pid {owner['pid']} is still running; not claiming the session.")
        elif answer == '3':
            return Choice('watch', watching=owner['pid'], session=state.get('session', ''))
        elif answer == '4':
            return Choice('quit')


def claimed(registry, me, state, mode):
    try:
        registry.claim(me)
    except Busy:                                   # someone else got there first
        return Choice('quit')
    children = {name: child for name, child in state.get('children', {}).items()
                if registry.alive(child['pid'])}
    if mode == 'adopt' and not children:    # nothing alive to adopt: a fresh bag, not the old directory
        mode = 'own'
    return Choice(mode, children, state.get('session', ''))


def stop_owner(registry, pid, wait=STOP_WAIT):
    """SIGTERM runs the owner's normal close (neutral, disarm, stop children, flush the bag)."""
    try:
        registry.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    end = registry.clock() + wait
    while registry.alive(pid) and registry.clock() < end:
        registry.sleep(0.2)
