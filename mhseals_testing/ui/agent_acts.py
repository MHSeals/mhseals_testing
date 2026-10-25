"""What an agent act does inside the TUI (spec agent-control sections 3 and 4): the same code as a key.

The desk (ui/feeds/agent.py) decides when an act may start; these functions do it. ``run``
calls ``app.run_procedure`` as the procedure key would; ``goal_*`` use ``app.goals`` (a
``harness.goals.Goals``), the one goal sender the cursor goal shares; ``choose`` is the
timed ``popup.choose`` for the confirm and ``ask`` popups. ``dry_run`` evaluates the live gates
and returns them without moving anything.
"""

from dataclasses import replace
import math
import threading
import time

from mhseals_hardware.menu import Cancelled
import numpy as np

from mhseals_testing.data import read_json
from mhseals_testing.harness import goals as goals_module
from mhseals_testing.harness import runs
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.runner import Aborted, fold
from mhseals_testing.ui import theme

# The key loop's stopping errors (ui/app.py STOPPING_ERRORS).
STOPPING = (Aborted, Cancelled, RuntimeError, OSError, ValueError, KeyError)
# Agent names for procedure keys; config/agent.yaml ``run`` lists the ones agents may request.
NAMES = {'square': 'w', 'station': 's', 'nav2_bringup': 'n', 'controller_check': 'v', 'perception': 'p',
         'mission': 't', 'characterize': 'f', 'compass_calibration': 'g', 'interference': 'e',
         'manual': 'm', 'thruster_set': 'i'}
NAV_NODES = ('bt_navigator', 'planner_server')
ABORT_JOIN_S = 1.0     # Ctrl+C during a background Nav2 step: how long to let a send land (then cancelled)


def procedure(name_or_key):
    """(Procedure, agent name) for ``square`` or its key ``w``; (None, None) when unknown."""
    key = NAMES.get(name_or_key, name_or_key)
    found = next((p for p in PROCEDURES if p.key == key), None)
    name = next((n for n, k in NAMES.items() if k == key), None)
    return (found, name) if found and name else (None, None)


def run_key(args):
    """The procedure a ``run`` request names: ``{key}`` (a key or an agent name)."""
    return str(args.get('key', ''))


class Closed(Exception):
    """A desk popup closed by its deadline or by the act's cancel cause."""


def choose(app, title, options, timeout_s, cause, message=None, danger=True, clock=time.monotonic):
    """A ``popup.choose`` that closes itself after ``timeout_s`` or once ``cause()`` names a reason.

    Returns (choice, None), or (None, why): 'refused' (Esc), 'timeout' or the cause. Space and E pass
    through as in every popup. The focus starts on the safe option (Refuse, or an agent's ``no``),
    so Enter alone never allows motion; the time left is a bar in the body.
    """
    from mhseals_testing.ui import popups
    deadline = clock() + timeout_s
    options = list(options)
    default = next((i for i, o in enumerate(options) if popups.safe(o)), 0)
    modal = popups.choice_modal(title, options, message=message, danger=danger, default=default)
    modal['hint'] = theme.SEPARATOR.join((f'{popups.UPDOWN} select', 'Enter choose', 'Esc refuse'))
    modal['countdown'] = (timeout_s, timeout_s)
    keys = popups.choose_keys()

    def handle(key):
        action = keys.get(fold(key))
        if action in ('up', 'left', 'down', 'right'):
            step = -1 if action in ('up', 'left') else 1
            modal['selected'] = (modal['selected'] + step) % len(options)
        elif action == 'select':
            return options[modal['selected']]
        elif action == 'cancel':
            raise Cancelled('refused by operator')

    def check():
        modal['countdown'] = (max(0.0, deadline - clock()), timeout_s)
        why = cause() or ('timeout' if clock() > deadline else None)
        if why:
            raise Closed(why)

    try:
        return app.popup(modal, handle, check), None
    except Cancelled:
        return None, 'refused'
    except Closed as closed:
        return None, str(closed)


def run(app, found):
    """Run ``found`` as its key would; returns (result, error). The run file names the outcome."""
    run_id = runs.next_run_id(app.paths.runs, found.title)
    error = None
    try:
        app.run_procedure(found)
    except STOPPING as stopped:
        error = str(stopped) or type(stopped).__name__
        app.stop(error, by='error')             # what the key loop does with a stopping error
    record = read_json(app.paths.runs / f'{run_id}.json') or {}
    notice = str(app.notice or '')
    result = {'run_id': run_id if record else None,
              'run_file': str(app.paths.runs / f'{run_id}.json') if record else None,
              'status': record.get('status'), 'results': (record.get('result') or {}).get('results', []),
              'invalid': 'window invalid' in notice, 'notice': notice}
    if error is None and notice.endswith('cancelled'):
        error = notice
    return result, error


# --- goals -----------------------------------------------------------------------------------

class Nav2Goals:
    """The Nav2 behind ``app.goals``, resolved on first use; a cancel never blocks the caller.

    The arbiter cancels from its ROS timer, on the same executor that would answer the cancel,
    so waiting there could only time out. The request is sent; Goals journals the stop.
    """

    def __init__(self, app):
        self.app = app

    def plan(self, goal, timeout=2.0):
        return self.app.nav2().plan(goal, timeout=timeout)

    def navigate(self, goals, timeout=10.0):
        return _Quick(self.app.nav2().navigate(goals, timeout=timeout))


class _Quick:
    def __init__(self, goal):
        self.goal = goal

    done = property(lambda self: self.goal.done)
    status = property(lambda self: self.goal.status)

    def cancel(self):
        self.goal.cancel(timeout=0.0)


def goal_facts(app, lifecycle=None):
    """``goals.Facts`` from the app. ``lifecycle`` None asks Nav2 (blocking, up to 0.5 s per server)."""
    nodes = {name.rsplit('/', 1)[-1] for name in app.ros.nodes()}
    nav2 = nav2_of(app)
    if lifecycle is None:
        lifecycle = nav2.lifecycle(timeout=0.5) if nav2 is not None else {}
    sensors = app.monitor.snapshot()['sensors']
    odometry = sensors.get('odometry', {})
    if app.profile.uses_fcu or app.profile.use_sim_time is not True:  # a real boat needs MAVROS's 3D fix
        fix = app.facts.get('gps.fix_type')
        fix = int(fix.value) if fix is not None else 0
    else:
        fix = navsat_fix(app.profile, sensors)
    driving = app.running or ('manual control' if app.manual else '')
    return goals_module.Facts(
        live=bool(app.source.live), owner=not app.watching, nav_running=set(NAV_NODES) <= nodes,
        lifecycle=lifecycle, unit=app.profile.command_units, armed=bool(app.armed), driving=driving or '',
        pose_age_s=odometry.get('age', math.inf) if odometry.get('status') == 'LIVE' else math.inf,
        fix=fix, costmap_age_s=costmap_age(nav2))


def costmap_age(nav2):
    """Receipt age of ``nav2``'s last costmap in seconds; inf when it has none."""
    at = getattr(nav2, 'costmap_time', None)
    return time.monotonic() - at if at is not None else math.inf


def navsat_fix(profile, sensors):
    """GPSRAW-style fix type from the profile's NavSatFix sensor, for a plant without an FCU (the sim).

    NavSatStatus -1 none, 0 fix, 1 SBAS, 2 GBAS reads as 0, 3 (3D), 4, 5; a stale or missing row is 0.
    """
    for spec in profile.sensors:
        row = sensors.get(spec.name) or {}
        status = (row.get('values') or {}).get('status')
        if spec.kind == 'gps' and row.get('status') != 'STALE' and status is not None:
            return 3 + int(status) if status >= 0 else 0
    return 0


def nav2_of(app):
    try:
        return app.nav2()
    except RuntimeError:
        return None


def grid(costmap):
    """A ``goals.Grid`` from a nav_msgs/OccupancyGrid, or None."""
    if costmap is None:
        return None
    info = costmap.info
    data = np.asarray(costmap.data, dtype=np.int16).reshape(info.height, info.width)
    return goals_module.Grid(data, info.resolution, (info.origin.position.x, info.origin.position.y))


def _plan(app, goal, facts, timeout=2.0):
    """(boat, grid, path_m, None) or (.., reason): the gates, the goal cell and a planned path.

    The costmap is fetched fresh (<= ``timeout``) and the planner asked (<= ``timeout`` per step), like
    the cursor goal; neither when a gate that needs no costmap already fails.
    """
    reason = goals_module.gate(facts, auto=True)
    if reason:
        return None, None, None, reason
    try:
        nav2 = app.nav2()
        pose, cells = nav2.pose(), grid(nav2.fetch_costmap(timeout=timeout))
    except RuntimeError as error:
        return None, None, None, str(error)
    if pose is None or cells is None:
        return None, None, None, 'no map pose' if pose is None else 'no costmap'
    facts = replace(facts, costmap_age_s=costmap_age(nav2))  # judged on the grid just fetched
    path, reason = app.goals.preview(goal, pose[:2], cells, facts, timeout=timeout)
    if reason:
        return pose[:2], cells, None, reason
    return pose[:2], cells, float(sum(math.dist(a, b) for a, b in zip(path, path[1:]))), None


def goal_preview(app, goal, facts, timeout=2.0):
    """(path_m, None) or (None, reason)."""
    return _plan(app, goal, facts, timeout)[2:]


def goal_send(app, goal, facts, claim, cancelled=lambda: None):
    """Send ``goal`` for ``claim`` through ``app.goals``; (path_m, None) or (None, reason).

    ``cancelled()`` is asked once more just before the send: a reason there sends nothing.
    """
    boat, cells, path_m, reason = _plan(app, goal, facts)
    reason = reason or cancelled()
    if reason is None:  # the same grid, now aged by the planner call
        facts = replace(facts, costmap_age_s=costmap_age(nav2_of(app)))
        reason = app.goals.send(goal, boat, cells, facts, claim, path_m)
    return (None, reason) if reason else (path_m, None)


def in_background(app, title, work, message=None, wait_s=0.2, on_abort=lambda: None):
    """``work()`` on a thread, so a slow Nav2 never stalls the keys; returns its value or raises its error.

    Past ``wait_s`` a popup shows and keeps reading keys (Space and E act as in every popup) until
    ``work`` ends. It always waits for the end, so nothing ``work`` starts is left unseen; on Ctrl+C
    it calls ``on_abort()``, waits at most ABORT_JOIN_S more, then re-raises.
    """
    box = {}

    def target():
        try:
            box['value'] = work()
        except Exception as error:      # noqa: BLE001 - handed to the caller's thread
            box['error'] = error
    thread = threading.Thread(target=target, name='agent-act', daemon=True)
    thread.start()
    thread.join(wait_s)
    if thread.is_alive():
        modal = dict(kind='message', title=title, message=message, rows=None, width=64,
                     hint=theme.SEPARATOR.join(('Space stops', 'E revokes')))

        def check():
            thread.join(0.02)
            if not thread.is_alive():
                raise Closed('done')
        try:
            app.popup(modal, lambda key: None, check)
        except Closed:
            pass
        except BaseException:
            on_abort()
            thread.join(ABORT_JOIN_S)
            raise
    if 'error' in box:
        raise box['error']
    return box.get('value')


# --- dry run ---------------------------------------------------------------------------------

def dry_run(app, op, args, motion_setting, lifecycle=None, timeout=2.0):
    """Everything a run or goal would check, without executing (spec section 3 --dry-run).

    The desk passes its cached ``lifecycle`` and a short Nav2 ``timeout``: it answers on the tick.
    """
    confirm = motion_setting == 'confirm'
    if op == 'run':
        found, name = procedure(run_key(args))
        blocking, advisory = (app.engine.gate(found.key, app.args.bench_override, 'armed' in found.needs)
                              if app.engine is not None else ([], []))
        needs_arm = 'armed' in found.needs and not app.armed
        return {'procedure': name, 'title': found.title, 'availability': app.availability(found) or None,
                'gates': list(blocking), 'advisories': list(advisory),
                'arm_blockers': app.arm_blockers() if needs_arm else [],
                'motion_blockers': app.motion_blockers() if app.armed else [],
                'settings': vars(app.settings), 'asks': {'arm': needs_arm, 'confirm': confirm},
                'hint': 'ask the operator to arm (a)' if needs_arm else ''}
    facts = goal_facts(app, lifecycle)
    path_m, reason = goal_preview(app, goal_args(args), facts, timeout)
    return {'goal': goal_args(args), 'gate': reason, 'path_m': path_m, 'asks': {'confirm': confirm},
            'hint': 'ask the operator to arm (a)' if reason == 'outputs are not armed' else ''}


def goal_args(args):
    """(x, y, yaw radians) from ``{'x', 'y', 'yaw'}``; ValueError when not finite numbers."""
    yaw = args.get('yaw')
    goal = (float(args.get('x', math.nan)), float(args.get('y', math.nan)), 0.0 if yaw is None else float(yaw))
    if not all(math.isfinite(v) for v in goal):
        raise ValueError('goal needs finite x, y (and yaw)')
    return goal
