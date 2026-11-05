"""Authority feed: runs the arbiter in the session owner and keeps its latest record (spec authority_hotreload 3).

Only the owner of a live session has an arbiter: a ``--watch`` guest and offline review get none
(``start`` returns None). The arbiter reads what the app knows (the procedure it runs, the manual
state, the task status, the /cmd_vel publishers) through ``sources``, publishes latched at 1 Hz and
on change, and journals its events through ``ros.emit``. The feed remembers the last record it
published and when, so ``view()['authority_record']`` is None once it is stale (paused).
"""

import time

from mhseals_testing.harness.authority import is_unknown
from mhseals_testing.ros import authority as arbiter_module
from mhseals_testing.profiles import MUX_INPUT
from mhseals_testing.ros.arming import muxed

CMD_WINDOW_S = 1.0
MISSION = 'Mission rehearsal'  # app.services.procedures.running holds the procedure's title (procedures 't')


class AuthorityFeed:
    """Holds the arbiter and the latest record; ``clock`` is the local wall clock."""

    def __init__(self, app, ros, *, clock=time.time, registry=None):
        self.app, self.ros, self.clock = app, ros, clock
        self.record, self.at = None, None
        self.cmd_at = self.cmd_nonzero = None     # last /cmd_vel sample (local time) and if it moved
        self.arbiter = arbiter_module.Arbiter(
            self.publish, ros.emit, registry=registry, sources=self.sources,
            neutral=app.plant.neutral if app.plant else (lambda: None), hold=self.hold,
            clock=clock, quiet=self.quiet)

    def quiet(self):
        """Disarmed and neutral (no procedure, manual drive or goal, /cmd_vel still or silent): a lapsed lease
        needs no stop and leaves no hold."""
        app = self.app
        moving = self.cmd_nonzero and self.cmd_at is not None and self.clock() - self.cmd_at <= CMD_WINDOW_S
        goals = getattr(getattr(app, 'desk', None), 'goals', None)
        return not (app.armed or app.services.procedures.running or app.services.manual.active or moving or (goals is not None and goals.active))

    def publish(self, record):
        """The arbiter's transport: remember the record, then send it (bind swaps in the publisher)."""
        self.record, self.at = record, self.clock()
        self.send(record)

    send = staticmethod(lambda record: None)

    def hold(self, cause, by):
        hold = getattr(self.app.plant, 'hold', None)   # the mux hold request lands with the mux publisher
        if hold is not None:
            hold(cause, by)

    def on_cmd(self, nonzero):
        """Note one /cmd_vel sample from the subscription."""
        self.cmd_at, self.cmd_nonzero = self.clock(), nonzero

    def sources(self):
        """The arbiter's inputs from the app's state (see harness.authority.derive)."""
        app = self.app
        procedure = {'name': app.services.procedures.running, 'run': app.services.procedures.run_id} if app.services.procedures.running else None
        act = getattr(getattr(app, 'desk', None), 'act', None)
        if procedure and act and act.get('op') == 'run':
            procedure['claim'] = act['claim']  # the agent's own requested run nests under its claim
        watcher = app.services.procedures.watcher
        task = (watcher.status if watcher is not None else None) or None
        # run_task's status names it 'task'.
        name = task and (task.get('task') or task.get('name'))
        children = getattr(app, 'processes', None)
        own = [n[len('task_'):] for n in getattr(children, 'processes', ())
               if n.startswith('task_') and children.running(n)] if app.services.procedures.running == MISSION else []
        if name in own:
            task = None                        # the Mission's own task: not a second source
        elif task:
            task = dict(task, name=name)       # derive reads 'name'
        if procedure and own and not (task and task.get('status') == 'RUNNING'):
            # Mission started it: the procedure's implicit claim. A different task that
            # runs meanwhile stays a source of its own, so it still shows as a conflict.
            procedure['task'] = own[0]
        topic = MUX_INPUT if muxed(app.profile) else app.profile.command_topic
        age = None if self.cmd_at is None else self.clock() - self.cmd_at
        publishers = self.ros.endpoints(topic) if hasattr(self.ros, 'endpoints') else []
        graph = {'cmd_vel': {'age_s': age, 'nonzero': bool(self.cmd_nonzero)},
                 'publishers': [{'node': p['node'], 'gid': p['gid']} for p in publishers],
                 'own_gids': [p['gid'] for p in publishers if p['self']]}
        goals = getattr(getattr(app, 'desk', None), 'goals', None)   # the one goal sender (the agent desk's)
        active = goals.active if goals is not None else None
        nav = {'goals': [{'id': f"goal {active['claim'] or 'tui'}", 'status': 'executing', 'by': 'nav',
                          'cancel': lambda: goals.cancel('revoke')}]} if active else None
        return {'procedure': procedure, 'task': task, 'graph': graph, 'nav': nav,
                'mux_expected': bool(app.armed), 'muxed': bool(muxed(app.profile)),
                'saved_holds': () if app.armed or not muxed(app.profile) else saved_holds(app)}

    def tick(self, app, snapshot, now):
        pass                                   # the arbiter runs on its own ROS timer

    def view(self):
        age = None if self.at is None else self.clock() - self.at
        stale = is_unknown(self.record, self.clock(), age)
        return {'authority_record': None if stale else self.record, 'authority_paused': stale}


def hold_file(app):
    """The file the stopped mux saved its holds in (hold_store, the state_file arming gives it: a sim session's own)."""
    from mhseals_hardware import hold_store
    from mhseals_testing.ros.arming import session_hardware
    config = getattr(getattr(app, 'args', None), 'hardware_config', None) or {}
    arming = getattr(app, 'arming', None)
    if arming is not None and getattr(arming, 'config_path', None):
        config = session_hardware(app.profile, config, arming.config_path, getattr(arming, 'remote', None))
    section = config.get('command_mux') or {}
    return section.get('state_file') or hold_store.default_path()


def saved_holds(app):
    """The holds the stopped mux saved (``hold_file``): ``[{cause, by, has_token}]``."""
    from mhseals_hardware import hold_store
    holds = hold_store.load(hold_file(app))
    return [{'cause': h['cause'], 'by': h['by'], 'has_token': h['token'] is not None}
            for h in sorted(holds.values(), key=lambda h: h['since'])]


def start(app, ros, bind=arbiter_module.bind, subscribe=None, registry=None):
    """The session owner's feed, bound to ``ros``; None for a guest or a review (nothing to run)."""
    if app.watching or app.review or app.args.review is not None:
        return None
    feed = AuthorityFeed(app, ros, registry=registry)
    feed.arbiter.publish = feed.send           # bind installs the ROS publisher in its place
    bind(ros, feed.arbiter)
    feed.send = feed.arbiter.publish           # bind installed the ROS publisher: keep it, wrap it
    feed.arbiter.publish = feed.publish
    if subscribe:
        subscribe(ros, feed)
    app.feeds.append(feed)                     # startup.py makes feed.arbiter app.arbiter, the one name others read
    # Watch /task/status from the start: a task started outside the TUI (another terminal,
    # an agent) must show as TASK, not only once a Mission has created the watcher.
    if getattr(app, 'watcher_factory', None) is not None:
        app.task_watcher()
    return feed


def subscribe_cmd_vel(ros, feed):
    """Sample the command input so a nonzero /cmd_vel with no claim shows as unclaimed."""
    from geometry_msgs.msg import Twist, TwistStamped
    profile = feed.app.profile
    stamped = profile.command_type == 'twist_stamped'
    topic = MUX_INPUT if muxed(profile) else profile.command_topic

    def on_cmd(message):
        twist = message.twist if stamped else message
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        feed.on_cmd(any(abs(v) > 1e-3 for v in values))

    ros.node.create_subscription(TwistStamped if stamped else Twist, topic, on_cmd, 10)


class ViewerFeed:
    """A viewer's authority record: the owner's, as published (latched), with the owner feed's staleness rule.

    The viewer derives nothing (one arbiter, in the owner); ``view`` has the same keys as AuthorityFeed's.
    """

    def __init__(self, clock=time.time):
        self.clock = clock
        self.record, self.at = None, None

    def on_record(self, record):
        if isinstance(record, dict) and 'owner' in record:
            self.record, self.at = record, self.clock()

    def tick(self, app, snapshot, now):
        pass

    def view(self):
        age = None if self.at is None else self.clock() - self.at
        stale = is_unknown(self.record, self.clock(), age)
        return {'authority_record': None if stale else self.record, 'authority_paused': stale}


def viewer(app, ros):
    """A watch guest's feed, subscribed to the owner's record; None for the owner or a review."""
    if not app.watching or app.review:
        return None
    import json
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    feed = ViewerFeed()

    def on_record(message):
        try:
            feed.on_record(json.loads(message.data))
        except ValueError:
            pass
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ros.node.create_subscription(String, arbiter_module.AUTHORITY_TOPIC, on_record, latched)
    app.feeds.append(feed)                      # startup.py makes it app.authority_view: the chip reads the record
    return feed
