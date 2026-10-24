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
from mhseals_testing.ros.arming import MUX_INPUT, muxed

CMD_WINDOW_S = 1.0


class AuthorityFeed:
    """Holds the arbiter and the latest record; ``clock`` is the local wall clock."""

    def __init__(self, app, ros, *, clock=time.time, registry=None):
        self.app, self.ros, self.clock = app, ros, clock
        self.record, self.at = None, None
        self.cmd_at = self.cmd_nonzero = None     # last /cmd_vel sample (local time) and if it moved
        self.arbiter = arbiter_module.Arbiter(
            self.publish, ros.emit, registry=registry, tui_owner=True, sources=self.sources,
            neutral=app.plant.neutral if app.plant else (lambda: None), hold=self.hold,
            clock=clock)

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
        procedure = {'name': app.running, 'run': app.run_id} if app.running else None
        task = getattr(getattr(app, '_watcher', None), 'status', None) or None
        topic = MUX_INPUT if muxed(app.profile) else app.profile.command_topic
        age = None if self.cmd_at is None else self.clock() - self.cmd_at
        publishers = self.ros.endpoints(topic) if hasattr(self.ros, 'endpoints') else []
        graph = {'cmd_vel': {'age_s': age, 'nonzero': bool(self.cmd_nonzero)},
                 'publishers': [{'node': p['node'], 'gid': p['gid']} for p in publishers],
                 'own_gids': [p['gid'] for p in publishers if p['self']]}
        return {'procedure': procedure, 'task': task, 'graph': graph, 'nav': None,
                'mux_expected': bool(app.armed), 'muxed': bool(muxed(app.profile))}

    def tick(self, app, snapshot, now):
        pass                                   # the arbiter runs on its own ROS timer

    def view(self):
        age = None if self.at is None else self.clock() - self.at
        stale = is_unknown(self.record, self.clock(), age)
        return {'authority_record': None if stale else self.record, 'authority_paused': stale}


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
    app.arbiter = feed.arbiter                 # the one name the popups, banner and Agents setting read
    app.feeds.append(feed)
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
