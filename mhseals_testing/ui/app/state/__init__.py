"""The App's state by owner: one small container per area, so a feature's state has one writer (finding #4).

``app.state`` holds them (``AppState``): ``render`` (View's redraw bookkeeping), ``bags`` (the Bags tab and the
bag a session records), ``manual`` (manual control, the Thrusters panel's edit and raw pulses), ``holds``
(the watchdog guard, the fence alert), ``arm`` (who armed, the last stop, the pin guard) and ``procedures``
(the run in progress). The Labels tab keeps its own (``app.labels``, actions/labels.py LabelsState).
core.py's OWNERS table names which mixin owns which group; test_app_state_ownership fails when a mixin writes another's. The flat names a mixin used before
(``app.bag_scan``) stay as aliases onto the container (``Alias``) until each area converts its mixin; a
converted area has no alias: its owner reads and writes ``app.state.<group>`` and other areas read it through
``app.services`` (services.py), never by writing it.
"""

from dataclasses import dataclass, field


@dataclass
class RenderState:
    """View's redraw bookkeeping (D-20261101-26): the last frame's time, its still key and its segments."""
    last_frame: float = 0.0
    last_drawn: object = None     # segments of the last idle frame, to skip identical ones; None redraws
    last_key: object = None       # still_key() of the last frame (render only on change)


@dataclass
class BagsState:
    """The Bags tab: the bag being recorded, the tab's scan of the disk and a bag being finalized."""
    recording_key: object = None
    scan: tuple = field(default_factory=lambda: (float('-inf'), [], (0, 0, 0)))   # (time, sessions, disk); 5 s
    finalizing: object = None
    finalize_thread: object = None


@dataclass
class ManualState:
    """Manual's (actions/manual.py): manual control, the command shown, and the Thrusters panel's edit and pulses."""
    active: bool = False             # manual control (m) is driving
    speed: object = None             # the last manual speed this session; None until manual ran
    chords: bool = False             # the manual mode, for the Command glyph's label
    command: tuple = (0.0, 0.0, 0.0)  # the commanded (surge, sway, yaw) the Command glyph shows
    desk_at: float = 0.0             # when manual control last ticked the agent desk
    draft: object = None             # unsaved thruster mapping: channel_map, reversed and pwm_chips
    grabbed: object = None           # thruster row being moved with up/down
    test_pulses: object = None       # raw pulses per output while testing, else None
    pulses_since: float = 0.0        # when the current test pulse started (its readback grace)
    pulse_verdict: object = None     # what the readback last said about it
    pulse_baseline: dict = field(default_factory=dict)   # the driver's refused/ended stamps the pulse started on


@dataclass
class ArmState:
    """Arm's (actions/arm.py): who armed, whether this session ever did, unexplained suite nodes, the last stop."""
    armed_by: object = None          # None is the operator's arm, else {'who', 'sim'} (an agent), for the ARMED chip
    ever_armed: bool = False         # a hold saved after this is this session's (sim stale holds)
    foreign_seen: dict = field(default_factory=dict)   # suite node -> first seen, no registry entry explains it
    stopped_by: object = None        # who stopped last: 'operator', 'agent <id>' or 'viewer <name>'
    pins_unverified: bool = False    # a saved pin change not read back yet: arming refused


@dataclass
class HoldsState:
    """Holds' (actions/holds.py): a watchdog off the graph while armed, the fence client and alert, a release."""
    watchdog_missing: object = None  # {since, asked, retried, held} while armed with a watchdog off the graph
    fence_client: object = None      # ros/fence.FenceClient on this owner's node, made on first use
    fence_out: bool = False          # outside the course area (until a position shows it back inside)
    fence_text: object = None        # the outside notice last said
    releasing: bool = False          # U's confirm is open: a second U waits


@dataclass
class ProceduresState:
    """Procedures' (actions/procedures.py): the run in progress, its runner's pause and cancel, lazy clients."""
    running: object = None           # the title of the procedure running, else None
    run_id: object = None            # the run in progress; its results carry this id
    run_fields: dict = field(default_factory=dict)   # run-level fields a procedure adds to its run record
    tags: dict = field(default_factory=dict)         # procedure, key and run_id, journaled with every event
    cancelled: object = False        # the first cause that cancels the run (True: the operator)
    paused: bool = False             # the runner is paused (the loop only polls)
    start_reset: object = None       # ros/sim_reset.StartReset, made on first use
    nav2: object = None              # the Nav2 client, made on first use
    watcher: object = None           # the task watcher, made on first use


@dataclass
class AppState:
    render: RenderState = field(default_factory=RenderState)
    bags: BagsState = field(default_factory=BagsState)
    manual: ManualState = field(default_factory=ManualState)
    procedures: ProceduresState = field(default_factory=ProceduresState)
    holds: HoldsState = field(default_factory=HoldsState)
    arm: ArmState = field(default_factory=ArmState)


def state_of(app):
    """``app.state``, made on first use (an App built without __init__ in a test still has one)."""
    found = app.__dict__.get('state')
    if found is None:
        found = app.__dict__['state'] = AppState()
    return found


class Alias:
    """A flat App attribute that lives in a container: ``bag_scan = Alias('bags', 'scan')``."""

    def __init__(self, group, name):
        self.group, self.name = group, name

    def __get__(self, app, owner=None):
        if app is None:
            return self
        return getattr(getattr(state_of(app), self.group), self.name)

    def __set__(self, app, value):
        setattr(getattr(state_of(app), self.group), self.name, value)


# flat name -> (container, field): the aliases App defines
ALIASES = {
    'last_frame': ('render', 'last_frame'), 'last_drawn': ('render', 'last_drawn'), 'last_key': ('render', 'last_key'),
    'bag_recording_key': ('bags', 'recording_key'), 'bag_scan': ('bags', 'scan'),
    'finalizing': ('bags', 'finalizing'), 'finalize_thread': ('bags', 'finalize_thread'),
}


def install(cls):
    """Define each alias on ``cls`` (App): the container is the one place the value lives."""
    for name, (group, attr) in ALIASES.items():
        setattr(cls, name, Alias(group, attr))
    return cls


def facades(app):
    """The read-only facades on ``app`` (services.py)."""
    from mhseals_testing.ui.app.state.services import Services
    return Services(app)
