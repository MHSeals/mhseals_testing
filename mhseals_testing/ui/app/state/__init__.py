"""The App's state by owner: one small container per area, so a feature's state has one writer (finding #4).

``app.state`` holds them (``AppState``). Each field is named for the mixin that owns it and is its one writer
(``view`` is View's, ``bags`` is Bags', ``health`` is the Health settings rows'); test_app_state_ownership derives
the owners from these names and fails when another area writes a container. The owner reads and writes
``self.state.<group>``; every other area reads ``app.services.<group>`` (services.py), never by writing it.
"""

from dataclasses import dataclass, field


@dataclass
class ViewState:
    """View's: the redraw bookkeeping (D-20261101-26), the live staging and the Fusion tab's feed."""
    last_frame: float = 0.0
    last_drawn: object = None     # segments of the last idle frame, to skip identical ones; None redraws
    last_key: object = None       # still_key() of the last frame (render only on change)
    staging: object = None        # the one LiveStaging of this TUI, made on first use
    fusion_feed: object = None    # the Fusion tab's feed, made when the tab first shows; False: none on this node


@dataclass
class LoopState:
    """Loop's: the last tick, the Fusion camera image toggle and whether quitting detaches (a handover)."""
    last_tick: float = 0.0
    fusion_image: object = None   # None until toggled (on locally, off over a link budget)
    detaching: bool = False       # the session hands its children to the next owner as it quits


@dataclass
class BagsState:
    """The Bags tab: the bag being recorded, the tab's scan of the disk and a bag being finalized."""
    recording_key: object = None
    scan: tuple = field(default_factory=lambda: (float('-inf'), [], (0, 0, 0)))   # (time, sessions, disk); 5 s
    finalizing: object = None
    finalize_thread: object = None
    rec: object = None             # the latest RecSample, drawn in the header strip
    review_live: object = None     # the live session's source, tabs and focus while a review plays over it
    review_fault: bool = False     # the review's reader failed: it stops ticking


@dataclass
class LabelsState:
    """The Labels tab's (actions/labels.py)."""
    session: object = None       # the session folder shown; None is the live one
    came_from: str = None        # the tab L or Esc returns to
    conditions: dict = None      # wind, waves, current as last set here (the next mark carries them)
    cache: tuple = (None, None)  # (stamp, manifest) of the session shown


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
    view: ViewState = field(default_factory=ViewState)
    loop: LoopState = field(default_factory=LoopState)
    bags: BagsState = field(default_factory=BagsState)
    labels: LabelsState = field(default_factory=LabelsState)
    manual: ManualState = field(default_factory=ManualState)
    procedures: ProceduresState = field(default_factory=ProceduresState)
    holds: HoldsState = field(default_factory=HoldsState)
    arm: ArmState = field(default_factory=ArmState)


def facades(app):
    """The read-only facades on ``app`` (services.py)."""
    from mhseals_testing.ui.app.state.services import Services
    return Services(app)
