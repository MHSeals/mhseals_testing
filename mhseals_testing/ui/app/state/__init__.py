"""The App's state by owner: one small container per area, so a feature's state has one writer (finding #4).

``app.state`` holds them (``AppState``): ``render`` (View's redraw bookkeeping), ``bags`` (the Bags tab and the
bag a session records), ``labels`` (the Labels tab), ``thrusters`` (the Thrusters panel's unsaved edit and raw
pulses) and ``arming_ui`` (the arm path's pin and watchdog guards). core.py's OWNERS table names which mixin
owns which attribute group; test_app_state_ownership fails when a mixin writes another's. The flat names a mixin used
before (``app.bag_scan``) stay as aliases onto the container (``Alias``) until each area converts its mixin,
so reads, writes and tests see one value. Other areas read a container through ``app.services`` (services.py),
never by writing it.
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
class LabelsState:
    """The Labels tab: the session it shows (None: the live one), its manifest cache, the tab L came from and
    the conditions last set."""
    folder: object = None
    cache: tuple = (None, None)
    came_from: object = None
    conditions_set: object = None


@dataclass
class ThrustersState:
    """The Thrusters panel's edit (Manual's keys): the unsaved mapping, the row being moved and raw pulses."""
    draft: object = None             # unsaved thruster mapping: channel_map, reversed and pwm_chips
    grabbed: object = None           # thruster row being moved with up/down
    test_pulses: object = None


@dataclass
class ArmingUiState:
    """The arm path's guards: a pin change not read back yet, and a watchdog off the graph while armed."""
    pins_unverified: bool = False    # a saved pin change not read back yet: arming refused
    watchdog_missing: object = None  # {since, asked, retried, held} while armed with a watchdog off the graph


@dataclass
class AppState:
    render: RenderState = field(default_factory=RenderState)
    bags: BagsState = field(default_factory=BagsState)
    labels: LabelsState = field(default_factory=LabelsState)
    thrusters: ThrustersState = field(default_factory=ThrustersState)
    arming_ui: ArmingUiState = field(default_factory=ArmingUiState)


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
    'labels_dir': ('labels', 'folder'), '_labels_cache': ('labels', 'cache'), 'labels_from': ('labels', 'came_from'),
    'conditions_set': ('labels', 'conditions_set'),
    'draft': ('thrusters', 'draft'), 'pins_unverified': ('arming_ui', 'pins_unverified'),
    'grabbed': ('thrusters', 'grabbed'), 'test_pulses': ('thrusters', 'test_pulses'),
    'watchdog_missing': ('arming_ui', 'watchdog_missing'),
}


def install(cls):
    """Define each alias on ``cls`` (App): the container is the one place the value lives."""
    for name, (group, attr) in ALIASES.items():
        setattr(cls, name, Alias(group, attr))
    return cls
