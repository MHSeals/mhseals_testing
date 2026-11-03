"""The view-model seam: the typed boundary between the session's state and rendering.

`Frame` is everything one draw consumes: ``snapshot`` (`Snapshot`, the boat's state: the monitor's
sensors, results and events plus the app's live rows) and ``view`` (`ViewState`, the station's: tab,
focus, hints, popups) at a terminal size. View.frame builds it and layout renders only it. Wave 2
splits along this line: a boat backend serves the Snapshot, the station client adds its ViewState.
Both are plain dicts at run time (TypedDict); test_view_model checks the keys a live App produces
against these declarations, so a new key is declared here first.
"""

from dataclasses import dataclass
from typing import Any, TypedDict


class Snapshot(TypedDict, total=False):
    """The boat's state (View.snapshot): monitor.snapshot() plus the live App's rows."""
    sensors: dict           # topic -> rate, age and value rows (monitor)
    tf: dict                # required transforms and their state
    advisories: dict        # name -> level, detail, age
    events: list            # the event log (newest last)
    plots: dict             # plot series
    results: list           # procedure results
    drift: Any              # the controller's drift estimate, or None when stale
    rtk: dict               # the RTK status and its age
    components: list        # Nav2 file and stack component rows
    control: dict           # live control state (View.control)
    fence: Any              # the course area and on-exit rows
    rec_mode: str           # the Recording row (owner only)
    graphics: tuple         # (theme.yaml setting, drawing tier)
    health: dict            # health check counts
    rtk_view: Any           # the RTK pane's current view


class ViewState(TypedDict, total=False):
    """The station's state (View.view): what this terminal shows and how. Keys of the monitor's
    shared feeds (feeds.merged) ride along under their own names; the App's keys win a collision."""
    # navigation: where this terminal is
    tab: str
    focus: str
    focus_title: Any
    cursors: dict
    position: str
    minimap: Any
    layerbox: Any
    checklist: Any
    modal: Any
    # messages
    notice: Any
    flash: Any              # a mark's short confirmation, bold before the notice (ui/app/actions/annotate.py)
    hints: list
    toasts: list
    toasts_more: int
    activity: Any
    now: float
    # control: who drives and how (the keyboard's mode is this terminal's)
    authority: Any
    operator: Any
    agent: Any
    armed: bool
    armed_by: Any
    arm_label: str
    command: tuple
    chords: bool
    chords_unavailable: Any
    speed: Any
    running: Any
    # the profile and host this station shows
    profile: str
    units: Any
    plot_scale: Any
    outputs: Any
    specs: Any
    deploy: dict
    host: str
    domain: str
    issues: Any
    # recording, review and bags
    recording: bool
    rec: Any
    review: Any
    bags: Any
    labels: Any             # the Labels tab's model (ui/panels/labels.py), on that tab only
    bag_usage: Any
    # thrusters and correction
    bench_override: Any
    hardware: dict
    matrix: Any
    test_pulses: list
    pwm_readback: list
    draft: Any
    grabbed: Any
    config_note: Any
    correction: Any
    procedures: list
    # the Fusion tab
    fusion: Any
    fusion_facts: Any


@dataclass(frozen=True)
class Frame:
    """One draw: the boat's ``snapshot``, the station's ``view``, at ``width`` x ``height`` cells."""
    snapshot: Snapshot
    view: ViewState
    width: int
    height: int

    def render(self):
        """The renderable for this frame (ui/layout.py)."""
        from mhseals_testing.ui import layout
        return layout.render(self.snapshot, self.view, self.width, self.height)
