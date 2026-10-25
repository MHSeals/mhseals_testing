"""Types shared by health probes, kinds, the engine and the surfaces."""

from dataclasses import dataclass, field

STATUSES = ('ok', 'info', 'unknown', 'warn', 'fail')    # ascending badness; sort worst first
SEVERITIES = ('info', 'warn', 'fail')                   # what a check's cap may be
MISSING = ('unknown', 'skip', 'fail')                   # policy when a needed fact is absent


@dataclass(frozen=True)
class Fact:
    """One named measurement published by a probe."""
    value: object                # scalar, bool, str or small dict
    label: str = ''              # who or what the value is about ("blastoise-jetson", "/gps/fix")
    data: dict = field(default_factory=dict)
    time: float = 0.0            # monotonic time of collection


@dataclass(frozen=True)
class Result:
    """What a kind concludes from facts."""
    status: str
    value: str = ''              # at most 10 chars, for inline cells: "3.1 Hz"
    detail: str = ''             # one line for the popup and toasts
    mark: str = ''               # one glyph for inline cells: ↓ low  ↑ high  ~ gaps  × none
    data: dict = field(default_factory=dict)


@dataclass(frozen=True)
class CheckSpec:
    """One configured check, after each-expansion and templating."""
    id: str
    kind: str
    params: object               # the kind's validated Params instance
    group: str = ''
    title: str = ''
    severity: str = 'warn'
    show: tuple = ()             # inline anchors this check may colour
    gates: tuple = ()            # procedure keys, or 'motion'
    when: dict = field(default_factory=dict)
    missing: str = 'unknown'
    raise_after: float | None = None     # None: the engine's default for the band
    clear_after: float = 3.0
    message: str = ''
    hint: str = ''
    profiles: tuple = ()         # empty: every profile
    advisory: str = ''           # dashboard System row this check feeds (checks sharing a name share a row)
    enabled: bool = True
