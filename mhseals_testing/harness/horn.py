"""Journal the simulator's harbor-alert horn (RoboBoat task 6) for the referee.

crane publishes the horn boat-side on ``/crane/sim/horn`` (std_msgs/String JSON
``{"on": bool, "freq_hz": Hz}``, one message per state change) and keeps the truth-side
change log in its own ``states.jsonl``; nothing on the harness side journaled it. Each
on/off edge becomes one session-journal event ``phase: horn`` with ``on``, ``freq_hz``,
``pattern`` (``one``/``two``: the course truth's horn program, which crane plays; None when
the truth names no horn), ``id`` (the truth's horn state id) and ``topic``. A sounding is an
``on: true`` event; its journal ``ros_time``/``t_ns`` say when. ROS-free.
"""

import json
from pathlib import Path

TOPIC = '/crane/sim/horn'


def program(truth_path):
    """``(state id, pattern)`` of the course truth's horn program, else ``(None, None)``."""
    try:
        truth = json.loads(Path(truth_path).read_text())
        for state in truth.get('states', []):
            prog = state.get('program') or {}
            if prog.get('type') == 'horn':
                return state.get('id'), prog.get('pattern')
    except (OSError, TypeError, ValueError, AttributeError):
        pass
    return None, None


class HornJournal:
    """Called with each ``/crane/sim/horn`` payload; journals the edges through ``emit(phase, **fields)``."""

    def __init__(self, emit, truth_path=None):
        self.emit = emit
        self.truth_path = truth_path
        self.on = None

    def __call__(self, text):
        try:
            message = json.loads(text)
            on, freq = bool(message['on']), float(message.get('freq_hz') or 0.0)
        except (ValueError, TypeError, KeyError, AttributeError):
            return
        if on == self.on:
            return
        first, self.on = self.on is None, on
        if first and not on:
            return   # the horn starts off: not an edge
        ident, pattern = program(self.truth_path) if self.truth_path else (None, None)
        self.emit('horn', on=on, freq_hz=freq, pattern=pattern, id=ident, topic=TOPIC)
