"""Hot-apply hardware config changes and prove the driver took them; ROS-free.

``HardwareApply.stage`` writes the changes to the operator's hardware YAML (the
driver hot-reloads it and moves its outputs), then polls the driver's own
parameters until they show the new values. Pin changes move real outputs, so
they are refused while armed (critique safety 1).
"""

from dataclasses import dataclass
import time

READBACK_TIMEOUT = 2.0  # seconds the driver gets to show the new values
POLL = 0.1


@dataclass
class Result:
    """``status`` is applied, saved, refused, invalid, rolled_back or timeout."""
    status: str
    detail: str = ''

    @property
    def ok(self):
        return self.status == 'applied'


class HardwareApply:
    """``save(changes)`` writes the YAML, ``read(keys)`` returns the driver's values or None.

    ``serving()`` says whether a driver answers parameter requests; without one
    (the TUI starts it only when arming) the file is just saved for the next arm.
    """

    def __init__(self, save, read, armed, current, serving=lambda: True, clock=time.monotonic,
                 sleep=time.sleep, timeout=READBACK_TIMEOUT):
        self.save, self.read, self.armed, self.current = save, read, armed, current
        self.serving = serving
        self.clock, self.sleep, self.timeout = clock, sleep, timeout

    def stage(self, changes):
        """Write ``changes``; applied when the readback matches, else the old values are restored.

        rolled_back: the driver answered but still shows the old values (its own
        switch failed and it kept the old pins; the reason is in its log).
        timeout: the driver gave no answer in time. Both restore the file.
        saved: no driver is running, so there is nothing to read back.
        """
        if self.armed():
            return Result('refused', 'disarm first: pins move real outputs')
        config = self.current()
        old = {key: config[key] for key in changes}
        if old == changes:
            return Result('applied', 'unchanged')
        try:
            self.save(changes)
        except (OSError, ValueError, TypeError) as error:
            return Result('invalid', str(error))
        if not self.serving():
            return Result('saved', 'applied at next arm')
        seen = None
        deadline = self.clock() + self.timeout
        while True:
            seen = self.read(list(changes))
            if seen == changes:
                return Result('applied')
            if self.clock() >= deadline:
                break
            self.sleep(POLL)
        try:
            self.save(old)
        except (OSError, ValueError, TypeError) as error:
            return Result('invalid', f'could not restore the old pins ({error}); fix hardware.yaml')
        if seen is None:
            return Result('timeout', f'no readback in {self.timeout:g} s; old pins restored')
        return Result('rolled_back', 'the driver kept the old pins; see its log')
