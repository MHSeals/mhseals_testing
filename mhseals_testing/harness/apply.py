"""Offline hardware file staging and legacy parameter-readback helpers; ROS-free.

HardwareApply records file/parameter outcomes, not effective output proof.
Staging backs up C+/C- and changes files for offline workflows; it cannot
establish the running driver's correction/model. RuntimeStaging refuses these
transitions until an owner-controlled runtime application can be acknowledged.
"""

from contextlib import contextmanager
from dataclasses import dataclass
import time

from mhseals_hardware.configuration import load_config, update_config

from mhseals_testing import data

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
        if self.armed() is not False:
            return Result('refused', 'disarm first: pins move real outputs')
        config = self.current()
        old = {key: config[key] for key in changes}
        if old == changes:
            if self.serving() and self.read(list(changes)) == changes:
                return Result('applied', 'unchanged parameter readback; effective outputs unverified')
            return Result('saved', 'file unchanged; consumer unverified')
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
            return Result('timeout', f'no readback in {self.timeout:g} s; previous file restored; consumer state unverified')
        restored = self.read(list(old))
        if restored == old:
            return Result('rolled_back', 'previous file and parameter readback restored; effective outputs unverified')
        return Result('rollback_unverified', 'previous file restored; consumer readback does not match')


SCHEMA = 'mhseals.state/1'
KEYS = ('decoupling', 'decoupling_negative')
IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


class Staging:
    """The backup of C and C- in ``state.json`` beside the data root, for one hardware YAML."""

    def __init__(self, config=None, state=None):
        self.config = config
        self.state = state or data.path('state.json')

    def pending(self):
        """The unrestored backup ({'decoupling': ..., 'decoupling_negative': ...}), or None."""
        return (data.read_json(self.state, {}) or {}).get('correction_backup')

    def raw_correction(self):
        """Back up C+/C- (once, so a repeat never saves the identity) and write identity C."""
        if self.pending() is None:
            current = load_config(self.config)
            backup = {key: current.get(key) for key in KEYS}
            self._write(correction_backup=backup)
        update_config(dict(decoupling=IDENTITY, decoupling_negative=None), self.config)

    def restore(self):
        """Put the backed-up C+/C- back, then drop the backup. False when nothing was staged."""
        backup = self.pending()
        if backup is None:
            return False
        update_config({key: backup.get(key) for key in KEYS}, self.config)
        self._clear()
        return True

    def commit(self, decoupling, decoupling_negative=None, **changes):
        """Write the survey's C+/C- (and e.g. characterization=...) in one save, then drop the backup."""
        update_config(dict(changes, decoupling=decoupling, decoupling_negative=decoupling_negative),
                      self.config)
        self._clear()

    def _clear(self):
        self._write(correction_backup=None)

    def _write(self, **changes):
        """Rewrite state.json with ``changes`` applied (None drops a key); other session state is kept."""
        record = dict(data.read_json(self.state, {}) or {})
        for key, value in changes.items():
            if value is None:
                record.pop(key, None)
            else:
                record[key] = value
        data.write_json(self.state, record, SCHEMA)

    def blockers(self):
        """Motion-refusal reasons; the shared guard adds these to every host's list."""
        if self.pending() is None:
            return []
        return ['an identity correction is staged and C is not restored: restore it first']

    @contextmanager
    def staged(self):
        """Raw efforts inside the block; any exit but ``commit`` restores (finally, so aborts too)."""
        self.raw_correction()
        try:
            yield self
        finally:
            self.restore()


CORRECTION_ACK_REQUIRED = (
    'Characterization unavailable: the driver correction C/C- is restart-only; '
    'runtime correction application/readback is unsupported. No correction transition was applied.')


class RuntimeStaging(Staging):
    """Fail closed until an owner-controlled runtime C transition can be acknowledged.

    Staging remains an offline file API. A file save or parameter echo cannot
    establish the immutable selected driver's active correction/model.
    """

    def _refuse(self):
        from mhseals_hardware.menu import Cancelled
        raise Cancelled(CORRECTION_ACK_REQUIRED)

    def raw_correction(self):
        self._refuse()

    def commit(self, decoupling, decoupling_negative=None, **changes):
        self._refuse()

    def restore(self):
        if self.pending() is None:
            return False
        self._refuse()

    def blockers(self):
        return (["Pending correction backup: runtime C is unverified; " + CORRECTION_ACK_REQUIRED]
                if self.pending() is not None else [])
