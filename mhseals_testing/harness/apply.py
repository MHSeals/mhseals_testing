"""Offline hardware file staging; ROS-free.

Staging backs up C+/C- and changes files for offline workflows; it cannot
establish the running driver's correction/model. RuntimeStaging refuses these
transitions until an owner-controlled runtime application can be acknowledged.
"""

from contextlib import contextmanager

from mhseals_hardware.configuration import load_config, update_config

from mhseals_testing import data

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
