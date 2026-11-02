"""Staging the thruster correction C for a characterization; ROS-free.

``Staging`` backs up C+/C- and changes the hardware YAML (offline workflows and tests).
``LiveStaging`` is the TUI's: it applies C to the running driver over SetParameters and
verifies the readback (the driver's live C, mhseals_hardware contract C4).
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


DRIVER = '/thruster_pwm_node'
TOLERANCE = 1e-9


class LiveStaging(Staging):
    """C+/C- on the running driver: SetParameters (``decoupling``, ``decoupling_negative``), then GetParameters
    must read the same values back, else Cancelled and nothing is assumed applied.

    ``params()`` returns the parameter client (ros/params.py ``Params``: ``set_atomically`` and ``get``).
    Identity is staged live only; the hardware YAML keeps the deployed C until ``commit``, so a driver that
    restarts after a crash runs the deployed C again. ``active`` is True while this TUI's own run stages C;
    a backup left by any other (an interrupted run) blocks motion until ``recover`` or A restores it.
    """

    def __init__(self, config=None, state=None, params=None, node=DRIVER):
        super().__init__(config, state)
        self.params, self.node, self.active, self.error = params, node, False, ''

    def apply_live(self, plus, minus=None):
        """Set C+/C- on the driver and verify the readback; Cancelled says why not."""
        from mhseals_hardware.menu import Cancelled
        values = {'decoupling': [float(v) for v in plus],
                  'decoupling_negative': [float(v) for v in (minus or plus)]}
        client = self.params() if self.params else None
        if client is None:
            raise Cancelled('no ROS link to the thruster driver: C cannot be applied live')
        try:
            ok, reason = client.set_atomically(self.node, values)
            back = client.get(self.node, list(values)) if ok else {}
        except (TimeoutError, ValueError, RuntimeError) as error:
            raise Cancelled(f'driver C not applied: {error}') from None
        if not ok:
            raise Cancelled(f'driver refused C: {reason or "no reason"}')
        for key, want in values.items():
            got = list(back.get(key) or [])
            if len(got) != len(want) or any(abs(a - b) > TOLERANCE for a, b in zip(got, want)):
                raise Cancelled(f'driver {key} reads back {got or "nothing"}, not {want}')

    def raw_correction(self):
        """Back up C+/C- (once) and stage identity on the driver; a refused stage leaves no new backup."""
        fresh = self.pending() is None
        if fresh:
            current = load_config(self.config)
            self._write(correction_backup={key: current.get(key) for key in KEYS})
        self.active = True
        try:
            self.apply_live(IDENTITY)
        except Exception:
            self.active = False
            if fresh:
                self._clear()
            raise

    @contextmanager
    def staged(self):
        """Identity inside the block; any exit but ``commit`` restores. A restore the driver did not take
        stays pending (a motion blocker naming why) instead of masking how the block ended."""
        from mhseals_hardware.menu import Cancelled
        self.raw_correction()
        try:
            yield self
        finally:
            try:
                self.restore()
            except Cancelled as error:
                self.active, self.error = False, str(error)

    def suspend(self):
        """The deployed C back on the driver while a human drives (a pause); the backup stays."""
        backup = self.pending()
        if backup is not None:
            self.apply_live(backup['decoupling'], backup.get('decoupling_negative'))

    def resume(self):
        """Identity again after a pause."""
        if self.pending() is not None:
            self.apply_live(IDENTITY)

    def restore(self):
        backup = self.pending()
        if backup is None:
            self.active = False
            return False
        self.apply_live(backup['decoupling'], backup.get('decoupling_negative'))
        self._clear()
        self.active = False
        return True

    def commit(self, decoupling, decoupling_negative=None, **changes):
        """Save the survey's C+/C- to the YAML, then put them on the driver and verify."""
        update_config(dict(changes, decoupling=decoupling, decoupling_negative=decoupling_negative),
                      self.config)
        self.apply_live(decoupling, decoupling_negative)
        self._clear()
        self.active = False

    def recover(self):
        """Disarmed (no driver runs): the backup goes back to the YAML, which the next driver starts from, and is
        cleared. Returns whether a backup was pending."""
        backup = self.pending()
        if backup is None:
            return False
        current = load_config(self.config)
        if any(current.get(key) != backup.get(key) for key in KEYS):   # an offline stage wrote the YAML
            update_config({key: backup.get(key) for key in KEYS}, self.config)
        self._clear()
        self.error = ''
        return True

    def blockers(self):
        if self.pending() is None or self.active:
            return []
        why = f' ({self.error})' if self.error else ''
        return [f'an interrupted characterization left C staged{why}: A on the Correction tab restores it']
