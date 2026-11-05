"""The Correction tab: its state, readiness and tracking side, focus line and keys."""

from mhseals_testing import calibration, data
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import correction


class Correction:
    """The Correction tab, mixed into Settings."""

    def correction_state(self):
        """What the Correction tab draws besides the config: the column and the state.json fits."""
        if self.tab != 'Correction':
            return None
        saved = (data.read_json(data.path('state.json'), {}) or {}).get('fits') or {}
        current = data.read_json(data.path('calibration', 'current.json'), {})
        derived = correction.provenance_fits(current, self.conditions.evaluate().get('ceilings'))
        fits = {d: {**derived.get(d, {}), **saved.get(d, {})} for d in {*derived, *saved}}
        shown = calibration.marks(self.args.hardware_config, self.history)
        fits = {**(fits or {}), **{d: {**(fits or {}).get(d, {}), 'mark': m} for d, m in shown.items()}} if shown else fits
        return dict(fits=fits, column=self.column, **self.correction_side())

    def correction_side(self):
        """Readiness (conditions + the characterize gates) and Tracking (feed + calibration file) models."""
        blocking, warnings = (), ()
        if self.engine is not None:
            # gates are keyed by the procedure letter, as availability() does
            procedure = next(p for p in PROCEDURES if p.title == 'Characterize')
            blocking, warnings = self.engine.gate(
                procedure.key, self.args.bench_override, 'armed' in procedure.needs)
        characterization = self.args.hardware_config.get('characterization') or {}
        provenance = (data.read_json(data.path('calibration', 'current.json'), {}) or {}).get('provenance') or {}
        return dict(
            readiness=correction.readiness_data(self.conditions.summary(), blocking, warnings,
                                                characterization.get('wave_floor')),
            tracking=correction.tracking_data(self.tracking.scores, self.tracking.axes,
                                              provenance.get('tracking') or {}, self.tracking.version))

    def correction_focus(self):
        """(title, position) of the footer's focus line: the cursor column, as in ui 5.7."""
        model = correction.tab_view(self.args.hardware_config, self.correction_state())
        return 'Correction', f"{model['columns'][model['cursor']]['key'].replace('-', theme.pick('−', '-'))} column"

    def on_correction(self, key):
        """Correction tab keys (keymap tab.correction); True when the key was one of them."""
        step = {'left': -1, 'h': -1, 'right': 1, 'l': 1}.get(key)
        if step is not None:
            model = correction.tab_view(self.args.hardware_config, self.correction_state())
            self.column = correction.move(model, step)
        elif key in ('r', 'R'):
            model = correction.tab_view(self.args.hardware_config, self.correction_state())
            self.characterize(model['columns'][model['cursor']]['key'] if key == 'r' else '')
        elif key == 'enter':
            model = correction.tab_view(self.args.hardware_config, self.correction_state())
            holds = correction.hold_rows(model)
            self.show(f"{model['columns'][model['cursor']]['key']} holds", rows=holds, width=50)
        elif key == 'A':
            self.apply_pending()
        elif key == 'I':
            self.hardware_changed()
        else:
            return False
        return True
