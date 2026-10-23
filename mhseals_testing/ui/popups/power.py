"""popup.power: the thruster-bus pack in full (ui.md 4.3, battery.md).

``rows`` turns the ``power.*`` facts of ``analysis.power.Power`` into label/value rows: pack and
cell voltage, percent, current, sag, the pack-swap detection and the voltage source. With no
source the facts are None and the popup says ``no_voltage_source`` (voltage scaling stays prior).
The Battery row on Status exists only with a source, so that is the only way in.
"""

from mhseals_testing.ui import theme

DASH = theme.DASH
NO_SOURCE = 'no_voltage_source'
SEP = theme.SEPARATOR.strip()


def _num(value, fmt, unit=''):
    return DASH if value is None else format(value, fmt) + unit


def rows(facts):
    """Label/value rows for one facts dict, or the no-source row for None."""
    if not facts:
        return [('Source', NO_SOURCE), ('Note', 'no /mavros/battery voltage; voltage scaling stays prior')]
    cells = facts.get('cells')
    pct = facts.get('pct')
    swaps = facts.get('swaps', 0)
    out = [('Source', facts.get('source') or DASH),
           ('Pack', f'{_num(facts.get("volt"), ".2f", " V")}{f" {SEP} {cells}S" if cells else ""}'),
           ('Cell', f'{_num(facts.get("cell_min"), ".2f", " V")} min'
                    + ('' if facts.get('cells_match') is not False
                       else f' {SEP} profile cells differ from detected {facts.get("cells_detected")}S')),
           ('Percent', _num(pct, '.0f', '%') + (' (from voltage)' if facts.get('pct_estimated') else '')),
           ('Current', f'{_num(facts.get("amps"), "+.1f", " A")} {SEP} {_num(facts.get("watts"), ".0f", " W")}'),
           ('Sag', _num(facts.get('sag_mV'), '.0f', ' mV')),
           ('Used', _num(facts.get('mah_used'), '.0f', ' mAh')),
           ('Left', _num(facts.get('remaining_min'), '.0f', ' min')),
           ('Pack swap', f'{swaps} detected {SEP} usage reset' if swaps else 'none detected')]
    return out


def modal(facts):
    return dict(kind='message', title='Power', rows=rows(facts), width=64, hint='Esc close')


def open_power(app):
    """Show the popup for the live facts, refreshed while open; Esc or q closes."""
    m = modal(app.power_facts())
    app.popup(m, lambda key: True if key in ('escape', 'q') else None,
              lambda: m.update(rows=rows(app.power_facts())))
