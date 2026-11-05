"""popup.conditions: the water-conditions estimate in full (ui.md 6.3, conditions_minimap.md 5.1).

``rows`` turns a ``Conditions.summary()`` and ``Conditions.detail()`` into label/value rows (panel
figures, D, beta with its z, the change state, then the window table); ``open_conditions`` is the
modal loop with up/down (k/j) scrolling. Nothing here estimates anything.
"""

import math


from mhseals_testing.health import config as health_config
from mhseals_testing.ui import keymap, theme

DASH = theme.DASH
GATED = {'f': 'Characterize start'}     # procedure key -> what starting it means
SEP = theme.SEPARATOR.strip()
UPDOWN = theme.pick('↑↓', 'Up/Down')


def gates(path=None):
    """Conditions checks that gate a procedure start: ``[(check id, severity, 'Characterize start', expected verdicts)]``."""
    out = []
    for group, section in health_config.sections(path):
        for check in section.get('checks', []) if group == 'Conditions' else ():
            for key in check.get('gates', ()):
                out.append((check['id'], check['severity'], GATED.get(key, key), check.get('expect', [])))
    return out


def _num(value, fmt):
    return DASH if value is None else format(value, fmt)


def _deg(rad):
    """rad/s -> deg/s, or None."""
    return None if rad is None else math.degrees(rad)


def table(windows):
    """Window table rows: label = time before the newest sample, value = aligned figures."""
    head = f'{"T":>4}  {"v_N":>7}  {"v_E":>7}  {"psi":>4}  {"r-bar":>7}  flag'
    out = [('Windows', head if windows else DASH)]
    for w in windows:
        out.append((f'-{w["ago"]:.0f} s', f'{w["T"]:4.0f}  {w["v_n"]:+7.3f}  {w["v_e"]:+7.3f}  {w["psi"]:4.0f}  '
                                          f'{w["r"]:+7.2f}  {w["flag"]}'))
    return out


def rows(summary, detail=None, gate_list=()):
    """Label/value rows for one summary dict, its estimator detail (or None) and the gates (loaded once)."""
    s, d = summary, detail or {}
    out = [('Verdict', f'{s.get("verdict", "unknown")} {SEP} ceiling {_num(s.get("ceiling"), ".0%")}')]
    if s.get('reason'):
        out.append(('Reason', s['reason']))
    drift = s.get('drift')
    if drift is None:
        out.append(('Drift', 'measuring'))
    else:
        speed = math.hypot(*drift)
        out.append(('Drift', f'{speed:.2f} m/s (E {drift[0]:+.2f}, N {drift[1]:+.2f}) {SEP} '
                             f'CI {_num(s.get("se"), ".3f")} {SEP} trust {_num(s.get("drift_trust"), ".0%")}'))
    trend = d.get('trend')
    out.append(('D', _num(d.get('dispersion'), '.2f')))
    out.append(('beta', DASH if trend is None else
                f'{math.hypot(*trend) * 60:.3f} m/s per min {SEP} z {d.get("trend_z", 0.0):.1f}'))
    out.append(('State', s.get('state', 'unknown') + (f' {SEP} {d["pool"]} windows pooled' if d.get('pool') else '')))
    out.append(('Wave floor', 'yaw ' + _num(_deg(s.get('floor')), '.2f') + ' deg/s'))
    out.append(('Sea state', _num(_deg(s.get('sea')), '.1f') + ' deg/s'))
    out.append(('Neutral', _num(s.get('neutral_s'), '.0f') + ' s'))
    for check_id, severity, what, expect in gate_list:
        out.append(('Gates', f'{what}: {severity} unless {"/".join(expect)}  ({check_id})'))
    return out + table(d.get('windows', ()))


def modal(summary, detail=None, gate_list=()):
    return dict(kind='message', title='Conditions', rows=rows(summary, detail, gate_list), width=76,
                hint=f'{UPDOWN} scroll{theme.SEPARATOR}Esc close')


def open_conditions(app):
    """Show the popup for the live estimate, refreshed while it is open; Esc or q closes, up/down scroll."""
    gate_list = gates()                         # read once at open
    rebuild = lambda: rows(app.conditions.summary(), app.conditions.detail(), gate_list)
    m = modal(app.conditions.summary(), app.conditions.detail(), gate_list)

    def handle(key):
        action = keymap.dispatch('popup.conditions', key)
        if action == 'close':
            return True
        app.scroll(m, action, ('up', 'down'))
    app.popup(m, handle, lambda: m.update(rows=rebuild()))
