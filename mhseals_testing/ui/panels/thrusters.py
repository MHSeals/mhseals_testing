"""Thrusters panel: output map, reversal, pulse and readback per position."""

from rich.text import Text

from mhseals_hardware.thruster_mixer import POSITIONS, thruster_pulses, validate_decoupling
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES
from mhseals_testing.ui.theme import ACCENT, MUTED, SEPARATOR


READBACK_SHORT = {'unavailable': 'n/a'}  # keeps the Pin column inside a half-width panel
NEUTRAL = 1500


def effort_bar(pulse, testing=False, width=9):
    """A centred bar: left of the tick is reverse, right is forward."""
    effort = max(-1.0, min(1.0, (pulse - NEUTRAL) / 400))
    half = width // 2
    cells = [' '] * width
    cells[half] = theme.BAR
    for step in range(1, round(abs(effort) * half) + 1):
        cells[half + step if effort > 0 else half - step] = theme.FILL
    return Text(''.join(cells), style=theme.WARN if testing else ACCENT)


def thruster_rows(view):
    """Per position: physical output, reversal and the pulse it gets now."""
    config = view['draft'] or view['hardware']
    channel_map, reversed_ = config['channel_map'], config.get('reversed', [False] * 4)
    if view['test_pulses'] is not None:
        by_output = view['test_pulses']
    elif view['units'] == 'effort' and view['armed']:
        by_output = thruster_pulses(view['command'], view['matrix'], channel_map, reversed_,
                                    validate_decoupling(view['hardware']['decoupling']))
    else:
        by_output = [NEUTRAL] * 4
    saved = view['hardware']
    return [dict(position=position.upper(), output=output, reversed=flip,
                 pulse=by_output[output - 1], readback=view['pwm_readback'][output - 1],
                 changed=(output != saved['channel_map'][index]
                          or flip != saved['reversed'][index]))
            for index, (position, output, flip) in enumerate(
                zip(POSITIONS, channel_map, reversed_))]


def thrusters_panel(view, compact=False, pins=True):
    """Thruster table; ``compact`` (also forced by the Pin column) drops the unit and effort bar.

    ``pins`` False drops the odroid Pin column where the panel is too narrow for it.
    """
    focused = view['focus'] == 'thrusters'
    cursor = view['cursors'].get('thrusters', 0)
    testing = view['test_pulses'] is not None
    pins = pins and view['outputs'] == 'odroid'
    compact = compact or pins  # the Pin column leaves no room for the bar
    columns = (['   Thr', 'Out', 'Rev', {'header': theme.pick('µs', 'us'), 'justify': 'right'}] if compact else
               ['    Thr', 'Out', 'Rev', {'header': 'Pulse', 'justify': 'right'}, 'Effort'])
    if pins:
        columns.append('Pin')
    table = theme.grid(*columns, padding=(0, 1))
    for index, row in enumerate(thruster_rows(view)):
        active = view['armed'] and row['pulse'] != NEUTRAL
        state = ('WARN' if testing else 'LIVE') if active else ('IDLE' if view['armed'] else 'OFF')
        grabbed = view['grabbed'] == index
        label = Text(row['position'] + (' ⇅' if grabbed else ''))
        cells = [theme.row(state, label, focused and index == cursor),
                 Text(str(row['output']), style=f'bold {theme.WARN}' if row['changed'] else None),
                 Text(theme.pick('↺', '(') if row['reversed'] else theme.MIDDOT, style=theme.WARN if row['reversed'] else MUTED),
                 Text(f"{row['pulse']}" + ('' if compact else theme.pick(' µs', ' us')), style='bold' if active else MUTED)]
        if not compact:
            cells.append(effort_bar(row['pulse'], testing))
        if pins:
            cells.append(Text(READBACK_SHORT.get(row['readback'], row['readback']), style=MUTED))
        table.add_row(*cells)
    subtitle = ('testing' if testing else view['outputs']) + SEPARATOR + (
        'unsaved' if view['draft'] else view['config_note'])
    return theme.block(table, TITLES['thrusters'], focused, subtitle)
