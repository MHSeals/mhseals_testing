"""Thrusters panel: output map, reversal, pulse and readback per position."""

import re

from rich.text import Text

from mhseals_hardware.odroid_pwm import J2_PINS
from mhseals_hardware.thruster_mixer import POSITIONS, thruster_pulses, validate_decoupling
from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES


SATURATED = 0.98    # |effort| at or above this is drawn as saturated
NEUTRAL = 1500


def effort_bar(pulse, testing=False, width=9, saturated=False):
    """A centred bar: left of the tick is reverse, right is forward."""
    effort = max(-1.0, min(1.0, (pulse - NEUTRAL) / 400))
    half = width // 2
    cells = [' '] * width
    cells[half] = theme.BAR
    for step in range(1, round(abs(effort) * half) + 1):
        cells[half + step if effort > 0 else half - step] = theme.FILL
    return Text(''.join(cells), style=theme.WARN if testing or saturated else theme.ACCENT)


READBACK_TOLERANCE = 2  # µs between the sysfs duty and the commanded pulse


def readback_glyph(text, pulse=None, armed=True):
    """Rb: ✓ when the sysfs duty matches the commanded ``pulse`` while armed, × when it does not
    (or is on while disarmed), · when off, ? when unreadable, a dash when not read yet."""
    if text.startswith('ON'):
        match = re.match(r'ON\s+(\d+)\s*us', text)
        good = (armed and match is not None and pulse is not None
                and abs(int(match.group(1)) - pulse) <= READBACK_TOLERANCE)
        return '✓' if good else '×'
    return (theme.MIDDOT if text.startswith('OFF') else '?' if text == 'unavailable'
            else theme.DASH)


def split_decoupling(config):
    """(decoupling, decoupling_negative or None) as validated matrices."""
    negative = config.get('decoupling_negative')
    return (validate_decoupling(config['decoupling']),
            None if negative is None else validate_decoupling(negative))


def thruster_rows(view):
    """Per position: physical output, reversal and the pulse it gets now."""
    config = view['draft'] or view['hardware']
    channel_map, reversed_ = config['channel_map'], config.get('reversed', [False] * 4)
    if view['test_pulses'] is not None:
        by_output = view['test_pulses']
    elif view['units'] == 'effort' and view['armed']:
        by_output = thruster_pulses(view['command'], view['matrix'], channel_map, reversed_,
                                    *split_decoupling(view['hardware']))
    else:
        by_output = [NEUTRAL] * 4
    saved = view['hardware']
    return [dict(position=position.upper(), output=output, reversed=flip,
                 pulse=by_output[output - 1], pin=J2_PINS[output][0],
                 saturated=abs(by_output[output - 1] - NEUTRAL) >= SATURATED * 400,
                 readback=view['pwm_readback'][output - 1],
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
        columns.insert(1, 'Pin')
        columns.append('Rb')
    table = theme.grid(*columns, padding=(0, 1))
    saturated = []
    for index, row in enumerate(thruster_rows(view)):
        active = view['armed'] and row['pulse'] != NEUTRAL
        state = ('WARN' if testing else 'LIVE') if active else ('IDLE' if view['armed'] else 'OFF')
        grabbed = view['grabbed'] == index
        if active and row['saturated']:
            saturated.append(row['position'])
        label = Text(row['position'] + (' ⇅' if grabbed else ''))
        cells = [theme.row(state, label, focused and index == cursor)]
        if pins:
            cells.append(Text(f"p{row['pin']}", style=theme.MUTED))
        cells += [
            Text(str(row['output']), style=f'bold {theme.WARN}' if row['changed'] else None),
            Text(theme.pick('−', '-') if row['reversed'] else theme.MIDDOT, style=theme.WARN if row['reversed'] else theme.MUTED),
            Text(f"{row['pulse']}" + ('' if compact else theme.pick(' µs', ' us')), style=theme.WARN if row['saturated'] and active else 'bold' if active else theme.MUTED)]
        if not compact:
            cells.append(effort_bar(row['pulse'], testing, saturated=row['saturated'] and active))
        if pins:
            cells += [Text(readback_glyph(row['readback'], row['pulse'], view['armed']))]
        table.add_row(*cells)
    driver = view.get('driver') or {}
    parts = ['testing' if testing else driver.get('owner') or view['outputs']]
    if pins and driver.get('mosfet') is not None:
        parts.append(f"p{J2_PINS['mosfet'][0]} {'on' if driver['mosfet'] else 'off'}")
    saved = view['hardware']
    if saved.get('decoupling_negative') not in (None, saved['decoupling']):
        parts.append('C± split')
    if saturated:
        parts.append('sat ' + ','.join(saturated))
    parts.append('unsaved' if view['draft'] else
                 'config out of sync' if driver.get('config_sync') is False else view['config_note'])
    subtitle = theme.SEPARATOR.join(parts)
    return theme.block(table, TITLES['thrusters'], focused, subtitle)
