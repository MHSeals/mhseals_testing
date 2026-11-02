"""Help and footer hints: the key tables the help popup and the hint grid show."""

from mhseals_testing.ui import keymap as keymap_module, theme, widgets
from mhseals_testing.ui.panels import TITLES

from mhseals_testing.ui.app.shared import PULSE_STEP


UPDOWN, LEFTRIGHT = theme.pick('↑↓', 'Up/Down'), theme.pick('←→', 'Left/Right')
MICRO, PLUSMINUS = theme.pick('µ', 'u'), theme.pick('±', '+/-')
GLOBAL_HINTS = [('Ctrl hjkl', 'Focus'), ('a', 'Arm panel'), ('Space', 'Stop'), ('?', 'Help'),
                ('q', 'Quit')]
PANEL_HINTS = {
    'status': [(UPDOWN, 'Select'), ('Enter', 'Details')],
    'motion': [('m', 'Manual'), ('+/-', 'Amplitude')],
    'conditions': [('Enter', 'Details')],
    'thrusters': [(LEFTRIGHT, 'Pulse'), ('Enter', 'Move'), ('1-4', 'Output'), ('r', 'Reverse'),
                  ('s', 'Save'), ('u', 'Undo'), ('0', 'Neutral'), ('i', 'Identify'),
                  ('c', 'Clear decoupling'), ('P', 'Pin'), ('M', 'Pins')],
    'procedures': [('Enter', 'Run'), (UPDOWN, 'Select')],
    'activity': [('Enter', 'Run'), ('v', 'View')],
    'minimap': [('z/Z', 'Zoom'), ('0', 'Reset zoom'), ('u', 'Heading-up'), ('F', 'Fov')],
    'map': [('c', 'Costmap'), ('d', 'Tracks'), ('y', 'Trail'), ('z/Z', 'Zoom'), ('u', 'Heading-up'),
            ('F', 'Fov'), ('w', 'Wake'), ('g', 'Rings'), ('T', 'Tasks')],
    'list': [(UPDOWN, 'Select'), ('p', 'Pin'), ('n', 'Title'), ('d', 'Delete'), ('P', 'Prune'), ('F', 'Finalize')],
}
FUSION_HINTS = [('i', 'Image')]
CORRECTION_HINTS = [(LEFTRIGHT, 'Direction'), ('Enter', 'Holds'), ('r', 'Rerun direction'),
                    ('R', 'Run all'), ('A', 'Apply pending'), ('I', 'Hardware changed')]
REVIEW_HINTS = [('p', 'Play/pause'), (f'{LEFTRIGHT} {{ }}', 'Seek 5 s / 30 s'),
                (', .', 'Prev/next mark'), ('< >', 'Rate'), ('Esc', 'Leave review'),
                ('?', 'Help'), ('q', 'Quit')]


MANUAL_HINTS = [(f'W/S {UPDOWN}', 'Surge'), ('A/D', 'Sway'), (LEFTRIGHT, 'Yaw'), ('+/-', 'Speed'),
                ('Space', 'Stop'), ('x / Esc', 'Leave manual')]
HELP = [   # '[Screen ]keys', as test_help_keys reads them against config/keymap.yaml
    ('Ctrl h j k l / arrows', 'Focus the panel to the left, below, above, right'),
    ('Tab / Shift Tab', 'Cycle focus through the panels'),
    ('[ / ]', 'Previous / next tab: Drive, Correction, Map, Bags, Fusion'),
    (f'{UPDOWN} / j k', 'Move within the focused panel'),
    ('Enter', 'Run, open or show details'),
    ('Esc', 'Back out (drops a picked-up thruster); never quits'),
    ('Space', 'Stop: hold, neutral and disarm; aborts a run'),
    ('a', 'Arm panel: outputs, interlocks, arm or disarm'),
    ('U', 'Release the hold (confirm names who resumes)'),
    ('E', "Revoke the agent's claim (stops first, no confirm)"),
    ('O', 'Take control (viewer; confirm: the owner stops at neutral, disarms, revokes the agent)'),
    ('S', 'Operator state: normal, manual, battery swap, power cycle (agents pause; m sets manual)'),
    ('o', 'Pose: what the EKFs fuse, and the sources'),
    ('!', 'Health checks: mute, recheck, acknowledge'),
    ('W', 'Preview checklist (boat_tui preview): check on/off; then p pass, f fail, n note, Esc'),
    ('H', 'Config history: hardware, Nav2, FCU, RTK, health, keymap'),
    ('V', 'Perception with the camera view open'),
    ('+ / -', 'Trial amplitude'),
    ('x', 'Write the session report (dismisses a toast first)'),
    ('b / B', 'Mark the moment (journal) / add a note to the last marker'),
    ('?', 'This help'),
    ('q', 'Quit: neutral, disarm, flush the bag'),
    ('Drive f d y g e v', 'Characterize, dock / thrust physics, compass cal / '
                          'interference, controller steps'),
    ('Drive m i n w s t p', 'Manual, thruster set, Nav2, square, station, mission, perception '
                            '(focused-panel keys win)'),
    (f'Status {LEFTRIGHT}', 'Change a setting row: Agents, agent motion, recording, graphics'),
    ('Status e', 'Nav2 parameters (on the Navigation row)'),
    (f'Thrusters {LEFTRIGHT}',
     f'Test pulse {PLUSMINUS}{PULSE_STEP} {MICRO}s on the selected thruster (armed)'),
    ('Thrusters 0', 'All test pulses back to neutral'),
    ('Thrusters Enter', f'Pick up the thruster; {UPDOWN} moves it, Enter drops it'),
    ('Thrusters 1-4 / r', 'Wire it to an output / reverse it (a draft until saved)'),
    ('Thrusters s / u',
     'Save the draft to the config (the next session starts from it) / undo it'),
    ('Thrusters M', 'Edit the PWM pins and MOSFET line; saved, then the driver restarts, sysfs '
                    'confirms and it stops (disarmed only; refused while armed or arming)'),
    ('Thrusters P', 'Move the thruster to another free PWM pin (a draft; s saves and reads it back)'),
    ('Thrusters i', 'Thruster set: identify outputs by pulsing each one'),
    ('Thrusters c', 'Clear the fitted decoupling (asks first)'),
    ('Activity v', 'Cycle results, event log and the agent desk (while it runs)'),
    ('Activity X', 'Exclude the selected result (asks a reason)'),
    ('Map c / C', 'Costmap layer / source: auto, local, global (also the Drive minimap)'),
    ('Map r', 'Costmap hits layer'),
    ('Map d / D', 'Detections: tracks, raw, off / confidence floor'),
    ('Map y / Y', 'Trail mode / restart the trail'),
    ('Map F / u', 'Camera field of view / north-up or heading-up'),
    ('Map z / Z / 0', 'Zoom in / out / fit'),
    ('Map w g T', 'Wake, rings and task layers (Map tab only)'),
    (f'Correction {LEFTRIGHT}', 'Pick a direction column; Enter shows its holds'),
    ('Correction r / R', 'Rerun the column / run all (both start Characterize for now)'),
    ('Correction A', 'Apply pending corrections (runtime apply is unsupported; says so)'),
    ('Correction I', 'Hardware changed: a note, then the epoch bump (every fit goes stale)'),
    ('Bags Enter', 'Review the selected session'),
    ('Bags p / n', 'Pin / title the session'),
    ('Bags d / P', 'Delete the session / prune old ones'),
    ('Bags F', 'Finalize the session'),
    ('Fusion i', 'Camera image under the points and boxes, on or off (kitty/sixel graphics; on by default '
                 'locally, off over SSH)'),
    ('Review p', 'Play / pause (arming, procedures and edits are off)'),
    (f'Review {LEFTRIGHT} / {{ }}', 'Seek 5 s / 30 s'),
    ('Review , .', 'Previous / next marker'),
    ('Review < >', 'Playback rate'),
    ('Review z', 'Activity loupe zoom'),
    ('Review Esc', 'Back to live'),
    (f'Manual w s {UPDOWN}', 'Surge (held keys drive; releasing them stops)'),
    (f'Manual a d / {LEFTRIGHT}', 'Sway / yaw'),
    ('Manual + / -', 'Speed'),
    ('Manual Space', 'Stop and hold; manual stays open'),
    ('Manual x / m / Esc', 'Leave manual'),
    ('Run Space / q / Esc', 'Abort the running procedure'),
    ('Run m', 'Take over in manual (the run ends at neutral)'),
]


def map_key_maps(doc):
    """``(drive, map)`` key -> action of the map views: view.minimap on the Drive column, and on
    the Map tab with tab.map's toggles (w g) over it, which are Drive procedure keys elsewhere.
    view.minimap's own e (centre) shadows Interference on both, so a focused map never starts it."""
    view = keymap_module.key_map(doc.get('view.minimap'))
    return view, {**keymap_module.key_map(doc.get('tab.map')), **view}


def help_rows(profile, entries):
    """Help as popup rows: the profile, the keys that work anywhere, then one section per panel.

    A ``Thrusters 0`` entry lists under a Thrusters rule as ``0``, so the panel is read once and the
    key column stays narrow. The entries' text is used as written.
    """
    panels = set(TITLES.values())
    rows, sections = [(profile.name, profile.description), (widgets.SECTION, 'Anywhere')], {}
    for key, text in entries:
        head, _, rest = key.partition(' ')
        if head in panels and rest:
            sections.setdefault(head, []).append((rest, text))
        else:
            rows.append((key, text))
    for panel, items in sections.items():
        rows += [(widgets.SECTION, panel), *items]
    return rows


def help_width(columns=None):
    """Wider help on a wide terminal, so fewer lines wrap and less of it scrolls; 76 without a width (no console)."""
    return max(76, min(104, columns - 12)) if columns else 76
