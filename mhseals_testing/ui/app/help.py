"""Help and footer hints: the key tables the help popup and the hint grid show, read from
config/keymap.yaml's help and hints sections (checked by keymap.validate, rule 7); README.md's key tables
are generated from the same rows (python3 -m mhseals_testing.ui.app.help --readme)."""

import re
import sys
from pathlib import Path

from mhseals_testing.strict_yaml import load_strict
from mhseals_testing.ui import keymap as keymap_module, theme, widgets
from mhseals_testing.ui.panels import TITLES

from mhseals_testing.ui.app.shared import PULSE_STEP


UPDOWN, LEFTRIGHT = theme.pick('↑↓', 'Up/Down'), theme.pick('←→', 'Left/Right')
MICRO, PLUSMINUS = theme.pick('µ', 'u'), theme.pick('±', '+/-')
FIELDS = dict(updown=UPDOWN, leftright=LEFTRIGHT, micro=MICRO, plusminus=PLUSMINUS, pulse_step=PULSE_STEP)


def tables(doc=None, fields=None):
    """``(help, hints)`` from config/keymap.yaml's help and hints sections, {fields} filled in:
    help is [(label, text)], hints maps a group (``global``, ``panel.status``, ``manual``) to its rows."""
    doc = doc if doc is not None else load_strict(keymap_module.shipped_file().read_text(), keymap_module.shipped_file())
    fields = FIELDS if fields is None else fields

    def rows(items):
        return [(key.format(**fields), text.format(**fields)) for key, text, *_ in items]
    return rows(doc.get('help', [])), {group: rows(items) for group, items in doc.get('hints', {}).items()}


HELP, _HINTS = tables()   # '[Screen ]keys', as keymap.validate reads them (rule 7)
GLOBAL_HINTS, FUSION_HINTS = _HINTS['global'], _HINTS['fusion']
CORRECTION_HINTS, REVIEW_HINTS, MANUAL_HINTS = _HINTS['correction'], _HINTS['review'], _HINTS['manual']
PANEL_HINTS = {group.removeprefix('panel.'): rows for group, rows in _HINTS.items() if group.startswith('panel.')}


README = Path(__file__).resolve().parents[3] / 'README.md'
README_BLOCK = re.compile(r'(<!-- keys: config/keymap.yaml -->\n).*?(<!-- /keys -->)', re.S)   # the table between
UNICODE = dict(updown='↑↓', leftright='←→', micro='µ', plusminus='±', pulse_step=PULSE_STEP)


def readme_tables(doc=None):
    """README.md's key table: the help rows that carry a README text (their third string)."""
    doc = doc if doc is not None else load_strict(keymap_module.shipped_file().read_text(), keymap_module.shipped_file())
    picked = [(key.format(**UNICODE).removeprefix('Manual '), readme) for key, _, *more in doc.get('help', [])
              for readme in more]
    return '\n'.join(['| Key | Does |', '| --- | --- |'] + ['| `%s` | %s |' % row for row in picked]) + '\n'


def readme(text, doc=None):
    """``text`` (README.md) with its generated key tables replaced."""
    return README_BLOCK.sub(lambda m: m[1] + readme_tables(doc) + m[2], text)


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


if __name__ == '__main__':   # python3 -m mhseals_testing.ui.app.help --readme: regenerate README.md's key tables
    if sys.argv[1:] != ['--readme']:
        sys.exit('usage: python3 -m mhseals_testing.ui.app.help --readme')
    README.write_text(readme(README.read_text()))
