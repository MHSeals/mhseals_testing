"""Palette, status tokens and glyph table from config/theme.yaml (ui.md section 3); no ROS.

Rendering code names tokens and glyph ids and asks this module for a Rich style, so
no hex value or glyph character lives in a panel. Tiers: ``truecolor``, ``256``
(hand-picked indices), ``16`` (names, no backgrounds) and ``mono`` (NO_COLOR:
attributes only). ASCII is a separate switch for glyphs and chips (``[WORD]``).
An overlay at ~/.config/mhseals/theme.yaml may set ``theme``, ``cvd`` and single
tokens or glyphs; a bad overlay is ignored and reported so the UI can toast it.
"""

import os
from pathlib import Path

import yaml
from rich.text import Text

from mhseals_testing.strict_yaml import load_strict

CONFIG = Path(__file__).resolve().parent.parent.parent / 'config' / 'theme.yaml'
OVERLAY = Path.home() / '.config' / 'mhseals' / 'theme.yaml'
TIERS = ('truecolor', '256', '16', 'mono')
FIELDS = ('dark', 'light', 'x256', 'ansi')
# Contrast against bg: 3:1 for foregrounds, 2:1 for trail ramps (drawn at .72 weight);
# chrome lines and background tints are decoration with a shape cue and are exempt.
EXEMPT = {'bg', 'faint', 'border', 'sel_bg', 'cost_bg', 'lethal_bg', 'grid',
          'cost.1', 'cost.2', 'cost.3', 'inscribed', 'lethal', 'trail.old'}   # backgrounds, quiet map structure
TRAIL = ('gps.', 'xte.', 'spd.', 'trail.', 'wake', 'fov')   # 2:1 floor: quiet map dots
GLYPH_IDS = ('ok', 'warn', 'fail', 'busy', 'off', 'focus', 'cursor', 'sep', 'spinner')   # the code uses these


def default_path():
    """config/theme.yaml in the source tree, else the installed share directory."""
    if CONFIG.is_file():
        return CONFIG
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'theme.yaml'
    except (ImportError, KeyError):
        return CONFIG


def ascii_mode(env=None):
    env = os.environ if env is None else env
    return env.get('TERM') == 'dumb' or any(
        env.get(name, '').lower() in ('1', 'true', 'yes', 'on') for name in ('MHS_ASCII', 'CX_ASCII'))


def detect_tier(env=None):
    """NO_COLOR gives ``mono``; otherwise COLORTERM and TERM pick the richest tier."""
    env = os.environ if env is None else env
    if env.get('NO_COLOR'):
        return 'mono'
    if env.get('COLORTERM', '').lower() in ('truecolor', '24bit'):
        return 'truecolor'
    return '256' if '256color' in env.get('TERM', '') else '16'


def detect_theme(env=None):
    """``light`` when COLORFGBG ends in a light background index, else ``dark``."""
    env = os.environ if env is None else env
    last = env.get('COLORFGBG', '').rsplit(';', 1)[-1]
    return 'light' if last.isdigit() and int(last) in (7, 15) else 'dark'


def _luminance(hex_colour):
    channels = [int(hex_colour[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    r, g, b = [c / 12.92 if c <= 0.03928 else ((c + 0.055) / 1.055) ** 2.4 for c in channels]
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def contrast(a, b):
    """WCAG contrast ratio of two ``#RRGGBB`` colours."""
    hi, lo = sorted((_luminance(a), _luminance(b)), reverse=True)
    return (hi + 0.05) / (lo + 0.05)


def _check_entry(name, entry):
    if not isinstance(entry, dict):
        raise ValueError(f'token {name}: expected a mapping')
    if 'alias' in entry:
        return
    missing = [f for f in FIELDS if f not in entry]
    if missing:
        raise ValueError(f'token {name}: missing {", ".join(missing)}')
    for mode in ('dark', 'light'):
        value = entry[mode]
        if not (isinstance(value, str) and len(value) == 7 and value[0] == '#' and int(value[1:], 16) >= 0):
            raise ValueError(f"token {name}: {mode} {value!r} must be a quoted '#RRGGBB'")
    index = entry['x256']
    if not (isinstance(index, list) and len(index) == 2 and all(isinstance(i, int) and 0 <= i < 256 for i in index)):
        raise ValueError(f'token {name}: x256 must be [dark, light] indices')


class Theme:
    """A validated theme: resolves tokens to Rich styles per tier, and glyphs per mode."""

    def __init__(self, data):
        self.data = data
        self.tokens = data['tokens']
        self.swap = data.get('cvd_swap') or {}
        self.mono = data.get('mono') or {}
        self.attr16 = data.get('attr16') or {}
        self.glyphs = data['glyphs']
        self.mode = data.get('theme', 'auto')
        self.cvd = bool(data.get('cvd', False))
        self.graphics = data.get('graphics', 'auto')   # ui/pixels.py tier: auto | kitty | sixel | text
        self.validate()

    def validate(self):
        if self.mode not in ('auto', 'dark', 'light'):
            raise ValueError(f'theme {self.mode!r} must be auto, dark or light')
        if self.graphics not in ('auto', 'kitty', 'sixel', 'text'):
            raise ValueError(f'graphics {self.graphics!r} must be auto, kitty, sixel or text')
        for name, entry in self.tokens.items():
            _check_entry(name, entry)
            self._leaf(name)
        for name in self.swap:
            _check_entry(f'cvd_swap.{name}', self.swap[name])
            if name not in self.tokens or self._fixed(name):
                raise ValueError(f'cvd_swap.{name}: not a swappable token')
        for name, pair in self.glyphs.items():
            if not (isinstance(pair, list) and len(pair) == 2):
                raise ValueError(f'glyph {name}: expected [unicode, ascii]')
        absent = [g for g in GLYPH_IDS if g not in self.glyphs]
        if absent:
            raise ValueError(f'missing glyph ids: {", ".join(absent)} (quote the key \'off\')')
        bg = self.tokens['bg']
        for name in self.tokens:
            if name in EXEMPT:
                continue
            floor = 2.0 if name.startswith(TRAIL) else 3.0
            entries = [self._leaf(name)] + ([self.swap[name]] if name in self.swap else [])
            for entry in entries:
                for mode in ('dark', 'light'):
                    if contrast(entry[mode], bg[mode]) < floor:
                        raise ValueError(f'token {name}: {mode} contrast below {floor}:1')

    def _fixed(self, name):
        return bool(self.tokens[name].get('fixed'))

    def _leaf(self, name, seen=()):
        """The token that carries values, following aliases; raises on a loop or unknown target."""
        if name in seen:
            raise ValueError(f'token {name}: alias loop')
        if name not in self.tokens:
            raise ValueError(f'token {name}: unknown (alias target)')
        entry = self.tokens[name]
        return self._leaf(entry['alias'], (*seen, name)) if 'alias' in entry else entry

    def _name(self, name, cvd):
        """Leaf token name after aliases; a CVD swap applies unless the alias is fixed."""
        seen = ()
        while 'alias' in self.tokens.get(name, {}):
            cvd = cvd and not self._fixed(name)
            seen, name = (*seen, name), self.tokens[name]['alias']
            if name in seen:
                break
        return name, cvd

    def _entry(self, token, cvd=None):
        if token not in self.tokens:
            raise KeyError(f'unknown colour token {token!r}')
        name, swapped = self._name(token, self.cvd if cvd is None else cvd)
        return self.swap[name] if swapped and name in self.swap else self._leaf(name), name

    def resolve(self, token, mode='dark', tier='truecolor', cvd=None):
        """The colour of a token for a theme mode and tier: ``#hex``, ``color(N)`` or a name."""
        entry, _ = self._entry(token, cvd)
        if tier == 'truecolor':
            return entry[mode]
        if tier == '256':
            return f'color({entry["x256"][mode == "light"]})'
        return entry['ansi'] or ''

    def style(self, token, mode=None, tier=None, cvd=None):
        """A Rich style string for a token."""
        tier = tier or detect_tier()
        if tier == 'mono':
            return self.mono.get(token, self.mono.get(self._entry(token, cvd)[1], ''))
        mode = mode or (detect_theme() if self.mode == 'auto' else self.mode)
        colour = self.resolve(token, mode, tier, cvd)
        if tier == '16' and colour in ('bold', 'default'):
            return '' if colour == 'default' else colour
        if tier == '16':
            # Only the attr16 tokens carry an attribute (ui.md: unknown = bright_black + dim).
            attr = self.attr16.get(token, '')
            return f'{colour} {attr}'.strip() if attr not in colour.split() else colour
        return colour

    def chip(self, word, token, mode=None, tier=None, ascii=None, cvd=None):
        """`` WORD `` as bg-coloured text on a solid token cell; reverse in 16/mono, ``[WORD]`` in ASCII."""
        if ascii_mode() if ascii is None else ascii:
            return Text(f'[{word}]')
        tier = tier or detect_tier()
        mode = mode or (detect_theme() if self.mode == 'auto' else self.mode)
        if tier in ('16', 'mono'):
            fore = self.style(token, mode, tier, cvd)
            return Text(f' {word} ', style=f'{fore} reverse'.strip())
        fore = self.resolve('bg', mode, tier)
        return Text(f' {word} ', style=f'{fore} on {self.resolve(token, mode, tier, cvd)}')

    def glyph(self, glyph_id, ascii=None):
        """The Unicode glyph, or its ASCII stand-in in ASCII mode."""
        pair = self.glyphs[glyph_id]
        return pair[1 if (ascii_mode() if ascii is None else ascii) else 0]


def _merge(base, overlay):
    merged = {key: (dict(value) if isinstance(value, dict) else value) for key, value in base.items()}
    for key, value in overlay.items():
        if key in ('tokens', 'cvd_swap', 'mono', 'attr16', 'glyphs'):
            if not isinstance(value, dict):
                raise ValueError(f'{key}: expected a mapping')
            merged.setdefault(key, {}).update(value)
        elif key in ('theme', 'cvd', 'graphics'):
            merged[key] = value
        elif key != 'schema':
            raise ValueError(f'unknown key {key!r}')
    return merged


def load_theme(path=None, overlay=None):
    """``(Theme, warning)``: the shipped file plus the user overlay; a bad overlay is ignored.

    A broken shipped file raises ValueError naming the file. ``overlay=False`` skips the overlay.
    """
    path = Path(path or default_path())
    try:
        base = load_strict(path.read_text(), path)
        if not isinstance(base, dict) or base.get('schema') != 'mhseals.theme/1':
            raise ValueError('expected schema mhseals.theme/1')
        theme = Theme(base)
    except (ValueError, yaml.YAMLError, KeyError, AttributeError, OSError) as error:
        raise ValueError(f'{path}: {error}') from error
    overlay_path = Path(OVERLAY if overlay is None else overlay) if overlay is not False else None
    if overlay_path is None or not overlay_path.is_file():
        return theme, None
    try:
        user = load_strict(overlay_path.read_text(), overlay_path) or {}
        if not isinstance(user, dict):
            raise ValueError('expected a mapping')
        return Theme(_merge(base, user)), None
    except (ValueError, yaml.YAMLError, KeyError, AttributeError, OSError) as error:
        return theme, f'theme overlay {overlay_path} ignored: {error}'
