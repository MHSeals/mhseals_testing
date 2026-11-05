import pytest

from mhseals_testing.analysis.classes import load_classes
from mhseals_testing.ui import palette


@pytest.fixture(scope='module')
def theme():
    return palette.load_theme(overlay=False)[0]


def test_status_tokens_map_onto_the_theme_constants():
    from mhseals_testing.ui import theme as ui
    assert (ui.GOOD, ui.WARN, ui.BAD, ui.BUSY) == tuple(
        ui.PALETTE.style(t) for t in ('ok', 'warn', 'fail', 'busy'))


def test_every_map_class_colour_is_a_theme_token(theme):
    assert all(symbol.colour in theme.tokens for symbol in load_classes())
    assert all(token in theme.tokens for token in ('ok', 'warn', 'fail', 'busy', 'muted'))


def test_class_with_an_unknown_colour_token_is_rejected(tmp_path):
    text = (palette.default_path().parent / 'minimap.yaml').read_text().replace('obj.blue', 'obj.bleu')
    path = tmp_path / 'minimap.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match='obj.bleu.*not a theme.yaml token'):
        load_classes(path)


def test_an_unknown_fov_colour_token_is_rejected(tmp_path):
    text = (palette.default_path().parent / 'minimap.yaml').read_text().replace('colour: fov', 'colour: fvo')
    path = tmp_path / 'minimap.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match="fov: colour 'fvo'"):
        load_classes(path)


# Hue families that must never be mistaken for each other: a buoy is not a health state is not
# who drives. Within a family hues may meet (a red buoy and a red pole are one colour).
FAMILIES = {'buoy': ('obj.red', 'obj.green', 'obj.yellow', 'obj.blue'),
            'severity': ('ok', 'warn', 'fail'),
            'authority': ('auth.ours', 'auth.nav', 'auth.agent'),
            'gps': ('gps.fix', 'gps.float', 'gps.dgps', 'gps.3d', 'gps.none'),   # the trail, on the map with buoys
            'mission': ('task.confirmed', 'task.failed', 'task.goal'),   # the decision words alias these
            'scope': ('scope',),     # the hw / cal tag is not a status, buoy or authority
            'stop': ('stop',),       # the safe stopped state is not a failure
            'armed': ('armed', 'arm.word')}     # and the hot armed state is neither
ALIKE = {frozenset(('severity', 'gps'))}   # fix quality reads as health on purpose: fix is ok, none is fail
MIN_DELTA_E = 7.5   # CIEDE2000; the reviewed dark theme's closest pair (yellow buoy, warn) is 7.9
FAMILIES['agent vendor'] = ('agent.claude', 'agent.codex')   # which coding agent ran a command


def _apart():
    """``((name, tokens), (name, tokens))`` for each pair of families that must stay apart."""
    import itertools
    return [(one, two) for one, two in itertools.combinations(FAMILIES.items(), 2)
            if frozenset((one[0], two[0])) not in ALIKE]


def _lab(rgb):
    def linear(c):
        c /= 255
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
    r, g, b = map(linear, rgb)
    xyz = ((0.4124 * r + 0.3576 * g + 0.1805 * b) / 0.95047, 0.2126 * r + 0.7152 * g + 0.0722 * b,
           (0.0193 * r + 0.1192 * g + 0.9505 * b) / 1.08883)
    fx, fy, fz = (t ** (1 / 3) if t > 216 / 24389 else (24389 / 27 * t + 16) / 116 for t in xyz)
    return 116 * fy - 16, 500 * (fx - fy), 200 * (fy - fz)


def delta_e(one, two):
    """CIEDE2000 between two Lab colours."""
    import math
    (l1, a1, b1), (l2, a2, b2) = one, two
    c_bar = (math.hypot(a1, b1) + math.hypot(a2, b2)) / 2
    g = 0.5 * (1 - math.sqrt(c_bar ** 7 / (c_bar ** 7 + 25 ** 7)))
    a1, a2 = a1 * (1 + g), a2 * (1 + g)
    c1, c2 = math.hypot(a1, b1), math.hypot(a2, b2)
    h1, h2 = math.degrees(math.atan2(b1, a1)) % 360, math.degrees(math.atan2(b2, a2)) % 360
    dh = 0.0 if c1 * c2 == 0 else (h2 - h1 + 180) % 360 - 180
    big_dh = 2 * math.sqrt(c1 * c2) * math.sin(math.radians(dh / 2))
    l_bar, c_bar = (l1 + l2) / 2, (c1 + c2) / 2
    h_bar = h1 + h2 if c1 * c2 == 0 else (h1 + h2) / 2 + (180 if abs(h1 - h2) > 180 else 0)
    t = (1 - 0.17 * math.cos(math.radians(h_bar - 30)) + 0.24 * math.cos(math.radians(2 * h_bar))
         + 0.32 * math.cos(math.radians(3 * h_bar + 6)) - 0.2 * math.cos(math.radians(4 * h_bar - 63)))
    sl = 1 + 0.015 * (l_bar - 50) ** 2 / math.sqrt(20 + (l_bar - 50) ** 2)
    sc, sh = 1 + 0.045 * c_bar, 1 + 0.015 * c_bar * t
    rt = (-2 * math.sqrt(c_bar ** 7 / (c_bar ** 7 + 25 ** 7))
          * math.sin(math.radians(60 * math.exp(-((h_bar % 360 - 275) / 25) ** 2))))
    dl, dc = (l2 - l1) / sl, (c2 - c1) / sc
    return math.sqrt(dl ** 2 + dc ** 2 + (big_dh / sh) ** 2 + rt * dc * big_dh / sh)


def _rgb(colour):
    """``#rrggbb`` or ``color(N)`` (xterm 256: the 6x6x6 cube and the grey ramp) to RGB."""
    if colour.startswith('#'):
        return tuple(int(colour[i:i + 2], 16) for i in (1, 3, 5))
    n = int(colour[6:-1])
    if n >= 232:
        return (8 + (n - 232) * 10,) * 3
    steps = (0, 95, 135, 175, 215, 255)
    return steps[(n - 16) // 36], steps[(n - 16) // 6 % 6], steps[(n - 16) % 6]


def test_known_delta_e_values():
    assert delta_e((50, 2.6772, -79.7751), (50, 0, -82.7485)) == pytest.approx(2.0425, abs=1e-4)
    assert delta_e((50, 0, 0), (50, -1, 2)) == pytest.approx(2.3669, abs=1e-4)


@pytest.mark.parametrize('cvd', [False, True])
@pytest.mark.parametrize('mode', ['dark', 'light'])
@pytest.mark.parametrize('tier', ['truecolor', '256'])
def test_hue_families_stay_apart_in_every_tier(theme, tier, mode, cvd):
    import itertools
    close = []
    for (_, one), (_, two) in _apart():
        for a, b in itertools.product(one, two):
            d = delta_e(*(_lab(_rgb(theme.resolve(t, mode, tier, cvd))) for t in (a, b)))
            if d < MIN_DELTA_E:
                close.append(f'{a}/{b} {d:.1f}')
    assert not close, close


@pytest.mark.parametrize('cvd', [False, True])
def test_hue_families_never_share_a_16_colour_name(theme, cvd):
    """Every pair of families kept apart, not only buoy against the rest; ``default`` and attribute
    names are no colour, so two families that both fall back to them still read apart by glyph."""
    plain = {'default', 'bold', 'dim', 'underline'}
    shared = []
    for (f1, one), (f2, two) in _apart():
        names = [{theme.resolve(t, tier='16', cvd=cvd) for t in tokens} - plain for tokens in (one, two)]
        shared += [f'{f1}/{f2} {name}' for name in sorted(names[0] & names[1])]
    assert not shared, shared


ARMED_FROM_AGENT = 20   # the ARMED chip sits next to the AGENT chip: well past MIN_DELTA_E


@pytest.mark.parametrize('cvd', [False, True])
@pytest.mark.parametrize('mode', ['dark', 'light'])
@pytest.mark.parametrize('tier', ['truecolor', '256'])
def test_the_armed_chip_is_far_from_the_agent_chip(theme, tier, mode, cvd):
    d = delta_e(*(_lab(_rgb(theme.resolve(t, mode, tier, cvd))) for t in ('armed', 'auth.agent')))
    assert d >= ARMED_FROM_AGENT, f'armed/auth.agent {d:.1f}'


def test_on_16_colours_armed_is_its_own_name_and_bold(theme):
    assert theme.resolve('armed', tier='16') != theme.resolve('auth.agent', tier='16')
    assert 'bold' in theme.attr16.get('armed', '')


# --- colour names stay in theme.yaml (audit finding 13) -------------------------------------------
# Rich colour names, '#rrggbb' and 'color(n)' written as style strings in ui/panels, ui/popups, ui/layout and
# ui/minimap bypass the tokens, so CVD swaps and the light theme never reach them. The allowlist only shrinks:
# a file on it must still hold exactly its count (drop the entry when it reaches 0).
COLOUR_ALLOW = {
    'minimap/emit.py': 8,   # the ANSI SGR names of its own emitter; waits for the hull work (follow-up)
}
_STYLE_WORDS = {'bold', 'dim', 'italic', 'underline', 'reverse', 'blink', 'strike', 'not', 'on', 'default'}


def _colour_literals(path):
    import ast
    import re
    from rich.color import ANSI_COLOR_NAMES
    spelled = re.compile(r'^(#[0-9a-fA-F]{6}|color\(\d+\)|rgb\(.*\))$')

    def colour(word):
        return (word in ANSI_COLOR_NAMES and word != 'default') or bool(spelled.match(word))
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            words = node.value.split()
            if words and all(w in _STYLE_WORDS or colour(w) for w in words) and any(map(colour, words)):
                yield node.lineno, node.value


def test_no_colour_names_outside_the_theme():
    from pathlib import Path
    import mhseals_testing.ui as ui
    root = Path(ui.__file__).parent
    files = [*root.glob('panels/**/*.py'), *root.glob('popups/**/*.py'), *root.glob('minimap/**/*.py'),
             *root.glob('layout*.py'), *root.glob('layout/**/*.py')]
    found = {}
    for path in files:
        hits = list(_colour_literals(path))
        if hits:
            found[path.relative_to(root).as_posix()] = hits
    over = {name: hits for name, hits in found.items() if len(hits) > COLOUR_ALLOW.get(name, 0)}
    assert not over, f'colour names outside theme.py/theme.yaml (use a token): {over}'
    shrunk = {name: n for name, n in COLOUR_ALLOW.items() if len(found.get(name, ())) < n}
    assert not shrunk, f'COLOUR_ALLOW can shrink: {shrunk}'


def test_the_fusion_range_ramp_is_the_spd_tones(theme):
    """spd.range.* differ from spd.* only in their 16-colour name (one hue per band); the rest is spd's."""
    for band in range(6):
        ramp, spd = theme.tokens[f'spd.range.{band}'], theme.tokens[f'spd.{band}']
        assert {k: ramp[k] for k in ('dark', 'light', 'x256')} == {k: spd[k] for k in ('dark', 'light', 'x256')}
    sixteen = [theme.style(f'spd.range.{band}', tier='16') for band in range(6)]
    assert len(set(sixteen)) == 6 and theme.style('obj.red', tier='16') not in sixteen


def test_score_grades_take_the_status_styles():
    from mhseals_testing.ui import theme as ui
    assert [ui.score_style(g) for g in ('green', 'yellow', 'red', None)] == [ui.GOOD, ui.WARN, ui.BAD, ui.MUTED]


# Panel literals live in theme.yaml tune: (schema theme.tune.*); the shipped values are the ones the code had.
TUNE_DEFAULTS = {
    'motion.tall': 20, 'motion.short': 40, 'motion.stale': 2.0, 'motion.label': 18,
    'fusion.greys': 6, 'fusion.link_blocks': (64, 32, 16, 8, 4), 'fusion.readout_rows': 8, 'fusion.around_min': 24,
    'fusion.around_share': 0.3, 'fusion.spark_columns': 12, 'fusion.stamp_match': 0.005, 'fusion.link_share': 0.9,
    'fusion.rate_every': 5.0, 'fusion.rate_start': 0.25, 'fusion.rate_step': 1.5, 'fusion.over_every': 0.5,
    'fusion.join_steps': 4,
}


def test_tune_ships_the_pinned_defaults_and_each_has_a_schema_entry():
    from pathlib import Path
    import yaml
    from mhseals_testing.ui import theme as ui
    assert {key: ui.tune(key) for key in TUNE_DEFAULTS} == TUNE_DEFAULTS
    shipped = {f'{panel}.{name}' for panel, keys in ui.PALETTE.data['tune'].items() for name in keys}
    assert shipped == set(TUNE_DEFAULTS)
    fragment = yaml.safe_load((Path(palette.default_path()).parent / 'schema' / 'theme.yaml').read_text())
    assert set(fragment) == {f'theme.tune.{key}' for key in TUNE_DEFAULTS}


def test_the_panels_read_their_sizes_from_tune():
    from mhseals_testing.ui.panels import fusion, motion
    assert (motion.TALL, motion.SHORT, motion.STALE, motion.LABEL) == (20, 40, 2.0, 18)
    assert (fusion.READOUT_ROWS, fusion.AROUND_MIN, fusion.LINK_BLOCKS, fusion.JOIN_STEPS) == (8, 24, (64, 32, 16, 8, 4), 4)
