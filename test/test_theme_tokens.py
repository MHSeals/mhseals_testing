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
            'armed': ('armed',)}     # and the hot armed state is neither
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
