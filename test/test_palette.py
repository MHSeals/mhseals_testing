import pytest
import yaml

from mhseals_testing.ui import palette


@pytest.fixture(scope='module')
def theme():
    return palette.load_theme(overlay=False)[0]


def test_off_glyph_key_is_loaded_as_a_string_not_a_boolean(theme):
    assert 'off' in theme.glyphs and True not in theme.glyphs
    assert theme.glyph('off', ascii=False) == '·' and theme.glyph('off', ascii=True) == '.'


def test_unquoted_off_key_is_rejected(tmp_path):
    text = palette.default_path().read_text().replace("'off':", 'off:')
    path = tmp_path / 'theme.yaml'
    path.write_text(text)
    with pytest.raises(ValueError, match='missing glyph ids: off'):
        palette.load_theme(path, overlay=False)


def test_shipped_theme_loads_and_passes_validation():
    assert palette.load_theme(overlay=False)[0].tokens


def test_low_contrast_overlay_is_ignored_with_a_warning(tmp_path):
    overlay = tmp_path / 'o.yaml'
    overlay.write_text(yaml.dump({'tokens': {'warn': {
        'dark': '#181C23', 'light': '#FAF9F5', 'x256': [234, 231], 'ansi': 'yellow'}}}))
    theme, warning = palette.load_theme(overlay=overlay)
    assert 'ignored' in warning and 'warn' in warning
    assert theme.resolve('warn') == '#EDCB78'


def test_overlay_can_turn_cvd_on_and_retint_a_token(tmp_path):
    overlay = tmp_path / 'o.yaml'
    overlay.write_text("cvd: true\ntheme: light\n")
    theme, warning = palette.load_theme(overlay=overlay)
    assert warning is None and theme.cvd and theme.mode == 'light'
    assert theme.style('ok', tier='truecolor') == '#2D63C4'


@pytest.mark.parametrize('mode', ['dark', 'light'])
def test_cvd_changes_status_but_never_object_tokens(theme, mode):
    objs = [name for name in theme.tokens if name.startswith('obj.')]
    assert objs
    for name in objs:
        for tier in ('truecolor', '256', '16'):
            assert theme.resolve(name, mode, tier, cvd=True) == theme.resolve(name, mode, tier, cvd=False)
    assert theme.resolve('ok', mode, cvd=True) != theme.resolve('ok', mode, cvd=False)
    assert theme.resolve('fail', mode, cvd=True) != theme.resolve('fail', mode, cvd=False)
    assert theme.resolve('busy', mode, cvd=False) == theme.resolve('accent', mode)


def test_cvd_swap_never_names_a_fixed_token():
    data = yaml.safe_load(palette.default_path().read_text())
    data['cvd_swap']['obj.red'] = data['cvd_swap']['ok']
    with pytest.raises(ValueError, match='not a swappable token'):
        palette.Theme(data)


@pytest.mark.parametrize('tier', palette.TIERS)
@pytest.mark.parametrize('mode', ['dark', 'light'])
def test_every_token_resolves_in_every_tier_to_a_valid_rich_style(theme, tier, mode):
    from rich.style import Style
    for name in theme.tokens:
        Style.parse(theme.style(name, mode, tier))


def test_unknown_token_and_alias_loops_are_rejected():
    data = yaml.safe_load(palette.default_path().read_text())
    data['tokens']['a'] = {'alias': 'b'}
    data['tokens']['b'] = {'alias': 'a'}
    with pytest.raises(ValueError, match='alias loop'):
        palette.Theme(data)
    with pytest.raises(KeyError):
        palette.load_theme(overlay=False)[0].style('nope')


def test_16_colour_attributes_are_only_on_unknown_and_sel_bg(theme):
    assert theme.style('unknown', 'dark', '16') == 'bright_black dim'
    assert theme.style('muted', 'dark', '16') == 'bright_black'
    assert theme.style('fail', 'dark', '16') == 'red'
    assert theme.style('faint', 'dark', '16') == 'bright_black'
    assert theme.style('border', 'dark', '16') == 'bright_black'
    assert theme.style('sel_bg', 'dark', '16') == 'underline'
    assert theme.style('sand', 'dark', '16') == 'bold'
    assert theme.style('fail', 'dark', 'mono') == 'bold'


def test_chip_tiers(theme):
    assert theme.chip('AGENT', 'auth.agent', 'dark', 'truecolor', False).style.__str__() == '#171B22 on #EFA6E2'
    assert theme.chip('AGENT', 'auth.agent', 'light', '256', False).style.__str__() == 'color(231) on color(133)'
    assert 'reverse' in str(theme.chip('HOLD', 'auth.hold', tier='16', ascii=False).style)
    assert 'reverse' in str(theme.chip('HOLD', 'auth.hold', tier='mono', ascii=False).style)
    assert theme.chip('AGENT', 'auth.agent', ascii=True).plain == '[AGENT]'


def test_tier_and_theme_detection():
    assert palette.detect_tier({'NO_COLOR': '1', 'COLORTERM': 'truecolor'}) == 'mono'
    assert palette.detect_tier({'COLORTERM': 'truecolor'}) == 'truecolor'
    assert palette.detect_tier({'TERM': 'xterm-256color'}) == '256'
    assert palette.detect_tier({'TERM': 'linux'}) == '16'
    assert palette.detect_theme({'COLORFGBG': '0;15'}) == 'light'
    assert palette.detect_theme({}) == 'dark'
    assert palette.ascii_mode({'TERM': 'dumb'}) and palette.ascii_mode({'CX_ASCII': '1'})
