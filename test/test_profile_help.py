"""Descriptions follow profile data into public help without operational dispatch."""
from types import SimpleNamespace

from mhseals_testing.profiles import profile_descriptions


def test_cli_profiles_uses_yaml_descriptions_without_health(monkeypatch, capsys):
    """Actual CLI listing reads descriptions and leaves health sampling untouched."""
    from mhseals_testing.harness import cli
    def forbidden(*args, **kwargs):
        raise AssertionError('profile help entered an operational path')
    monkeypatch.setattr(cli, 'load_profile', forbidden)
    monkeypatch.setattr(cli.health_config, 'load', forbidden)
    monkeypatch.setattr(cli, 'host_collector', forbidden)
    assert cli.main(['profiles'], collect=forbidden) == 0
    assert capsys.readouterr().out == ''.join(
        f'{name}: {description}\n' for name, description in profile_descriptions())


def test_tui_help_dispatch_shows_selected_description():
    """The actual ? handler carries even a custom profile description into its rows."""
    from mhseals_testing.ui.app import App
    captured = {}
    profile = SimpleNamespace(name='custom', description='Custom output and sensor path')
    app = SimpleNamespace(profile=profile, tab='Status', review=False, panels=lambda: [],
                          show=lambda title, **kwargs: captured.update(title=title, **kwargs))
    App.dispatch(app, '?')
    assert captured['title'] == 'Help'
    assert captured['rows'][0] == ('custom', profile.description)


def test_both_cli_parsers_expose_profile_description_help():
    """The executable parsers advertise the live YAML-backed explanation command."""
    from mhseals_testing.harness.cli import build_parser
    from mhseals_testing.ui.app import build_parser as tui_parser
    check = build_parser()._subparsers._group_actions[0].choices['check']
    assert 'boat_trial profiles' in check.format_help()
    assert 'boat_trial profiles' in tui_parser().format_help()
