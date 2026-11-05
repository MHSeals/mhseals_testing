"""The Settings mixin stays glue: its row groups are one module each under settings_rows/ (audit finding 9)."""
from pathlib import Path

from mhseals_testing.ui.app.actions import settings, settings_rows
from mhseals_testing.ui.app.core import App

ROWS = Path(settings_rows.__file__).parent


def test_settings_is_glue_and_each_row_group_is_under_the_file_limit():
    assert len(Path(settings.__file__).read_text().splitlines()) < 250
    modules = sorted(p for p in ROWS.glob('*.py') if p.name != '__init__.py')
    assert {p.stem for p in modules} == {'health', 'files', 'hardware', 'status', 'pose', 'correction'}
    for path in modules:
        assert len(path.read_text().splitlines()) < 500, path.name


def test_every_row_group_is_mixed_into_the_app_once():
    groups = settings.Settings.__bases__
    assert len(groups) == 6
    names = [k for group in groups for k in vars(group) if not k.startswith('__')]
    assert len(names) == len(set(names))                 # no method is defined in two groups
    assert all(group in App.__mro__ for group in groups)
