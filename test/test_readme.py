"""The README stays short and its relative links resolve."""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
README_LINES_TODAY = 70


def test_readme_not_longer_than_before():
    assert len((ROOT / 'README.md').read_text().splitlines()) <= README_LINES_TODAY


def test_readme_links_exist():
    text = (ROOT / 'README.md').read_text()
    for target in re.findall(r'\]\((docs/[^)#]+)\)', text):
        assert (ROOT / target).exists(), target


def test_help_lists_the_thrusters_clear_key():
    """The README calls `?` the reference, so every Thrusters key must be in it."""
    from mhseals_testing.ui import app
    assert any(k == 'Thrusters c' for k, _ in app.HELP)
