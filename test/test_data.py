from pathlib import Path

from mhseals_testing import data
from mhseals_testing.ui.app import parse_args


def test_default_root_is_not_cwd_relative(monkeypatch, tmp_path):
    monkeypatch.delenv('MHSEALS_DATA', raising=False)
    monkeypatch.chdir(tmp_path)
    assert data.data_root().is_absolute()
    assert data.data_root().name == 'bags'


def test_env_overrides_root(monkeypatch, tmp_path):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'd'))
    assert data.data_root() == tmp_path / 'd'


def test_default_session_bag_lands_under_root(monkeypatch, tmp_path):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path / 'd'))
    monkeypatch.chdir(tmp_path)
    args, _ = parse_args([])
    assert Path(args.output).parent == tmp_path / 'd'
