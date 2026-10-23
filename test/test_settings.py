"""Settings table: validation, persistence (not command mode or bench), procedures read ctx.settings."""

import pytest

from mhseals_testing.harness import settings as store
from mhseals_testing.procedures import Settings as ProcedureSettings
from mhseals_testing.ui.app import build_parser, parse_args


def test_attribute_access_and_validation():
    settings = store.Settings(settle=2, command=3)
    assert (settings.settle, settings.command, settings.amplitude) == (2, 3, 0.25)
    assert ProcedureSettings is store.Settings
    for assignment in ('amplitude=0', 'amplitude=nan', 'repeats=9', 'command_mode=fast',
                       'bench_override=maybe', 'nope=1', 'amplitude'):
        with pytest.raises(ValueError):
            settings.set(assignment)
    settings.set('bench_override=on')
    settings.set('repeats=3')
    assert settings.bench_override is True and settings.repeats == 3


def test_settings_persist_except_command_and_bench(tmp_path):
    path = tmp_path / 'boat_test.yaml'
    settings = store.Settings(amplitude=0.4, command_mode='velocity', bench_override=True)
    assert store.save(settings, path)
    text = path.read_text()
    assert 'command_mode' not in text and 'bench_override' not in text
    loaded = store.load(path)
    assert loaded.amplitude == 0.4
    assert loaded.command_mode == 'effort' and loaded.bench_override is False


def test_broken_file_or_value_falls_back(tmp_path):
    path = tmp_path / 'boat_test.yaml'
    path.write_text('amplitude: [1\n')
    assert store.load(path).amplitude == 0.25
    path.write_text('amplitude: 7\nsettle: 2\ncommand_mode: velocity\nbench_override: true\n')
    loaded = store.load(path)
    assert loaded.amplitude == 0.25 and loaded.settle == 2
    assert loaded.command_mode == 'effort' and not loaded.bench_override


def test_parse_args_reads_saved_values_and_set_overrides(tmp_path):
    store.save(store.Settings(amplitude=0.4, settle=2))
    args, profile = parse_args([])
    assert args.settings.amplitude == 0.4 and args.stale_after == 2.0
    assert profile.command_units == 'effort' and not args.bench_override  # reset at start
    args, profile = parse_args(['--set', 'amplitude=0.1', '--set', 'command_mode=velocity'])
    assert args.settings.amplitude == 0.1 and profile.command_units == 'velocity'
    with pytest.raises(SystemExit):
        parse_args(['--set', 'repeats=9'])


def test_flags_shrink():
    flags = {option for action in build_parser()._actions for option in action.option_strings}
    assert {'--profile', '--set', '--output'} <= flags
    assert not flags & {'--amplitude', '--velocity-control', '--bench-override', '--repeats', '--stale-after'}
