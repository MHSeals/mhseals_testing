"""Fault injection for the crane quality gate: faulted boats and corrupted records, sim only."""
import hashlib
import json

import pytest
import yaml

from physics_faults import CASES, KINDS, FaultBoat, corrupt
from test_physics_procedure import make_ctx, saved
from mhseals_testing.procedures import physics

K = (30.0, 30.0, 10.0)


def survey(tmp_path, boat, sim=True):
    ctx, _ = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='survey', amplitude=1.0)
    ctx.profile.use_sim_time = sim
    return saved(physics.thrust(ctx))


@pytest.fixture(scope='module')
def good(tmp_path_factory):
    root = tmp_path_factory.mktemp('good')
    record = survey(root, FaultBoat('reference', k=K, noise=0.002))
    return root / 'char', record


def cross(record, label):
    """Mean off-axis speed over the main speed, over the highest-level holds of one direction."""
    axis = ('surge', 'sway', 'yaw').index(label[:-1])
    holds = [h for h in record['holds'] if h['name'].split()[0] == label and not h.get('overridden')]
    top = max(abs(h['applied'][axis]) for h in holds)
    holds = [h for h in holds if abs(h['applied'][axis]) == top]
    mean = [sum(h[k] for h in holds) / len(holds) for k in ('vx', 'vy', 'wz')]
    return [abs(v) / max(abs(mean[axis]), 1e-9) for i, v in enumerate(mean) if i != axis]


def test_the_reference_case_is_the_unfaulted_boat(tmp_path):
    record = survey(tmp_path, FaultBoat('reference', k=K))
    assert record['speed_curves']['surge+']['d2'] == pytest.approx(12 / 30, rel=0.05)
    assert max(cross(record, 'surge+')) < 0.02


def test_hull_drag_scales_each_axis_fit(tmp_path):
    record = survey(tmp_path, FaultBoat('hull_drag', k=K))
    assert record['speed_curves']['surge+']['d2'] == pytest.approx(3 * 12 / 30, rel=0.06)
    assert record['speed_curves']['yaw+']['d2'] == pytest.approx(2 * 1.5 / 10, rel=0.1)


@pytest.mark.parametrize('case', ['weak_rr', 'angled_fr', 'combined'])
def test_a_thruster_fault_shows_as_cross_coupling_the_reference_lacks(tmp_path, case):
    record = survey(tmp_path, FaultBoat(case, k=K))
    assert max(cross(record, 'surge+')) > 0.05, case


def test_sim_profiles_mark_faulted_records_sim_derived(tmp_path):
    assert survey(tmp_path, FaultBoat('weak_rr', k=K))['hardware']['sim_derived'] is True


def folder(tmp_path, name='out'):
    return tmp_path / 'sim' / name


def test_every_kind_writes_a_labelled_copy_and_leaves_the_source_alone(good, tmp_path):
    root, record = good
    name = f'{record["fit"]["id"]}-survey'
    original = (root / f'{name}.yaml').read_bytes()
    for kind in KINDS:
        path = corrupt(root, folder(tmp_path, kind), name, kind, 2, seed=3)
        made = yaml.safe_load(path.read_text())
        assert made['schema'] == record['schema'] and made['fault_injection'] == dict(kind=kind, severity=2, seed=3)
        assert (root / f'{name}.yaml').read_bytes() == original


def test_corruption_is_deterministic_and_grows_with_severity(good, tmp_path):
    root, record = good
    name = f'{record["fit"]["id"]}-survey'
    side = record['samples_file']['path']
    base = json.loads((root / side).read_text())['odometry']

    count = iter(range(1000))

    def changed(kind, severity, seed):
        out = folder(tmp_path, f'{kind}{severity}{seed}-{next(count)}')
        corrupt(root, out, name, kind, severity, seed)
        rows = json.loads((out / side).read_text())['odometry']
        return len(base) - len(rows) if kind == 'dropout' else sum(a != b for a, b in zip(rows, base))

    for kind in ('nan', 'dropout', 'spikes', 'stuck'):
        assert changed(kind, 2, 1) == changed(kind, 2, 1)
        assert changed(kind, 1, 1) < changed(kind, 3, 1), kind


def test_sample_edits_pass_the_integrity_hash_and_byte_edits_fail_it(good, tmp_path):
    root, record = good
    name = f'{record["fit"]["id"]}-survey'

    def intact(kind):
        out = folder(tmp_path, kind)
        made = yaml.safe_load(corrupt(root, out, name, kind, 3, 0).read_text())
        file = made['samples_file']
        return file['sha256'] == 'sha256:' + hashlib.sha256((out / file['path']).read_bytes()).hexdigest()

    assert all(intact(k) for k in ('nan', 'dropout', 'skew', 'spikes', 'scale', 'stuck', 'overrides'))
    assert not intact('sha_mismatch') and not intact('truncated')
    nan = json.loads((folder(tmp_path, 'nan') / record['samples_file']['path']).read_text())['odometry']
    assert any(None in row for row in nan)


def test_record_kinds_change_the_provenance_the_gate_reads(good, tmp_path):
    root, record = good
    name = f'{record["fit"]["id"]}-survey'
    stale = yaml.safe_load(corrupt(root, folder(tmp_path, 'a'), name, 'stale_config', 3).read_text())
    assert stale['hardware']['hash'] != record['hardware']['hash']
    assert stale['hardware']['keys']['hardware_epoch'] != record['hardware']['keys']['hardware_epoch']
    assert yaml.safe_load(corrupt(root, folder(tmp_path, 'b'), name, 'muted', 1).read_text())['gate_muted'] is True
    wet = yaml.safe_load(corrupt(root, folder(tmp_path, 'c'), name, 'conditions', 3).read_text())
    assert wet['conditions'] == dict(verdict='unknown') and wet['battery_v'] is None


def test_a_record_from_the_boat_is_never_corrupted(tmp_path):
    record = survey(tmp_path, FaultBoat('reference', k=K), sim=False)
    assert record['hardware']['sim_derived'] is False
    with pytest.raises(PermissionError, match='sim only'):
        corrupt(tmp_path / 'char', folder(tmp_path), f'{record["fit"]["id"]}-survey', 'nan')
    assert not folder(tmp_path).exists()


def test_corruption_refuses_to_overwrite_or_leave_a_sim_folder(good, tmp_path):
    root, record = good
    name = f'{record["fit"]["id"]}-survey'
    with pytest.raises(FileExistsError):
        corrupt(root, root, name, 'nan')
    corrupt(root, folder(tmp_path), name, 'nan')
    with pytest.raises(FileExistsError):
        corrupt(root, folder(tmp_path), name, 'nan')
    with pytest.raises(PermissionError, match='sim or temp'):
        corrupt(root, __import__('pathlib').Path('/home/x/data/characterization'), name, 'nan')
    with pytest.raises(ValueError):
        corrupt(root, folder(tmp_path, 'k'), name, 'bogus')
    with pytest.raises(ValueError):
        corrupt(root, folder(tmp_path, 's'), name, 'nan', 4)


def test_the_helper_is_not_part_of_the_installed_package():
    import pathlib
    here = pathlib.Path(__file__).parent
    setup = (here.parent / 'setup.py').read_text()
    assert (here / 'physics_faults.py').is_file()
    assert "find_packages(exclude=['test'])" in setup and "package_files('test')" not in setup
    assert not list((here.parent / 'mhseals_testing').rglob('physics_faults*'))
    assert set(CASES) >= {'reference', 'weak_rr', 'angled_fr', 'hull_drag', 'combined'}
