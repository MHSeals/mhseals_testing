"""The characterization procedure against a fake boat whose coupling is known exactly."""
from contextlib import contextmanager
import math
import sys
from types import SimpleNamespace

import pytest
import yaml

from mhseals_hardware.configuration import load_config, update_config
from mhseals_hardware.thruster_mixer import decouple, validate_decoupling
from mhseals_testing.harness.apply import Staging
from mhseals_testing.procedures import Settings
from mhseals_testing.procedures.characterize import run
from mhseals_testing.runner import Aborted

GAIN = (1.2, 0.8, 1.5)
C_OLD = [1.0, 0.0, 0.0, 0.0, 1.0, 0.3, 0.0, 0.0, 1.0]  # an earlier fit the survey must not see
# The exact columns: surge pushes sway by 0.1 (0.08 astern), sway turns the boat by -0.2.
EXPECTED = {(0, 1): (-0.125, -1 / 60), (0, -1): (-0.1, -1 / 75), (1, 1): (0.0, 2 / 15),
            (1, -1): (0.0, 2 / 15), (2, 1): (0.0, 0.0), (2, -1): (0.0, 0.0)}


def boat(effort):
    """Steady velocity for raw per-axis efforts."""
    sway_from_surge = 0.1 if effort[0] >= 0 else 0.08
    return [GAIN[0] * effort[0], GAIN[1] * effort[1] + sway_from_surge * effort[0],
            GAIN[2] * effort[2] - 0.2 * effort[1]]


class Fake:
    """A runner whose boat reads C+/C- from the hardware YAML, as the driver does."""

    def __init__(self, config, abort_after=None, noise=0.0, coast=False):
        self.config, self.clock, self.odometry, self.commands = config, 0.0, [], []
        self.noise, self.coast, self.last = noise, coast, [0.0] * 3   # coast: a hold's lag starts from the last velocity
        self.abort_after = abort_after

    def hold(self, command, seconds, label=''):
        if self.abort_after is not None and len(self.commands) >= self.abort_after:
            raise Aborted('cancelled by operator')
        start, self.clock = self.clock, self.clock + seconds
        self.commands.append(tuple(command))
        settings = load_config(self.config)
        applied = decouple(command, validate_decoupling(settings['decoupling']),
                           validate_decoupling(settings['decoupling_negative'] or settings['decoupling']))
        velocity = boat(applied) if any(command) else [0.0] * 3
        for step in range(int(seconds * 2)):  # 2 Hz, first-order lag (tau 1 s) from the previous velocity
            t = start + step / 2 + 0.25
            decay = math.exp(-(t - start)) if self.coast or any(command) else 0.0
            now = [v + (p - v) * decay for v, p in zip(velocity, self.last)]
            self.odometry.append((t, dict(zip(('vx', 'vy', 'wz'), (v + self.noise * (-1) ** step for v in now)))))
        if self.coast:
            self.last = [v + (p - v) * math.exp(-seconds) for v, p in zip(velocity, self.last)]
        return start, self.clock

    def stop(self):
        pass

    def invalid(self, start, end):
        return False    # no human override in these runs


def context(tmp_path, runner, answer='save', direction='', nav2=None):
    config = tmp_path / 'hardware.yaml'
    calls = []

    @contextmanager
    def motion(name, **details):
        yield runner

    def choose(title, options, shortcuts=None, **kwargs):
        calls.append(dict(overlay=bool(nav2) and (tmp_path / 'overlay.yaml').exists()))
        return answer

    records = []
    services = dict(staging=Staging(config, tmp_path / 'state.json'), user_config=config,
                    hardware_config=load_config(config))
    if nav2:
        services.update(nav2_params=nav2, nav2_overlay=tmp_path / 'overlay.yaml')
    return SimpleNamespace(
        settings=Settings(amplitude=0.5, settle=2, command=3, direction=direction), motion=motion,
        prompt=lambda text: None, choose=choose, record=records.append, services=services,
        history=lambda since, until: [s for s in runner.odometry if since <= s[0] <= until]), records, calls


@pytest.fixture
def config(tmp_path):
    path = tmp_path / 'hardware.yaml'
    update_config(dict(decoupling=C_OLD), path)
    return path


def columns(matrix):
    """{(axis, sign): (cross entries)} out of a flat C+ (sign 1) or C- (sign -1)."""
    return {axis: tuple(matrix[3 * k + axis] for k in range(3) if k != axis) for axis in range(3)}


def test_survey_writes_exact_columns_and_speed_curves(config, tmp_path):
    ctx, records, _ = context(tmp_path, Fake(config))
    result = run(ctx)
    saved = load_config(config)
    for (axis, sign), expected in EXPECTED.items():
        matrix = saved['decoupling'] if sign > 0 else saved['decoupling_negative']
        assert columns(matrix)[axis] == pytest.approx(expected, abs=0.02 if axis == 2 else 2e-3)  # yaw stops inside its 0.10 tolerance
        assert matrix[4 * axis] == 1.0  # unit diagonal
    assert set(result['speed_curves']) == {a + s for a in ('surge', 'sway', 'yaw') for s in '+-'}
    assert saved['characterization']['speed_curves'].keys() == result['speed_curves'].keys()
    assert result['motion_limits']['velocity']['surge'] > 0
    assert records[-1]['verdict'] == 'SAVED' and records[-1]['verify']['ok']
    assert Staging(config, tmp_path / 'state.json').pending() is None


def test_survey_stores_the_lag_and_the_per_sample_noise(config, tmp_path):
    """Without t95 the predictor has no lag and without wave_floor sigma is 0: a healthy boat scores low."""
    ctx, _, _ = context(tmp_path, Fake(config, noise=0.02, coast=True))
    result = run(ctx)
    # The fake lags with tau = 1 s, so 95 % takes ln(20) = 3.0 s (2 Hz samples and the noise move it by a sample).
    assert set(result['t95']) == {'surge', 'sway', 'yaw'}
    assert all(2.0 <= lag <= 4.0 for lag in result['t95'].values())
    # (yaw, surge, sway), per sample: the coast after each hold must not leak into it
    assert result['wave_floor'] == pytest.approx([0.02] * 3, rel=0.1)
    assert load_config(config)['characterization']['t95'] == result['t95']


def test_survey_runs_raw_efforts_not_the_old_correction(config, tmp_path):
    runner = Fake(config)
    run(context(tmp_path, runner)[0])
    assert runner.commands[2] == (0.5, 0.0, 0.0)  # surge at a_hi, with no cross effort added
    # Surge's own deployed column is zero, so the hold is the boat's own sway 0.1 * 0.5 with nothing added.
    hold = [v for t, v in runner.odometry if 22 < t < 25]
    assert hold[-1]['vy'] == pytest.approx(0.05, abs=0.01)


def test_abort_restores_the_old_correction(config, tmp_path):
    ctx, _, _ = context(tmp_path, Fake(config, abort_after=14))
    with pytest.raises(Aborted):
        run(ctx)
    assert load_config(config)['decoupling'] == C_OLD
    assert Staging(config, tmp_path / 'state.json').pending() is None


def test_declining_the_save_keeps_the_old_correction(config, tmp_path):
    before = load_config(config)['characterization']
    ctx, records, _ = context(tmp_path, Fake(config), answer='no')
    run(ctx)
    assert load_config(config)['decoupling'] == C_OLD and load_config(config)['characterization'] == before
    assert not any(r.get('verdict') == 'SAVED' for r in records)


def test_overlay_is_written_before_the_save_question(config, tmp_path):
    nav2 = tmp_path / 'nav2.yaml'
    nav2.write_text(yaml.safe_dump({'controller_server': {'ros__parameters': {'FollowPath': {}}},
                                    'velocity_smoother': {'ros__parameters': {}}}))
    ctx, _, calls = context(tmp_path, Fake(config), answer='no', nav2=nav2)
    run(ctx)
    assert calls == [dict(overlay=True)]


def test_direction_reruns_only_that_column(config, tmp_path):
    runner, before = Fake(config), load_config(config)['characterization']
    result = run(context(tmp_path, runner, direction='surge-')[0])
    saved = load_config(config)
    assert columns(saved['decoupling_negative'])[0] == pytest.approx(EXPECTED[(0, -1)], abs=2e-3)
    assert saved['decoupling'] == C_OLD  # the positive columns are untouched
    assert {k: v for k, v in saved['characterization'].items() if k != 'fit'} == before and set(result['directions']) == {'surge-'}
    assert all(c[0] <= 0 for c in runner.commands if any(c))


def test_a_reversed_axis_refuses_and_restores(config, tmp_path):
    runner = Fake(config)
    original = runner.hold

    def hold(command, seconds, label=''):
        return original((command[0], -command[1], command[2]), seconds, label)  # sway wired backwards
    runner.hold = hold
    ctx, records, _ = context(tmp_path, runner)
    result = run(ctx)
    assert result['verdict'] == 'REFUSED' and 'sway' in result['reason']
    assert load_config(config)['decoupling'] == C_OLD


def test_unknown_direction_is_refused_without_staging(config, tmp_path):
    ctx, _, _ = context(tmp_path, Fake(config), direction='up')
    assert run(ctx)['verdict'] == 'REFUSED'
    assert Staging(config, tmp_path / 'state.json').pending() is None


def test_survey_sends_each_directions_deployed_column(config, tmp_path):
    runner = Fake(config)
    run(context(tmp_path, runner)[0])
    yaw_hi = [c for c in runner.commands if c[2] == 0.5][0]
    assert yaw_hi == (0.0, 0.15, 0.5)  # C_OLD's yaw column (0, 0.3) times the effort


def test_an_amplitude_with_no_room_for_the_correction_is_refused(config, tmp_path):
    ctx, _, _ = context(tmp_path, Fake(config))
    ctx.settings = Settings(amplitude=1.0, settle=2, command=3)
    assert run(ctx)['verdict'] == 'REFUSED'
    assert load_config(config)['decoupling'] == C_OLD


def test_a_corrected_boat_runs_only_the_survey_holds(config, tmp_path, monkeypatch):
    """Every survey point already inside tolerance: neutral + 12 survey holds, no refine or low holds."""
    monkeypatch.setattr(sys.modules[__name__], 'boat', lambda effort: [GAIN[0] * effort[0], GAIN[1] * effort[1], GAIN[2] * effort[2]])
    update_config(dict(decoupling=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]), config)
    runner = Fake(config)
    ctx, records, _ = context(tmp_path, runner, answer='no')
    run(ctx)
    moving = [c for c in runner.commands if any(c)]
    assert len(moving) == 12
    assert all(h['label'] in {a + s for a in ('surge', 'sway', 'yaw') for s in '+-'} or h['label'] == 'neutral'
               for h in records[-1]['holds'])


def _coast_history(tau, settle=4.0, hold=3.0, v=1.0):
    """first-order (tau) velocity response at 20 Hz: surge+ held to steady v, neutral coast, then surge- to -v from the coast."""
    dt, rows, t, x = 0.05, [], 0.0, 0.0
    for command, seconds in ((v, 10 * tau), (0.0, settle), (-v, hold)):
        for _ in range(round(seconds / dt)):
            x += (command - x) * dt / tau
            rows.append((t, dict(vx=x, vy=0.0, wz=0.0)))
            t += dt
    return rows, (10 * tau, 10 * tau + settle), (10 * tau + settle, 10 * tau + settle + hold)


def test_response_skips_the_previous_holds_coast():
    from mhseals_testing.procedures.characterize import response
    history, settle, hold = _coast_history(0.6)
    biased = response(history, settle, hold, 1.0)[0]
    fixed = response(history, settle, hold, 1.0, coast=2.0)[0]
    assert abs(biased + 1.0) > 0.07  # the whole settle reads the coast
    assert fixed == pytest.approx(-1.0, abs=0.06)


def test_unconverged_direction_makes_the_verdict_partial_and_names_it(config, tmp_path, monkeypatch):
    from mhseals_testing.procedures import characterize
    real = characterize.refine

    def refine(measure, begin, *args):
        found = real(measure, begin, *args)
        return dict(found, converged=False)  # every refine ends unconverged
    monkeypatch.setattr(characterize, 'refine', refine)
    ctx, records, _ = context(tmp_path, Fake(config), answer='no')
    run(ctx)
    entry = next(r for r in records if r.get('verdict') in ('FITTED', 'PARTIAL'))
    assert entry['verdict'] == 'PARTIAL'
    assert set(entry['directions_unfitted']) == {a + s for a in ('surge', 'sway', 'yaw') for s in '+-'}


def test_converged_directions_stay_fitted(config, tmp_path):
    ctx, records, _ = context(tmp_path, Fake(config), answer='no')
    run(ctx)
    entry = next(r for r in records if r.get('verdict') in ('FITTED', 'PARTIAL'))
    assert entry['verdict'] == 'FITTED' and entry['directions_unfitted'] == []


def test_a_dead_direction_is_marked_and_keeps_its_old_curve(config, tmp_path):
    update_config(dict(characterization=dict(version=2, speed_curves={'sway-': dict(d1=1.0, d2=0.5)})), config)
    runner = Fake(config)
    original = runner.hold

    def hold(command, seconds, label=''):
        return original((command[0], max(command[1], 0.0), command[2]), seconds, label)  # sway- does nothing
    runner.hold = hold
    ctx, records, _ = context(tmp_path, runner)
    ctx.services['hardware_config'] = load_config(config)
    run(ctx)
    saved = load_config(config)
    result = next(r for r in records if r.get('verdict') == 'PARTIAL')
    assert result['dead_directions'] == ['sway-'] and result['directions']['sway-']['dead']
    assert saved['characterization']['speed_curves']['sway-'] == dict(d1=1.0, d2=0.5)
    assert saved['characterization']['speed_curves']['sway+'] != dict(d1=1.0, d2=0.5)  # live directions are fitted
    assert saved['decoupling_negative'] is None or columns(saved['decoupling_negative'])[1] == (0.0, 0.0)


def test_a_dead_single_direction_keeps_its_column_and_skips_verify(config, tmp_path):
    runner, before = Fake(config), load_config(config)
    original = runner.hold

    def hold(command, seconds, label=''):
        return original((command[0], max(command[1], 0.0), command[2]), seconds, label)  # sway- does nothing
    runner.hold = hold
    ctx, records, _ = context(tmp_path, runner, direction='sway-')
    result = run(ctx)
    saved = load_config(config)
    assert result['verdict'] == 'PARTIAL' and result['dead_directions'] == ['sway-']
    assert result['directions']['sway-']['dead'] and not result['directions']['sway-']['converged']
    assert saved['decoupling_negative'] == before.get('decoupling_negative')  # the deployed column is kept
    assert saved['decoupling'] == before['decoupling']
    assert not any('verify' in r for r in records)
    assert len(runner.commands) == 3  # settle, the one probe hold, final settle: no refine or low holds


def test_save_records_fit_provenance(config, tmp_path, monkeypatch):
    from mhseals_testing import calibration
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path))
    ctx, _, _ = context(tmp_path, Fake(config))
    ctx.services['fit_context'] = lambda: dict(session='s1', battery_v=15.5, muted=False)
    run(ctx)
    saved = load_config(config)['characterization']
    stored = calibration.load(tmp_path / 'calibration' / 'current.json')['directions']
    assert set(stored) == {a + s for a in ('surge', 'sway', 'yaw') for s in '+-'}
    assert set(saved['fit']) == set(stored) and saved['fit']['sway+']['id'] == stored['sway+']['fit']['id']
    assert stored['sway+']['battery_v'] == 15.5 and stored['sway+']['gate'] == {'muted': False}
    assert calibration.marks(load_config(config), current={'directions': stored}) == {}
