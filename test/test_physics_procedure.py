"""The physics characterization steps against fakes: records, provenance, refusals."""
from contextlib import contextmanager
import json
import math
from types import SimpleNamespace

import pytest
import yaml

from mhseals_hardware.configuration import config_sha, load_config, update_config
from mhseals_testing import physics_record
from mhseals_testing.analysis import physics as fits
from mhseals_testing.procedures import Settings, physics


class Boat:
    """A planar boat at 20 Hz: M v' = W - D1 v - D2 v|v| per axis, W = k e|e| (efforts) or B T (pulses).

    ``runner`` and monitor clock in one: ``hold`` drives an effort, ``supervise`` the last raw pulses.
    ``tied`` keeps it still. ``takeover=('hold' or 'pulse', n)`` has a human take over the n-th one
    as the real Runner does: a hold re-runs and returns the clean window with ``overrides`` grown,
    supervise calls ``on_pause``. ``abort=('pulse', n)`` raises Aborted (an abort key) mid-pulse.
    """
    DT = 0.05

    def __init__(self, k=(1.0, 1.0, 1.0), mass=(25.0, 35.0, 4.0), d1=(5.0, 9.0, 1.0), d2=(12.0, 25.0, 1.5),
                 noise=0.0, thrust=(40.0, 40.0, 40.0, 40.0), drift=(0.0, 0.0), tied=False, takeover=None,
                 abort=None):
        self.k, self.mass, self.d1, self.d2, self.noise = k, mass, d1, d2, noise
        self.thrust, self.drift, self.tied = thrust, drift, tied
        self.t, self.v, self.yaw, self.step = 0.0, [0.0, 0.0, 0.0], 0.0, 0
        self.series = dict(odometry=[], gyro=[])
        self.sent, self.pulses = [], None
        self.takeover, self.abort, self.calls, self.overrides = takeover, abort, dict(hold=0, pulse=0), []
        self.on_resume, self.resumed = lambda: None, None

    def wrench(self, command):
        if command is not None:
            return [k * e * abs(e) for k, e in zip(self.k, command)]
        efforts = [(p - 1500) / 400 for p in self.pulses or [1500] * 4]
        force = [t * e * abs(e) for t, e in zip(self.thrust, efforts)]
        # outputs FL FR RR RL at 45 degrees, arm 0.43 m
        return [0.7 * (force[0] + force[1] + force[2] + force[3]),
                0.7 * (force[0] - force[1] + force[2] - force[3]),
                0.43 * (-force[0] + force[1] + force[2] - force[3])]

    def advance(self, seconds, command=None, sample=None):
        for _ in range(int(round(seconds / self.DT))):
            w = self.wrench(command)
            self.v = [0.0, 0.0, 0.0] if self.tied else [v + (wi - a * v - b * v * abs(v)) / m * self.DT
                                                         for v, wi, m, a, b in zip(self.v, w, self.mass, self.d1, self.d2)]
            self.yaw += self.v[2] * self.DT
            self.t += self.DT
            self.step += 1
            jitter = self.noise * (1 if self.step % 2 else -1)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            ground = (self.v[0] + self.drift[0] * c + self.drift[1] * s, self.v[1] - self.drift[0] * s + self.drift[1] * c)
            self.series['odometry'].append((self.t, dict(vx=ground[0] + jitter, vy=ground[1] - jitter,
                                                         wz=self.v[2] + jitter, yaw=self.yaw)))
            self.series['gyro'].append((self.t, dict(wz=self.v[2])))
            if sample:
                sample(self.t)

    def hold(self, command, seconds, label=''):
        self.calls['hold'] += 1
        if self.takeover == ('hold', self.calls['hold']):
            began = self.t
            self.sent.append(tuple(command))
            self.advance(seconds / 2, tuple(command))
            self.advance(5.0, (0.0, 0.0, 0.0))      # the human drives, then the settle
            self.resumed = self.t
            self.on_resume()                        # as Runner._pause: a raise here ends at neutral
            self.overrides.append(dict(by='manual', start=began, end=self.t, window=(began - 0.5, self.t)))
        start = self.t
        self.sent.append(tuple(command))
        self.advance(seconds, tuple(command))
        return start, self.t

    def supervise(self, seconds, label='', sample=None, on_pause=lambda: None, **kwargs):
        from mhseals_testing.runner import Aborted
        self.calls['pulse'] += 1
        start = self.t
        if ('pulse', self.calls['pulse']) in (self.takeover, self.abort):
            self.advance(seconds / 2, None, sample)
            if self.abort == ('pulse', self.calls['pulse']):
                raise Aborted('motion cancelled by operator')
            on_pause()
        self.advance(seconds, None, sample)
        return start, self.t, False

    def invalid(self, start, end):
        return any(start < o['window'][1] and end > o['window'][0] for o in self.overrides)

    def stop(self):
        pass


def make_ctx(tmp_path, answers=(), runner=None, status=None, **settings):
    config = tmp_path / 'hardware.yaml'
    if not config.exists():
        update_config(dict(reverse_thrust_ratio=0.78), config)
    answers, records = (answers if callable(answers) else list(answers)), []

    def ask_text(title, message, text='', check=None):
        answer = answers(message) if callable(answers) else answers.pop(0)
        if answer is not None and check:
            check(answer)
        return answer

    @contextmanager
    def motion(name, **details):
        yield runner

    def pulses(values):
        runner.pulses = values

    sha = config_sha(load_config(config))
    status = dict(config_sha=sha, effective_config_sha=sha, pending_restart_keys=[]) if status == 'ok' else status
    services = dict(user_config=config, physics_root=tmp_path / 'char', ask_text=ask_text,
                    fit_context=lambda: dict(session='s1', run='r1', battery_v=15.9, muted=False),
                    driver_status=lambda: status, pulses=pulses)
    ctx = SimpleNamespace(
        settings=Settings(**settings), services=services, record=records.append, motion=motion,
        observe=motion, prompt=lambda *a, **k: '', profile=SimpleNamespace(outputs='odroid'),
        monitor=SimpleNamespace(clock=lambda: runner.t if runner else 0.0, gyro_sensor='gyro'),
        history=lambda since=float('-inf'), until=float('inf'), name='odometry':
            [s for s in (runner.series.get(name, []) if runner else []) if since <= s[0] <= until])
    return ctx, records


def saved(result):
    record = yaml.safe_load(open(result['path']))
    assert record['schema'] == physics_record.SCHEMA
    return record


def test_weigh_in_records_mass_cg_and_bifilar_inertia_with_provenance(tmp_path):
    ctx, records = make_ctx(tmp_path, ['24.6', '0.05', '0', '-0.02', '2.0', '0.4', '1.5'])
    result = physics.dock(ctx)
    record = saved(result)
    assert record['step'] == 'weigh_in' and record['mass'] == 24.6 and record['cg'] == [0.05, 0.0, -0.02]
    assert record['iz']['value'] == pytest.approx(24.6 * 9.80665 * 0.16 * 4 / (16 * math.pi ** 2 * 1.5))
    assert record['units']['mass'] == 'kg' and record['counts']['entries'] == 7
    hardware = record['hardware']
    assert hardware['keys']['reverse_thrust_ratio'] == 0.78 and hardware['sim_derived'] is False
    assert hardware['hash'].startswith('sha256:') and hardware['physics_hash'].startswith('sha256:')
    assert record['fit']['session'] == 's1' and record['battery_v'] == 15.9
    assert record['conditions'] == {'verdict': 'unknown'}
    assert records[-1]['verdict'] == 'SAVED'


def test_weigh_in_without_a_swing_keeps_the_inertia_a_prior(tmp_path):
    ctx, _ = make_ctx(tmp_path, ['24.6', '0', '0', '0', ''])
    assert saved(physics.dock(ctx))['iz'] == {'value': None, 'method': 'none: prior'}


def test_cancel_or_out_of_range_entry_refuses_and_writes_nothing(tmp_path):
    ctx, records = make_ctx(tmp_path, [None])
    assert physics.dock(ctx)['verdict'] == 'REFUSED'
    ctx, _ = make_ctx(tmp_path, ['500'])
    with pytest.raises(ValueError):
        physics.dock(ctx)       # the popup's check keeps it open on a value out of range
    assert not (tmp_path / 'char').exists()


def test_driver_check_refuses_a_pending_restart_or_another_file(tmp_path):
    config = tmp_path / 'hardware.yaml'
    update_config(dict(reverse_thrust_ratio=0.78), config)
    sha = config_sha(load_config(config))
    good = dict(config_sha=sha, effective_config_sha=sha, pending_restart_keys=[])
    assert physics_record.driver_check(good, config, True) == (
        {'config_sha': sha, 'effective_config_sha': sha, 'pending_restart_keys': []}, '')
    assert 'restart' in physics_record.driver_check(dict(good, pending_restart_keys=['decoupling']), config, True)[1]
    assert 'another hardware file' in physics_record.driver_check(dict(good, config_sha='0'), config, True)[1]
    assert physics_record.driver_check(None, config, True)[1] and not physics_record.driver_check(None, config, False)[1]


def test_a_record_needs_units_and_counts_and_hashes_its_samples(tmp_path):
    header = dict(step='x', fit=dict(id='f1'))
    with pytest.raises(ValueError):
        physics_record.write(header, dict(units={}), root=tmp_path)
    path = physics_record.write(header, dict(units={}, counts={}), samples=[[0.0, 1.0]], root=tmp_path)
    record = yaml.safe_load(path.read_text())
    text = (tmp_path / record['samples_file']['path']).read_text()
    assert record['samples_file']['sha256'] == physics_record.sha256(text) and json.loads(text) == [[0.0, 1.0]]


def test_log_stamps_commanded_pulses_battery_and_raw_gyro_on_one_clock(tmp_path):
    boat = Boat()
    ctx, _ = make_ctx(tmp_path, runner=boat)
    log = physics.Log(ctx)
    segment = log.hold(boat, (0.5, 0.0, 0.0), 2.0, 'surge+')
    raw = log.pulse(boat, [1700, 1500, 1500, 1500], 1.0, 'out1')
    assert segment['pulses'] == physics.commanded_pulses((0.5, 0.0, 0.0), load_config(tmp_path / 'hardware.yaml'))
    assert segment['pulses'] != [1500] * 4 and raw['pulses'] == [1700, 1500, 1500, 1500]
    assert segment['battery'] == [[0.0, 15.9], [pytest.approx(2.0), 15.9]] and not segment['overridden']
    samples = log.samples()
    assert samples['pulse_source'] == 'commanded' and len(samples['odometry']) == len(samples['gyro']) == 60
    assert samples['odometry'][0][0] == samples['gyro'][0][0]
    assert log.counts() == dict(segments=2, overridden=0)


def test_require_driver_refuses_on_the_boat_without_a_status(tmp_path):
    ctx, _ = make_ctx(tmp_path, status=None)
    with pytest.raises(physics.Refused):
        physics.require_driver(ctx)
    ctx, _ = make_ctx(tmp_path, status='ok')
    assert physics.require_driver(ctx)['pending_restart_keys'] == []


def test_dock_noise_records_sigma_per_channel_after_removing_a_slow_swing(tmp_path):
    boat = Boat(noise=0.004)
    boat.v = [0.0, 0.0, 0.0]
    ctx, records = make_ctx(tmp_path, runner=boat, dock_step='dock_noise')
    result = physics.dock(ctx)
    record = saved(result)
    noise = record['noise']
    assert noise['odometry.vx']['sigma'] == pytest.approx(0.004, rel=0.05) and noise['odometry.vx']['n'] == 1200
    assert noise['odometry.vx']['lag1'] == pytest.approx(-1.0, abs=0.01)    # alternating, white-ish
    assert noise['gyro.wz']['sigma'] == pytest.approx(0.0, abs=1e-9)
    assert record['counts']['odometry.vy'] == 1200 and 'sigma' in record['units']
    assert record['samples_file']['path'].endswith('dock_noise.samples.json')


def test_env_floor_never_attributes_when_the_sensor_exceeds_the_floor():
    assert fits.env_floor(0.05, 0.03) == (pytest.approx(0.04), True)
    assert fits.env_floor(0.02, 0.03) == (0.0, False)


def test_bollard_fits_forward_and_reverse_strength_deadband_and_ratio_per_output(tmp_path):
    import re
    strength = {1: 40.0, 2: 38.0, 3: 41.0, 4: 39.0}

    def answer(message):
        if 'angle' in message:
            return '0'
        output, pulse = map(int, re.search(r'Output (\d) \(\w+\) at (\d+) us', message).groups())
        u = abs(pulse - 1500) / 400
        k = strength[output] * (1.0 if pulse > 1500 else 0.78)
        return f'{(k * u * u if u > 0.05 else 0.0) / 9.80665:.5f}'
    boat = Boat(tied=True)
    ctx, records = make_ctx(tmp_path, answer, runner=boat, status='ok', amplitude=1.0)
    record = saved(physics.thrust(ctx))
    one = record['thrusters']['1']
    assert one['thruster'] == 'FL' and one['forward']['K'] == pytest.approx(40.0, rel=1e-3)
    assert one['reverse']['K'] == pytest.approx(31.2, rel=1e-3) and one['reverse_ratio'] == pytest.approx(0.78, rel=1e-3)
    assert one["forward"]["db_us"] == pytest.approx(60.0) and one["o_us"] == pytest.approx(0.0)  # 0.05 kgf hides 0.06, 0.1
    assert one["forward"]["dof"] == 4 and one['forward']['residual_sd'] == pytest.approx(0.0, abs=1e-3)
    assert record['counts']['readings'] == 64 and record['pack_v'] == {'min': 15.9, 'max': 15.9}
    assert record['hardware']['driver']['pending_restart_keys'] == [] and boat.pulses is None
    assert record['units']['K'].startswith('N')


def test_thrust_steps_refuse_when_the_driver_runs_another_file(tmp_path):
    ctx, records = make_ctx(tmp_path, runner=Boat(), status=dict(config_sha='0', pending_restart_keys=[]))
    assert physics.thrust(ctx)['verdict'] == 'REFUSED' and 'another hardware file' in records[-1]['reason']


def test_pulses_give_estimate_mixer_experiments_in_canonical_order(tmp_path):
    boat = Boat(noise=0.0005)
    update_config(dict(reversed=[False, True, False, False]), tmp_path / 'hardware.yaml')
    ctx, _ = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='pulses')
    record = saved(physics.thrust(ctx))
    assert record['counts']['experiments'] == 16 and record['skipped'] == []
    first = record['experiments'][0]
    assert first['thruster'] == 'FL' and first['cmd4'] == [0.3, 0.0, 0.0, 0.0]
    # FL at +30 %: 40 N * 0.09 = 3.6 N; surge 0.7 * 3.6 / 25 kg, yaw -0.43 * 3.6 / 4 (less drag at onset)
    assert first['W_hat'][0] == pytest.approx(0.7 * 3.6 / 25, rel=0.15)
    assert first['W_hat'][2] == pytest.approx(-0.43 * 3.6 / 4, rel=0.15)
    flipped = next(e for e in record['experiments'] if e['thruster'] == 'FR')
    assert flipped['cmd4'] == [0.0, -0.3, 0.0, 0.0] and min(first['sigma']) >= 0.005
    assert record['order'] == ['FL', 'FR', 'RR', 'RL'] and record['units']['W_hat'].startswith('m/s^2')


def test_survey_fits_curves_with_errors_and_coast_down_drag_over_mass(tmp_path):
    boat = Boat(k=(30.0, 30.0, 10.0), noise=0.002)
    ctx, records = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='survey', amplitude=1.0)
    record = saved(physics.thrust(ctx))
    surge = record['speed_curves']['surge+']
    assert surge['n'] == 8 and surge['levels'] == 4 and surge['dof'] == 6 and len(surge['residuals']) == 8
    assert surge['d1'] == pytest.approx(5 / 30, rel=0.1) and surge['d2'] == pytest.approx(12 / 30, rel=0.05)
    assert len(surge['covariance']) == 2 and surge['pure_error_var'] is not None
    sway = record['speed_curves']['sway-']
    assert sway['d2'] == pytest.approx(25 / 30, rel=0.05)
    coast = record['coast_down']['surge+']
    assert coast['d1_over_m'] == pytest.approx(5 / 25, abs=0.05) and coast['d2_over_m'] == pytest.approx(12 / 25, rel=0.1)
    assert record['coast_down']['yaw-']['d2_over_m'] == pytest.approx(1.5 / 4, rel=0.1)
    assert record['counts']['holds'] == 48 and len(record['holds']) == 48
    assert record['wave_floor']['vx'] == pytest.approx(0.002, rel=0.2) and record['env_floor'] is None
    assert record['holds'][0]['applied'] == [1.0, 0.0, 0.0] and boat.sent[1] == (1.0, 0.0, 0.0)


def test_survey_splits_the_wave_floor_against_the_dock_sigma(tmp_path):
    root = tmp_path / 'char'
    root.mkdir()
    (root / 'f1-dock_noise.yaml').write_text(yaml.safe_dump(dict(noise={'odometry.vx': dict(sigma=0.001)})))
    ctx, _ = make_ctx(tmp_path, runner=Boat(k=(30.0, 30.0, 10.0), noise=0.002), status='ok',
                      thrust_step='survey', amplitude=1.0)
    env = saved(physics.thrust(ctx))['env_floor']['vx']
    assert env['ok'] and env['value'] == pytest.approx((0.002 ** 2 - 0.001 ** 2) ** 0.5, rel=0.2)


def test_survey_refuses_a_backwards_axis(tmp_path):
    ctx, records = make_ctx(tmp_path, runner=Boat(k=(-30.0, 30.0, 10.0)), status='ok', thrust_step='survey',
                            amplitude=1.0)
    assert physics.thrust(ctx)['verdict'] == 'REFUSED' and 'backwards' in records[-1]['reason']


def test_survey_turns_between_repeats_so_headings_spread():
    assert fits.heading_spread([0.0, 0.0]) == pytest.approx(1.0)
    assert fits.heading_spread([0.0, math.pi]) == pytest.approx(0.0, abs=1e-12)


def test_survey_records_the_heading_spread_it_reached(tmp_path):
    boat = Boat(k=(30.0, 30.0, 10.0))
    ctx, _ = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='survey', amplitude=1.0)
    record = saved(physics.thrust(ctx))
    spread = record['heading_spread']
    assert spread['n'] == 48 and spread['ok'] and spread['R'] < 0.9
    turns = [s for s in record['holds'] if s['repeat'] == 1]
    assert abs(math.remainder(turns[0]['yaw'] - record['holds'][23]['yaw'], 2 * math.pi)) > 2.0


def test_maneuvers_are_saved_withheld_with_circles_both_ways_and_zigzags(tmp_path):
    boat = Boat(k=(30.0, 30.0, 10.0))
    ctx, _ = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='maneuvers', amplitude=0.5)
    record = saved(physics.thrust(ctx))
    assert record['withheld'] is True and 'never fitted' in record['use']
    kinds = [(r['kind'], r.get('sign')) for r in record['runs']]
    assert kinds == [('circle', 1), ('circle', -1), ('zigzag', None)] * 2
    zigzag = record['runs'][2]
    assert zigzag['switches'] == 6 and zigzag['segments'][1] > zigzag['segments'][0]
    samples = json.loads((tmp_path / 'char' / record['samples_file']['path']).read_text())
    first, last = zigzag['segments']
    assert {s['label'].split()[0] for s in samples['segments'][first:last]} == {'zigzag+', 'zigzag-'}


def test_records_from_the_sim_or_its_config_are_marked_sim_derived(tmp_path):
    import pathlib
    sim = pathlib.Path(physics.__file__).resolve().parents[2] / 'config' / 'sim' / 'hardware_crane.yaml'
    assert physics_record.hardware_block(sim)['sim_derived'] is True
    assert load_config(sim)['characterization']['source'].startswith('sim:')
    ctx, _ = make_ctx(tmp_path, ['24.6', '0', '0', '0', ''])
    assert saved(physics.dock(ctx))['hardware']['sim_derived'] is False
    ctx, _ = make_ctx(tmp_path, ['24.6', '0', '0', '0', ''])
    ctx.profile.use_sim_time = True
    assert saved(physics.dock(ctx))['hardware']['sim_derived'] is True


def test_every_record_names_the_config_history_snapshot_at_fit_time(tmp_path):
    head = dict(id='c7', after='abc123def456')
    ctx, _ = make_ctx(tmp_path, ['24.6', '0', '0', '0', ''])
    ctx.services['history'] = SimpleNamespace(head=lambda sub: head if sub == 'hardware' else None)
    hardware = saved(physics.dock(ctx))['hardware']
    assert hardware['history'] == {'id': 'c7', 'sha': 'abc123def456'}
    assert set(hardware['keys']) == set(physics_record.KEYS) and hardware['thrusters'] == ['FL', 'FR', 'RR', 'RL']


def test_driver_check_refuses_launch_overrides_beyond_the_output_routing(tmp_path):
    config = tmp_path / 'hardware.yaml'
    update_config(dict(reverse_thrust_ratio=0.78), config)
    loaded = load_config(config)
    sha = config_sha(loaded)
    flipped = config_sha(dict(loaded, reversed=[True, False, False, False]))
    reason = physics_record.driver_check(dict(config_sha=sha, effective_config_sha=flipped), config, True)[1]
    assert 'launch overrides' in reason and 'reversed' in reason
    assert physics_record.driver_check(dict(config_sha=sha), config, True)[1]      # no effective hash: unconfirmed
    routed = dict(output_backend='log')
    status = dict(config_sha=sha, effective_config_sha=config_sha(dict(loaded, **routed)), outputs=routed)
    assert physics_record.driver_check(status, config, True)[1] == ''               # only the routing differs


def bollard_answer(message):
    return '0' if 'angle' in message else '1.0'


def test_bollard_axial_pull_is_the_reading_times_the_cosine_of_the_line_angle(tmp_path):
    ctx, _ = make_ctx(tmp_path, lambda m: '60' if 'angle' in m else '2.0', runner=Boat(tied=True), status='ok',
                      amplitude=1.0)
    record = saved(physics.thrust(ctx))
    assert record['readings'][0][2] == pytest.approx(2.0 * 9.80665 * 0.5)
    samples = json.loads((tmp_path / 'char' / record['samples_file']['path']).read_text())
    hold = next(s for s in samples['segments'] if 'force_n' in s)
    assert hold['angle_deg'] == 60.0 and hold['force_n'] == pytest.approx(9.80665)


def test_bollard_scales_raw_pulses_by_the_amplitude(tmp_path):
    boat = Boat(tied=True)
    sent = []
    ctx, _ = make_ctx(tmp_path, bollard_answer, runner=boat, status='ok', amplitude=0.5)
    pulses = ctx.services['pulses']
    ctx.services['pulses'] = lambda values: (sent.append(values), pulses(values))
    saved(physics.thrust(ctx))
    raw = [p for values in sent if values for p in values if p != 1500]
    assert max(raw) == 1700 and min(raw) == 1300 and boat.pulses is None


def test_bollard_levels_outside_the_pulse_range_are_refused_at_load():
    cfg = physics.load_settings()
    assert cfg['bollard']['max_speed'] > 0
    for levels in ([0.5, 1.2], [0.0, 0.5], [-0.3]):
        with pytest.raises(physics.Refused, match='bollard levels'):
            physics.check_settings(dict(cfg, bollard=dict(cfg['bollard'], levels=levels)))


def test_bollard_aborts_and_clears_pulses_when_the_boat_is_not_tied(tmp_path):
    boat = Boat()
    ctx, records = make_ctx(tmp_path, bollard_answer, runner=boat, status='ok', amplitude=1.0)
    assert physics.thrust(ctx)['verdict'] == 'REFUSED' and 'not tied' in records[-1]['reason']
    assert boat.pulses is None and not (tmp_path / 'char').exists()


def test_an_abort_mid_pulse_clears_the_raw_pulses(tmp_path):
    from mhseals_testing.runner import Aborted
    for step in ('bollard', 'pulses'):
        boat = Boat(tied=step == 'bollard', abort=('pulse', 3))
        ctx, _ = make_ctx(tmp_path, bollard_answer, runner=boat, status='ok', thrust_step=step, amplitude=1.0)
        with pytest.raises(Aborted):
            physics.thrust(ctx)
        assert boat.pulses is None


def test_a_takeover_during_raw_pulses_clears_them_and_aborts_the_step(tmp_path):
    boat = Boat(takeover=('pulse', 2))
    cleared = []
    ctx, records = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='pulses')
    pulses = ctx.services['pulses']
    ctx.services['pulses'] = lambda values: (cleared.append(values is None), pulses(values))
    assert physics.thrust(ctx)['verdict'] == 'REFUSED'
    assert 'took over' in records[-1]['reason'] and records[-1]['invalidated'] == ['rest', 'output 1 +30%']
    assert cleared[:3] == [False, False, True] and boat.pulses is None    # cleared in the pause, before finally
    assert not (tmp_path / 'char').exists()


def test_a_takeover_during_a_survey_hold_aborts_the_step_and_invalidates_its_pair(tmp_path):
    boat = Boat(k=(30.0, 30.0, 10.0), takeover=('hold', 3))      # neutral, surge+ hold, then its coast
    ctx, records = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='survey', amplitude=1.0)
    assert physics.thrust(ctx)['verdict'] == 'REFUSED'
    assert records[-1]['taken_over'] == 'coast' and records[-1]['invalidated'] == ['surge+ 1.00', 'coast']
    assert boat.sent[-1] == (0.0, 0.0, 0.0) and not (tmp_path / 'char').exists()


def test_a_takeover_in_a_maneuver_aborts_and_invalidates_the_whole_run(tmp_path):
    boat = Boat(k=(30.0, 30.0, 10.0), takeover=('hold', 2))      # the first circle, after its approach
    ctx, records = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='maneuvers', amplitude=0.5)
    assert physics.thrust(ctx)['verdict'] == 'REFUSED'
    assert records[-1]['invalidated'] == ['approach', 'circle+ 0.25']


def test_log_recheck_flags_a_segment_inside_an_override_guard_window(tmp_path):
    boat = Boat()
    ctx, _ = make_ctx(tmp_path, runner=boat)
    log = physics.Log(ctx)
    log.hold(boat, (0.2, 0.0, 0.0), 1.0, 'surge+')
    boat.overrides.append(dict(window=(boat.t - 0.5, boat.t + 3.0)))     # a takeover 0.5 s after it ended
    with pytest.raises(physics.TakenOver):
        log.recheck(boat)
    assert log.segments[0]['overridden']


def test_thrust_physics_is_unavailable_unarmed():
    from mhseals_testing.procedures import PROCEDURES, unavailable
    from mhseals_testing.profiles import load_profile
    thrust = next(p for p in PROCEDURES if p.key == 'y')
    assert 'arm' in unavailable(thrust, load_profile('boat'), armed=False)


def test_survey_refuses_when_the_first_hold_has_no_speed(tmp_path):
    boat = Boat(k=(30.0, 30.0, 10.0))
    advance = boat.advance

    def blank(*args, **kwargs):
        advance(*args, **kwargs)
        for _, v in boat.series['odometry'][-int(round(args[0] / boat.DT)):]:
            v['vx'] = None
    boat.advance = blank
    ctx, records = make_ctx(tmp_path, runner=boat, status='ok', thrust_step='survey', amplitude=1.0)
    assert physics.thrust(ctx)['verdict'] == 'REFUSED' and 'no odometry vx' in records[-1]['reason']


def test_a_hardware_change_during_the_step_refuses_the_record(tmp_path):
    ctx, records = make_ctx(tmp_path, runner=Boat(noise=0.0005), status='ok', thrust_step='pulses')
    statuses = [ctx.services['driver_status'](), dict(ctx.services['driver_status'](), config_sha='changed')]
    ctx.services['driver_status'] = lambda: statuses.pop(0) if len(statuses) > 1 else statuses[0]
    assert physics.thrust(ctx)['verdict'] == 'REFUSED' and not (tmp_path / 'char').exists()
    assert 'another hardware file' in records[-1]['reason']     # the end check reads the file on disk again


def test_survey_and_maneuvers_say_their_efforts_are_commanded(tmp_path):
    ctx, _ = make_ctx(tmp_path, runner=Boat(k=(30.0, 30.0, 10.0)), status='ok', thrust_step='maneuvers',
                      amplitude=0.5)
    assert saved(physics.thrust(ctx))['effort_source'] == 'commanded'


def test_after_a_takeover_no_hold_runs_again(tmp_path):
    for step, takeover, amplitude in (('survey', ('hold', 2), 1.0), ('maneuvers', ('hold', 2), 0.5)):
        boat = Boat(k=(30.0, 30.0, 10.0), takeover=takeover)
        ctx, records = make_ctx(tmp_path, runner=boat, status='ok', thrust_step=step, amplitude=amplitude)
        sent = len(boat.sent)
        assert physics.thrust(ctx)['verdict'] == 'REFUSED' and 'took over' in records[-1]['reason']
        assert boat.calls['hold'] == 2 and boat.t == boat.resumed          # nothing driven after the release
        assert len(boat.sent) == sent + 2 and boat.overrides == []         # the interrupted hold never re-ran
        assert records[-1]['invalidated'][-1] == records[-1]['taken_over']
