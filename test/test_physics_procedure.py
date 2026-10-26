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
    """
    DT = 0.05

    def __init__(self, k=(1.0, 1.0, 1.0), mass=(25.0, 35.0, 4.0), d1=(5.0, 9.0, 1.0), d2=(12.0, 25.0, 1.5),
                 noise=0.0, thrust=(40.0, 40.0, 40.0, 40.0), drift=(0.0, 0.0)):
        self.k, self.mass, self.d1, self.d2, self.noise = k, mass, d1, d2, noise
        self.thrust, self.drift = thrust, drift
        self.t, self.v, self.yaw, self.step = 0.0, [0.0, 0.0, 0.0], 0.0, 0
        self.series = dict(odometry=[], gyro=[])
        self.sent, self.pulses = [], None

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
            self.v = [v + (wi - a * v - b * v * abs(v)) / m * self.DT
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
        start = self.t
        self.sent.append(tuple(command))
        self.advance(seconds, tuple(command))
        return start, self.t

    def supervise(self, seconds, label='', sample=None, **kwargs):
        start = self.t
        self.advance(seconds, None, sample)
        return start, self.t, False

    def invalid(self, start, end):
        return False

    def stop(self):
        pass


def make_ctx(tmp_path, answers=(), runner=None, status=None, **settings):
    config = tmp_path / 'hardware.yaml'
    if not config.exists():
        update_config(dict(reverse_thrust_ratio=0.78), config)
    answers, records = list(answers), []

    def ask_text(title, message, text='', check=None):
        answer = answers.pop(0)
        if answer is not None and check:
            check(answer)
        return answer

    @contextmanager
    def motion(name, **details):
        yield runner

    def pulses(values):
        runner.pulses = values

    status = dict(config_sha=config_sha(load_config(config)), pending_restart_keys=[]) if status == 'ok' else status
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
    good = dict(config_sha=config_sha(load_config(config)), effective_config_sha='x', pending_restart_keys=[])
    assert physics_record.driver_check(good, config, True) == (
        {'config_sha': good['config_sha'], 'effective_config_sha': 'x', 'pending_restart_keys': []}, '')
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
