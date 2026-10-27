import json

import pytest

from mhseals_testing.harness import mission

ELEMENTS = [dict(name='gate 1', done=True, t=30.0), dict(name='gate 2', done=True, t=80.0),
            dict(name='dock', done=False, t=None)]


def test_summary_line_has_no_points():
    line = mission.summary_line('pair_test', 'complete', 3, 4, 142.0, ['late'])
    assert line == 'pair_test: complete, 3/4 elements, 142 s, 1 failure'
    assert 'pts' not in line


def test_block_starts_with_summary_and_counts_completion():
    block = mission.mission_block('pair_test', 'timeout', 90.0, ELEMENTS, failures=['dock missed @90s'])
    assert next(iter(block)) == 'summary'
    assert block['summary'] == 'pair_test: timeout, 2/3 elements, 90 s, 1 failure'
    assert block['completion']['done'] == 2


def test_write_run_is_a_run_file_with_mission_block(tmp_path):
    block = mission.mission_block('pair_test', 'complete', 10.0, ELEMENTS[:2])
    target = mission.write_run(tmp_path, 'r1', block)
    assert target == tmp_path / 'runs' / 'r1.json'
    record = json.loads(target.read_text())
    assert record['schema'] == 'mhseals.run/1' and record['mission']['summary'] == block['summary']
    assert record['procedure_key'] == 'mission'
    assert list(record['mission'])[0] == 'summary'


@pytest.mark.parametrize('domain', [42, 142, 0, 199, 229])
def test_domain_guard_refuses_boat_and_out_of_range(domain):
    with pytest.raises(ValueError):
        mission.check_domain(domain)


def test_domain_guard_accepts_sim_range():
    assert mission.check_domain(200) == 200 and mission.check_domain('228') == 228


def test_batch_checks_every_domain_before_running(tmp_path):
    calls = []
    with pytest.raises(ValueError):
        mission.sim_batch(tmp_path, 't', lambda *a: calls.append(a), repeat=3, domain=227)
    assert calls == [] and not (tmp_path / 'runs').exists()


def test_batch_records_one_run_per_repeat(tmp_path):
    def run_one(task, domain, profile, index):
        return dict(outcome='complete', duration=10.0 * index, elements=ELEMENTS[:2])
    batch = mission.sim_batch(tmp_path, 'pair_test', run_one, repeat=2,
                              profile='crane_full_stack', name='b')
    assert [r['domain'] for r in batch['runs']] == [200, 201]
    assert sorted(p.name for p in (tmp_path / 'runs').iterdir()) == ['b_pair_test_1.json', 'b_pair_test_2.json']
    assert batch['summary'] == 'pair_test: 2 sim runs'


def test_existing_run_is_never_overwritten(tmp_path):
    block = mission.mission_block('t', 'complete', 1.0)
    mission.write_run(tmp_path, 'r1', block)
    with pytest.raises(FileExistsError):
        mission.write_run(tmp_path, 'r1', block)
    calls = []

    def run_one(*a):
        calls.append(a)
        return dict(outcome='complete', duration=1.0)

    mission.sim_batch(tmp_path, 't', run_one, name='b')
    calls.clear()
    with pytest.raises(FileExistsError):
        mission.sim_batch(tmp_path, 't', run_one, name='b')
    assert calls == []


def test_clash_on_a_later_repeat_runs_nothing(tmp_path):
    block = mission.mission_block('t', 'complete', 1.0)
    mission.write_run(tmp_path, 'b_t_2', block)
    calls = []
    run_one = lambda *a: calls.append(a) or dict(outcome='complete', duration=1.0)
    with pytest.raises(FileExistsError):
        mission.sim_batch(tmp_path, 't', run_one, repeat=3, name='b')
    assert calls == []
    assert [p.name for p in (tmp_path / 'runs').iterdir()] == ['b_t_2.json']
