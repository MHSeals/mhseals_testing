import json

from mhseals_testing.harness import runs


def test_one_file_per_procedure_run(tmp_path):
    first = runs.write_run(tmp_path, dict(procedure='axis', verdict='SIGN MATCH'), settings={'amp': 0.3})
    second = runs.write_run(tmp_path, dict(procedure='axis', verdict='SIGN MATCH'))
    assert sorted(p.name for p in tmp_path.glob('*.json')) == ['001-axis.json', '002-axis.json']
    saved = json.loads((tmp_path / '001-axis.json').read_text())
    assert saved['schema'] == 'mhseals.run/1'
    assert saved['settings'] == {'amp': 0.3} and second['run_id'] == '002-axis'


def test_aborted_keeps_its_status(tmp_path):
    record = runs.write_run(tmp_path, dict(procedure='step', verdict='ABORTED'))
    assert record['status'] == runs.ABORTED and not runs.usable(record)


def test_existing_run_is_never_rewritten(tmp_path):
    runs.write_run(tmp_path, dict(procedure='axis', verdict='A', run_id='r1'))
    again = runs.write_run(tmp_path, dict(procedure='axis', verdict='B', run_id='r1'))
    assert again['result']['verdict'] == 'A'


def test_exclusion_appends_and_keeps_measurements(tmp_path):
    before = runs.write_run(tmp_path, dict(procedure='axis', verdict='SIGN MATCH', gain=1.5))
    after = runs.exclude_run(tmp_path, before['run_id'], 'wind gust', 'operator')
    assert after['status'] == runs.EXCLUDED and after['excluded'] == dict(reason='wind gust', by='operator')
    assert after['result'] == before['result'] and after['settings'] == before['settings']
    assert after['history'][-1]['from_status'] == runs.MEASURED
    assert not runs.usable(after) and runs.load_runs(tmp_path) == [after]


def test_run_id_is_slugified(tmp_path):
    assert runs.write_run(tmp_path, dict(procedure='nav2 bring-up'))['run_id'] == '001-nav2-bring-up'
