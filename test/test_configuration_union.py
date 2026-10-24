"""Real pure domain adapters; no ROS or powered output operations."""
from pathlib import Path

import pytest
import yaml

from mhseals_testing.configuration import Selection, bootstrap, hardware, navigation


def preparations(root, scope, tmp_path, backend='topic'):
    operator = tmp_path / 'hardware.yaml'
    operator.write_text('manual_yaw_scale: 0.5\n')
    nav = Path(navigation.__file__).parents[1] / 'config/nav2_params.yaml'
    return {
        'hardware': hardware.prepare_inputs(operator, session=(
            yaml.safe_dump({'manual_yaw_scale': 0.8, 'output_backend': backend}), 'profile invocation')),
        'navigation': navigation.prepare_inputs(root, scope, nav, None, sim='true', use_lidar='true'),
    }


def test_empty_root_union_and_immutable_adoption(tmp_path):
    root = tmp_path / 'store'
    prepared = preparations(root, 'session', tmp_path)
    reference = bootstrap(root, 'session', prepared)
    selected = reference.validate()
    assert len(selected.documents) == 4
    effective = selected.documents['hardware-pure']
    assert effective['baseline']['manual_yaw_scale'] == 0.5
    assert effective['effective']['manual_yaw_scale'] == 0.8
    assert effective['effective']['output_backend'] == 'topic'
    assert effective['baseline']['output_backend'] != 'topic'
    (tmp_path / 'hardware.yaml').write_text('manual_yaw_scale: 0.1\n')
    adopted = Selection.from_record(reference.record())
    assert adopted.validate().documents == selected.documents
    document, subset = hardware.startup_inputs(root=root, scope='session',
                                              config_candidate=reference.candidate)
    assert subset.ticket == selected.ticket
    assert document['effective']['manual_yaw_scale'] == 0.8
    nav = navigation.select_snapshot(root, 'session', 'not-read', 'not-read',
                                     sim='false', use_lidar='false',
                                     config_candidate=reference.candidate)
    assert nav['invocation']['sim'] is True


def test_corrupt_foreign_artifact_blocks_owner_and_hardware(tmp_path):
    root = tmp_path / 'store'
    reference = bootstrap(root, 'session', preparations(root, 'session', tmp_path))
    files = list(root.rglob('nav2-baseline.json'))
    assert len(files) == 1
    files[0].write_text('{}')
    with pytest.raises(ValueError):
        reference.validate()
    with pytest.raises(ValueError):
        hardware.startup_inputs(root=root, scope='session', config_candidate=reference.candidate)


def test_missing_selection_never_bootstraps(tmp_path):
    reference = Selection(str(tmp_path), 'missing', 'a' * 64, ('hardware', 'navigation'))
    with pytest.raises(Exception):
        reference.validate()
    assert list(tmp_path.iterdir()) == []


def test_preparation_failure_does_not_reserve(tmp_path):
    root = tmp_path / 'store'
    prepared = preparations(root, 'session', tmp_path)
    prepared['navigation']['documents'].pop('nav2-baseline')
    with pytest.raises(ValueError, match='incomplete'):
        bootstrap(root, 'session', prepared)
    assert not root.exists()


def test_partial_union_evidence_blocks_complete_selection(tmp_path, monkeypatch):
    root = tmp_path / 'store'
    prepared = preparations(root, 'partial', tmp_path)
    observe = navigation.observe_inputs

    def incomplete(*args):
        result = observe(*args)
        result.pop('nav2-baseline')
        return result

    monkeypatch.setattr(navigation, 'observe_inputs', incomplete)
    with pytest.raises(ValueError):
        bootstrap(root, 'partial', prepared)
    assert not (root / 'partial/active.json').exists()
    with pytest.raises(Exception):
        hardware.startup_inputs(root=root, scope='partial', config_candidate='a' * 64)
    assert list(root.glob('partial'))  # reservation retained for owner recovery


def test_session_overlay_does_not_publish_effective_as_baseline(tmp_path):
    root = tmp_path / 'store'
    first = preparations(root, 'first', tmp_path)
    one = bootstrap(root, 'first', first).validate()
    second = preparations(root, 'second', tmp_path)
    alternate = tmp_path / 'alternate.yaml'
    alternate.write_text('manual_yaw_scale: 0.2\n')
    second['hardware'] = hardware.prepare_inputs(alternate, session=(
        yaml.safe_dump({'manual_yaw_scale': 0.8, 'output_backend': 'topic'}), 'session'))
    two = bootstrap(root, 'second', second).validate()
    assert one.documents['hardware-pure']['effective']['manual_yaw_scale'] == 0.8
    assert two.documents['hardware-pure']['effective']['manual_yaw_scale'] == 0.8
    assert one.documents['hardware-baseline']['effective']['manual_yaw_scale'] == 0.5
    assert two.documents['hardware-baseline']['effective']['manual_yaw_scale'] == 0.2
    # A separate persistent baseline can share E without becoming the same B.
    assert one.candidate['baseline'] != two.candidate['baseline']
