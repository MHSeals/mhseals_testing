"""The authority UI's knobs come from config/agent.yaml tune.authority and lease.motion_s; shipped == code default."""
import pytest
import yaml

from mhseals_testing.agent import config as agent_config
from mhseals_testing.ui import authority_chip, authority_tune, takeover


@pytest.fixture(autouse=True)
def fresh():
    authority_tune.tune.cache_clear()
    yield
    authority_tune.tune.cache_clear()


def test_shipped_values_match_the_code_defaults():
    shipped = agent_config.load()
    assert shipped['tune']['authority'] == agent_config.TUNE['authority'] == \
        {'working_s': 60.0, 'stop_wait_s': 40.0, 'view_wait_s': 15.0}
    assert shipped['lease']['motion_s'] == authority_tune.MOTION_S == 10
    assert authority_tune.tune() == {**agent_config.TUNE['authority'], 'motion_s': 10.0}


def test_an_unreadable_config_falls_back_to_the_code_defaults(monkeypatch, tmp_path):
    monkeypatch.setattr(agent_config, 'config_path', lambda: tmp_path / 'absent.yaml')
    assert authority_tune.tune() == {**agent_config.TUNE['authority'], 'motion_s': authority_tune.MOTION_S}


def _with(tmp_path, monkeypatch, **tune):
    doc = yaml.safe_load(agent_config.config_path().read_text())
    doc['tune'] = {'authority': tune}
    path = tmp_path / 'agent.yaml'
    path.write_text(yaml.safe_dump(doc))
    monkeypatch.setattr(agent_config, 'config_path', lambda: path)


def test_the_chip_and_takeover_read_the_setting(tmp_path, monkeypatch):
    _with(tmp_path, monkeypatch, working_s=5, view_wait_s=2)
    note = {'notes': [{'who': 'w', 't_ns': 90e9}]}
    assert authority_chip.working(note, {}, 94) == ('w', 4)
    assert authority_chip.working(note, {}, 96) is None

    class Registry:
        t = 0.0
        def clock(self): return self.t
        def sleep(self, s): self.t += s
        def state(self): return {}
    registry, printed = Registry(), []
    assert takeover.watch(registry, type('C', (), {'print': lambda self, s: printed.append(s)})()).mode == 'quit'
    assert 2 <= registry.t < 2.5


@pytest.mark.parametrize('tune', [{'other': {}}, {'authority': {'nope': 1}}, {'authority': {'working_s': 0}},
                                  {'authority': {'working_s': True}}, {'authority': []}])
def test_bad_tune_is_refused_with_the_key(tune):
    doc = yaml.safe_load(agent_config.config_path().read_text())
    with pytest.raises(ValueError, match='tune'):
        agent_config.validate({**doc, 'tune': tune})
