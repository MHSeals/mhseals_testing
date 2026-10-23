"""The params kind and probe (read only), the threshold bands, and {share:...} templating."""

import pytest

from mhseals_testing.health import config, registry
from mhseals_testing.health.kinds import params as params_kind, threshold
from mhseals_testing.health.model import Fact
from mhseals_testing.health.probes import params as probe

NODE = '/mavros/param'


def facts(**values):
    return {probe.fact_name(NODE, name): Fact(value) for name, value in values.items()}


def check(reference, names, **kw):
    return params_kind.Params(node=NODE, names=names, reference=str(reference), **kw)


@pytest.fixture
def reference(tmp_path):
    path = tmp_path / 'fcu.param'
    path.write_text('# header\nEK3_SRC1_YAW,1\nCOMPASS_USE,1\nARMING_CHECK 1\n')
    return path


def test_matching_parameters_are_ok(reference):
    result = params_kind.evaluate(check(reference, ['EK3_SRC1_YAW', 'COMPASS_USE']),
                                  facts(EK3_SRC1_YAW=1.0, COMPASS_USE=1.0))
    assert result.status == 'ok'


def test_the_ts035_incident_values_differ(reference):
    result = params_kind.evaluate(check(reference, ['EK3_SRC1_YAW', 'COMPASS_USE']),
                                  facts(EK3_SRC1_YAW=8.0, COMPASS_USE=0.0))
    assert result.status == 'warn' and 'EK3_SRC1_YAW 8 (want 1)' in result.detail


def test_unread_parameters_are_unknown_unless_something_differs(reference):
    names = ['EK3_SRC1_YAW', 'COMPASS_USE']
    assert params_kind.evaluate(check(reference, names), facts(COMPASS_USE=1)).status == 'unknown'
    assert params_kind.evaluate(check(reference, names), facts(COMPASS_USE=0)).status == 'warn'


def test_inline_expect_overrides_the_file(reference):
    spec = check(reference, ['COMPASS_USE'], expect={'COMPASS_USE': 0})
    assert params_kind.evaluate(spec, facts(COMPASS_USE=0)).status == 'ok'


def test_the_kind_tells_the_probe_what_to_read(reference):
    spec = check(reference, ['A', 'B'])
    assert registry.KINDS['params'].watch(spec) == {'params': ['/mavros/param:A', '/mavros/param:B']}


class FakeClient:
    """Records calls; any write or pull fails the test."""

    def __init__(self, values):
        self.values, self.calls = values, []

    def get_parameters(self, node, names):
        self.calls.append((node, names))
        return {n: self.values[n] for n in names if n in self.values}

    def __getattr__(self, name):
        raise AssertionError(f'the params probe called {name}')


def test_the_probe_only_reads_and_skips_names_the_fcu_has_not_loaded():
    client = FakeClient({'COMPASS_USE': 1.0})
    found = probe.collect(client, {NODE: ['COMPASS_USE', 'EK3_SRC1_YAW']}, clock=lambda: 5.0)
    assert client.calls == [(NODE, ['COMPASS_USE', 'EK3_SRC1_YAW'])]
    assert list(found) == [probe.fact_name(NODE, 'COMPASS_USE')] and found[list(found)[0]].time == 5.0


def test_threshold_ranges_expect_scale_and_abs():
    spec = threshold.Params(fact='x', warn=[0.8, 1.25], fail=[0.6, 1.8], scale=470, fmt='{:.2f}×')
    run = lambda v: threshold.evaluate(spec, {'x': Fact(v)})
    assert [run(v).status for v in (470, 330, 100, 900)] == ['ok', 'warn', 'fail', 'fail']
    assert run(470).value == '1.00×'
    mode = threshold.Params(fact='m', expect=['MANUAL', 'HOLD'])
    assert threshold.evaluate(mode, {'m': Fact('AUTO')}).status == 'fail'
    assert threshold.evaluate(mode, {'m': Fact('HOLD')}).status == 'ok'
    absolute = threshold.Params(fact='c', warn=0.1, below=False, abs=True)
    assert threshold.evaluate(absolute, {'c': Fact(-0.3)}).status == 'warn'
    assert threshold.evaluate(absolute, {}).status == 'unknown'


def test_share_template_names_the_package_directory(monkeypatch, tmp_path):
    monkeypatch.setattr(config, 'share_dir', lambda package: tmp_path / package)
    assert config.render('{share:mhseals_nav}/x', share=config.Share()) == f'{tmp_path}/mhseals_nav/x'
