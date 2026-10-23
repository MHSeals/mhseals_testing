from mhseals_testing.health import registry
from mhseals_testing.health.model import Fact

registry.discover()
KINDS = registry.KINDS
TOPIC = '/cmd_vel_mux'


def fact(value, **data):
    return Fact(value, data=data)


def pwm_facts(nodes=(), subscribers=(), armed=True, managed=True):
    return {'graph.nodes': fact(list(nodes)), f'graph.subscribers:{TOPIC}': fact(list(subscribers)),
            'session.armed': fact(armed), 'session.driver_managed': fact(managed)}


def owner(facts):
    params = registry.build_params('pwm_owner', {'topic': TOPIC, 'driver': 'thruster_pwm_node'})
    return KINDS['pwm_owner'].evaluate(params, facts)


def endpoint(node, own=False):
    return {'node': node, 'gid': node, 'self': own}


def test_one_managed_driver_is_ok():
    sub = endpoint('/thruster_pwm_node')
    assert owner(pwm_facts(['/thruster_pwm_node'], [sub])).status == 'ok'


def test_own_endpoint_is_skipped_but_other_drivers_count():
    subs = [endpoint('/thruster_pwm_node', own=True), endpoint('/thruster_pwm_node')]
    assert owner(pwm_facts(['/thruster_pwm_node'], subs)).status == 'ok'
    twice = [endpoint('/thruster_pwm_node'), endpoint('/ns/thruster_pwm_node')]
    result = owner(pwm_facts(['/thruster_pwm_node'], twice))
    assert result.status == 'fail' and 'DUPLICATE' in result.detail


def test_unmanaged_driver_while_armed_fails():
    result = owner(pwm_facts(['/thruster_pwm_node'], [endpoint('/thruster_pwm_node')], managed=False))
    assert result.status == 'fail' and 'UNMANAGED' in result.detail


def test_missing_facts_are_unknown():
    assert owner({}).status == 'unknown'


def test_readback_maps_the_legacy_level():
    params = registry.build_params('pwm_readback', {})
    run = KINDS['pwm_readback'].evaluate
    assert run(params, {'pwm.readback': Fact('WARN', data={'detail': '4/4 ON; x'})}).status == 'warn'
    assert run(params, {'pwm.readback': Fact('N/A')}).status == 'unknown'
    assert run(params, {}).status == 'unknown'


def processes(rows, installed=None, **raw):
    params = registry.build_params('processes', {'match': 'thruster_pwm_node', **raw})
    facts = {'host.processes': fact(rows)}
    facts.update({f'install.{k}': fact(v) for k, v in (installed or {}).items()})
    return KINDS['processes'].evaluate(params, facts)


def row(pid, managed=True, start=100.0, cmd='python thruster_pwm_node'):
    return {'pid': pid, 'cmd': cmd, 'start': start, 'managed': managed}


def test_adopted_child_counts_as_managed_and_orphan_does_not():
    assert processes([row(1), row(9, cmd='bash')], managed=True).status == 'ok'
    orphan = processes([row(1, managed=False)], managed=True)
    assert orphan.status == 'warn' and orphan.data['orphans'] == [1]


def test_stale_process_started_before_the_install():
    result = processes([row(1, start=100.0)], installed={'mhseals_hardware': 200.0},
                       newer_than=('mhseals_hardware',))
    assert result.status == 'warn' and result.data['stale'] == [1]
    assert processes([row(1, start=300.0)], installed={'mhseals_hardware': 200.0},
                     newer_than=('mhseals_hardware',)).status == 'ok'


def test_more_than_max_is_a_duplicate():
    assert processes([row(1), row(2)]).status == 'warn'
    assert processes([row(1), row(2)], max=2).status == 'ok'
