"""CFG-NAV-LIVE1: the live Nav2 apply sequence, with an in-memory world behind the ports."""

import copy
import io
import json
from types import SimpleNamespace

import pytest

from mhseals_testing.harness import nav2_live as live_module
from mhseals_testing.harness.nav2_live import Nav2Live, Ports
from mhseals_testing.ros.params import coerce

FOLLOW = 'controller_server.FollowPath.'
LIVE = {
    '/controller_server': {
        'controller_frequency': 10.0, 'FollowPath.vx_max': 0.4, 'FollowPath.vy_max': 0.15,
        'FollowPath.wz_max': 0.3, 'FollowPath.vx_min': -0.1, 'FollowPath.time_steps': 48,
        'FollowPath.model_dt': 0.1, 'FollowPath.prune_distance': 3.0, 'FollowPath.vx_std': 0.2,
        'FollowPath.vy_std': 0.1, 'FollowPath.wz_std': 0.2,
        'FollowPath.rotate_to_heading_angular_vel': 0.3,
        'FollowPath.motion_model': 'Omni', 'FollowPath.critics': ['CostCritic'],
        'FollowPath.batch_size': 1000,
        'FollowPath.CostCritic.cost_weight': 3.81, 'general_goal_checker.xy_goal_tolerance': 0.4},
    '/velocity_smoother': {
        'max_velocity': [0.4, 0.15, 0.3], 'min_velocity': [-0.1, -0.15, -0.3],
        'max_accel': [0.3, 0.3, 0.5], 'max_decel': [-0.3, -0.3, -0.5]},
    '/planner_server': {'GridBased.tolerance': 0.5, 'expected_planner_frequency': 20.0},
}
QUIET = {'owner': 'none', 'input': 'auto', 'procedure': None, 'task': None, 'claims': [],
         'conflict': False}


class Rosout:
    def __init__(self, world):
        self.world, self.closed = world, False

    def errors(self, nodes, params):
        return list(self.world.logs)

    def close(self):
        self.closed = True


class World(Ports):
    """Nodes, a mux and an authority record that answer like the real ones, and a call log."""

    def __init__(self):
        self.nodes = copy.deepcopy(LIVE)
        self.owner_record = {'pid': 4242}
        self.record = dict(QUIET)
        self.goal_list = []
        self.mux_state = None
        self.expected = False
        self.calls, self.events = [], []
        self.reject, self.timeout, self.drift, self.logs = {}, None, {}, []
        self.silent_mux = False
        self.refuse_lifecycle, self.settle = set(), {}   # refuse: 1-based call numbers; settle: command -> {(node, name): value}
        self.flip = None            # (reads, change): the authority record changes after them
        self.reads = 0
        self.rosouts = []
        self.journal_error = None

    def owner(self):
        return self.owner_record

    def authority(self):
        self.reads += 1
        if self.flip and self.reads > self.flip[0]:
            self.record.update(self.flip[1])
        return self.record

    def goals(self):
        return self.goal_list

    def mux(self):
        return self.mux_state

    def mux_expected(self):
        return self.expected

    def mux_request(self, request):
        self.calls.append(('mux', request))
        if self.silent_mux:
            return
        holds = self.mux_state['holds']
        if request['op'] == 'hold':
            holds.append({'cause': request['cause'], 'by': request['by'], 'has_token': True,
                          'token': request['token']})
        else:
            holds[:] = [h for h in holds if h.get('token') != request.get('token')]

    def read(self, node):
        if node not in self.nodes:
            raise TimeoutError(f'{node} is not available')
        return dict(self.nodes[node]), set(self.nodes[node])

    def get(self, node, names):
        self.calls.append(('get', node, list(names)))
        if self.timeout == ('get', node):
            raise TimeoutError('get timed out')
        return {n: self.drift.get((node, n), self.nodes[node][n]) for n in names
                if n in self.nodes[node]}

    def put(self, node, values):
        self.calls.append(('set', node, copy.deepcopy(values)))
        if self.timeout == ('set', node):
            raise TimeoutError('set timed out')
        if node in self.reject:
            return False, self.reject[node]
        self.nodes[node].update(copy.deepcopy(values))
        return True, ''

    def lifecycle(self, command):
        self.calls.append(('lifecycle', command))
        if self.timeout == ('lifecycle', command):
            raise TimeoutError(f'{command} timed out')
        number = sum(c[0] == 'lifecycle' for c in self.calls)
        for (node, name), value in self.settle.get(command, {}).items():
            self.nodes[node][name] = value
        return number not in self.refuse_lifecycle

    def rosout_open(self):
        self.rosouts.append(Rosout(self))
        return self.rosouts[-1]

    def journal(self, phase, **fields):
        if self.journal_error:
            raise OSError(self.journal_error)
        self.events.append((phase, fields, len(self.calls)))

    def sets(self):
        return [(c[1], c[2]) for c in self.calls if c[0] == 'set']


@pytest.fixture
def world():
    return World()


def make(world, recorded=None, **kwargs):
    ticks = iter(x * 0.5 for x in range(10 ** 6))
    record = (lambda: recorded.append(1)) if recorded is not None else None
    return Nav2Live(world, me=kwargs.pop('me', 4242), record=record,
                    sleep=lambda s: None, clock=lambda: next(ticks), **kwargs)


SMOOTHER, CONTROLLER = '/velocity_smoother', '/controller_server'


def chooser(asked):
    """Return an ``App.choose`` that records the popup and picks the first option."""
    def choose(title, options, shortcuts=None, **kw):
        asked.append((options, kw['rows']))
        return options[0]
    return choose


def ask(**keys):
    return {FOLLOW + name: value for name, value in keys.items()}


# ---- success

def test_raising_a_limit_sets_the_controller_then_the_smoother_and_reads_both_back(world):
    recorded = []
    receipt = make(world, recorded).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'verified' and receipt['live']
    assert [(n, sorted(v)) for n, v in world.sets()] == [
        ('/controller_server', ['FollowPath.vx_max']), ('/velocity_smoother', ['max_velocity'])]
    assert world.nodes['/velocity_smoother']['max_velocity'] == [0.5, 0.15, 0.3]
    assert [c[0] for c in world.calls] == ['set', 'get', 'set', 'get']
    assert receipt['verified'] == ask(vx_max=0.5) and recorded == [1]
    derived = [r['key'] for r in receipt['rows'] if r['derived']]
    assert derived == ['velocity_smoother.max_velocity']


def test_lowering_a_limit_sends_the_smoother_first_at_the_restrictive_value(world):
    make(world).apply(ask(vx_max=0.15))
    assert [n for n, _ in world.sets()] == ['/velocity_smoother', '/controller_server']
    assert world.sets()[0][1] == {'max_velocity': [0.15, 0.15, 0.3]}
    assert world.sets()[1][1]['FollowPath.vx_std'] == 0.15          # derived with the limit


def test_mixed_change_sends_the_restrictive_smoother_value_then_the_rest_last(world):
    world.nodes['/velocity_smoother']['max_velocity'] = [0.4, 0.15, 0.3]
    make(world).apply(ask(vx_max=0.5, wz_max=0.2))
    nodes = [n for n, _ in world.sets()]
    assert nodes == ['/velocity_smoother', '/controller_server', '/velocity_smoother']
    assert world.sets()[0][1] == {'max_velocity': [0.4, 0.15, 0.2],
                                  'min_velocity': [-0.1, -0.15, -0.2]}
    assert world.sets()[2][1] == {'max_velocity': [0.5, 0.15, 0.2]}


def test_one_atomic_set_per_node(world):
    make(world).apply({**ask(vx_max=0.5, wz_max=0.4), 'planner_server.GridBased.tolerance': 0.6})
    nodes = [n for n, _ in world.sets()]
    assert sorted(set(nodes)) == ['/controller_server', '/planner_server', '/velocity_smoother']
    assert nodes.count('/controller_server') == 1 and nodes.count('/planner_server') == 1


def test_the_ros_parameters_level_of_the_file_is_not_part_of_the_name(world):
    receipt = make(world).apply({'controller_server.ros__parameters.FollowPath.vx_max': 0.5})
    assert receipt['status'] == 'verified'
    assert receipt['verified'] == {'controller_server.ros__parameters.FollowPath.vx_max': 0.5}


def test_values_already_live_need_no_write_and_no_hold(world):
    world.mux_state = {'holds': []}
    receipt = make(world).apply(ask(vx_max=0.4))
    assert receipt['status'] == 'verified' and world.calls == [] and not world.events


def test_integer_for_a_double_is_accepted_after_coercion():
    assert coerce(1, 'DOUBLE') == 1.0 and isinstance(coerce(1, 'DOUBLE'), float)
    assert coerce([1, 2], 'DOUBLE_ARRAY') == [1.0, 2.0]
    assert coerce(3.0, 'INTEGER') == 3 and isinstance(coerce(3.0, 'INTEGER'), int)
    for value, kind in ((True, 'DOUBLE'), (2.5, 'INTEGER'), ('1', 'DOUBLE'), (1, 'BOOL'),
                        (1, 'STRING'), (1, 'DOUBLE_ARRAY')):
        with pytest.raises(ValueError):
            coerce(value, kind)


# ---- readback, rejection, rosout, timeout

def test_a_rejected_key_rolls_back_the_earlier_nodes_in_reverse_with_readback(world):
    world.reject['/controller_server'] = 'FollowPath.vx_max: out of range'
    receipt = make(world).apply(ask(vx_max=0.15))
    assert receipt['status'] == 'rolled_back' and 'rejected the set' in receipt['error']
    assert world.nodes == LIVE
    sent = world.sets()
    assert [n for n, _ in sent] == [SMOOTHER, CONTROLLER, SMOOTHER]
    assert sent[2][1] == {'max_velocity': [0.4, 0.15, 0.3]}            # the captured prior value
    assert world.calls[-1][0] == 'get'                                # the rollback was read back


def test_rollback_runs_in_reverse_order_over_every_step_that_went_through(world):
    world.logs.append('controller_server: Invalid value')
    receipt = make(world).apply(ask(vx_max=0.5, wz_max=0.2))
    assert receipt['status'] == 'rolled_back'
    assert [n for n, _ in world.sets()] == [SMOOTHER, CONTROLLER, SMOOTHER,
                                            SMOOTHER, CONTROLLER, SMOOTHER]
    assert world.nodes == LIVE


def test_readback_mismatch_rolls_back(world):
    world.drift[('/controller_server', 'FollowPath.vx_max')] = 0.4
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'rolled_back' and 'readback differs' in receipt['error']
    assert world.nodes == LIVE


def test_a_bad_rosout_line_rolls_back(world):
    world.logs.append('controller_server: Parameter FollowPath.vx_max not found')
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'rolled_back' and '/rosout' in receipt['error']
    assert world.nodes == LIVE and all(r.closed for r in world.rosouts)


def test_failed_rollback_is_reported(world):
    world.logs.append('x: Invalid value')
    original, restores = world.put, []

    def deny_the_last_restore(node, values):
        if node == '/velocity_smoother' and restores:
            return False, 'denied'
        restores.append(node) if node == '/velocity_smoother' else None
        return original(node, values)
    world.put = deny_the_last_restore
    receipt = make(world).apply(ask(vx_max=0.15))
    assert receipt['status'] == 'rollback_failed'
    assert 'could not restore velocity_smoother' in receipt['error']


def test_timeout_is_unknown_with_no_retry_and_no_rollback(world):
    world.timeout = ('set', '/velocity_smoother')
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'unknown' and 'nothing was retried' in receipt['error']
    assert [n for n, _ in world.sets()] == [CONTROLLER, SMOOTHER]          # one try each
    receipt = make(world).apply(ask(vx_max=0.45))
    assert receipt['status'] == 'unknown'


def test_readback_timeout_is_unknown(world):
    world.timeout = ('get', '/controller_server')
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'unknown'
    assert [n for n, _ in world.sets()] == ['/controller_server']


def test_a_value_that_cannot_be_coerced_rolls_back_what_went_through(world):
    original = world.put

    def typed(node, values):
        if node == '/velocity_smoother':
            raise ValueError('bad type')
        return original(node, values)
    world.put = typed
    receipt = make(world).apply(ask(vx_max=0.15))
    assert receipt['status'] == 'rolled_back' and world.nodes == LIVE


# ---- refusals

@pytest.mark.parametrize('change,text', [
    ({'procedure': {'name': 'drive_to'}}, 'procedure running'),
    ({'claims': [{'id': 'c1', 'who': 'agent', 'state': 'active'}]}, 'claim held'),
    ({'task': {'status': 'RUNNING'}}, 'task'),
    ({'input': 'rc', 'owner': 'rc'}, 'someone is driving'),
    ({'input': 'manual', 'owner': 'keyboard'}, 'someone is driving'),
    ({'owner': 'unknown'}, 'unknown'),
    ({'owner': 'external'}, 'something else is driving'),
    ({'conflict': True}, 'conflict'),
])
def test_refuses_while_someone_else_has_the_boat(world, change, text):
    world.record.update(change)
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'refused' and text in receipt['refused']
    assert world.calls == [] and not world.events


def test_refuses_without_an_owner_or_for_another_owner_or_stale_authority(world):
    world.owner_record = None
    assert 'no live session owner' in make(world).apply(ask(vx_max=0.5))['refused']
    world.owner_record = {'pid': 1}
    assert 'does not own the session' in make(world).apply(ask(vx_max=0.5))['refused']
    world.owner_record = {'pid': 4242}
    world.record = None
    assert 'authority state unknown' in make(world).apply(ask(vx_max=0.5))['refused']
    assert world.calls == []


def test_a_guest_needs_only_a_live_owner(world):
    assert make(world, me=None).apply(ask(vx_max=0.5))['status'] == 'verified'


def test_an_active_goal_refuses_and_now_allows_only_limits(world):
    world.goal_list = ['navigate_to_pose:ab12']
    assert 'a goal is active' in make(world).apply(ask(vx_max=0.5))['refused']
    assert world.calls == []
    world.mux_state = {'holds': []}
    live = make(world)
    assert live.apply(ask(vx_max=0.5), now=True)['status'] == 'verified'
    world.calls.clear()
    other = {FOLLOW + 'CostCritic.cost_weight': 5.0}
    assert 'a goal is active' in make(world).apply(other, now=True)['refused']
    assert world.calls == []


def test_now_with_a_goal_and_no_mux_to_hold_refuses_and_sets_nothing(world):
    world.goal_list = ['navigate_to_pose:abcd']
    world.mux_state, world.expected = None, False
    receipt = make(world).apply(ask(vx_max=0.35), now=True)
    assert receipt['status'] == 'refused' and 'no mux to hold' in receipt['refused']
    assert not any(c[0] in ('set', 'mux') for c in world.calls)
    world.goal_list = []
    assert make(world).apply(ask(vx_max=0.35), now=True)['status'] == 'verified'


def test_unknown_goal_state_refuses(world):
    world.goal_list = None
    assert 'cannot tell' in make(world).apply(ask(vx_max=0.5))['refused']


def test_a_key_the_node_does_not_declare_refuses_the_whole_request(world):
    receipt = make(world).apply({FOLLOW + 'Ghost.weight': 1.0, FOLLOW + 'vx_max': 0.5})
    assert receipt['status'] == 'refused' and 'not declared' in receipt['refused']
    assert FOLLOW + 'Ghost.weight' in receipt['refused'] and world.calls == []


def reset_calls(world):
    return [c[1] if c[0] == 'lifecycle' else c[0] for c in world.calls
            if c[0] in ('lifecycle', 'set')]


def test_a_non_live_key_is_set_then_navigation_resets_starts_and_reads_back(world):
    world.mux_state = {'holds': []}
    receipt = make(world).apply({FOLLOW + 'batch_size': 500, FOLLOW + 'CostCritic.cost_weight': 4.0})
    assert receipt['status'] == 'verified' and receipt['reset'], receipt
    assert world.nodes[CONTROLLER]['FollowPath.batch_size'] == 500
    kinds = [c[1] if c[0] == 'lifecycle' else c[0] for c in world.calls]
    # hold, set, readback, reset, startup, readback again, release: the hold covers the whole reset
    assert kinds == ['mux', 'set', 'get', 'reset', 'startup', 'get', 'mux']
    assert receipt['hold']['released'] and not world.mux_state['holds']


def test_a_live_only_request_never_resets(world):
    receipt = make(world).apply(ask(vx_max=0.35))
    assert receipt['status'] == 'verified' and not receipt['reset']
    assert 'lifecycle' not in {c[0] for c in world.calls}


def test_a_reset_needs_no_goal_even_with_now(world):
    world.goal_list = ['navigate_to_pose:abcd']
    receipt = make(world).apply({FOLLOW + 'batch_size': 500, FOLLOW + 'vx_max': 0.35}, now=True)
    assert receipt['status'] == 'refused' and 'a goal is active' in receipt['refused']
    assert world.calls == []


def test_a_reset_that_fails_restores_the_values_and_resets_again(world):
    world.refuse_lifecycle = {2}                       # the first startup
    before = copy.deepcopy(world.nodes)
    receipt = make(world).apply({FOLLOW + 'batch_size': 500})
    assert receipt['status'] == 'rolled_back' and 'startup failed' in receipt['error'], receipt
    assert reset_calls(world) == ['set', 'reset', 'startup', 'set', 'reset', 'startup']
    assert world.nodes == before


def test_a_value_that_differs_after_startup_rolls_back(world):
    world.settle = {'startup': {(CONTROLLER, 'FollowPath.batch_size'): 1000}}
    receipt = make(world).apply({FOLLOW + 'batch_size': 500})
    assert receipt['status'] == 'rolled_back' and 'FollowPath.batch_size' in receipt['error'], receipt


def test_a_rollback_whose_reset_fails_is_reported(world):
    world.refuse_lifecycle = {2, 4}
    receipt = make(world).apply({FOLLOW + 'batch_size': 500})
    assert receipt['status'] == 'rollback_failed' and 'Navigation' in receipt['error'], receipt


def test_a_reset_timeout_is_unknown_with_no_retry(world):
    world.timeout = ('lifecycle', 'reset')
    receipt = make(world).apply({FOLLOW + 'batch_size': 500})
    assert receipt['status'] == 'unknown' and 'nothing was retried' in receipt['error'], receipt
    assert reset_calls(world) == ['set', 'reset']


@pytest.mark.parametrize('value', [
    '[[0.1, 0.1], [0.1', '[[0.1, 0.1], [0.1, -0.1]]', '[[0.1, 0.1], [0.1, -0.1], [1]]',
    '[[0.1, 0.1], [0.1, -0.1], [NaN, 0]]', 'square', 0.3, None])
def test_a_malformed_footprint_refuses_before_any_write(world, value):
    world.nodes['/global_costmap/global_costmap'] = {'footprint': '[[1, 1], [1, -1], [-1, 0]]'}
    receipt = make(world).apply({'global_costmap.global_costmap.footprint': value})
    assert receipt['status'] == 'refused' and 'footprint must be' in receipt['refused']
    assert world.calls == []


def test_a_good_footprint_is_set_live(world):
    world.nodes['/global_costmap/global_costmap'] = {'footprint': '[[1, 1], [1, -1], [-1, 0]]'}
    receipt = make(world).apply({'global_costmap.footprint': '[[0.5, 0.5], [0.5, -0.5], [-0.5, 0.0]]'})
    assert receipt['status'] == 'verified' and not receipt['reset'], receipt


def test_unreachable_node_refuses(world):
    del world.nodes['/planner_server']
    receipt = make(world).apply({'planner_server.GridBased.tolerance': 0.6})
    assert receipt['status'] == 'refused' and 'did not answer' in receipt['refused']


def test_a_conflicting_derived_value_refuses(world):
    receipt = make(world).apply(ask(vx_max=0.5, model_dt=0.9))
    assert receipt['status'] == 'refused'


def test_changed_live_values_since_the_preview_refuse(world):
    live = make(world)
    previewed = live.preview(ask(vx_max=0.5))
    world.nodes['/controller_server']['FollowPath.vx_max'] = 0.45
    receipt = live.apply(ask(vx_max=0.5), previewed)
    assert receipt['status'] == 'refused' and 'changed since the preview' in receipt['refused']
    assert world.calls == []


def test_a_new_owner_since_the_preview_refuses(world):
    live = make(world, me=None)
    previewed = live.preview(ask(vx_max=0.5))
    world.owner_record = {'pid': 7}
    assert 'owner changed' in live.apply(ask(vx_max=0.5), previewed)['refused']


def test_the_ownership_check_is_repeated_right_before_the_first_write(world):
    world.mux_state = {'holds': []}
    # The preview's read is clear; the recheck after the hold is not.
    world.flip = (1, {'procedure': {'name': 'drive_to'}})
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'refused' and 'procedure running' in receipt['refused']
    assert world.sets() == []
    assert [c[1]['op'] for c in world.calls if c[0] == 'mux'] == ['hold', 'release']   # let go
    assert world.mux_state['holds'] == []


def test_a_guest_refuses_when_the_owner_changes_during_the_hold(world):
    world.mux_state = {'holds': []}
    original = world.mux_request

    def swapping(request):
        original(request)
        if request['op'] == 'hold':
            world.owner_record = {'pid': 7}
    world.mux_request = swapping
    receipt = make(world, me=None).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'refused' and 'owner changed' in receipt['refused']
    assert world.sets() == []
    assert [c[1]['op'] for c in world.calls if c[0] == 'mux'] == ['hold', 'release']
    assert world.mux_state['holds'] == []


def test_an_unexpected_exception_mid_apply_is_unknown_journaled_and_released(world, tmp_path):
    world.mux_state = {'holds': []}
    original = world.put

    def breaking(node, values):
        if node == SMOOTHER:
            raise RuntimeError('invalid handle')
        return original(node, values)
    world.put = breaking
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'unknown' and 'RuntimeError' in receipt['error']
    assert [n for n, _ in world.sets()] == [CONTROLLER]
    assert world.events[-1][0] == 'nav2_live_outcome' and world.events[-1][1]['status'] == 'unknown'
    assert world.mux_state['holds'] == [] and receipt['hold']['released']
    world.nodes = copy.deepcopy(LIVE)        # the CLI applies FILE from a fresh start
    code, _ = run_cli(world, tmp_path, '--yes', '--json')
    assert code == 4


# ---- the mux hold

def test_a_tokened_config_hold_is_confirmed_and_released_with_the_same_token(world):
    world.mux_state = {'holds': []}
    seen = []
    original = world.put

    def watching(node, values):
        seen.append([h['cause'] for h in world.mux_state['holds']])
        return original(node, values)
    world.put = watching
    receipt = make(world).apply(ask(vx_max=0.5))
    hold, release = [c[1] for c in world.calls if c[0] == 'mux']
    assert hold['op'] == 'hold' and hold['cause'] == 'config' and hold['token']
    assert release['op'] == 'release'
    assert release['token'] == hold['token'] and release['cause'] == 'config'
    assert seen == [['config'], ['config']]                  # held across every write
    assert world.mux_state['holds'] == [] and receipt['hold']['released']
    assert receipt['status'] == 'verified'


def test_the_hold_is_released_after_a_failure_too(world):
    world.mux_state = {'holds': []}
    world.reject['/controller_server'] = 'no'
    assert make(world).apply(ask(vx_max=0.5))['status'] == 'rolled_back'
    assert world.mux_state['holds'] == []


def test_an_unconfirmed_hold_refuses_and_writes_nothing(world):
    world.mux_state = {'holds': []}
    world.silent_mux = True
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'refused' and 'did not confirm' in receipt['refused']
    assert world.sets() == []


def test_an_existing_config_hold_refuses(world):
    world.mux_state = {'holds': [{'cause': 'config', 'by': 'x', 'has_token': True}]}
    assert 'already in place' in make(world).apply(ask(vx_max=0.5))['refused']
    assert world.calls == []


def test_armed_with_no_mux_state_refuses_and_unmuxed_needs_no_hold(world):
    world.expected = True
    assert 'mux state is unknown' in make(world).apply(ask(vx_max=0.5))['refused']
    world.expected = False
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'verified' and not any(c[0] == 'mux' for c in world.calls)


def test_an_unconfirmed_release_is_reported(world):
    world.mux_state = {'holds': []}
    original = world.mux_request

    def stuck(request):
        if request['op'] == 'release':
            world.calls.append(('mux', request))
            return
        original(request)
    world.mux_request = stuck
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'verified' and 'press U' in receipt['warning']


# ---- journal

def test_intent_is_journaled_before_the_first_write_and_the_outcome_after(world):
    make(world).apply(ask(vx_max=0.5))
    (intent, fields, at), (outcome, result, after) = world.events
    assert intent == 'nav2_live_intent' and at == 0 and fields['before'][FOLLOW + 'vx_max'] == 0.4
    assert fields['keys'][FOLLOW + 'vx_max'] == 0.5 and 'token' not in json.dumps(fields)
    assert outcome == 'nav2_live_outcome' and result['status'] == 'verified'
    assert after == len(world.calls)


def test_an_intent_that_cannot_be_journaled_refuses_before_any_write(world):
    world.journal_error = 'disk full'
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'refused' and 'journaled' in receipt['refused']
    assert world.calls == []


def test_a_failed_apply_records_no_history(world):
    recorded = []
    world.reject['/controller_server'] = 'no'
    make(world, recorded).apply(ask(vx_max=0.5))
    assert recorded == []


# ---- the CLI verb

FILE = 'controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.5\n'


def run_cli(world, tmp_path, *flags, text=FILE):
    from mhseals_testing.harness import cli
    from mhseals_testing import nav2_limits
    path = nav2_limits.path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    out = io.StringIO()
    args = cli.build_parser().parse_args(['config', 'apply', 'nav2', *flags])
    import contextlib
    with contextlib.redirect_stdout(out):
        ticks = iter(x * 0.5 for x in range(10 ** 6))
        code = cli.apply_nav2(args, ports=world, sleep=lambda s: None, clock=lambda: next(ticks))
    return code, out.getvalue()


def test_cli_without_yes_prints_the_preview_and_sends_nothing(world, tmp_path):
    code, out = run_cli(world, tmp_path, '--json')
    doc = json.loads(out)
    assert code == 2 and doc['outcome'] == 'preview'
    assert doc['changes'][0]['key'] == FOLLOW + 'vx_max'
    assert world.calls == []


def test_cli_yes_applies_and_records_applied_live_session(world, tmp_path):
    from mhseals_testing.history import History
    code, out = run_cli(world, tmp_path, '--yes', '--json')
    doc = json.loads(out)
    assert code == 0 and doc['outcome'] == 'applied' and doc['live'] == 'until Navigation restarts'
    change = History().head('nav2')
    assert change['applied'] == 'live-session' and change['source'] == 'cli:apply-nav2'


def test_cli_exit_codes_for_refusal_rollback_unknown_and_nothing(world, tmp_path):
    world.record['procedure'] = {'name': 'x'}
    assert run_cli(world, tmp_path, '--yes', '--json')[0] == 2
    world.record['procedure'] = None
    world.reject['/controller_server'] = 'no'
    assert run_cli(world, tmp_path, '--yes', '--json')[0] == 3
    world.reject.clear()
    world.timeout = ('set', '/controller_server')
    assert run_cli(world, tmp_path, '--yes', '--json')[0] == 4
    world.timeout = None
    world.nodes['/controller_server']['FollowPath.vx_max'] = 0.5
    world.nodes['/velocity_smoother']['max_velocity'][0] = 0.5
    assert run_cli(world, tmp_path, '--yes', '--json')[0] == 1


def test_cli_expects_the_mux_from_the_record_and_the_profile(tmp_path, monkeypatch):
    from mhseals_testing.harness import cli
    quiet = {'input': 'auto', 'rc': None, 'holds': []}
    registry = SimpleNamespace(state=lambda: {})
    assert cli.cli_mux_expected(registry, None)                      # no record: assume muxed
    assert cli.cli_mux_expected(registry, {**quiet, 'rc': {'a': 1}})
    assert cli.cli_mux_expected(registry, {**quiet, 'holds': [{'cause': 'keys'}]})
    assert not cli.cli_mux_expected(registry, {'input': None, 'rc': None, 'holds': []})
    for profile, muxed in (('muxed', True), ('plain', False)):
        header = tmp_path / profile
        header.mkdir()
        (header / 'session.json').write_text(json.dumps({'profile': profile}))
        registry = SimpleNamespace(state=lambda h=header: {'session': str(h)})
        loaded = SimpleNamespace(muxed=muxed)
        monkeypatch.setattr(cli, 'load_profile', lambda name, loaded=loaded: loaded)
        assert cli.cli_mux_expected(registry, quiet) is muxed
    assert cli.cli_mux_expected(SimpleNamespace(state=lambda: {}), quiet)   # profile unknown


def test_cli_now_flag_lets_a_limits_change_run_during_a_goal(world, tmp_path):
    world.goal_list = ['navigate_to_pose:ab12']
    world.mux_state = {'holds': []}
    assert run_cli(world, tmp_path, '--yes', '--json')[0] == 2
    assert run_cli(world, tmp_path, '--yes', '--now', '--json')[0] == 0


# ---- the TUI seam

def test_nav2_file_applies_through_the_live_path_and_reads_live_until_restart(world, tmp_path):
    from mhseals_testing.harness.nav2_edit import Nav2File
    path = tmp_path / 'nav2.yaml'
    path.write_text('controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.5\n')
    live = make(world)
    nav = Nav2File(path, live.applier)
    live.preview(nav.requested())
    assert nav.apply() == 'Nav2 applied 1 key(s) live'
    row = nav.row()
    assert row['value'] == 'live' and not nav.pending
    assert 'live until Navigation restarts' in row['detail']


def test_nav2_file_shows_refusals_and_failures(world, tmp_path):
    from mhseals_testing.harness.nav2_edit import Nav2File
    path = tmp_path / 'nav2.yaml'
    path.write_text('controller_server:\n  ros__parameters:\n    FollowPath:\n      vx_max: 0.5\n')
    nav = Nav2File(path, make(world).applier)
    world.goal_list = ['navigate_to_pose:ab12']
    assert nav.apply().startswith('Nav2 apply refused: a goal is active') and nav.pending
    world.goal_list = []
    world.reject['/controller_server'] = 'no'
    assert nav.apply().startswith('Nav2 apply rolled back:') and nav.pending
    world.reject.clear()
    world.timeout = ('set', '/velocity_smoother')
    assert nav.apply().startswith('Nav2 apply unknown:')


def test_the_popup_previews_from_the_live_nodes_and_applies_live(tmp_path):
    from test_app import nav2_app
    world = World()
    app, path = nav2_app(tmp_path)
    live = make(world)
    app.nav2_live_factory = lambda: live
    app.nav2_file.applier = app.apply_nav2_live
    asked = []
    app.choose = chooser(asked)
    app.nav2_popup()
    options, rows = asked[0]
    assert options == ['Apply live', 'Close']
    assert (FOLLOW + 'vx_max', '0.4 → 0.5') in rows or any(k == FOLLOW + 'vx_max' for k, _ in rows)
    assert app.notice == 'Nav2 applied 1 key(s) live'
    assert world.nodes['/controller_server']['FollowPath.vx_max'] == 0.5


def test_the_popup_offers_only_close_when_refused_and_a_watcher_never_applies(tmp_path):
    from test_app import nav2_app
    world = World()
    world.goal_list = ['navigate_to_pose:ab12']
    app, path = nav2_app(tmp_path)
    app.nav2_live_factory = lambda: make(world)
    app.nav2_file.applier = app.apply_nav2_live
    asked = []
    app.choose = chooser(asked)
    app.nav2_popup()
    assert asked[0][0] == ['Close'] and any(k == 'refused' for k, _ in asked[0][1])
    app.watching = 123
    assert 'only watches' in app.nav2_file.apply()
    assert world.calls == []


def test_live_module_exports_the_documented_limit_keys():
    assert FOLLOW + 'vx_max' in live_module.LIMIT_KEYS
    assert 'velocity_smoother.max_velocity' in live_module.LIMIT_KEYS
    assert FOLLOW + 'CostCritic.cost_weight' not in live_module.LIMIT_KEYS


# ---- /rosout filter

def test_rosout_keeps_failure_lines_about_what_was_set_and_skips_mppi_noise():
    from mhseals_testing.ros.nav2_live import Rosout
    lines = [(1.0, 'controller_server', 'Parameter failure_tolerance not found', 30),  # earlier
             # MPPI listing a key it does not own:
             (5.0, 'controller_server', 'Parameter failure_tolerance not found', 30),
             (5.1, 'controller_server', 'Parameter FollowPath.batch_size not found', 30),  # not set
             (5.2, 'velocity_smoother', 'Invalid size of max_velocity', 30),
             (5.3, 'other_node', 'something else is Invalid', 30),
             (5.4, 'bt_navigator', 'param max_velocity not found', 30),
             (5.5, 'local_costmap.local_costmap', 'all fine', 30),
             (5.6, 'global_costmap.global_costmap', 'Error parsing footprint parameter', 40)]
    errors = Rosout(lines, 2.0).errors({'controller_server', 'velocity_smoother'},
                                       ['failure_tolerance', 'max_velocity'])
    assert errors == ['controller_server: Parameter FollowPath.batch_size not found',
                      'velocity_smoother: Invalid size of max_velocity',
                      'bt_navigator: param max_velocity not found',
                      'global_costmap.global_costmap: Error parsing footprint parameter']


def test_a_missing_domain_package_refuses_instead_of_crashing(world, monkeypatch):
    from mhseals_testing.harness import nav2_params

    def missing(*args, **kwargs):
        raise ImportError('No module named mhseals_nav')
    monkeypatch.setattr(nav2_params, 'plan', missing)
    receipt = make(world).apply(ask(vx_max=0.5))
    assert receipt['status'] == 'refused' and 'cannot derive the request' in receipt['refused']
    assert world.calls == []


def test_a_malformed_profile_yaml_reads_as_unknown_mux(tmp_path, monkeypatch):
    from mhseals_testing.harness import cli
    (tmp_path / 'session.json').write_text('{"profile": "x"}')
    registry = SimpleNamespace(state=lambda: {'session': str(tmp_path)})
    monkeypatch.setattr(cli, 'load_profile', lambda name: (_ for _ in ()).throw(__import__('yaml').YAMLError('bad')))
    assert cli.session_muxed(registry) is None
