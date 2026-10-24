"""FcuParams against a fake /mavros/param link: the guarded read and apply paths."""

import math

import pytest
import yaml

from mhseals_testing.ros.fcu import FIXED_MAG_CAL_YAW, META as SHIPPED, FcuParams, MavrosLink, load_meta, same_float32


class FakeLink:
    def __init__(self, params, armed=False, fail_on=None, drift=None):
        self.params, self.is_armed, self.fail_on, self.drift = dict(params), armed, fail_on, drift or {}
        self.sets, self.commands, self.listings = [], [], []
        self.clock, self.serial = lambda: 0., 0

    def names(self):
        listing = self.listings.pop(0) if self.listings else list(self.params)
        return listing

    def get(self, names):
        return {n: self.params[n] for n in names if n in self.params}

    def safety_state(self):
        return dict(armed=self.is_armed, connected=True, received=self.clock(),
                    target='boat', connection=1, boot=1)

    def observe_fresh(self, names, after, identity, request):
        self.serial += 1
        return dict(values=self.get(names), request=request, identity=identity, source='fcu',
                    observed=self.clock(), received=self.clock(), observation_id=self.serial)

    def set(self, name, value):
        self.sets.append((name, value))
        if name == self.fail_on and value != self.params[name] and self.fail_on:
            self.fail_on = None     # fail once; the rollback set then works
            return False
        self.params[name] = self.drift.get(name, value)
        return True

    def armed(self):
        return self.is_armed

    def command(self, command, params):
        self.commands.append((command, params))
        return True


class Clock:
    def __init__(self):
        self.t = 0.

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


META = {'count': 3, 'params': {'A': {'min': 0, 'max': 10}, 'B': {}, 'C': {'reboot': True}}}
PARAMS = {'A': 1, 'B': 2.5, 'C': 3}


def fcu(link, **options):
    clock = Clock()
    link.clock = clock
    return FcuParams(link, meta=META, clock=clock, sleep=clock.sleep, **options)


def test_partial_load_gives_none_then_stable_count_gives_values():
    link = FakeLink(PARAMS)
    link.listings = [['A'], ['A', 'B'], ['A', 'B', 'C']]   # still growing, then stable
    assert fcu(link, timeout=3.).read_all(minimum=3) is None
    assert fcu(link).read_all(minimum=3) == PARAMS


def test_count_below_the_last_known_count_is_not_accepted():
    assert fcu(FakeLink(PARAMS), timeout=8.).read_all(minimum=4) is None


def test_unreadable_names_make_read_all_none():
    link = FakeLink(PARAMS)
    link.get = lambda names: {'A': 1}
    assert fcu(link).read_all() is None


def test_read_never_sets_or_commands():
    link = FakeLink(PARAMS)
    reader = fcu(link)
    reader.read(['A', 'zzz'])
    reader.read_all()
    assert link.sets == [] and link.commands == []


def test_bad_name_is_refused_before_any_set():
    link = FakeLink(PARAMS)
    result = fcu(link).apply({'A': 2, 'NOPE': 1})
    assert result['status'] == 'refused' and 'NOPE' in result['detail']
    assert link.sets == []


def test_out_of_range_and_non_finite_values_are_refused():
    link = FakeLink(PARAMS)
    assert fcu(link).apply({'A': 11})['status'] == 'refused'
    assert fcu(link).apply({'B': math.nan})['status'] == 'refused'
    assert fcu(link).apply({})['status'] == 'refused'
    assert link.sets == []


def test_armed_or_unknown_state_is_refused():
    for state in (True, None):
        link = FakeLink(PARAMS, armed=state)
        assert fcu(link).apply({'A': 2})['status'] == 'refused'
        assert link.sets == []


def test_apply_sets_in_order_and_reads_back():
    link = FakeLink(PARAMS)
    result = fcu(link).apply({'A': 2, 'B': 0.1})
    assert result['status'] == 'applied' and result['written'] == ['A', 'B']
    assert link.sets == [('A', 2), ('B', 0.1)]


def test_reboot_required_parameter_is_pending():
    assert fcu(FakeLink(PARAMS)).apply({'C': 4})['status'] == 'pending'


def test_readback_compares_at_float32():
    assert same_float32(0.1, 0.10000000149011612) and not same_float32(0.1, 0.1001)
    link = FakeLink(PARAMS, drift={'B': 0.10000000149011612})
    assert fcu(link).apply({'B': 0.1})['status'] == 'applied'


def test_mid_batch_failure_rolls_back_in_reverse():
    link = FakeLink(PARAMS, fail_on='C')
    result = fcu(link).apply({'A': 2, 'B': 3.5, 'C': 4})
    assert result['status'] == 'rolled_back'
    assert link.sets == [('A', 2), ('B', 3.5), ('C', 4), ('C', 3), ('B', 2.5), ('A', 1)]
    assert link.params == PARAMS


class RaisingLink(FakeLink):
    """Raises on the n-th set (1-based), as MavrosLink does on a client timeout; after that on every set if ``always``."""

    def __init__(self, params, raise_on=2, always=False):
        super().__init__(params)
        self.raise_on, self.always, self.raised = raise_on, always, False

    def set(self, name, value):
        if len(self.sets) + 1 == self.raise_on or (self.always and self.raised):
            self.sets.append((name, value))
            self.raised = True
            raise TimeoutError('set timed out')
        return super().set(name, value)


def test_set_that_raises_counts_as_unacknowledged_and_rolls_back_in_reverse():
    link = RaisingLink(PARAMS, raise_on=2)
    result = fcu(link).apply({'A': 2, 'B': 3.5, 'C': 4})
    assert result['status'] == 'rolled_back'
    assert link.sets == [('A', 2), ('B', 3.5), ('B', 2.5), ('A', 1)]
    assert link.params == PARAMS


def test_rollback_that_raises_is_unknown():
    link = RaisingLink(PARAMS, raise_on=2, always=True)
    assert fcu(link).apply({'A': 2, 'B': 3.5})['status'] == 'unknown'


def test_raising_readback_rolls_back_and_raising_before_read_refuses():
    link = FakeLink(PARAMS)
    real, calls = link.get, []

    def get(names):
        calls.append(names)
        if len(calls) == 2:
            raise TimeoutError('get timed out')
        return real(names)
    link.get = get
    assert fcu(link).apply({'A': 2})['status'] == 'recovery_required'
    assert link.params['A'] == 2
    broken = FakeLink(PARAMS)
    broken.get = lambda names: (_ for _ in ()).throw(TimeoutError('x'))
    assert fcu(broken).apply({'A': 2})['status'] == 'refused' and broken.sets == []


def test_command_is_done_only_when_the_ack_is_accepted():
    link = object.__new__(MavrosLink)
    request = type('Request', (), {})
    link.types = (None, None, None, type('T', (), {'Request': request}))
    link.commanding = None
    for success, result, expected in ((True, 0, True), (True, 2, False), (False, 0, False)):
        response = type('Response', (), {'success': success, 'result': result})()
        link._call = lambda client, req, what, r=response: r
        assert link.command(FIXED_MAG_CAL_YAW, [1, 0, 0, 0, 0, 0, 0]) is expected


def test_readback_mismatch_rolls_back_when_the_old_values_return():
    link = FakeLink(PARAMS, drift={'B': 9.})
    link.set('B', 2.5)
    link.drift = {}
    link.get = lambda names, real=link.get: {n: (9. if n == 'B' and link.sets[-1][1] != 2.5 else v) for n, v in real(names).items()}
    assert fcu(link).apply({'B': 3.5})['status'] == 'rolled_back'


def test_readback_mismatch_that_cannot_be_undone_is_unknown():
    link = FakeLink(PARAMS, drift={'A': 9})
    assert fcu(link).apply({'A': 2})['status'] == 'unknown'     # the FCU keeps 9 after the rollback set too


def test_mag_cal_needs_confirmation_and_disarmed():
    link = FakeLink(PARAMS)
    assert not fcu(link).fixed_mag_cal(90.)
    assert not fcu(FakeLink(PARAMS, armed=True)).fixed_mag_cal(90., confirmed=True)
    assert not fcu(link).fixed_mag_cal(400., confirmed=True)
    assert link.commands == []
    assert fcu(link).fixed_mag_cal(90., confirmed=True)
    assert link.commands == [(FIXED_MAG_CAL_YAW, [90., 1., 0, 0, 0, 0, 0])]


def test_shipped_meta_lists_the_blastoise_parameters():
    meta = load_meta()
    assert meta['count'] == len(meta['params']) > 900 and 'EK3_SRC1_YAW' in meta['params']
    assert 'COMPASS_OFS_X' in meta['params']


def test_shipped_classes_name_the_calibration_and_ignored_parameters():
    classes = yaml.safe_load((SHIPPED.parent / 'fcu.yaml').read_text())
    assert 'COMPASS_OFS*' in classes['calibration'] and 'STAT_*' in classes['ignore']


class RemoteLink(FakeLink):
    """Trusted test adapter: remote state and the display cache are independent."""
    def __init__(self, error=False):
        super().__init__(PARAMS)
        self.remote, self.error = dict(PARAMS), error
        self.bad_receipt = None

    def set(self, name, value):
        self.sets.append((name, value))
        self.remote[name] = value
        if len(self.sets) == 1:
            if self.error:
                raise TimeoutError('lost ACK after remote effect')
            return False
        return True

    def observe_fresh(self, names, after, identity, request):
        receipt = super().observe_fresh(names, after, identity, request)
        receipt['values'] = {n: self.remote[n] for n in names}
        if self.bad_receipt and self.sets:
            self.bad_receipt(receipt)
        return receipt


def test_false_ack_and_exception_after_effect_restore_remote_not_cache():
    for error in (False, True):
        link = RemoteLink(error)
        result = fcu(link).apply({'A': 2})
        assert result['status'] == 'rolled_back' and result['written'] == ['A']
        assert link.remote == PARAMS and link.params == PARAMS
        assert link.sets == [('A', 2), ('A', 1)]


def test_cache_equality_and_stale_or_wrong_epoch_receipts_do_not_certify():
    for corrupt in (lambda r: r.update(observed=-1),
                    lambda r: r.update(identity=('boat', 2, 1)),
                    lambda r: r.update(observation_id=1),
                    lambda r: r.update(source='cache'),
                    lambda r: r.update(values={})):
        link = RemoteLink()
        link.bad_receipt = corrupt
        result = fcu(link).apply({'A': 2})
        assert result['status'] == 'unknown' and 'rollback_unverified' in result['detail']
        assert link.remote['A'] == 2 and link.params['A'] == 1
        assert link.sets == [('A', 2)]


def test_legacy_link_refuses_before_effects():
    link = FakeLink(PARAMS)
    link.observe_fresh = None
    assert fcu(link).apply({'A': 2})['status'] == 'refused'
    assert link.sets == []


def test_safety_changes_stop_forward_and_compensation():
    for change in (dict(armed=True), dict(boot=2), dict(connection=2),
                   dict(target='other'), dict(received=-10), dict(connected=False)):
        link = RemoteLink()
        original = link.safety_state
        link.safety_state = lambda: dict(original(), **(change if link.sets else {}))
        result = fcu(link).apply({'A': 2, 'B': 4})
        assert result['status'] == 'unknown' and result['written'] == ['A']
        assert link.sets == [('A', 2)] and link.remote['A'] == 2
    link = FakeLink(PARAMS)
    link.safety_state = lambda: None
    assert fcu(link).apply({'A': 2})['status'] == 'refused' and link.sets == []


def test_successful_first_write_then_safety_loss_stops_remaining_forward_sets():
    for field, value in (('armed', True), ('boot', 2), ('connection', 2), ('received', -10)):
        link = FakeLink(PARAMS)
        original = link.safety_state
        link.safety_state = lambda: dict(original(), **({field: value} if link.sets else {}))
        result = fcu(link).apply({'A': 2, 'B': 4})
        assert result['status'] == 'recovery_required' and result['written'] == ['A']
        assert link.sets == [('A', 2)]


def test_missing_safety_mid_batch_and_arm_flip_during_reverse_restoration():
    link = FakeLink(PARAMS, fail_on='B')
    original = link.safety_state
    link.safety_state = lambda: None if len(link.sets) >= 3 else original()
    result = fcu(link).apply({'A': 2, 'B': 4})
    assert result['status'] == 'unknown'
    assert result['written'] == ['A', 'B']
    assert link.sets == [('A', 2), ('B', 4), ('B', 2.5)]
    assert link.params['A'] == 2


def test_compensation_requires_its_own_new_receipt():
    link = RemoteLink()
    link.bad_receipt = lambda r: r.update(observation_id=2) if len(link.sets) > 1 else None
    result = fcu(link).apply({'A': 2})
    assert link.remote['A'] == 1
    assert result['status'] == 'unknown' and 'rollback_unverified' in result['detail']


INVALID_NUMBERS = [None, True, False, math.nan, math.inf, -math.inf, '', 10 ** 10000]


@pytest.mark.parametrize('bad', INVALID_NUMBERS, ids=lambda v: type(v).__name__)
@pytest.mark.parametrize('phase', ['predecessor', 'forward', 'uncertain'])
def test_malformed_remote_values_never_become_compensation(bad, phase):
    link = RemoteLink()
    observe = link.observe_fresh

    def malformed(names, after, identity, request):
        receipt = observe(names, after, identity, request)
        if phase == 'predecessor' or link.sets:
            receipt['values']['A'] = bad
        return receipt

    link.observe_fresh = malformed
    if phase == 'forward':
        write = link.set
        link.set = lambda name, value: (write(name, value), True)[1]
    result = fcu(link, settle=0).apply({'A': 2})
    assert result['status'] == {'predecessor': 'refused', 'forward': 'recovery_required',
                                'uncertain': 'unknown'}[phase]
    assert link.sets == ([] if phase == 'predecessor' else [('A', 2)])
    assert link.params['A'] == 1
    assert link.remote['A'] == (1 if phase == 'predecessor' else 2)


@pytest.mark.parametrize('bad', INVALID_NUMBERS, ids=lambda v: type(v).__name__)
@pytest.mark.parametrize('field', ['received', 'target', 'connection', 'boot'])
@pytest.mark.parametrize('after_effect', [False, True])
@pytest.mark.parametrize('acked', [False, True])
def test_malformed_safety_evidence_stops_remote_effects(bad, field, after_effect, acked):
    link = RemoteLink()
    if acked:
        write = link.set
        link.set = lambda name, value: (write(name, value), True)[1]
    safety = link.safety_state
    link.safety_state = lambda: dict(safety(), **(
        {field: bad} if not after_effect or link.sets else {}))
    result = fcu(link, settle=0).apply({'A': 2})
    expected = 'recovery_required' if acked else 'unknown'
    assert result['status'] == (expected if after_effect else 'refused')
    assert link.sets == ([('A', 2)] if after_effect else [])
    assert link.params['A'] == 1


@pytest.mark.parametrize('bad', INVALID_NUMBERS, ids=lambda v: type(v).__name__)
@pytest.mark.parametrize('field', ['observed', 'received', 'identity'])
@pytest.mark.parametrize('after_effect', [False, True])
@pytest.mark.parametrize('acked', [False, True])
def test_malformed_observation_evidence_stops_remote_effects(bad, field, after_effect, acked):
    link = RemoteLink()
    if acked:
        write = link.set
        link.set = lambda name, value: (write(name, value), True)[1]
    observe = link.observe_fresh

    def malformed(names, after, identity, request):
        receipt = observe(names, after, identity, request)
        if not after_effect or link.sets:
            receipt[field] = (identity[0], bad, identity[2]) if field == 'identity' else bad
        return receipt

    link.observe_fresh = malformed
    result = fcu(link, settle=0).apply({'A': 2})
    expected = 'recovery_required' if acked else 'unknown'
    assert result['status'] == (expected if after_effect else 'refused')
    assert link.sets == ([('A', 2)] if after_effect else [])
    assert link.params['A'] == 1


def test_oversized_requested_integer_refuses_without_raising():
    link = RemoteLink()
    assert fcu(link).apply({'B': 10 ** 10000})['status'] == 'refused'
    assert link.sets == []
