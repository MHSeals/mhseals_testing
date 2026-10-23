"""FcuParams against a fake /mavros/param link: the guarded read and apply paths."""

import math

import yaml

from mhseals_testing.ros.fcu import FIXED_MAG_CAL_YAW, META as SHIPPED, FcuParams, load_meta, same_float32


class FakeLink:
    def __init__(self, params, armed=False, fail_on=None, drift=None):
        self.params, self.is_armed, self.fail_on, self.drift = dict(params), armed, fail_on, drift or {}
        self.sets, self.commands, self.listings = [], [], []

    def names(self):
        listing = self.listings.pop(0) if self.listings else list(self.params)
        return listing

    def get(self, names):
        return {n: self.params[n] for n in names if n in self.params}

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
    assert link.sets == [('A', 2), ('B', 3.5), ('C', 4), ('B', 2.5), ('A', 1)]
    assert link.params == PARAMS


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
