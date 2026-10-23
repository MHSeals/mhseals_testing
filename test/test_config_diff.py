from mhseals_testing.history.render import diff

PINS = {1: (7, 'GPIO0_D0', 'PWM7', 'x'), 2: (12, 'a', 'b', 'c'), 3: (15, 'd', 'e', 'f'), 4: (33, 'g', 'h', 'i')}
IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
BASE = {'channel_map': [1, 2, 3, 4], 'reversed': [False] * 4, 'command_timeout': 0.5, 'decoupling': IDENTITY,
        'output_backend': 'odroid', 'decoupling_negative': None}


def hw(**changes):
    return {'overrides': {}, 'effective': {**BASE, **changes}}


def test_per_thruster_rows_carry_the_pin():
    rows = diff('hardware', hw(), hw(channel_map=[3, 2, 3, 4], reversed=[False, False, True, False]), PINS)
    assert [(r.group, r.label, r.before, r.after, r.note) for r in rows] == [
        ('Thrusters', 'FL output', 1, 3, '(pin 7 → 15)'), ('Thrusters', 'RR reversed', False, True, '')]


def test_scalar_units_and_group_order():
    rows = diff('hardware', hw(), hw(command_timeout=0.3, channel_map=[2, 1, 3, 4]), PINS)
    assert [r.group for r in rows] == ['Thrusters', 'Thrusters', 'Timing']
    assert (rows[-1].label, rows[-1].unit, rows[-1].before, rows[-1].after) == ('Command timeout', 's', 0.5, 0.3)


def test_matrix_row_reshapes_and_unknown_key_lands_in_other():
    new = hw(decoupling=[1, 0, 0, 0, 1, 0, -0.0375, 0.1, 1], mystery=3)
    rows = diff('hardware', hw(), new)
    matrix = next(r for r in rows if r.kind == 'matrix')
    assert matrix.label == 'Correction C⁺' and matrix.grid['after'][2] == [-0.0375, 0.1, 1]
    assert matrix.grid['rows'] == ['surge', 'sway', 'yaw']
    other = next(r for r in rows if r.group == 'Other')
    assert (other.label, other.kind, other.after) == ('mystery', 'added', 3)


def test_nav2_nests_into_group_and_label():
    old = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_max': 0.4}}}}
    new = {'controller_server': {'ros__parameters': {'FollowPath': {'vx_max': 0.38}}}}
    [row] = diff('nav2', old, new)
    assert (row.group, row.label, row.unit) == ('Controller', 'FollowPath · vx_max', 'm/s')


def test_equal_versions_have_no_rows():
    assert diff('hardware', hw(), hw()) == []
