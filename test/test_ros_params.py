from types import SimpleNamespace as NS

from mhseals_testing.ros.params import NOT_SET, collect


def test_undeclared_parameters_are_left_out():
    values = [NS(type=3, v=1.5), NS(type=NOT_SET, v=None)]
    assert collect(['a', 'b'], values, lambda value: value.v) == {'a': 1.5}
