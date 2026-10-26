"""fusion_campaign id metric: one-way switches over all matches, alternation apart."""
from mhseals_testing.perception.fusion_campaign import id_changes
import pytest


@pytest.mark.parametrize('seq, expected', [
    (['A'], (0, 0)),
    (['A', 'B'], (1, 0)),                       # lost and re-acquired
    (['A', 'B', 'A', 'B', 'A'], (0, 3)),        # a 0.6 m pair credited by turns
    (['A', 'B', 'A', 'C'], (1, 1)),             # a real loss on a paired buoy still counts
    (['A', 'B', 'C'], (2, 0)),
])
def test_id_changes_split_one_way_switches_from_alternation(seq, expected):
    assert id_changes(seq) == expected
