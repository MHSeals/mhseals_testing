"""The hull shape library (hull_shapes.py) and the vessel profile reader it is fed from (mhseals_hardware.vessel)."""

import numpy as np
import pytest

from mhseals_hardware import vessel
from mhseals_testing.ui.minimap import hull_shapes

CAT = {'shape': 'catamaran', 'length': 1.34, 'beam': 0.9, 'pontoon': 0.23, 'deck': 0.63}


def test_the_vessel_profile_sets_the_hull_over_its_fleet(tmp_path):
    (tmp_path / 'fleet').mkdir()
    (tmp_path / 'fleet' / 'profile.yaml').write_text('hull: {shape: monohull, length: 2.0}\n')
    (tmp_path / 'fleet' / 'hull.yaml').write_text('{length: 9.0, beam: 1.0}\n')       # the kind file, under it
    (tmp_path / 'boat').mkdir()
    (tmp_path / 'boat' / 'profile.yaml').write_text('extends: fleet\nhull: {shape: catamaran, pontoon: 0.2}\n')
    (tmp_path / 'boat' / 'network.env').write_text('')
    found = vessel.section('hull', str(tmp_path / 'boat' / 'network.env'))
    assert found == {'shape': 'catamaran', 'length': 2.0, 'beam': 1.0, 'pontoon': 0.2}
    with pytest.raises(ValueError, match='missing'):                  # a named vessel never reads as none
        vessel.section('hull', str(tmp_path / 'missing.env'))


def test_catamaran_is_two_pontoons_bridged_by_a_deck():
    parts = hull_shapes.parts('catamaran', CAT)
    decks, hulls = [p for p in parts if p.deck], [p for p in parts if not p.deck]
    assert len(decks) == 1 and len(hulls) == 2 and parts[0].deck      # decks first: hulls draw over them
    assert all(abs(abs(p.offset) - (0.9 - 0.23) / 0.9 / 2) < 1e-9 for p in hulls)
    assert len(hull_shapes.sterns('catamaran', CAT)) == 2 and len(hull_shapes.sterns('monohull', CAT)) == 1
    along, across = np.array([0.0, 0.0, 0.0]), np.array([0.0, hulls[0].offset, 0.49])
    on_hull, on_deck = hull_shapes.coverage('catamaran', CAT, along, across)
    assert list(on_deck) == [True, False, False] and list(on_hull) == [False, True, True]


@pytest.mark.parametrize('env', [{}, {'ROS_WS': '/ws'}, {'ASTRO_NETWORK_ENV': '~/boat/network.env', 'ROS_WS': '/ws'}])
def test_the_reader_finds_the_network_env_as_mhseals_nav_does(env, monkeypatch):
    from mhseals_nav import boat_network
    for key, value in env.items():
        monkeypatch.setenv(key, value)
    assert vessel.env_path() == boat_network.path()
