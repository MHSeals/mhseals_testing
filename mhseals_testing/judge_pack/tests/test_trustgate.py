"""trustgate statistics and the G3 sample-size rule."""
import unittest

from mhseals_testing.judge_pack import trustgate as tg


def pair(task, variant, seed, fast_pts, unity_pts):
    side = lambda pts: {'scored': True, 'points': pts, 'possible': 100.0, 'attempted': {'t': True}, 'earned': {'t': pts > 0},
                        'duration_s': 10.0, 'strikes': 0, 'path_m': 5.0, 'flags': [], 'nav': variant}
    return {'pair_id': f'{variant}{seed}', 'task': task, 'seed': seed, 'unity_session': f'u{variant}{seed}',
            'fast': side(fast_pts), 'unity': side(unity_pts), 'matched': True, 'match': 'commit',
            'variant': variant, 'scored': True}


class StatsTest(unittest.TestCase):
    def test_rank_statistics(self):
        a, b = [1, 2, 3, 4], [10, 20, 30, 40]
        self.assertAlmostEqual(tg.spearman(a, b), 1.0)
        self.assertAlmostEqual(tg.kendall_tau_b(a, list(reversed(b))), -1.0)
        self.assertIsNone(tg.pearson([1, 1], [2, 3]))

    def test_mmrv_is_the_reference_gap_of_the_worst_reversal(self):
        low, ref = {'a': 1, 'b': 2, 'c': 3}, {'a': 0.1, 'b': 0.5, 'c': 0.4}
        self.assertAlmostEqual(tg.mmrv(low, ref), (0 + 0.1 + 0.1) / 3)
        self.assertEqual(tg.mmrv(ref, ref), 0.0)

    def test_g3_needs_8_variants_by_5_seeds(self):
        few = [pair('mission', f'v{v}', s, v, v) for v in range(3) for s in range(5)]
        self.assertEqual(tg.g3(few)['scopes']['mission']['status'], 'insufficient')
        enough = [pair('mission', f'v{v}', s, v, v) for v in range(8) for s in range(5)]
        row = tg.g3(enough)['scopes']['mission']
        self.assertEqual(row['status'], 'measurable')
        self.assertAlmostEqual(row['descriptive']['spearman'], 1.0)

    def test_g4_bands(self):
        r = tg.g4([pair('mission', 'v', s, 50, 50) for s in range(4)])
        self.assertEqual(r['verdict'], 'pass')


if __name__ == '__main__':
    unittest.main()
