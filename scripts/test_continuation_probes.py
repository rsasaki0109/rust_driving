import copy
import unittest
from inspect_continuation_probes import inspect

def fixture():
    points = [
        dict(position=dict(x=0., y=0.), speed=0.21, time=0.),
        dict(position=dict(x=1., y=0.), speed=0., time=2.),
        dict(position=dict(x=1., y=0.), speed=0., time=10.),
    ]
    row = dict(kind='probe', tick=1, time=.2, new_adoption=True,
        permit_alive=True, healthy_unrestricted_output=True, closed_loop=False,
        endpoint_bound=[dict(x=2., y=0.), 2.], ego=dict(speed=.21),
        candidate=dict(mode='Yield', points=points),
        original=dict(points=[dict(speed=.21)]))
    # Add a peak purely to test comparison accounting, not physical feasibility.
    row['candidate']['points'].insert(1, dict(position=dict(x=.5,y=0.),speed=1.,time=1.))
    second = copy.deepcopy(row)
    second.update(tick=2, time=.25, new_adoption=False)
    end = dict(kind='end', ticks=4, adoptions=1, permits=1,
        attempted_ticks=2, candidate_ticks=2, legacy_verified=True, closed_loop=False)
    return [dict(kind='configuration'), row, second, end]

class ProbeAudit(unittest.TestCase):
    def test_same_tick_and_later_comparisons_are_separate(self):
        result = inspect(fixture())
        self.assertEqual(result['candidate_ticks'], 2)
        self.assertEqual(result['later_candidate_ticks'], 1)
        self.assertEqual(result['higher_peak_than_original_ticks'], 2)
        self.assertEqual(result['later_low_speed_higher_peak_ticks'], 1)

    def test_rejected_candidate_counts_attempt_and_requires_revocation(self):
        rows = fixture()
        rows[2]['candidate'] = None
        rows[2]['permit_alive'] = False
        rows[-1]['candidate_ticks'] = 1
        self.assertEqual(inspect(rows)['later_candidate_ticks'], 0)
        rows[2]['permit_alive'] = True
        with self.assertRaises(ValueError):
            inspect(rows)

    def test_corrupted_structure_counts_and_hold_are_rejected(self):
        for case in range(10):
            rows = fixture()
            if case == 0: rows[-1]['candidate_ticks'] += 1
            elif case == 1: rows[2]['tick'] = rows[1]['tick']
            elif case == 2: rows[1]['endpoint_bound'] = None
            elif case == 3: rows[1]['candidate']['points'][-1]['time'] = 9.
            elif case == 4: rows[1]['healthy_unrestricted_output'] = False
            elif case == 5: rows[1]['candidate']['points'][0]['speed'] = float('nan')
            elif case == 6: rows.pop()
            elif case == 7: rows[2]['endpoint_bound'][1] = 3.
            elif case == 8: rows[2]['time'] = 8.2
            else: rows[1]['new_adoption'] = False
            with self.assertRaises(ValueError):
                inspect(rows)

if __name__ == '__main__':
    unittest.main()
