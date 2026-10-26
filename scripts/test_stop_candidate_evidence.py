#!/usr/bin/env python3
import unittest
from stop_candidate_evidence import classify, summarize

class StopEvidenceTests(unittest.TestCase):
    def sample(self,contact=None,gain=2.):
        return dict(hold_start_s=2.,hold_end_s=10.,progress_gain_m=gain,first_contact_s=contact)
    def test_arrival_boundary_belongs_to_stationary_tail(self):
        self.assertEqual(classify(self.sample(1.99)),'moving_conflict')
        self.assertEqual(classify(self.sample(2.)),'stationary_tail_conflict')
        self.assertEqual(classify(self.sample(10.)),'stationary_tail_conflict')
    def test_clear_small_progress_is_distinct_from_collision(self):
        result=summarize([self.sample(gain=1.99),self.sample(gain=2.),self.sample(5.)])
        self.assertEqual(result['sweep_counts'],dict(clear_below_2m=1,clear_at_least_2m=1,stationary_tail_conflict=1))
        self.assertEqual(result['maximum_clear_progress_gain_m'],2.)
        self.assertFalse(result['confirmation_authorized'])
    def test_incomplete_tail_and_invalid_clocks_fail(self):
        for change in [dict(hold_end_s=9.9),dict(first_contact_s=-1.),dict(first_contact_s=11.),
                       dict(first_contact_s=float('nan')),dict(progress_gain_m=float('inf'))]:
            with self.assertRaises(ValueError):classify(dict(self.sample(),**change))
    def test_absent_search_is_not_a_feasibility_result(self):
        with self.assertRaises(ValueError):summarize([])
if __name__=='__main__':unittest.main()
