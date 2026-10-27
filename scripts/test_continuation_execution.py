#!/usr/bin/env python3
import copy
import unittest
from inspect_continuation_execution import inspect
from test_approach_execution import tick


def selected(t):
    row = tick(t, speed=1, acceleration=2)
    out = row['expected']
    out.pop('planning_diagnostics')
    out['continuation'] = dict(now_s=t, expires_s=8., original_endpoint=dict(x=4,y=0),
                               original_end_progress_m=4.)
    return row


class ContinuationExecutionTests(unittest.TestCase):
    def test_selection_without_lattice_diagnostics_and_following_stop(self):
        rows = [tick(0, True), selected(.1), selected(.2), tick(.3)]
        saved = copy.deepcopy(rows)
        result = inspect(rows)
        self.assertEqual(rows, saved)
        span = result['spans'][0]
        self.assertEqual(span['selected_ticks'], 2)
        self.assertEqual(span['positive_acceleration_ticks'], 2)
        self.assertEqual(span['previous']['time_s'], 0)
        self.assertEqual(span['following']['selected_disposition'], 'CheckedStop')
        self.assertFalse(span['first']['planning_diagnostics_available'])
        self.assertIsNone(span['first']['obstruction_stop_active'])
        self.assertEqual(span['end_observation'], 'selection_absent')
        self.assertAlmostEqual(span['expiry_remaining_at_last_s'], 7.8)

    def test_gaps_and_changed_permits_form_distinct_spans(self):
        rows = [selected(0), tick(.1), selected(.2), selected(.3)]
        rows[-1]['expected']['continuation']['expires_s'] = 8.1
        spans = inspect(rows)['spans']
        self.assertEqual(len(spans), 3)
        self.assertEqual(spans[1]['end_observation'], 'different_permit')
        self.assertEqual(spans[2]['end_observation'], 'recording_end')
        self.assertIsNone(spans[2]['following'])

    def test_terminal_window_and_no_selections(self):
        result = inspect([tick(0), tick(5), tick(6), tick(10)])
        self.assertEqual(result['spans'], [])
        self.assertEqual(result['selected_ticks'], 0)
        self.assertEqual(result['terminal_dispositions'], {'CheckedStop': 3})

    def test_invalid_clocks_and_expired_selection_rejected(self):
        wrong = selected(.1)
        wrong['expected']['continuation']['now_s'] = .2
        for rows in [[], [tick(1), tick(1)], [tick(2), tick(1)],
                     [selected(8)], [wrong], [tick(float('nan'))]]:
            with self.subTest(rows=rows):
                with self.assertRaises(ValueError):
                    inspect(rows)


if __name__ == '__main__':
    unittest.main()
