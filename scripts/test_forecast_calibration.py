#!/usr/bin/env python3
"""Independent temporal, dependence-group and failure controls for calibration."""
import copy
import math
import unittest

from forecast_calibration import case_scores, descriptive, forecast_at, grouped_split, quantile


def fixture():
    estimate = dict(pose=dict(position=dict(x=0., y=0.), yaw=0.), speed=0.)
    actor = dict(id=11, position=dict(x=1., y=0.), radius=.5)
    track = dict(id=7, position=dict(x=1., y=0.), velocity=dict(x=0., y=0.), radius=.5, last_seen=0.)
    prediction = dict(id=7, positions=[dict(x=1., y=0.) for _ in range(41)],
                      radius=.5, additional_radius_m=[1.]*41, dt=.2)
    frames, ticks = [], []
    for i in range(181):
        frames.append(dict(time=i*.05, estimate=copy.deepcopy(estimate), objects=[copy.deepcopy(actor)]))
        ticks.append(dict(expected=dict(time=i*.05, estimate=copy.deepcopy(estimate), health=[],
            tracks=[copy.deepcopy(track)], predictions=[copy.deepcopy(prediction)])))
    run = dict(frames=frames, summary=dict(steps=len(frames), seed=7, reached_goal=False, passed=False))
    config = dict(forecast_envelope=dict(position_error_m=.2, velocity_error_m_s=.3, acceleration_error_m_s2=.1))
    return run, config, ticks


class CalibrationControls(unittest.TestCase):
    def test_finite_sample_rank_uses_independent_groups_and_an_infinite_boundary(self):
        for n, rank, bounded in [(0, 1, False), (1, 2, False), (98, 99, False),
                                  (99, 99, True), (100, 100, True), (199, 198, True)]:
            result = quantile([(str(i), float(i)) for i in range(n)])
            self.assertEqual(result['order_statistic_rank'], rank)
            self.assertEqual(result['unbounded'], not bounded)
            self.assertEqual(result['minimum_groups_for_finite_rank'], 99)
            if bounded:
                self.assertEqual(result['scale'], float(rank-1))
            self.assertFalse(result['confidence_established'])

    def test_repeated_times_plants_and_cases_cannot_inflate_the_group_count(self):
        rows = [('seed:7', .1)]*10000 + [('seed:7', 1.2)] + [('seed:42', .8)]*10000
        result = quantile(rows)
        self.assertEqual(result['group_count'], 2)
        self.assertTrue(result['unbounded'])
        self.assertEqual(quantile([('a', .1), ('a', .9)], '0.5')['scale'], .9)

    def test_unmatched_scores_stay_in_the_order_statistic(self):
        rows = [(str(i), .1) for i in range(99)]
        rows[-1] = ('98', None)
        self.assertTrue(quantile(rows)['unbounded'])
        self.assertIsNone(quantile([('a', None), ('a', 0.)], '0.5')['scale'])

    def test_split_rejects_leakage_duplicates_and_missing_frozen_groups(self):
        cases = [dict(case_id='one', seed=7, joint_nonconformity=.2),
                 dict(case_id='two', seed=42, joint_nonconformity=.3)]
        result = grouped_split(cases, [7], [42])
        self.assertEqual(result['calibration']['group_count'], 1)
        self.assertEqual(result['evaluation_group_count'], 1)
        self.assertIsNone(result['evaluation_covered_groups'])
        self.assertFalse(result['fresh_holdout'])
        for values, cal, test in [(cases, [7], [7, 42]), (cases+cases[:1], [7], [42]),
                                  (cases, [7, 99], [42]), (cases, [7], [99])]:
            with self.subTest(cal=cal, test=test), self.assertRaises(ValueError):
                grouped_split(values, cal, test)

    def test_cached_forecasts_supply_one_origin_and_radius_components_have_units(self):
        run, config, ticks = fixture()
        result = case_scores(run, config, ticks)
        self.assertEqual(result['acquired_identities'], 1)
        self.assertEqual(result['eligible_origins'], 1)
        self.assertEqual(result['horizons']['8.0']['covered'], 1)
        self.assertAlmostEqual(result['horizons']['8.0']['declared_cv_error_radius_m']['maximum'], 5.8)
        self.assertEqual(result['joint_nonconformity'], 0.)
        self.assertEqual(result['original_summary'], run['summary'])
        self.assertFalse(result['confidence_established'])

    def test_joint_score_retains_a_single_bad_horizon_among_many_covered_times(self):
        run, config, ticks = fixture()
        run['frames'][160]['objects'][0]['position']['x'] = 3.
        result = case_scores(run, config, ticks)
        self.assertEqual(result['joint_nonconformity'], 2.)
        self.assertFalse(result['joint_observed_covered'])
        self.assertEqual(result['horizons']['0.2']['covered'], 1)
        self.assertEqual(result['horizons']['8.0']['covered'], 0)

    def test_association_is_fixed_before_the_future_is_seen(self):
        run, config, ticks = fixture()
        run['frames'][40]['objects'][0]['position']['x'] = 5.
        run['frames'][40]['objects'].append(dict(id=12, position=dict(x=1., y=0.), radius=.5))
        result = case_scores(run, config, ticks)
        witness = result['horizons']['2.0']['worst_utilization']
        self.assertEqual(witness['actor_id'], 11)
        self.assertEqual(witness['score'], 4.)

    def test_ambiguity_and_future_absence_cannot_be_removed_from_the_denominator(self):
        for which in ['ambiguous', 'absent']:
            run, config, ticks = fixture()
            if which == 'ambiguous':
                run['frames'][0]['objects'].append(dict(id=12, position=dict(x=1.1, y=0.), radius=.5))
            else:
                run['frames'][40]['objects'] = []
            result = case_scores(run, config, ticks)
            self.assertIsNone(result['joint_nonconformity'])
            self.assertEqual(result['horizons']['2.0']['eligible'], 1)
            self.assertEqual(result['horizons']['2.0']['covered'], 0)

    def test_right_censoring_and_empty_scoring_domain_are_explicit(self):
        run, config, ticks = fixture()
        for tick in ticks[178:]:
            tick['expected']['tracks'][0]['last_seen'] = 8.9
        result = case_scores(run, config, ticks)
        self.assertEqual(result['eligible_origins'], 1)
        self.assertEqual(result['right_censored_origins'], 1)
        for tick in ticks:
            tick['expected']['tracks'] = []
            tick['expected']['predictions'] = []
        self.assertIsNone(case_scores(run, config, ticks)['joint_nonconformity'])

    def test_interpolation_is_in_forecast_time_and_zero_radius_does_not_hide_error(self):
        run, config, ticks = fixture()
        p = ticks[0]['expected']['predictions'][0]
        p['positions'][1]['x'] = 3.
        self.assertEqual(forecast_at(p, .1)[0], (2., 0.))
        for tick in ticks:
            tick['expected']['predictions'][0]['additional_radius_m'] = [0.]*41
        run['frames'][20]['objects'][0]['position']['x'] = 2.
        self.assertIsNone(case_scores(run, config, ticks)['joint_nonconformity'])

    def test_clock_ownership_identity_and_nonfinite_evidence_are_rejected(self):
        for which in ['clock', 'future', 'duplicates', 'nan', 'short']:
            run, config, ticks = fixture()
            out = ticks[0]['expected']
            if which == 'clock':
                run['frames'][1]['time'] += .001
            elif which == 'future':
                out['tracks'][0]['last_seen'] = .1
            elif which == 'duplicates':
                out['tracks'].append(copy.deepcopy(out['tracks'][0]))
            elif which == 'nan':
                out['predictions'][0]['additional_radius_m'][0] = math.nan
            else:
                out['predictions'][0]['positions'].pop()
                out['predictions'][0]['additional_radius_m'].pop()
            with self.subTest(which=which), self.assertRaises(ValueError):
                case_scores(run, config, ticks)

    def test_descriptive_percentiles_are_not_confidence_bounds(self):
        self.assertEqual(descriptive(range(1, 101)), dict(count=100, minimum=1, median=50, p95=95, maximum=100))
        self.assertIsNone(descriptive([])['maximum'])


if __name__ == '__main__':
    unittest.main()
