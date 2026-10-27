#!/usr/bin/env python3
import copy
import unittest
from audit_continuation_ownership import audit

CONFIG = dict(continue_checked_approaches=True)


def recording():
    adoption = dict(adopted=True, supporting_acquisitions=3, first_acquisition_s=0.,
        last_acquisition_s=.2, proposal_endpoint=dict(x=5.,y=0.),
        proposal_end_progress_m=5., proposal_lateral_target_m=0.,
        blocking_prediction_ids=[7])
    def output(t):
        return dict(time=t, localization_epoch=0, health=[], emergency=False,
                    checked_approach=dict(adopted=False))
    rows = [output(.2), output(.25), output(.3)]
    rows[0]['checked_approach'] = adoption
    for row in rows[1:]:
        row['continuation'] = dict(now_s=row['time'], acquisition_s=.2, expires_s=8.2,
            original_endpoint=dict(x=5.,y=0.), original_end_progress_m=5.,
            selected_end_progress_m=4.9, lateral_target_m=0., blocking_ids=[7])
    return [dict(expected=row) for row in rows]


class OwnershipTests(unittest.TestCase):
    def test_binds_cached_and_fresh_selections_without_mutation(self):
        rows = recording()
        rows[-1]['expected']['continuation']['acquisition_s'] = .3
        for row in rows: row['expected'].pop('localization_epoch')
        before = copy.deepcopy(rows)
        result = audit(CONFIG, rows)
        self.assertEqual(result['selected_ticks'], 2)
        self.assertEqual(result['adoptions'], 1)
        self.assertEqual(rows, before)

    def test_rejects_corrupted_binding(self):
        changes = [
            ('expires_s', 8.25), ('now_s', .26), ('acquisition_s', .1),
            ('acquisition_s', .4), ('original_endpoint', dict(x=6.,y=0.)),
            ('original_end_progress_m', 6.), ('lateral_target_m', 1.),
            ('blocking_ids', [8]), ('selected_end_progress_m', 5.1)]
        for key, value in changes:
            with self.subTest(key=key, value=value):
                rows = recording()
                rows[1]['expected']['continuation'][key] = value
                with self.assertRaises(ValueError): audit(CONFIG, rows)

    def test_absence_fault_or_epoch_cannot_revive_authority(self):
        for change in ['absence', 'emergency', 'health', 'epoch']:
            rows = recording()
            if change == 'absence': rows[1]['expected'].pop('continuation')
            elif change == 'emergency': rows[1]['expected']['emergency'] = True
            elif change == 'health': rows[1]['expected']['health'] = ['fault']
            else: rows[2]['expected']['localization_epoch'] = 1
            with self.subTest(change=change):
                with self.assertRaises(ValueError): audit(CONFIG, rows)
        rows = recording()[:2]
        rows[-1]['expected']['emergency'] = True
        self.assertEqual(audit(CONFIG, rows)['selected_guarded_ticks'], 1)

    def test_unsupported_disabled_expired_or_unowned_selection_rejected(self):
        with self.assertRaises(ValueError): audit({}, recording())
        with self.assertRaises(ValueError): audit(CONFIG, recording()[1:])
        for change in ['support', 'expiry', 'clock', 'simultaneous']:
            rows = recording()
            if change == 'support': rows[0]['expected']['checked_approach']['supporting_acquisitions'] = 2
            elif change == 'expiry':
                rows[1]['expected']['time'] = 8.2
                rows[1]['expected']['continuation'].update(now_s=8.2, acquisition_s=8.2)
            elif change == 'clock': rows[1]['expected']['time'] = .2
            else: rows[1]['expected']['checked_approach']['adopted'] = True
            with self.subTest(change=change):
                with self.assertRaises(ValueError): audit(CONFIG, rows)


if __name__ == '__main__':
    unittest.main()
