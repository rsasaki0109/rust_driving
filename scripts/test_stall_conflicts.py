#!/usr/bin/env python3
import unittest
from inspect_stall_conflicts import witness

class ConflictClockTests(unittest.TestCase):
    def sample(self,time=2.,stamp=9.9):
        config=dict(forecast_envelope=dict(position_error_m=.2,velocity_error_m_s=.3,acceleration_error_m_s2=.1))
        out=dict(time=10.,tracks=[dict(id=7,last_seen=stamp)],
            predictions=[dict(id=7,dt=.2,positions=[{}]*41,radius=1.)])
        candidate=dict(index=1,lateral_target_m=3.9,disposition='CheckedStop',
            nominal_forecast_conflict=dict(prediction_id=7,relative_time_s=time,
                forecast_radius_m=3.,minimum_clearance_floor_m=0.))
        return config,out,candidate

    def test_uses_actual_conflict_time_and_consumes_measurement_age(self):
        result=witness(*self.sample())
        self.assertAlmostEqual(result['declared_cv_radius_m'],.2+.3*2.1+.05*2.1**2)
        self.assertLess(result['declared_cv_radius_m'],5.8)

    def test_endpoint_hold_does_not_invent_extrapolated_forecast_growth(self):
        result=witness(*self.sample(time=12.,stamp=10.))
        self.assertTrue(result['endpoint_held'])
        self.assertAlmostEqual(result['declared_cv_radius_m'],5.8)

    def test_future_acquisition_and_nonfinite_time_are_rejected(self):
        for sample in [self.sample(stamp=11.),self.sample(time=float('nan'))]:
            with self.assertRaises(ValueError):witness(*sample)

    def test_absent_or_duplicated_track_identity_is_rejected(self):
        for duplicate in [False,True]:
            config,out,candidate=self.sample()
            out['tracks']=out['tracks']*2 if duplicate else []
            with self.assertRaises(ValueError):witness(config,out,candidate)

if __name__=='__main__':unittest.main()
