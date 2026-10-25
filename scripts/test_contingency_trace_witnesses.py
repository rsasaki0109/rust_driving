#!/usr/bin/env python3
"""Analytic controls for the read-only stationary-envelope diagnosis."""
import math
import unittest
from inspect_contingency_failure import stationary_guard

class StationaryWitnesses(unittest.TestCase):
    def ego(self,yaw=0.):
        return dict(speed=0.,pose=dict(position=dict(x=0.,y=0.),yaw=yaw))
    def object(self,velocity=(0.,0.)):
        return dict(id=1,radius=.5,dt=8.,positions=[
            dict(x=2.6,y=0.),dict(x=2.6+8*velocity[0],y=8*velocity[1])])
    def test_static_and_parallel_are_clear_but_slow_transverse_motion_retains_reserve(self):
        self.assertFalse(stationary_guard(self.ego(),[self.object()],1.)['existing_reserve_contact_witness'])
        parallel=self.object((.01,0.))
        self.assertFalse(stationary_guard(self.ego(),[parallel],1.)['existing_reserve_contact_witness'])
        crossing=stationary_guard(self.ego(),[self.object((0.,.01))],1.)
        self.assertTrue(crossing['without_directional_reserve_certified'])
        self.assertTrue(crossing['existing_reserve_contact_witness'])
        self.assertAlmostEqual(crossing['witness']['directional_reserve_m'],.7)
    def test_rigid_rotation_preserves_the_witness(self):
        base=self.object((0.,.01))
        original=stationary_guard(self.ego(),[base],1.)['witness']['actual_gap_m']
        for yaw in [-2.4,-.3,.8,2.7]:
            rotated=dict(base,positions=[dict(x=p['x']*math.cos(yaw)-p['y']*math.sin(yaw),
                y=p['x']*math.sin(yaw)+p['y']*math.cos(yaw)) for p in base['positions']])
            actual=stationary_guard(self.ego(yaw),[rotated],1.)
            self.assertAlmostEqual(actual['witness']['actual_gap_m'],original)
    def test_interior_radius_peak_is_not_an_endpoint_only_clearance(self):
        obj=dict(id=1,radius=.5,dt=4.,positions=[dict(x=3.,y=0.)]*3,
            additional_radius_m=[0.,2.,0.])
        proof=stationary_guard(self.ego(),[obj],1.)
        self.assertTrue(proof['existing_reserve_contact_witness'])
        self.assertFalse(proof['without_directional_reserve_certified'])
        self.assertAlmostEqual(proof['witness']['time_s'],4.,places=5)
    def test_moving_ego_is_not_misreported_as_a_stationary_certificate(self):
        ego=self.ego();ego['speed']=.01
        self.assertIsNone(stationary_guard(ego,[self.object()],1.))
    def test_interior_quadratic_minimum_detects_a_crossing(self):
        obj=dict(id=1,radius=.5,dt=8.,positions=[dict(x=-5.,y=1.),dict(x=5.,y=1.)])
        proof=stationary_guard(self.ego(),[obj],1.)
        self.assertTrue(proof['existing_reserve_contact_witness'])
        self.assertFalse(proof['without_directional_reserve_certified'])
        self.assertLess(proof['witness']['actual_gap_m'],0.)
        self.assertGreater(proof['witness']['time_s'],0.)
        self.assertLess(proof['witness']['time_s'],8.)

if __name__=='__main__':
    unittest.main()
