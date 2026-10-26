#!/usr/bin/env python3
"""Analytic controls for independent velocity-disc support."""
import math
import unittest
from directional_reserve_oracle import directional_reserve

class DirectionalSupport(unittest.TestCase):
    def test_underresolved_direction_has_the_full_crossing_reserve(self):
        for angle in [0.,.3,1.8,3.,5.4]:
            velocity=(.01*math.cos(angle),.01*math.sin(angle))
            self.assertEqual(directional_reserve(velocity,.3,(1.,0.)),.7)
        self.assertEqual(directional_reserve((0.,0.),0.,(1.,0.)),0.)
    def test_known_parallel_velocity_has_the_analytic_tangent_circle_bound(self):
        self.assertAlmostEqual(directional_reserve((1.,0.),.2,(1.,0.)),.7*.2)
        self.assertAlmostEqual(directional_reserve((.3,0.),.3,(1.,0.)),.7)
        self.assertAlmostEqual(directional_reserve((0.,.001),0.,(1.,0.)),.7)
    def test_circle_sampling_cannot_exceed_support(self):
        for velocity in [(0.,0.),(.01,-.02),(2.,.1),(-4.,.7)]:
            for radius in [0.,.05,.3,1.]:
                for heading in [-2.3,-.4,.8,2.6]:
                    tangent=(math.cos(heading),math.sin(heading))
                    upper=directional_reserve(velocity,radius,tangent)
                    for i in range(256):
                        angle=2*math.pi*i/256
                        candidate=(velocity[0]+radius*math.cos(angle),velocity[1]+radius*math.sin(angle))
                        self.assertLessEqual(directional_reserve(candidate,0.,tangent),upper+1e-12)
    def test_joint_rotation_and_growing_error_preserve_order(self):
        original=directional_reserve((3.,.7),.3,(.8,.6))
        for angle in [-2.,.1,1.8]:
            c,s=math.cos(angle),math.sin(angle)
            def rotate(v):return (c*v[0]-s*v[1],s*v[0]+c*v[1])
            self.assertAlmostEqual(directional_reserve(rotate((3.,.7)),.3,rotate((.8,.6))),original)
        values=[directional_reserve((4.,0.),r,(1.,0.)) for r in [0.,.1,.3,1.,4.,5.]]
        self.assertEqual(values,sorted(values))
    def test_invalid_context_is_rejected(self):
        for value in [-1.,math.nan,math.inf]:
            with self.assertRaises(ValueError):directional_reserve((0.,0.),value,(1.,0.))

if __name__=='__main__':
    unittest.main()
