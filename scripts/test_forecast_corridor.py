#!/usr/bin/env python3
import math
import unittest
from inspect_forecast_corridor import section, inspect


class CorridorTests(unittest.TestCase):
    def test_analytic_intersection_and_union(self):
        value = section((0.,0.), (1.,0.), 2., [(1,(0.,0.),1.)])
        self.assertEqual(value['free_width_m'],2.)
        self.assertEqual(value['free_intervals_m'],[[-2.,-1.],[1.,2.]])
        self.assertFalse(value['fully_covered'])
        full = section((0.,0.), (1.,0.), 2., [(1,(0.,-1.),1.),(2,(0.,1.),1.)])
        self.assertTrue(full['fully_covered'])
        self.assertEqual(full['free_width_m'],0.)
        self.assertEqual(section((0.,0.),(1.,0.),2.,[(1,(2.,0.),1.)])['free_width_m'],4.)
        tangent = section((0.,0.),(1.,0.),2.,[(1,(1.,0.),1.)])
        self.assertEqual(tangent['free_width_m'],4.)
        self.assertFalse(tangent['fully_covered'])

    def test_joint_rotation_preserves_width(self):
        original = section((2.,3.),(1.,0.),2.,[(1,(2.6,3.5),1.)])
        angle = .71
        def rotate(p):
            x,y=p
            return (x*math.cos(angle)-y*math.sin(angle),x*math.sin(angle)+y*math.cos(angle))
        rotated = section(rotate((2.,3.)),rotate((1.,0.)),2.,[(1,rotate((2.6,3.5)),1.)])
        self.assertAlmostEqual(original['free_width_m'],rotated['free_width_m'])

    def test_prediction_interpolation_and_endpoint_hold(self):
        route = dict(points=[dict(x=0.,y=0.),dict(x=20.,y=0.)],half_width=3.)
        prediction = dict(id=1,positions=[dict(x=10.,y=0.)]*2,
            radius=.5,additional_radius_m=[0.,2.],dt=4.)
        result = inspect(route,[prediction],.5,10.)
        widths = [r['free_width_m'] for r in result['samples'] if r['ahead_m']==0.]
        self.assertEqual(widths,[3.,1.,0.,0.])
        self.assertFalse(result['continuous_space_time_certified'])

    def test_malformed_inputs_are_rejected(self):
        route = dict(points=[dict(x=0.,y=0.),dict(x=20.,y=0.)],half_width=3.)
        for radius,progress in [(-1.,0.),(3.,0.),(.5,-1.),(.5,float('nan'))]:
            with self.assertRaises(ValueError): inspect(route,[],radius,progress)
        with self.assertRaises(ValueError): section((0.,0.),(0.,0.),2.,[])
        with self.assertRaises(ValueError): section((0.,0.),(1.,0.),2.,[(1,(0.,0.),float('inf'))])


if __name__ == '__main__':
    unittest.main()
