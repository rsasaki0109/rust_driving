#!/usr/bin/env python3
"""Independent endpoint-time semantics; dense integration is the physical control."""
import copy
import math
import unittest

from check_gnss_latency import Filter, chronological

CONFIG = dict(initial_pose=dict(position=dict(x=12., y=-7.), yaw=.4),
              rear_axle_offset_m=1.5, instantaneous_odometry=True)

class EndpointTests(unittest.TestCase):
    def test_linear_rates_integrate_yaw_and_report_current_course_speed(self):
        oracle = Filter(CONFIG)
        first = dict(stamp=2., speed=1., yaw_rate=-.6)
        last = dict(stamp=2.05, speed=5., yaw_rate=1.)
        oracle.seed_motion(first, 2.)
        oracle.predict(last, .05)
        self.assertAlmostEqual(oracle.x[2], .41, places=12)
        self.assertAlmostEqual(oracle.speed, math.hypot(5., 1.5), places=12)
        # Independent continuous integral for time-linear wheel and gyro rates.
        x, y, count = 12., -7., 10000
        for i in range(count):
            t = (i+.5)*.05/count
            wheel, rate = 1.+4.*t/.05, -.6+1.6*t/.05
            yaw = .4-.6*t+.8*t*t/.05
            x += (wheel*math.cos(yaw)-1.5*rate*math.sin(yaw))*.05/count
            y += (wheel*math.sin(yaw)+1.5*rate*math.cos(yaw))*.05/count
        self.assertLess(math.dist(oracle.x[:2], [x,y]), .001)
        held = Filter(dict(CONFIG, instantaneous_odometry=False))
        held.predict(last, .05)
        self.assertGreater(math.dist(held.x[:2], oracle.x[:2]), .05)

    def test_constant_rates_preserve_original_chassis_state_and_covariance(self):
        sampled = Filter(CONFIG)
        held = Filter(dict(CONFIG, instantaneous_odometry=False))
        sampled.seed_motion(dict(stamp=0., speed=3., yaw_rate=.4), 0.)
        for i in range(1, 51):
            motion = dict(stamp=i*.05, speed=3., yaw_rate=.4)
            sampled.predict(motion, .05); held.predict(motion, .05)
            self.assertEqual(sampled.x, held.x)
            self.assertEqual(sampled.speed, held.speed)
            self.assertEqual(sampled.p, held.p)

    def test_missing_seed_repeated_or_shifted_endpoint_is_rejected(self):
        motion = dict(stamp=.05, speed=2., yaw_rate=.2)
        oracle = Filter(CONFIG)
        with self.assertRaises(ValueError): oracle.predict(motion, .05)
        for last in [dict(motion, stamp=0.), dict(motion, stamp=.1),
                     dict(motion, yaw_rate=math.nan)]:
            oracle = Filter(CONFIG)
            oracle.seed_motion(dict(motion, stamp=0.), 0.)
            before = copy.deepcopy((oracle.x, oracle.p, oracle.previous_motion))
            with self.assertRaises(ValueError): oracle.predict(last, .05)
            self.assertEqual(before, (oracle.x, oracle.p, oracle.previous_motion))
        for initial in [None, dict(motion, stamp=-1.), dict(motion, stamp=.02)]:
            with self.assertRaises(ValueError): Filter(CONFIG).seed_motion(initial, 0.)

    def test_unsupported_rewind_and_missing_body_calibration_are_rejected(self):
        for extra in [dict(rear_axle_offset_m=None), dict(gnss_latency_compensation=True),
                      dict(heading_latency_compensation=True), dict(localization_map={'unsupported':True})]:
            with self.assertRaises(ValueError): Filter(dict(CONFIG, **extra))
        with self.assertRaises(ValueError): chronological(CONFIG, [], [])

if __name__ == '__main__':
    unittest.main()
