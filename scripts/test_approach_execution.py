#!/usr/bin/env python3
import unittest
from inspect_approach_execution import events, original_projection

def tick(t,adopt=False,speed=0,acceleration=0,endpoint=1):
    return dict(expected=dict(time=t,estimate=dict(speed=speed),
        command=dict(acceleration=acceleration,steering=0),emergency=False,health=[],
        checked_approach=dict(adopted=adopt),
        planning_diagnostics=dict(route_progress_m=t,selected_candidate=0,
            obstruction_stop_requested=False,obstruction_stop_active=False,
            candidates=[dict(disposition='CheckedStop',profile=dict(searched_stop=False))]),
        trajectory=dict(mode='Yield',lateral_target=0,points=[
            dict(position=dict(x=0,y=0),time=0,speed=0),
            dict(position=dict(x=endpoint,y=0),time=1,speed=speed)])))

class ExecutionTraceTests(unittest.TestCase):
    def test_intervals_exclude_next_adoption_and_keep_final_censoring(self):
        result=events([tick(0),tick(1,True,1,2),tick(2,speed=.1,endpoint=2),
                       tick(3,True,9,2),tick(4,speed=.2)])
        self.assertEqual([e['adoption_time_s'] for e in result],[1,3])
        self.assertEqual(result[0]['maximum_estimated_speed_m_s'],1)
        self.assertEqual(result[0]['observed_progress_gain_m'],1)
        self.assertEqual(result[0]['next_adoption_time_s'],3)
        self.assertEqual(result[0]['first_endpoint_change']['time_s'],2)
        self.assertEqual(result[0]['first_stationary_profile']['time_s'],2)
        self.assertEqual(result[1]['end_reason'],'recording_end')
        self.assertEqual(result[1]['last_observed_time_s'],4)
    def test_request_transition_retains_both_ticks_and_does_not_mean_active_hold(self):
        ticks=[tick(0,True),tick(.05,speed=.1),tick(.1,speed=.21)]
        d=ticks[-1]['expected']['planning_diagnostics']
        d['obstruction_stop_requested']=True
        d['candidates'][0]['profile']['searched_stop']=True
        event=events(ticks)[0]
        transition=event['first_new_stop_request']
        self.assertEqual(transition['before']['time_s'],.05)
        self.assertEqual(transition['after']['time_s'],.1)
        self.assertFalse(transition['after']['obstruction_stop_active'])
        self.assertTrue(transition['after']['selected_searched_stop'])
        projected=original_projection(event)
        self.assertNotIn('first_new_stop_request',projected)
        self.assertNotIn('obstruction_stop_requested',projected['trace'][0])
        self.assertEqual(projected['trace'][-1]['estimated_speed_m_s'],.21)
    def test_existing_request_and_next_adoption_are_not_new_requests(self):
        ticks=[tick(0,True),tick(.05),tick(.1,True)]
        for t in ticks:t['expected']['planning_diagnostics']['obstruction_stop_requested']=True
        self.assertTrue(all(e['first_new_stop_request'] is None for e in events(ticks)))
        ticks[0]['expected']['planning_diagnostics']['obstruction_stop_requested']=False
        ticks[1]['expected']['planning_diagnostics']['obstruction_stop_requested']=False
        self.assertIsNone(events(ticks)[0]['first_new_stop_request'])
    def test_empty_duplicate_or_reversed_clock_rejected(self):
        for ticks in [[],[tick(1),tick(1)],[tick(2),tick(1)]]:
            with self.assertRaises(ValueError):events(ticks)
    def test_unadopted_trace_produces_no_event(self):
        self.assertEqual(events([tick(0),tick(1)]),[])
    def test_terminal_adoption_is_censored_not_success(self):
        event=events([tick(0),tick(1,True)])[0]
        self.assertEqual(event['observed_progress_gain_m'],0)
        self.assertIsNone(event['first_endpoint_change'])
        self.assertEqual(event['end_reason'],'recording_end')

if __name__=='__main__':
    unittest.main()
