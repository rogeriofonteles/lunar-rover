"""Arrival timing, dwell boundaries, interruptions and recorded-case regressions."""
from pathlib import Path
import json
import unittest

import numpy as np

from arm.buttons import button_target
from arm.route_metrics import evaluate_arrivals,arrival_counts


class ArrivalTests(unittest.TestCase):
    def fixture(self,n=10):
        times=np.arange(n)*.02
        states=np.zeros((n,12))
        tip=np.tile(button_target('red')['position_m'],(n,1))
        arrivals=[dict(button='red',index=0),dict(button='red',index=2)]
        return arrivals,times,states,tip

    def test_on_time_and_initial_pose_not_counted(self):
        args=self.fixture()
        visits=evaluate_arrivals(*args,.02,.06)
        self.assertEqual(visits[1]['status'],'on_time')
        self.assertEqual(visits[1]['delay_s'],0)
        self.assertEqual(arrival_counts(visits),dict(on_time=1,delayed=0,missed=0,total=1,reached=1))

    def test_late_arrival_needs_position_and_speed_at_same_sample(self):
        arrivals,times,states,tip=self.fixture()
        states[2:4,6]=.05  # equality is outside the strict speed criterion
        tip[4,0]+=.02     # slow enough, but outside the position criterion
        visits=evaluate_arrivals(arrivals,times,states,tip,.02,.06)
        v=visits[1]
        self.assertEqual(v['status'],'delayed')
        self.assertFalse(v['reached'])  # legacy field still means on-time
        self.assertTrue(v['reached_within_window'])
        self.assertAlmostEqual(v['actual_arrival_time_s'],.1)
        self.assertAlmostEqual(v['delay_s'],.06)  # last dwell sample is included
        self.assertEqual(arrival_counts(visits)['delayed'],1)

    def test_no_credit_before_schedule_or_after_dwell(self):
        arrivals,times,states,tip=self.fixture()
        states[2:6,6]=.1
        visits=evaluate_arrivals(arrivals,times,states,tip,.02,.06)
        self.assertEqual(visits[1]['status'],'missed')
        self.assertIsNone(visits[1]['delay_s'])
        self.assertEqual(visits[1]['reason'],'criteria_not_met')

    def test_repeated_button_visits_are_independent(self):
        arrivals,times,states,tip=self.fixture(16)
        arrivals.append(dict(button='red',index=12))
        states[2:6,6]=.1
        visits=evaluate_arrivals(arrivals,times,states,tip,.02,.06)
        self.assertEqual([v['status'] for v in visits[1:]],['missed','on_time'])
        self.assertEqual(arrival_counts(visits)['total'],2)

    def test_interruption_before_and_during_window(self):
        args=self.fixture(2)
        visits=evaluate_arrivals(*args,.02,.06)
        self.assertEqual(visits[1]['reason'],'interrupted_before_arrival')
        self.assertIsNone(visits[1]['tip_error_m'])
        arrivals,times,states,tip=self.fixture(4)
        states[2:,6]=.1
        visits=evaluate_arrivals(arrivals,times,states,tip,.02,.06)
        self.assertEqual(visits[1]['reason'],'interrupted_during_window')
        self.assertAlmostEqual(visits[1]['observation_end_time_s'],.06)

    def test_collision_sample_cannot_count_as_arrival(self):
        arrivals,times,states,tip=self.fixture(4)
        states[2,6]=.1
        clearance=np.array([.1,.1,.1,-.001])
        visits=evaluate_arrivals(arrivals,times,states,tip,.02,.06,clearance)
        self.assertEqual(visits[1]['status'],'missed')
        self.assertEqual(visits[1]['reason'],'interrupted_during_window')

    def test_recorded_torque_case_distinguishes_delay_from_failure(self):
        folder=Path(__file__).resolve().parents[1]/'path_planning_lab/cases/torque'
        common=json.loads((folder/'route.json').read_text())
        for method in ('mpc_sqp','mpc_slsqp','mpc_dual_ascent','ilqr_receding','infinite','ilqr'):
            with self.subTest(method=method),np.load(folder/'runs'/f'{method}.npz') as a:
                visits=evaluate_arrivals(common['arrivals'],a['times'],a['states'],a['actual_tip'],
                                         common['timestep'],common['dwell_s'],a['clearance'])
                if method.startswith('mpc_') or method=='ilqr_receding':
                    self.assertEqual([v['status'] for v in visits[1:]],['on_time','on_time','delayed'])
                    self.assertAlmostEqual(visits[-1]['delay_s'],.16)
                    self.assertEqual(arrival_counts(visits)['reached'],3)
                else:
                    self.assertEqual(visits[-1]['status'],'missed')
                    self.assertEqual(arrival_counts(visits)['reached'],2)


if __name__=='__main__':
    unittest.main()
