"""Planning validity, moving references, and affine tracking regressions."""
from contextlib import redirect_stdout
import io
import unittest
from dataclasses import replace

import mujoco
import numpy as np
from numpy.testing import assert_allclose

from arm.model import ArmModel
from arm.planning import CollisionWorld,plan_route,rrt_connect,segment_distance
from arm.simulate import load_config
from arm.tracking import prepare_nominal,condense,TrackingProblem,TrajectoryController
from arm.route_simulation import run_route
from arm.route_cases import case_settings, external_torque, make_plant


class PlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=load_config(mode='mpc',solver='sqp',horizon=20,ilqr_max_iterations=2)
        cls.arm=ArmModel(torque_limit=cls.config['torque_limit'])
        cls.route,cls.world=plan_route(cls.arm)
        cls.nominal=prepare_nominal(cls.arm,cls.route,cls.config)

    def test_rrt_detours_validate_every_edge_and_respect_time_bounds(self):
        self.assertFalse(self.route.planning[0]['direct_valid'])
        self.assertFalse(self.route.planning[2]['direct_valid'])
        for path in self.route.paths:
            for a,b in zip(path,path[1:]):
                self.assertTrue(self.world.edge(a,b))
        self.assertLessEqual(np.max(np.abs(self.route.states[:,6:])),.65+1e-12)
        self.assertLessEqual(np.max(np.abs(self.route.accelerations)),1.2+1e-12)
        for arrival in self.route.arrivals:
            assert_allclose(self.route.states[arrival['index'],6:],0,atol=1e-12)
        for q in self.route.states[::5,:6]:
            self.assertGreaterEqual(self.world.clearance(q),self.world.margin)

    def test_rrt_reproducibility_and_failure_has_no_unchecked_fallback(self):
        def valid(q): return np.linalg.norm(q)>.3
        def edge(a,b): return all(valid(q) for q in np.linspace(a,b,50))
        args=(np.array([-1.,0]),np.array([1.,0]),valid,edge,-np.ones(2),np.ones(2))
        first,_=rrt_connect(*args,seed=2)
        second,_=rrt_connect(*args,seed=2)
        assert_allclose(first,second,atol=0,rtol=0)
        with self.assertRaises(RuntimeError):
            rrt_connect(args[0],args[1],lambda q:True,lambda a,b:False,args[-2],args[-1],max_iterations=5)
        with self.assertRaises(ValueError):
            rrt_connect(np.zeros(2),args[1],valid,edge,args[-2],args[-1])

    def test_collision_and_inverse_dynamics(self):
        self.assertAlmostEqual(segment_distance(np.array([0.,0,0]),np.array([1.,0,0]),
                                               np.array([.5,-1.,0]),np.array([.5,1.,0])),0.)
        for idx in (30,150,350):
            x=self.route.states[idx]
            acceleration=self.arm.derivative(x,self.nominal.controls[idx])[6:]
            assert_allclose(acceleration,self.route.accelerations[idx],atol=1e-9)
        midpoint=(self.route.states[0,:6]+self.route.states[self.route.arrivals[1]['index'],:6])/2
        self.assertFalse(self.world.valid(midpoint))

    def test_affine_condensing_matches_time_varying_recursion(self):
        rng=np.random.default_rng(4);N=4
        A=rng.normal(size=(N,12,12))*.1;B=rng.normal(size=(N,12,6))*.1
        d=rng.normal(size=(N,12))*.01;x=rng.normal(size=12);u=rng.normal(size=(N,6))
        S,T,D=condense(A,B,d);expected=[];state=x.copy()
        for k in range(N):
            state=A[k]@state+B[k]@u[k]+d[k];expected.extend(state)
        assert_allclose(S@u.ravel()+T@x+D,expected,atol=1e-12)

    def test_ilqr_tracking_cost_uses_each_reference_sample(self):
        p=TrackingProblem(self.arm,self.nominal,50,8)
        self.assertEqual(p.cost(p.reference_states,p.reference_controls),0.)
        constant=np.tile(p.reference_states[-1],(9,1))
        self.assertGreater(p.cost(constant,p.reference_controls),0.)
        c=dict(self.config,mode='ilqr_receding')
        controller=TrajectoryController(self.arm,self.nominal,c)
        u=controller.command(self.route.states[50],50)
        self.assertTrue(np.isfinite(u).all())
        self.assertTrue(np.all(np.abs(u)<=self.arm.limit))

    def test_continuous_route_reaches_each_button_without_resets(self):
        with redirect_stdout(io.StringIO()):
            arrays,result=run_route(self.config,self.arm,self.route,self.nominal)
        self.assertTrue(result['completed'])
        self.assertTrue(all(v['reached'] for v in result['visits']))
        self.assertGreater(result['min_clearance_m'],.025)
        self.assertLess(result['rms_tip_error_m'],.002)
        self.assertEqual(result['nonconverged_solves'],0)
        # No teleporting at button transitions: every sample is a dynamics step.
        for visit in self.route.arrivals[1:]:
            k=visit['index']
            assert_allclose(arrays['states'][k],self.arm.step(arrays['states'][k-1],arrays['controls'][k-1]),atol=1e-12)

    def test_payload_changes_only_plant_mass_gravity_and_inertia(self):
        config=dict(self.config,experiment=case_settings('payload'))
        before=self.arm.model.body_mass.copy()
        plant=make_plant(config)
        assert_allclose(self.arm.model.body_mass,before,atol=0,rtol=0)
        self.assertAlmostEqual(plant.model.body_mass.sum()-before.sum(),.5)
        body=plant.model.body('tool_roll').id
        self.assertTrue(np.all(plant.model.body_inertia[body]>0))
        self.assertGreater(plant.model.body_ipos[body,2],self.arm.model.body_ipos[body,2])
        x=self.route.states[0];u=self.nominal.controls[0]
        self.assertGreater(np.linalg.norm(plant.derivative(x,u)-self.arm.derivative(x,u)),.1)
        # Independent static check: extra gravity torque equals -J_tip^T m g.
        extra=plant.equilibrium_control(x)-self.arm.equilibrium_control(x)
        jacp=np.zeros((3,6));jacr=np.zeros((3,6))
        mujoco.mj_jacSite(self.arm.model,self.arm.data,jacp,jacr,self.arm.model.site('button_tip').id)
        assert_allclose(extra,-jacp.T@(.5*self.arm.model.opt.gravity),atol=1e-10)
        same=make_plant(self.config)
        assert_allclose(same.step(x,u),self.arm.step(x,u),atol=1e-12)

    def test_external_pulse_duration_and_reset(self):
        experiment=case_settings('disturbance')
        forces=np.array([external_torque(experiment,k*.02) for k in range(400)])
        self.assertEqual(np.count_nonzero(np.linalg.norm(forces,axis=1)),15)
        assert_allclose(forces[300],[6,-10,5,0,0,0])
        assert_allclose(forces[315],0)
        x=self.route.states[0];u=self.nominal.controls[0]
        baseline=self.arm.step(x,u)
        pushed=self.arm.step(x,u,forces[300])
        self.assertGreater(np.linalg.norm(pushed-baseline),.001)
        assert_allclose(self.arm.step(x,u),baseline,atol=1e-12)

    def test_route_applies_disturbance_to_plant_without_resetting_state(self):
        experiment=case_settings('disturbance')
        experiment['pulse'].update(start_s=.02,duration_s=.04)
        config=dict(self.config,mode='infinite',experiment=experiment)
        route=replace(self.route,states=self.route.states[:5],accelerations=self.route.accelerations[:5],
                      arrivals=self.route.arrivals[:1])
        with redirect_stdout(io.StringIO()):
            arrays,summary=run_route(config,self.arm,route,self.nominal)
        self.assertEqual(summary['case'],'disturbance')
        self.assertTrue(summary['completed'])
        self.assertEqual(np.count_nonzero(np.linalg.norm(arrays['external_torques'],axis=1)),2)
        plant=make_plant(config)
        for k in range(4):
            assert_allclose(arrays['states'][k+1],plant.step(arrays['states'][k],arrays['controls'][k],
                            arrays['external_torques'][k]),atol=1e-12)

    def test_infeasible_nominal_uses_unclipped_affine_dynamics(self):
        config=dict(self.config,torque_limit=[.1]*6,experiment=dict(allow_infeasible=True))
        arm=ArmModel(torque_limit=config['torque_limit'])
        route=replace(self.route,states=self.route.states[:3],accelerations=self.route.accelerations[:3])
        nominal=prepare_nominal(arm,route,config)
        self.assertTrue(np.any(np.abs(nominal.controls)>arm.limit))
        self.assertTrue(arm.model.actuator_ctrllimited.all())
        limited=arm.step(route.states[0],nominal.controls[0])-route.states[1]
        self.assertGreater(np.linalg.norm(nominal.defect[0]-limited),.01)
        arm.model.actuator_ctrllimited[:]=False
        expected=arm.step(route.states[0],nominal.controls[0])-route.states[1]
        assert_allclose(nominal.defect[0],expected,atol=1e-12)


if __name__=='__main__':
    unittest.main()
