import unittest
from unittest.mock import patch
import numpy as np
from pydrake.examples import AcrobotPlant
from ilqr import nonlinear_step, iterative_lqr
from nonlinear_sqp import ShootingProblem, nonlinear_sqp, RecedingHorizonSQP


class NonlinearSQPTests(unittest.TestCase):
    def setUp(self):
        self.plant=AcrobotPlant()
        self.Q=np.diag([10.,10.,1.,1.])
        self.target=np.array([np.pi,0.,0.,0.])
        self.x=np.array([2.95,.15,.1,-.2])
        self.options=dict(timestep=.05,horizon=8,Q=self.Q,R=.1,terminal_cost=self.Q,
                          initialization='lqr',max_iterations=15,tolerance=1e-5)

    def test_rk4_autodiff_matches_independent_finite_difference(self):
        step=nonlinear_step(self.plant,.05)
        for x,u in [(self.x,.8),(np.array([1.,-.7,2.,-3.]),-2.)]:
            A,B=step.jacobian(x,u);eps=1e-6
            numeric=np.column_stack([(step(x+np.eye(4)[j]*eps,u)-step(x-np.eye(4)[j]*eps,u))/(2*eps) for j in range(4)])
            np.testing.assert_allclose(A,numeric,atol=1e-8,rtol=1e-7)
            np.testing.assert_allclose(B[:,0],(step(x,u+eps)-step(x,u-eps))/(2*eps),atol=1e-8)

    def test_full_nonlinear_cost_gradient_and_curvature(self):
        problem=ShootingProblem(self.plant,self.x,self.Q,.1,self.Q,.05,self.target,3.)
        u=np.linspace(-.3,.5,8);g,H=problem.derivatives(u);eps=1e-6
        numerical=np.array([(problem.value(u+np.eye(8)[j]*eps)[1]-problem.value(u-np.eye(8)[j]*eps)[1])/(2*eps) for j in range(8)])
        np.testing.assert_allclose(g,numerical,rtol=1e-6,atol=1e-6)
        self.assertGreater(np.linalg.eigvalsh(H).min(),0.)

    def test_both_solvers_relinearize_and_return_nonlinear_feasible_plan(self):
        seed=iterative_lqr(self.plant,self.x,initialize_only=True,**self.options)
        for solver in ('sqp','slsqp'):
            for bound in (3.,None):
                with self.subTest(solver=solver,bound=bound):
                    r=nonlinear_sqp(self.plant,self.x,solver=solver,torque_limit=bound,**self.options)
                    expected=iterative_lqr(self.plant,self.x,initialize_only=True,torque_limit=bound,**self.options)
                    np.testing.assert_array_equal(r.initial_controls,expected.controls)
                    np.testing.assert_array_equal(r.terminal_cost,seed.terminal_cost)
                    self.assertGreater(len(set(v['trajectory_hash'] for v in r.linearizations)),1)
                    self.assertTrue(any((v['max_A_change'] or 0)>1e-6 for v in r.linearizations))
                    step=nonlinear_step(self.plant,.05)
                    for k,u in enumerate(r.controls):np.testing.assert_allclose(step(r.states[k],u),r.states[k+1],atol=1e-12)
                    if bound:self.assertLessEqual(max(abs(r.controls)),bound+1e-10)
                    problem=ShootingProblem(self.plant,self.x,self.Q,.1,self.Q,.05,self.target,np.inf)
                    self.assertAlmostEqual(problem.value(r.controls)[1],r.returned_cost,places=9)
                    self.assertLess(r.returned_cost,r.initial_cost)
                    if solver=='sqp':self.assertTrue(np.all(np.diff(r.costs)<0))

    def test_receding_restarts_at_measurement_with_shifted_plan(self):
        for solver in ('sqp','slsqp'):
            controller=RecedingHorizonSQP(self.plant,solver=solver,torque_limit=3.,**self.options)
            controller.command(self.x);last=controller.last_result
            x=last.states[1]+np.array([0.,0.,.3,-.2])
            with patch('nonlinear_sqp.nonlinear_sqp',wraps=nonlinear_sqp) as solve:controller.command(x)
            np.testing.assert_array_equal(solve.call_args.kwargs['initial_controls'],np.r_[last.controls[1:],last.controls[-1]])
            np.testing.assert_array_equal(controller.last_result.states[0],x)

    def test_local_solution_agrees_with_ilqr(self):
        opts=self.options|dict(horizon=12,max_iterations=100,torque_limit=3.,tolerance=1e-7)
        x=self.target+np.array([-.005,.005,0.,0.])
        reference=iterative_lqr(self.plant,x,**opts)
        for solver in ('sqp','slsqp'):
            r=nonlinear_sqp(self.plant,x,solver=solver,**opts)
            np.testing.assert_allclose(r.returned_cost,reference.costs[-1],rtol=1e-5,atol=1e-8)
            np.testing.assert_allclose(r.controls,reference.controls,atol=2e-4)

if __name__=='__main__':unittest.main()
