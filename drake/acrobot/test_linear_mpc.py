import unittest
import math
import numpy as np
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from pydrake.examples import AcrobotPlant
from acrobot.linear_mpc import LinearMPC
from acrobot.lqr import infinite_horizon_lqr


class LinearMPCTest(unittest.TestCase):
    def difficult_case(self,challenge='far',step=30):
        cases=json.loads((Path(__file__).parent/'control_lab/solver_benchmark_cases.json').read_text())['cases']
        p=next(p for p in cases if p['config']['challenge_id']==challenge and p['step']==step)
        return np.array(p['initial_state']),np.array(p['initial_guess'])

    def assert_returned_plan_cost(self,c,r):
        errors=r.states-c.target
        cost=.5*sum(e@c.Q@e+c.R*u*u for e,u in zip(errors[:-1],r.controls))+.5*errors[-1]@c.Qf@errors[-1]
        np.testing.assert_allclose(r.returned_cost,cost,rtol=1e-12)
        np.testing.assert_allclose(errors[1:],errors[:-1]@c.A.T+r.controls[:,None]*c.B[:,0],atol=1e-8)

    def test_difficult_scaled_slsqp_against_clarabel(self):
        from pydrake.solvers import MathematicalProgram, ClarabelSolver, SolverOptions
        for challenge,step in [('far',10),('far',30),('terminal',110)]:
            x,seed=self.difficult_case(challenge,step)
            c=LinearMPC(AcrobotPlant(),solver='slsqp');r=c.solve(x,initial=seed)
            from acrobot.lqr import wrapped_state_error
            error=wrapped_state_error(x,c.target);scale=max(1.,max(abs(seed)))
            prog=MathematicalProgram();y=prog.NewContinuousVariables(c.N,'y')
            prog.AddQuadraticCost(np.eye(c.N),np.zeros(c.N),y)
            h=c.constraints(error);prog.AddLinearConstraint(c.G,np.full(len(h),-np.inf),h/scale,y)
            solver=ClarabelSolver();opts=SolverOptions()
            for name in ['tol_gap_abs','tol_gap_rel','tol_feas','reduced_tol_gap_abs','reduced_tol_gap_rel','reduced_tol_feas']:opts.SetOption(solver.id(),name,1e-12)
            opts.SetOption(solver.id(),'static_regularization_enable',False)
            opts.SetOption(solver.id(),'max_iter',500)
            independent=solver.Solve(prog,None,opts)
            self.assertTrue(independent.is_success())
            u=c.D@error+c.L@(independent.GetSolution(y)*scale)
            np.testing.assert_allclose(r.controls,u,atol=1e-5)
            self.assertTrue(r.converged,(r.residual,r.native_status))
            self.assertNotEqual(r.recovery,'initial_guess')
            self.assertLess(r.controls[0],-2.99)
            self.assert_returned_plan_cost(c,r)

    def test_small_bound_violation_repairs_plan_without_reversing_torque(self):
        x,seed=self.difficult_case();c=LinearMPC(AcrobotPlant(),solver='slsqp')
        ref=LinearMPC(AcrobotPlant(),solver='sqp').solve(x,initial=seed)
        from acrobot.lqr import wrapped_state_error
        e=wrapped_state_error(x,c.target);u=np.clip(ref.controls,-3,3);u[0]=-3.00005
        y=c.from_controls(e,u)/max(1.,max(abs(seed)))
        def proposed(*args,**kwargs):
            kwargs['callback'](y)
            return SimpleNamespace(x=y,nit=1,message='test small violation',success=False)
        with patch('acrobot.linear_mpc.minimize',side_effect=proposed):r=c.solve(x,initial=seed)
        self.assertEqual(r.recovery,'box_projection');self.assertAlmostEqual(r.controls[0],-3.)
        self.assertLess(r.returned_cost,r.initial_cost)
        self.assert_returned_plan_cost(c,r)

    def test_fallback_cost_is_not_discarded_candidate_history(self):
        x,seed=self.difficult_case();c=LinearMPC(AcrobotPlant(),solver='slsqp')
        def broken(*args,**kwargs):
            kwargs['callback'](np.zeros(c.N))
            return SimpleNamespace(x=np.full(c.N,np.nan),nit=1,message='test nonfinite result',success=False)
        with patch('acrobot.linear_mpc.minimize',side_effect=broken):r=c.solve(x,initial=seed)
        self.assertEqual(r.recovery,'initial_guess');self.assertFalse(r.converged)
        self.assertGreater(r.returned_cost,10*r.costs[-1])
        np.testing.assert_allclose(r.returned_cost,r.initial_cost,rtol=1e-8)
        self.assert_returned_plan_cost(c,r)

    def test_completion_of_squares_and_dynamics(self):
        c=LinearMPC(AcrobotPlant(),horizon=40)
        e=np.array([-.03,.02,.01,-.01]);v=np.random.default_rng(7).normal(0,.1,c.N)
        us=c.D@e+c.L@v
        xs=np.einsum('kij,j->ki',c.T,e)+np.einsum('kij,j->ki',c.S,v)
        np.testing.assert_allclose(xs[1:],xs[:-1]@c.A.T+us[:,None]*c.B[:,0],atol=1e-10)
        cost=.5*sum(x@c.Q@x+c.R*u*u for x,u in zip(xs[:-1],us))+.5*xs[-1]@c.Qf@xs[-1]
        self.assertAlmostEqual(cost,.5*e@c.P0@e+.5*v@v,places=8)
        np.testing.assert_allclose(c.from_controls(e,us),v,atol=1e-8)

    def test_riccati_matches_infinite_with_dare_terminal(self):
        p=AcrobotPlant();c=LinearMPC(p,solver='riccati',torque_limit=None)
        x=np.array([math.pi-.03,.03,0.,0.]);K=infinite_horizon_lqr(p,timestep=.05,R=.1)
        self.assertAlmostEqual(c.command(x),-(K@(x-c.target)).item(),places=6)

    def test_sqp_and_slsqp_solve_same_constrained_problem(self):
        for offset in [.03,.3,1.]:
            a=LinearMPC(AcrobotPlant(),solver='sqp');b=LinearMPC(AcrobotPlant(),solver='slsqp')
            x=np.array([math.pi-offset,offset,0.,0.]);ra=a.solve(x);rb=b.solve(x)
            self.assertLess(ra.bound_violation,1e-6);self.assertLess(rb.bound_violation,1e-6)
            self.assertTrue(ra.converged,(offset,ra.status,ra.residual))
            self.assertLess(rb.residual,1e-4,(offset,rb.status,rb.residual))
            np.testing.assert_allclose(ra.controls,rb.controls,atol=1e-3)

    def test_against_independent_drake_qp_solver(self):
        from pydrake.solvers import MathematicalProgram, OsqpSolver, SolverOptions
        c=LinearMPC(AcrobotPlant(),horizon=12,solver='sqp')
        x=np.array([math.pi-.15,.15,0.,0.]);e=x-c.target
        r=c.solve(x);h=c.constraints(e)
        prog=MathematicalProgram();v=prog.NewContinuousVariables(c.N,'v')
        prog.AddQuadraticCost(np.eye(c.N),np.zeros(c.N),v)
        prog.AddLinearConstraint(c.G,np.full(len(h),-np.inf),h,v)
        opts=SolverOptions();solver=OsqpSolver()
        opts.SetOption(solver.id(),'eps_abs',1e-8);opts.SetOption(solver.id(),'eps_rel',1e-8)
        opts.SetOption(solver.id(),'max_iter',100000)
        result=solver.Solve(prog,None,opts)
        self.assertTrue(result.is_success())
        independent=c.D@e+c.L@result.GetSolution(v)
        np.testing.assert_allclose(r.controls,independent,atol=1e-5)

    def test_no_bounds_same_as_riccati(self):
        x=[math.pi-.1,.1,.1,-.1]
        results=[LinearMPC(AcrobotPlant(),solver=m,torque_limit=None).solve(x) for m in ['riccati','sqp','slsqp']]
        for r in results[1:]:np.testing.assert_allclose(r.controls,results[0].controls,atol=1e-12)

    def test_iteration_limit_reports_nonconvergence(self):
        c=LinearMPC(AcrobotPlant(),solver='sqp',max_iterations=1)
        r=c.solve([math.pi-.3,.3,0,0])
        self.assertFalse(r.converged);self.assertLess(r.bound_violation,1e-6)

if __name__=='__main__':unittest.main()
