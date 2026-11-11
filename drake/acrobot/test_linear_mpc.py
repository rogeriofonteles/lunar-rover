import unittest
import math
import numpy as np
from pydrake.examples import AcrobotPlant
from acrobot.linear_mpc import LinearMPC
from acrobot.lqr import infinite_horizon_lqr


class LinearMPCTest(unittest.TestCase):
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
