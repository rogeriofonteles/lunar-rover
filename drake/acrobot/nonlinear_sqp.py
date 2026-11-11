"""Nonlinear shooting MPC: trajectory SQP (Gauss–Newton) and SciPy SLSQP.

States are eliminated by a fresh nonlinear RK4 rollout. At each derivative
request, A_k and B_k are recomputed along that rollout with Drake AutoDiff.
SQP explicitly solves a new bound-constrained QP; SLSQP uses the nonlinear
objective/gradient and its own quasi-Newton SQP updates. Neither uses the fixed
upright linear model. All three nonlinear solvers share iLQR initialization.
"""
from dataclasses import dataclass
import hashlib
import numpy as np
from scipy.optimize import minimize
from pydrake.solvers import MathematicalProgram, ClarabelSolver, SolverOptions
if __package__:
    from .ilqr import iterative_lqr, nonlinear_step
    from .lqr import wrapped_state_error
else:
    from ilqr import iterative_lqr, nonlinear_step
    from lqr import wrapped_state_error


@dataclass
class NonlinearResult:
    states: np.ndarray
    controls: np.ndarray
    costs: list
    converged: bool
    iterations: int
    status: str
    terminal_cost: np.ndarray
    initialization: str
    initial_controls: np.ndarray
    initial_cost: float
    candidate_cost: float | None
    returned_cost: float
    native_status: str
    native_success: bool
    residual: float
    bound_violation: float
    linearizations: list
    recovery: str
    formulation: str = 'nonlinear_shooting'


class ShootingProblem:
    """Nonlinear objective with exact RK4 first derivatives and GN curvature."""
    def __init__(self,plant,x0,Q,R,Qf,timestep,target,bound):
        self.dynamics=nonlinear_step(plant,timestep)
        self.x0=np.asarray(x0,dtype=float);self.Q=Q;self.R=R;self.Qf=Qf
        self.target=target;self.bound=bound;self.cached_u=None;self.cached=None
        self.linearizations=[];self.previous_A=None
        self.best=None

    def value(self,u):
        u=np.asarray(u,dtype=float)
        if self.cached_u is not None and np.array_equal(u,self.cached_u):return self.cached
        xs=[self.x0.copy()];cost=0.
        try:
            for torque in u:
                e=wrapped_state_error(xs[-1],self.target)
                cost+=.5*(e@self.Q@e+self.R*torque*torque)
                xs.append(self.dynamics(xs[-1],torque))
            e=wrapped_state_error(xs[-1],self.target);cost+=.5*e@self.Qf@e
            if not np.isfinite(cost):raise FloatingPointError('Nonfinite cost')
            xs=np.asarray(xs);cost=float(cost)
        except (FloatingPointError,OverflowError):
            xs=None;cost=1e30+float(min(1e29,np.linalg.norm(u)**2))
        self.cached_u=u.copy();self.cached=(xs,cost)
        if xs is not None and np.max(abs(u))<=self.bound+1e-9:
            if self.best is None or cost<self.best[2]:self.best=(xs.copy(),u.copy(),cost)
        return self.cached

    def derivatives(self,u):
        xs,cost=self.value(u);n=len(u)
        if xs is None:return 2*np.asarray(u),np.eye(n)
        sensitivity=np.zeros((4,n));gradient=self.R*np.asarray(u).copy();H=self.R*np.eye(n);As=[]
        for k in range(n):
            e=wrapped_state_error(xs[k],self.target)
            gradient+=sensitivity.T@self.Q@e;H+=sensitivity.T@self.Q@sensitivity
            A,B=self.dynamics.jacobian(xs[k],u[k]);As.append(A)
            sensitivity=A@sensitivity;sensitivity[:,k]+=B[:,0]
        e=wrapped_state_error(xs[-1],self.target)
        gradient+=sensitivity.T@self.Qf@e;H+=sensitivity.T@self.Qf@sensitivity
        As=np.asarray(As)
        change=None if self.previous_A is None else float(np.max(abs(As-self.previous_A)))
        self.linearizations.append(dict(evaluation=len(self.linearizations),trajectory_hash=hashlib.sha256(xs.tobytes()+np.asarray(u).tobytes()).hexdigest()[:12],max_A_change=change,A_first_norm=float(np.linalg.norm(As[0])),A_last_norm=float(np.linalg.norm(As[-1]))))
        self.previous_A=As
        return gradient,(H+H.T)/2

    def stationarity(self,u,scale):
        gradient,_=self.derivatives(u)
        projected=u-np.clip(u-gradient/scale,-self.bound,self.bound)
        return float(np.max(abs(projected)))


def quadratic_step(H,g,lower,upper):
    """Scaled convex QP for a *new* trajectory-local model at each SQP iteration."""
    diagonal=np.sqrt(np.maximum(np.diag(H),1e-12))
    matrix=H/diagonal[:,None]/diagonal[None,:];vector=g/diagonal
    prog=MathematicalProgram();z=prog.NewContinuousVariables(len(g),'du_scaled')
    prog.AddQuadraticCost(matrix,vector,z)
    prog.AddBoundingBoxConstraint(lower*diagonal,upper*diagonal,z)
    solver=ClarabelSolver();options=SolverOptions()
    for key in ['tol_gap_abs','tol_gap_rel','tol_feas']:options.SetOption(solver.id(),key,1e-11)
    options.SetOption(solver.id(),'static_regularization_enable',False)
    result=solver.Solve(prog,None,options)
    if not result.is_success():return None,str(result.get_solution_result())
    du=result.GetSolution(z)/diagonal
    if not np.isfinite(du).all():return None,'nonfinite QP direction'
    return np.clip(du,lower,upper),'QP solved'


def nonlinear_sqp(plant,initial_state,*,solver='sqp',**options):
    if solver not in ('sqp','slsqp'):raise ValueError('solver must be sqp or slsqp')
    # This call selects the exact same seed/Qf as iLQR, without an optimization pass.
    seed=iterative_lqr(plant,initial_state,initialize_only=True,**options)
    u=seed.controls.copy();Q=np.asarray(options.get('Q') if options.get('Q') is not None else np.diag([10.,10.,1.,1.]))
    R=options.get('R',1.);target=np.asarray(options.get('equilibrium_state') if options.get('equilibrium_state') is not None else [np.pi,0.,0.,0.])
    limit=options.get('torque_limit');bound=np.inf if limit is None else float(limit)
    problem=ShootingProblem(plant,initial_state,Q,R,seed.terminal_cost,options.get('timestep',.01),target,bound)
    _,initial_cost=problem.value(u);objective_scale=max(1.,initial_cost/len(u))
    history=[initial_cost];budget=options.get('max_iterations',50);tol=options.get('tolerance',1e-6)
    damping=options.get('regularization',1e-5);status='maximum iterations reached';success=False
    candidate_cost=initial_cost
    if solver=='sqp':
        for iteration in range(1,budget+1):
            xs,cost=problem.value(u);g,H=problem.derivatives(u)
            pg=float(np.max(abs(u-np.clip(u-g/objective_scale,-bound,bound))))
            if pg<=tol:status='projected-gradient tolerance';success=True;break
            # Trust bound is on the optimization increment, not actuator torque.
            radius=10. if not np.isfinite(bound) else max(1.,2*bound)
            lo=np.maximum(-bound-u,-radius);hi=np.minimum(bound-u,radius)
            du,qp_status=quadratic_step((H+damping*np.eye(len(u)))/objective_scale,g/objective_scale,lo,hi)
            if du is None:damping*=10.;status=qp_status;continue
            accepted=False
            for alpha in [1.,.5,.25,.125,.0625,.03125,.015625,.0078125]:
                trial=np.clip(u+alpha*du,-bound,bound);_,value=problem.value(trial)
                if value<cost and value<=cost+1e-4*alpha*min(0.,g@du):
                    u=trial;history.append(value);candidate_cost=value;accepted=True
                    damping=max(options.get('regularization',1e-5),damping/2.)
                    break
            if not accepted:
                damping*=10.
                if damping>1e12:status='line search failed';break
    else:
        # Positive scaling changes neither the nonlinear objective minimizer nor bounds.
        variable_scale=max(1.,float(np.max(abs(u))))
        def fun(y):return problem.value(y*variable_scale)[1]/objective_scale
        def jac(y):return problem.derivatives(y*variable_scale)[0]*variable_scale/objective_scale
        def callback(y):history.append(problem.value(y*variable_scale)[1])
        result=minimize(fun,u/variable_scale,jac=jac,method='SLSQP',
                        bounds=None if not np.isfinite(bound) else [(-bound/variable_scale,bound/variable_scale)]*len(u),
                        callback=callback,options={'maxiter':budget,'ftol':min(1e-9,tol*.01)})
        iteration=int(result.nit);status=str(result.message);success=bool(result.success)
        _,candidate_cost=problem.value(np.clip(result.x*variable_scale,-bound,bound))
    states,controls,cost=problem.best
    residual=problem.stationarity(controls,objective_scale)
    return NonlinearResult(states=states,controls=controls,costs=history,converged=residual<=tol,
                           iterations=iteration,status='stationarity satisfied' if residual<=tol else 'stationarity tolerance not reached',
                           terminal_cost=seed.terminal_cost,initialization=seed.initialization,initial_controls=seed.controls,
                           initial_cost=initial_cost,candidate_cost=candidate_cost,returned_cost=cost,native_status=status,native_success=success,
                           residual=residual,bound_violation=0. if not np.isfinite(bound) else float(max(0.,max(abs(controls))-bound)),
                           linearizations=problem.linearizations,recovery='best_nonlinear_rollout')


class RecedingHorizonSQP:
    def __init__(self,plant,solver='sqp',**options):
        self.plant=plant;self.solver=solver;self.solver_options=dict(options);self.last_result=None
        self.terminal_cost=options.get('terminal_cost')
    def command(self,state):
        seed=None if self.last_result is None else np.r_[self.last_result.controls[1:],self.last_result.controls[-1]]
        options=dict(self.solver_options);options['terminal_cost']=self.terminal_cost
        self.last_result=nonlinear_sqp(self.plant,state,solver=self.solver,initial_controls=seed,**options)
        self.terminal_cost=self.last_result.terminal_cost
        return float(self.last_result.controls[0])
