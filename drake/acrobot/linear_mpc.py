"""Acrobot linear receding-horizon control: Riccati, exact-Hessian SQP, SLSQP.

SQP is specialized to a convex quadratic problem with linear torque bounds;
it is NOT a nonlinear MPC solver. Riccati completion of squares yields a
well-conditioned identity Hessian instead of condensing unstable open-loop A.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import minimize

from .lqr import _linear_model, _riccati_step, _positive_integer, _torque_limit, _state_cost, wrapped_state_error


@dataclass
class LinearMPCResult:
    states: np.ndarray
    controls: np.ndarray
    costs: list
    converged: bool
    iterations: int
    status: str
    initialization: str
    residual: float
    bound_violation: float
    unconstrained: bool
    initial_cost: float
    candidate_cost: float | None
    returned_cost: float
    native_status: str
    native_success: bool | None
    variable_scale: float
    candidate_violation: float | None
    repair_max_nm: float
    recovery: str
    relative_dual_gap: float


class LinearMPC:
    def __init__(self, plant, timestep=.05, horizon=40, Q=None, R=.1,
                 torque_limit=3., solver='riccati', terminal_cost=None,
                 max_iterations=300, tolerance=1e-6):
        if solver not in ('riccati','sqp','slsqp'):
            raise ValueError('solver must be riccati, sqp, or slsqp')
        self.N=_positive_integer(horizon,'horizon')
        self.max_iterations=_positive_integer(max_iterations,'max_iterations')
        if not np.isfinite(tolerance) or tolerance<=0:raise ValueError('tolerance must be positive')
        self.tolerance=tolerance;self.solver=solver
        self.bound=np.inf if torque_limit is None else _torque_limit(torque_limit)
        self.A,self.B,self.Q=_linear_model(plant,timestep,Q,R,None)
        self.R=R;self.target=np.array([math.pi,0.,0.,0.]);self.last_result=None
        value=self.Q.copy()
        if terminal_cost is None:
            for _ in range(10000):
                _,new=_riccati_step(self.A,self.B,self.Q,R,value)
                if np.max(abs(new-value))<1e-11*max(1.,np.max(abs(value))):
                    value=new;break
                value=new
            else:raise RuntimeError('DARE terminal cost did not converge')
        else:value=_state_cost(terminal_cost,'terminal_cost')
        self.Qf=value.copy();self.gains=np.empty((self.N,1,4));self.sqrtW=np.empty(self.N)
        for k in range(self.N-1,-1,-1):
            self.sqrtW[k]=np.sqrt(R+(self.B.T@value@self.B).item())
            self.gains[k],value=_riccati_step(self.A,self.B,self.Q,R,value)
        self.P0=value
        # u = D e0 + L v; J = .5 e0' P0 e0 + .5 v'v.
        self.D=np.zeros((self.N,4));self.L=np.zeros((self.N,self.N))
        self.T=np.empty((self.N+1,4,4));self.S=np.zeros((self.N+1,4,self.N));self.T[0]=np.eye(4)
        for k in range(self.N):
            self.D[k]=-self.gains[k]@self.T[k]
            self.L[k]=-self.gains[k]@self.S[k];self.L[k,k]+=1/self.sqrtW[k]
            self.T[k+1]=self.A@self.T[k]+self.B@self.D[k:k+1]
            self.S[k+1]=self.A@self.S[k]+self.B@self.L[k:k+1]
        self.row_norm=np.linalg.norm(self.L,axis=1)
        self.G=np.vstack((self.L,-self.L))/np.tile(self.row_norm,2)[:,None]

    def constraints(self,error):
        d=self.D@error
        return np.concatenate((self.bound-d,self.bound+d))/np.tile(self.row_norm,2)

    def from_controls(self,error,controls):
        x=error.copy();v=np.empty(self.N)
        for k,u in enumerate(controls):
            v[k]=(u+(self.gains[k]@x).item())*self.sqrtW[k]
            x=self.A@x+self.B[:,0]*u
        return v

    def initial_guess(self,error):
        x=error.copy();controls=[]
        for K in self.gains:
            u=float(np.clip(-(K@x).item(),-self.bound,self.bound))
            controls.append(u);x=self.A@x+self.B[:,0]*u
        v=self.from_controls(error,controls);label='clipped_lqr'
        if self.last_result is not None:
            old=self.last_result.controls
            shifted=np.clip(np.r_[old[1:],old[-1]],-self.bound,self.bound)
            warm=self.from_controls(error,shifted)
            if warm@warm < v@v:v=warm;label='provided'
        return v,label

    def optimality(self,v,h):
        return self.certificate(v,h)[0]

    def certificate(self,v,h):
        """Convex-QP certificate: stationarity, primal feasibility and relative dual gap.

        Nonnegative multipliers give a dual lower bound. The gap equals
        .5*||v+G.T*lambda||² + lambda.T*(h-G*v). Scale by max(1,.5*v.T*v),
        not by a fixed objective tolerance. Near-active rows are candidates for
        the multiplier fit; the dual gap also checks their complementarity.
        """
        if not np.isfinite(self.bound):return float(np.max(abs(v))),0.
        slack=h-self.G@v;active=np.flatnonzero(slack<1e-5)
        if len(active):
            from scipy.optimize import nnls
            multipliers,_=nnls(self.G[active].T,-v,maxiter=max(100,10*len(active)))
            stationarity=v+self.G[active].T@multipliers
            complementarity=float(multipliers@np.maximum(slack[active],0.))
        else:stationarity=v;complementarity=0.
        gap=float((.5*stationarity@stationarity+complementarity)/max(1.,.5*v@v))
        residual=float(max(np.max(abs(stationarity))/max(1.,np.max(abs(v))),max(0.,-np.min(slack)),gap))
        return residual,gap

    def solve_sqp(self,initial,h):
        """Feasible active-set SQP for identity Hessian and linear inequalities.

        Each subproblem has the exact Hessian. QR projects the Newton direction
        onto the current active face; blocking constraints restrict its step.
        Iterations count face changes/Newton steps, not nonlinear SQP passes.
        """
        v=initial.copy();G=self.G;active=[];history=[.5*v@v]
        for i in np.flatnonzero(abs(h-G@v)<1e-8):
            candidate=active+[int(i)]
            if np.linalg.matrix_rank(G[candidate],tol=1e-9)==len(candidate):active=candidate
        status='maximum iterations reached'
        for iteration in range(1,self.max_iterations+1):
            if active:
                Q,R=np.linalg.qr(G[active].T,mode='reduced')
                p=-v+Q@(Q.T@v)
                lam=np.linalg.solve(R,-Q.T@v)
            else:p=-v;lam=np.array([])
            if np.max(abs(p))<=self.tolerance*.1:
                if not active or np.min(lam)>=-self.tolerance*.1:
                    status='KKT tolerance';break
                active.pop(int(np.argmin(lam)));continue
            gp=G@p;slack=h-G@v
            ratios=np.full(len(h),np.inf)
            candidates=gp>1e-10
            if active:candidates[active]=False
            ratios[candidates]=np.maximum(0.,slack[candidates])/gp[candidates]
            blocking=int(np.argmin(ratios));alpha=min(1.,float(ratios[blocking]))
            v+=alpha*p;history.append(float(.5*v@v))
            if ratios[blocking]<1.-1e-10:
                candidate=active+[blocking]
                if np.linalg.matrix_rank(G[candidate],tol=1e-9)==len(candidate):active=candidate
                else:status='dependent active constraints';break
        return v,iteration,status,history

    def solve(self,measured_state,initial=None):
        measured_state=np.asarray(measured_state,dtype=float)
        if measured_state.shape!=(4,) or not np.isfinite(measured_state).all():
            raise ValueError('state must be finite and length 4')
        error=wrapped_state_error(measured_state,self.target)
        if initial is None:seed,label=self.initial_guess(error)
        else:seed=np.asarray(initial,dtype=float).copy();label='paired_initial_guess'
        if seed.shape!=(self.N,) or not np.isfinite(seed).all():
            raise ValueError('initial guess must be finite and length horizon')
        constant=float(.5*error@self.P0@error)
        scale=1.;repair=0.;recovery='none';gap=0.;native_success=None
        candidate_violation=None
        if self.solver=='riccati' or not np.isfinite(self.bound):
            v=np.zeros(self.N);history=[constant];iteration=0
            status='Riccati analytical solution' if self.solver=='riccati' else 'unconstrained analytical solution'
            native_status=status;residual=0.;converged=True
            initial_cost=constant;candidate_cost=constant
        else:
            h=self.constraints(error)
            if np.max(self.G@seed-h)>1e-5:raise ValueError('Initial guess must be feasible')
            # Keep an exactly bounded recovery plan, independently of the solver.
            seed_controls=np.clip(self.D@error+self.L@seed,-self.bound,self.bound)
            feasible_seed=self.from_controls(error,seed_controls)
            initial_cost=constant+float(.5*seed@seed)
            if self.solver=='sqp':
                candidate,iteration,native_status,hist=self.solve_sqp(seed,h)
                native_success=native_status=='KKT tolerance'
            else:
                # v can be O(1e6), despite an identity Hessian. Solve in y=v/scale
                # with objective J/scale² and bounds h/scale. The QP is unchanged.
                scale=max(1.,float(np.max(abs(seed))))
                hist=[float(.5*seed@seed)]
                res=minimize(lambda y:.5*y@y,seed/scale,jac=lambda y:y,
                             constraints={'type':'ineq','fun':lambda y:h/scale-self.G@y,'jac':lambda y:-self.G},
                             method='SLSQP',callback=lambda y:hist.append(float(.5*(y*scale)@(y*scale))),
                             options={'maxiter':self.max_iterations,'ftol':1e-13})
                candidate=res.x*scale;iteration=int(res.nit)
                native_status=str(res.message);native_success=bool(res.success)
            candidate_cost=None
            v=feasible_seed;controls=seed_controls;recovery='initial_guess'
            if np.isfinite(candidate).all():
                candidate_cost=constant+float(.5*candidate@candidate)
                raw_controls=self.D@error+self.L@candidate
                candidate_violation=float(max(0.,np.max(abs(raw_controls))-self.bound))
                # Project the physical controls, then rebuild the dynamically
                # consistent plan. Small roundoff does not discard an entire plan.
                projected=np.clip(raw_controls,-self.bound,self.bound)
                repair=float(np.max(abs(projected-raw_controls)))
                repaired=self.from_controls(error,projected) if repair>1e-8 else candidate
                if np.isfinite(repaired).all() and repaired@repaired <= feasible_seed@feasible_seed+1e-12*max(1.,feasible_seed@feasible_seed):
                    v=repaired;controls=projected if repair>1e-8 else raw_controls
                    recovery='box_projection' if repair>1e-8 else 'none'
                else:repair=0.
            history=[constant+float(c) for c in hist]
            if candidate_cost is not None and history[-1]!=candidate_cost:history.append(candidate_cost)
            residual,gap=self.certificate(v,h)
            physical_violation=float(max(0.,np.max(abs(controls))-self.bound))
            converged=residual<=self.tolerance and physical_violation<=1e-8
            status='QP certificate satisfied' if converged else 'QP certificate not satisfied'
            if recovery=='initial_guess':status+='; feasible initial-guess fallback'
            elif recovery=='box_projection':status+='; physical torque projection'
        if self.solver=='riccati' or not np.isfinite(self.bound):controls=self.D@error+self.L@v
        # Roll out the actual returned controls, rather than a rejected candidate.
        states=np.empty((self.N+1,4));states[0]=self.target+error
        for k,u in enumerate(controls):
            states[k+1]=self.target+self.A@(states[k]-self.target)+self.B[:,0]*u
        returned_cost=float(.5*sum((x-self.target)@self.Q@(x-self.target)+self.R*u*u
                                  for x,u in zip(states[:-1],controls))+
                            .5*(states[-1]-self.target)@self.Qf@(states[-1]-self.target))
        violation=0. if not np.isfinite(self.bound) else float(max(0.,np.max(abs(controls))-self.bound))
        result=LinearMPCResult(states,controls,history,converged,iteration,status,label,residual,violation,
                              self.solver=='riccati',initial_cost,candidate_cost,returned_cost,native_status,
                              native_success,scale,candidate_violation,repair,recovery,gap)
        self.last_result=result
        return result

    def command(self,measured_state):
        result=self.solve(measured_state)
        # Final actuator guard; for Riccati the planned solution is unconstrained.
        return float(np.clip(result.controls[0],-self.bound,self.bound))
